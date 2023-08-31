/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*
* 		ESP32 TFG main project
* 		----------------------
*
* 		This project is the main where I have programmed three temperature, pressure, humidity and aire quality sensors.
*
* 		These sensors will take data from time to time, connect to a previously configured Wifi network, and send this data
* 		in different "topics" using the MQTT protocol.
*
* 		They are also programmed to take the different timestamps from an SMTP server, and are also sent by MQTT to then be
* 		able to treat them using a script in MATLAB to generate a well-ordered Excel document and be able to graph the solutions obtained.
*
*
* 		
* 		Electronic Industry and Automatic Engineering
* 		University of Almería, 2022
*
*
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


#include <time.h>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "driver/i2c.h"
#include "esp_attr.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_sleep.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include "CCS811.h"
#include "BME680.h"
#include "MS5611.h"

#ifdef __cplusplus
extern "C" {
#endif

#define		number_samples			5
#define		MAX_DELAY			3600000		// 3600000 	milliseconds = 3600 seconds = 1 hour
#define		MIN_DELAY			5000		// 5000		milliseconds = 5	seconds
#define		DELAY_SIZE			20
#define		TIMEZONE			2		// timezone-zone for spain: UTC +2
#define		YEAR_OFFSET			1900
#define		MONTH_OFFSET			1

static 	const char 			*TAG 		= 		"MQTT_TFG";
static	int				GLOBAL_DELAY	=		1200000;		// 30 minutes in this moment

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

void 		hal_i2c_init();
static void 	log_error_if_nonzero(const char *message, int error_code);
static void 	mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void 		initialize_sntp();
void 		obtain_time();
void 		send_mqtt_time_data(esp_mqtt_client_handle_t client, int sec, int min, int hour, int day, int month, int year);


// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/*
 * @brief 	Event handler registered to receive MQTT events
 *
 *  		This function is called by the MQTT client event loop.
 *
 * @param[in] handler_args	: 	user data registered to the event.
 * @param[in] base			:	Event base for the handler(always MQTT Base in this example).
 * @param[in] event_id 		:	The id for the received event.
 * @param[in] event_data 	:	The data for the event, esp_mqtt_event_handle_t.
 *
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
	int			i;
	char			read_topic[50], read_data[50];
	esp_err_t		ret;

	ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

        msg_id = esp_mqtt_client_subscribe(client, "ESP_control/run", 1);
        ESP_LOGI(TAG, "sent subscribe to ESP_control successful!!, msg_id = %d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "ESP_control/delay", 1);
		ESP_LOGI(TAG, "sent subscribe to ESP_delay successful!!, msg_id = %d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);

        for(i = 0; i <= event->topic_len; i++) {
        	if(i == event->topic_len) read_topic[i] = '\0';
        	else read_topic[i] = event->topic[i];
        }

        for(i = 0; i <= event->data_len; i++) {
        	if(i == event->data_len) read_data[i] = '\0';
        	else read_data[i] = event->data[i];
        }

        if(strcmp(read_topic, "ESP_control/run") == 0) {
        	if(strcmp(read_data, "STOP") == 0) {
				ret = esp_mqtt_client_publish(client, "ESP_control", "MQTT SERVICE STOPPED", 0, 1, 0);
					if(ret == -1) {
						printf("ERROR sending 'MQTT SERVICE STOPPED' to topic 'ESP_control'\n");
					}
				ESP_LOGI(TAG, "MQTT SERVICE STOPPED received message");

				ret = esp_mqtt_client_disconnect(client);
				if(ret != ESP_OK) printf("ERROR disconnecting MQTT client: %x\n", ret);
				else ESP_LOGI(TAG, "MQTT client succesful disconnected !!");
			}
			else if (strcmp(read_data, "START") == 0) {
				ret = esp_mqtt_client_start(client);
				if(ret != ESP_OK) {
					ret = esp_mqtt_client_reconnect(client);
					if(ret != ESP_OK) {
						ESP_LOGE(TAG, "ERROR reconnecting MQTT client!!!");
					}
				}

				ret = esp_mqtt_client_publish(client, "ESP_control", "MQTT SERVICE STARTED", 0, 1, 0);
					if(ret == -1) {
						printf("ERROR sending 'MQTT SERVICE STARTED' to topic 'ESP_control'\n");
					}
				ESP_LOGI(TAG, "MQTT SERVICE STARTED");
			}
        }
        else if(strcmp(read_topic, "ESP_control/delay") == 0) {
        	GLOBAL_DELAY = atoi(event->data);	//	convert char* to float variable

			if(GLOBAL_DELAY > MAX_DELAY) {
				GLOBAL_DELAY = MAX_DELAY;
			}
			else if(GLOBAL_DELAY < MIN_DELAY) {
				GLOBAL_DELAY = MIN_DELAY;
			}

			ESP_LOGI(TAG, "DELAY SELECTED: %d", GLOBAL_DELAY);
        }


        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}


/*
 * @brief		Obtain online time data
 *
 */
void initialize_sntp()
{
	ESP_LOGI(TAG, "Initializing SNTP");
	sntp_setoperatingmode(SNTP_OPMODE_POLL);
	sntp_setservername(0, "es.pool.ntp.org");
	ESP_LOGI(TAG, "Notification of a time synchronization event");
	sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);

	sntp_init();
}


/*
 * @brief		Obtain local time data from SNTP server
 *
 */
void obtain_time()
{
	time_t	now = 0;
	struct tm timeinfo = {0};
	int retry = 0;
	const int retry_count = 2;		// delay seconds to obtain the current date/time data

	while(sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
		ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
		vTaskDelay(2000/portTICK_RATE_MS);
	}

	time(&now);
	localtime_r(&now, &timeinfo);		// Obtain the local time of the machine and write a function to save it on -> timeinfo

}


/*
 * @brief		Send data time function by mqtt
 *
 */
void send_mqtt_time_data(esp_mqtt_client_handle_t client, int sec, int min, int hour, int day, int month, int year)
{
	char			*topic;
	char 			mqtt_data[100];
	esp_err_t		ret;

	sprintf(mqtt_data, "%d", year+YEAR_OFFSET);
	topic = "sensor/time/year";
	ret = esp_mqtt_client_publish(client, topic, mqtt_data, 0, 1, 0);
		if(ret == -1) {
			printf("ERROR sending data to topic %s\n", topic);
		}

	sprintf(mqtt_data, "%d", month+MONTH_OFFSET);
	topic = "sensor/time/month";
	ret = esp_mqtt_client_publish(client, topic, mqtt_data, 0, 1, 0);
		if(ret == -1) {
			printf("ERROR sending data to topic %s\n", topic);
		}

	sprintf(mqtt_data, "%d", day);
	topic = "sensor/time/day";
	ret = esp_mqtt_client_publish(client, topic, mqtt_data, 0, 1, 0);
		if(ret == -1) {
			printf("ERROR sending data to topic %s\n", topic);
		}

	sprintf(mqtt_data, "%d", hour);
	topic = "sensor/time/hour";
	ret = esp_mqtt_client_publish(client, topic, mqtt_data, 0, 1, 0);
		if(ret == -1) {
			printf("ERROR sending data to topic %s\n", topic);
		}

	sprintf(mqtt_data, "%d", min);
	topic = "sensor/time/min";
	ret = esp_mqtt_client_publish(client, topic, mqtt_data, 0, 1, 0);
		if(ret == -1) {
			printf("ERROR sending data to topic %s\n", topic);
		}

	sprintf(mqtt_data, "%d", sec);
	topic = "sensor/time/sec";
	ret = esp_mqtt_client_publish(client, topic, mqtt_data, 0, 1, 0);
		if(ret == -1) {
			printf("ERROR sending data to topic %s\n", topic);
		}
}


void app_main(void)
{
	wifi_ap_record_t			ap_info;
	BME680_calib_t				NVM_coef;
	uint16_t				mode_number, eco2, TVOC;
	float					hum_comp, BME680_t, BME680_p, MS5611_t, MS5611_p;
	esp_err_t				ret;
	char					*topic;
	char					mqtt_data[100], strftime_buf[64];

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

	mode_number 	= 	1;

	hal_i2c_init();				// Master initialition

	vTaskDelay(100/portTICK_RATE_MS);

	BME680_init(&NVM_coef);			// BME680 slave initialition

	vTaskDelay(100/portTICK_RATE_MS);

	CCS811_init(mode_number);		// CCS811 slave initialition

	vTaskDelay(100/portTICK_RATE_MS);

	MS5611_init();				// MS5611 slave initialition

	vTaskDelay(100/portTICK_RATE_MS);



    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    	// SNTP configuring

		time_t now;
		struct tm timeinfo;
		time(&now);
		localtime_r(&now, &timeinfo);

		sntp_servermode_dhcp(1);	// Set 1 if you want to request that the servers to be used for SNTP should be requested from the DHCP. Set 0 to not
		initialize_sntp();		// Function to initialize sntp and obtain online time data from server

	// end SNTP configuring

    // Initialition of mqtt

    esp_mqtt_client_config_t mqtt_cfg = {
			.uri = CONFIG_BROKER_URL,
		};
	#if CONFIG_BROKER_URL_FROM_STDIN
		char line[128];

		if (strcmp(mqtt_cfg.uri, "FROM_STDIN") == 0) {
			int count = 0;
			printf("Please enter url of mqtt broker\n");
			while (count < 128) {
				int c = fgetc(stdin);
				if (c == '\n') {
					line[count] = '\0';
					break;
				} else if (c > 0 && c < 127) {
					line[count] = c;
					++count;
				}
				vTaskDelay(10 / portTICK_PERIOD_MS);
			}
			mqtt_cfg.uri = line;
			printf("Broker url: %s\n", line);
		} else {
			ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
			abort();
		}
	#endif /* CONFIG_BROKER_URL_FROM_STDIN */

	esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
	/* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
	esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
	esp_mqtt_client_start(client);

    // End initialition of mqtt

    vTaskDelay(1000/portTICK_RATE_MS);

    while(1) {
    	if(ap_info.rssi == 0) {
			do {
				ESP_ERROR_CHECK(example_connect());
				vTaskDelay(4000/portTICK_RATE_MS);
			} while(ap_info.rssi == 0);
		}

    	printf("Connecting power: %i [dB]\n\n", ap_info.rssi);

    	CCS811_read_all_variables(mode_number, &eco2, &TVOC, &hum_comp, &BME680_t, &BME680_p, &NVM_coef);

    	vTaskDelay(100/portTICK_RATE_MS);

    	MS5611_t = MS5611_read_temperature();
    	MS5611_p = MS5611_read_pressure();

		sprintf(mqtt_data, "%d", eco2);
		topic = "sensor/CCS811/eco2";
		ret = esp_mqtt_client_publish(client, topic, mqtt_data, 0, 1, 0);
			if(ret == -1) {
				printf("ERROR sending data to topic %s\n", topic);
			}

		sprintf(mqtt_data, "%d", TVOC);
		topic = "sensor/CCS811/TVOC";
		ret = esp_mqtt_client_publish(client, topic, mqtt_data, 0, 1, 0);
			if(ret == -1) {
				printf("ERROR sending data to topic %s\n", topic);
			}


		sprintf(mqtt_data, "%f", BME680_t);
		topic = "sensor/BME680/Temperature";
		ret = esp_mqtt_client_publish(client, topic, mqtt_data, 0, 1, 0);
			if(ret == -1) {
				printf("ERROR sending data to topic %s\n", topic);
			}

		sprintf(mqtt_data, "%f", BME680_p);
		topic = "sensor/BME680/Pressure";
		ret = esp_mqtt_client_publish(client, topic, mqtt_data, 0, 1, 0);
			if(ret == -1) {
				printf("ERROR sending data to topic %s\n", topic);
			}

		sprintf(mqtt_data, "%f", hum_comp);
		topic = "sensor/BME680/Humidity";
		ret = esp_mqtt_client_publish(client, topic, mqtt_data, 0, 1, 0);
			if(ret == -1) {
				printf("ERROR sending data to topic %s\n", topic);
			}

		sprintf(mqtt_data, "%f", MS5611_t);
		topic = "sensor/MS5611/Temperature";
		ret = esp_mqtt_client_publish(client, topic, mqtt_data, 0, 1, 0);
			if(ret == -1) {
				printf("ERROR sending data to topic %s\n", topic);
			}

		sprintf(mqtt_data, "%f", MS5611_p);
		topic = "sensor/MS5611/Pressure";
		ret = esp_mqtt_client_publish(client, topic, mqtt_data, 0, 1, 0);
			if(ret == -1) {
				printf("ERROR sending data to topic %s\n", topic);
			}

		// Function for obtain current time
		obtain_time();
		time(&now);

		setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1); // https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv
		tzset();
		localtime_r(&now, &timeinfo);
		strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
		ESP_LOGI(TAG, "The current date/time in Almeria is: %s", strftime_buf);
		// end of function of obtaining current date/time in Almeria

		send_mqtt_time_data(client, timeinfo.tm_sec, timeinfo.tm_min, timeinfo.tm_hour, timeinfo.tm_mday, timeinfo.tm_mon, timeinfo.tm_year);

		printf("eco2 (CCS811): %d\nTVOC (CCS811): %d ppm\nTemperature (BME680): %f Celsius\nTemperature (MS5611): %f Celsius\n", eco2, TVOC, BME680_t, MS5611_t);
		printf("Pressure (BME680): %f hPa\nPressure (MS5611): %f hPa\nRelative humidity (BME680): %f\n\n", BME680_p, MS5611_p, hum_comp);

		ret = esp_wifi_sta_get_ap_info(&ap_info);
		if(ret != ESP_OK) {
			printf("ERROR reading wifi info....\n");
		}

		vTaskDelay(GLOBAL_DELAY/portTICK_RATE_MS);

    }
}


#ifdef __cplusplus
}
#endif //__cplusplus


