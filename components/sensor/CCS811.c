/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*	CCS811 sensor DRIVER writed in C
*	-------------------------------------------
*
*	***************************************************
*	*		 FAST USER GUIDE		  *
*	***************************************************
*  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*
*			1. Master initialition with function "	hal_i2c_init();	".
*			2. CCS811 slave initialition with function "	CCS811_init(mode_number);	".
*			3. Create UINT8_T variables to read eCO2 and TVOC
*			4. Direct read from the sensors:		CCS811_read_CO2_TVOC(mode_number, &eco2, &TVOC);
*
*  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*
*
*	It has 1 communicating protocol:
*		- I2C: 	PS pin to HIGH level (Vcc).
*
*	I2C MODE:
*	---------
*		- WAK pin: used to wake up the sensor. It must be connected to GND.
*		- INT: it my be connected to GND for CCS811. It is an interruption function.
*		- RST: RESET pin that is connected to Vcc by default.
*			   is OPTIONAL, but needs some pull-up resistor of 4.7 kohms to noise-cancelling
*		- ADD: defines last I2C addres bit.
*				* ADD = GND -> CCS811_I2C_ADDRESS = 0x5A (HEX)
*				* ADD = Vcc -> CCS811_I2C_ADDRESS = 0x5B (HEX)
*
*	Steps to use CCS811
*	------------------------
*		1. Read STATUS-REGISTER (it is going to fail... but it is not a problem!!).
*		2. Read hardware-ID. It must be 0x81 (HEX).
*		3. Read STATUS-REGISTER -> APP_VALID (bit 4) == 1
*		4. Write APP_START register.
*		5. Read STATUS-REGISTER, bits 4 y 7 (APP_VALID and FW_MODE == 1 both).
*		   If bit 7 == 1 , you can continue.
*		6. Sensor mode write.
*		7. BASELINE configuration (0x11 HEX).
*		8. Modify initial temperature and humidity values in ENV_DATA register.
*		9. Read ALG_RESULT_DATA register. Convert the values with bit-masking methods.
*
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


#include "CCS811.h"
#include "BME680.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief	Data writing in CCS811 (Define register and write data in it)
 *
 * @param[in]	CCS811_register			:	(uint8_t)		command of the register where we want to write
 * @param[in]	CCS811_register_value		:	(uint8_t)		value to introduce inside the register
 *
 * @param[out]	ret				:	(esp_err_t)		variable that indicates if there was a problem
 *
 */
esp_err_t CCS811_write_register(uint8_t CCS811_register, uint8_t CCS811_register_value)
{
	esp_err_t 	ret;

	i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(i2c_cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, (CCS811_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, CCS811_register, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, CCS811_register_value, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_stop(i2c_cmd));
	ret = i2c_master_cmd_begin(I2C_NUM_1, i2c_cmd, 1000/portTICK_RATE_MS);
	i2c_cmd_link_delete(i2c_cmd);

	return ret;
}


/**
 * 	@brief		Data writing in CCS811 (only defines data register direction)
 *
 *  @param[in]		CCS811_register			:	(uint8_t)		command of the register where we want to write
 *  @param[in]	 	size				: 	(unsigned)		number of bytes that you whant to write
 *
 *  @param[out]		ret				:	(uint8_t)		variable that indicates if there was a problem
 *
 */
esp_err_t CCS811_write_byte(uint8_t CCS811_register, unsigned size)
{
	esp_err_t 	ret;

	i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(i2c_cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, (CCS811_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, CCS811_register, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_stop(i2c_cmd));
	ret = i2c_master_cmd_begin(I2C_NUM_1, i2c_cmd, 1000/portTICK_RATE_MS);
	i2c_cmd_link_delete(i2c_cmd);

	return ret;
}


/**
 * @brief		Function for write multiple bytes in a register
 *
 * @param[in]		CCS811_register			:	(uint8_t)		register addres where we want to write
 * @param[in]		buffer_in			:	(uint8_t)		buffer that we whant to write
 * @param[in]		size				:	(unsigned)		number of bytes that we whant to write
 *
 */
esp_err_t CCS811_write_data_register(uint8_t CCS811_register, uint8_t *buffer_in, unsigned size)
{
	esp_err_t 	ret;

	i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(i2c_cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, (CCS811_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, CCS811_register, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_write(i2c_cmd, buffer_in, size,
			I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_stop(i2c_cmd));
	ret = i2c_master_cmd_begin(I2C_NUM_1, i2c_cmd, 1000/portTICK_RATE_MS);
	i2c_cmd_link_delete(i2c_cmd);

	return ret;
}

/**
 * @brief		Type 2 of data writing and reading in registers - reading of 1 byte
 *
 * @param[in]		*buffer_out			: (uint8_t)		pointer to buffer_out array where we save the read byte
 * @param[in]		CCS811_register			: (uint8_t)		register where we want to operate
 * @param[in]		size				: (unsigned)	number of bytes that we are going to read
 *
 */
esp_err_t CCS811_write_read_byte(uint8_t * buffer_out, uint8_t CCS811_register, unsigned size)
{
	esp_err_t 	ret;

	i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(i2c_cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, (CCS811_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, CCS811_register, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_start(i2c_cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, (CCS811_I2C_ADDRESS << 1) | I2C_MASTER_READ, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_read(i2c_cmd, buffer_out, size, I2C_MASTER_NACK));
	ESP_ERROR_CHECK(i2c_master_stop(i2c_cmd));
	ret = i2c_master_cmd_begin(I2C_NUM_1, i2c_cmd, 1000/portTICK_RATE_MS);
	i2c_cmd_link_delete(i2c_cmd);

	return ret;
}


/**
 * @brief		Type 2.5 of writing and reading in registers - Reading of 1 byte
 *
 * @param[in]		*buffer_out			: (uint16_t)		pointer of the variable where we are going to save the data
 * @param[in]		CCS811_register			: (uint8_t)			register where we want to operate
 * @param[in]		size				: (unsigned)		number of bytes that we are going to read
 *
 */
esp_err_t CCS811_write_read_byte2(uint8_t *buffer_out, uint8_t CCS811_register, unsigned size)
{
	int			i;
	esp_err_t 	ret;

	i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(i2c_cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, (CCS811_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, CCS811_register, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_start(i2c_cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, (CCS811_I2C_ADDRESS << 1) | I2C_MASTER_READ, I2C_MASTER_ACK));
	for(i = 0; i < (size - 1); i++) {
		ESP_ERROR_CHECK(i2c_master_read_byte(i2c_cmd, &buffer_out[i], I2C_MASTER_ACK));
	}

	ESP_ERROR_CHECK(i2c_master_read_byte(i2c_cmd, &buffer_out[size-1], I2C_MASTER_NACK));
	ESP_ERROR_CHECK(i2c_master_stop(i2c_cmd));
	ret = i2c_master_cmd_begin(I2C_NUM_1, i2c_cmd, 1000/portTICK_RATE_MS);
	i2c_cmd_link_delete(i2c_cmd);

	return ret;
}


/**
 * @brief		Software reset of sensor for switch it to boot-mode
 *
 */
esp_err_t CCS811_software_reset()
{
	uint8_t		sequence[4] = {0x11, 0xE5, 0x72, 0x8A};
	esp_err_t 	ret;

	i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(i2c_cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, (CCS811_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, CCS811_REG_SW_RESET, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_write(i2c_cmd, sequence, 4, I2C_MASTER_NACK));
	ESP_ERROR_CHECK(i2c_master_stop(i2c_cmd));
	ret = i2c_master_cmd_begin(I2C_NUM_1, i2c_cmd, 1000/portTICK_RATE_MS);
	i2c_cmd_link_delete(i2c_cmd);

	return ret;
}


/**
 * @brief		Select operating mode of the CCS811.
 * 			First it reads the LSB, then it compares if the value introduced.
 * 			Because we need to stop the sensor for some time to use it correctly.
 *
 * @param[in]	mode_number	:	(uint8_t)	sensor operation mode number. It is indicated in CCS811.h
 *							It will work if it is equal to global values
 *
 */
int CCS811_mode(uint8_t mode_number)
{
	uint8_t		buffer_out[1];
	uint8_t		mode_read;
	esp_err_t	ret;

	CCS811_write_read_byte(buffer_out, CCS811_REG_MEAS_MODE, 1);
	//printf("\n%d - %d\n",buffer_out[0],buffer_out[1]);
	mode_read = (buffer_out[0] >> 4) & 0x07;

	printf("Initial mode read: %d\n",mode_read);

	if(mode_read < mode_number) {
		ret = CCS811_write_register(CCS811_REG_MEAS_MODE, CCS811_MODE_IDLE);	// Switch to mode 0 and stay in it during 10 min
		if(ret != ESP_OK) {
			printf("ERROR switch to mode 0 with delay\n");
			return -1;
		}
		//vTaskDelay(600000/portTICK_RATE_MS);
	}

	switch(mode_number) {
		case 0:
			ret = CCS811_write_register(CCS811_REG_MEAS_MODE, CCS811_MODE_IDLE);
			vTaskDelay(100/portTICK_RATE_MS);
			if(ret != ESP_OK) {
				printf("Error selecting mode 0\n");
				return -1;
			}
			break;
		case 1:
			ret = CCS811_write_register(CCS811_REG_MEAS_MODE, CCS811_MODE_1S);
			vTaskDelay(100/portTICK_RATE_MS);
			if(ret != ESP_OK) {
				printf("Error selecting mode 1\n");
				return -1;
			}
			break;
		case 2:
			ret = CCS811_write_register(CCS811_REG_MEAS_MODE, CCS811_MODE_10S);
			vTaskDelay(100/portTICK_RATE_MS);
			if(ret != ESP_OK) {
				printf("Error selecting mode 2\n");
				return -1;
			}
			break;
		case 3:
			ret = CCS811_write_register(CCS811_REG_MEAS_MODE, CCS811_MODE_60S);
			vTaskDelay(100/portTICK_RATE_MS);
			if(ret != ESP_OK) {
				printf("Error selecting mode 3\n");
				return -1;
			}
			break;
		case 4:
			ret = CCS811_write_register(CCS811_REG_MEAS_MODE, CCS811_MODE_250MS);
			vTaskDelay(100/portTICK_RATE_MS);
			if(ret != ESP_OK) {
				printf("Error selecting mode 4\n");
				return -1;
			}
			break;
		default: printf("Invalid mode.\n");
	}

	CCS811_write_read_byte(buffer_out, CCS811_REG_MEAS_MODE, 1);
	//printf("\n%d - %d\n",buffer_out[0],buffer_out[1]);
	mode_read = (buffer_out[0] >> 4) & 0x07;

	printf("New mode read: %d\n",mode_read);

	printf("Everything OK initializing mode\n");
	vTaskDelay(100/portTICK_RATE_MS);

	return 0;
}


/**
 * @brief	Baseline configuration. It is automatically by the sensor, but only read and then write.
 * 		The value of the baseline is transparent for the engineer.
 *
 */
void CCS811_configuring_baseline()
{
	uint8_t		baseline_reg[2];
	esp_err_t	ret;

	i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(i2c_cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, (CCS811_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, CCS811_REG_BASELINE, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_start(i2c_cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, (CCS811_I2C_ADDRESS << 1) | I2C_MASTER_READ, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_read(i2c_cmd, baseline_reg, 2, I2C_MASTER_NACK));
	ESP_ERROR_CHECK(i2c_master_stop(i2c_cmd));
	ret = i2c_master_cmd_begin(I2C_NUM_1, i2c_cmd, 1000/portTICK_RATE_MS);
	i2c_cmd_link_delete(i2c_cmd);

	if(ret != ESP_OK) {
		printf("ERROR reading baseline...\n");
	}

	i2c_cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(i2c_cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, (CCS811_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, CCS811_REG_BASELINE, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_write(i2c_cmd, baseline_reg, 2, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_stop(i2c_cmd));
	ret = i2c_master_cmd_begin(I2C_NUM_1, i2c_cmd, 1000/portTICK_RATE_MS);
	i2c_cmd_link_delete(i2c_cmd);

	if(ret != ESP_OK) {
		printf("ERROR writing baseline...\n");
	}
}


/**
 * @brief	CCS811 slave initialition
 *
 * @param[in]	mode_number	:	(uint8_t)	sensor operating mode (0,1,2,3,4)
 *
 */
int CCS811_init(uint8_t mode_number)
{
	uint8_t		buffer_out, aux;
	esp_err_t 	ret;

	int i2c_slave_port = I2C_SLAVE_NUM;
	i2c_config_t conf_slave;
	memset(&conf_slave, 0, sizeof(i2c_config_t));
		conf_slave.sda_io_num = 21;
		conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
		conf_slave.scl_io_num = 22;
		conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
		conf_slave.mode = I2C_MODE_SLAVE;
		conf_slave.slave.addr_10bit_en = 0;
		conf_slave.slave.slave_addr = CCS811_I2C_ADDRESS;
	i2c_param_config(i2c_slave_port, &conf_slave);
	i2c_driver_install(i2c_slave_port, I2C_MODE_SLAVE,
						I2C_SLAVE_RX_BUF_LEN,
						I2C_SLAVE_TX_BUF_LEN, 0);

	vTaskDelay(1000/portTICK_RATE_MS);

	// First read the status of the sensor
	ret = CCS811_write_read_byte(&buffer_out, CCS811_REG_STATUS, 1);
	if(ret != ESP_OK) {
		printf("ERROR on first reading of status register: %x\n",ret);
	}

	// RESET to be sure that boot-mode is selected
	ret = CCS811_software_reset();
	if(ret != ESP_OK) {
		printf("ERROR in software-reset: %x\n",ret);

		ret = CCS811_write_read_byte(&buffer_out, CCS811_REG_ERROR_ID, 1);
		if(ret != ESP_OK) {
			printf("ERROR reading ERROR register: %x\n",ret);
		}
		printf("ERROR code reading ID: %x\n",buffer_out);

		return -1;
	}
	else {
		printf("RESET well executed\n");
	}


	vTaskDelay(100/portTICK_RATE_MS);

	// Reading the hardware id, that must be 0x81
	ret = CCS811_write_read_byte(&buffer_out, CCS811_REG_HW_ID, 1);
	if(ret != ESP_OK) {
		printf("ERROR reading hardware-id: %x\n",ret);

		ret = CCS811_write_read_byte(&buffer_out, CCS811_REG_ERROR_ID, 1);
		if(ret != ESP_OK) {
			printf("ERROR reading error register: %x\n",ret);
		}
		printf("ERROR code: %x",buffer_out);

		return -1;
	}
	else {
		printf("Hardware-ID: %x (HEX)\n",buffer_out);
	}

	vTaskDelay(100/portTICK_RATE_MS);

	// Reading status register
	ret = CCS811_write_read_byte(&buffer_out, CCS811_REG_STATUS, 1);
	if(ret != ESP_OK) {
		printf("ERROR reading status register: %x\n",ret);

		ret = CCS811_write_read_byte(&buffer_out, CCS811_REG_ERROR_ID, 1);
		if(ret != ESP_OK) {
			printf("ERROR writing the error register: %x\n",ret);
		}
		printf("ERROR code: %x",buffer_out);

		return -1;
	}
	else printf("Status: %x (HEX)\n",buffer_out);

	vTaskDelay(100/portTICK_RATE_MS);

	// Initialize firmware mode, where we can load programs and data inside the sensor
	ret = CCS811_write_byte(CCS811_REG_APP_START, 1);
	if(ret != ESP_OK) {
		printf("ERROR switching to firmware mode: %x\n",ret);

		ret = CCS811_write_read_byte(&buffer_out, CCS811_REG_ERROR_ID, 1);
		if(ret != ESP_OK) {
			printf("ERROR reading error register: %x\n",ret);
		}
		printf("ERROR code: %x",buffer_out);

		return -1;
	}
	else {
		// Checking the status register again to see if the firmware-mode has been started correctly (Bit 7 register STATUS == 1)
		CCS811_write_read_byte(&buffer_out, CCS811_REG_STATUS, 1);

		if(((buffer_out >> 4) & 0x01) == 1) {
			printf("Bit FW_MODE == 1, we can load !!\n");
		}
		else {
			printf("We are not in firmware mode...\n");

			ret = CCS811_write_read_byte(&buffer_out, CCS811_REG_ERROR_ID, 1);
			if(ret != ESP_OK) {
				printf("ERROR reading error register: %x\n",ret);
			}
			printf("ERROR code: %x",buffer_out);

			return -1;
		}
	}

	CCS811_mode(mode_number);

	vTaskDelay(1000/portTICK_RATE_MS);	// Waiting a little time for the sensor to collect data and the status-register detects it

	ret = CCS811_write_read_byte(&buffer_out, CCS811_REG_STATUS, 1);
	if(ret != ESP_OK) {
		printf("ERROR reading status register\n");
	}
	else {
		printf("Status register (after selecting mode): %x (HEX)\n",buffer_out);

		aux = (buffer_out >> 3) & 0x01;
		if(aux == 1) {
			printf("DATA-READY bit = 1; there are data to read!!!\n");
		}
		else printf("DATA-READY bit = 0....\n");
	}

	CCS811_configuring_baseline();

	return 0;
}


/**
 * @brief	Temperature and humidity compensation using BME680 temperature, pressure and humidity sensor
 *
 * @param[in]	*NVM_coef		:	(BME680_calib_t)	pointer to BME680_calib_t which stores the BME680 calibration data
 *
 */
void CCS811_temperature_humidity_compensation(BME680_calib_t *NVM_coef, float *hum_comp, float *tempe_comp, float *press_comp)
{
	uint16_t		temperature;
	uint8_t			buffer_in[4];
	esp_err_t		ret;

	BME680_get_data(press_comp, hum_comp, tempe_comp, NVM_coef);

	// Calculate fraction and non-faction part of both sensors
	buffer_in[0] = (((uint32_t)hum_comp % 1000) / 100) > 7 ? ((uint32_t)hum_comp/1000 + 1) << 1 : ((uint32_t)hum_comp/1000) << 1;
	buffer_in[1] = 0;

	if((((uint32_t)hum_comp % 1000) / 100) > 2 && ((((uint32_t)hum_comp % 1000) / 100) < 8)) {
		buffer_in[0] |= 1;
	}

	temperature = ((*tempe_comp)*1000);
	temperature += 25000;

	buffer_in[2] = ((temperature % 1000) / 100) > 7 ? (temperature / 1000 + 1)<<1 : (temperature / 1000) << 1;
	buffer_in[3] = 0;

	if(((temperature % 1000) / 100) > 2 && (((temperature % 1000) / 100) < 8)) {
		buffer_in[2] |= 1;
	}


	// Writing environment temperature and humidity to register
	ret = CCS811_write_data_register(CCS811_REG_ENV_DATA, buffer_in, 4);
	if(ret != ESP_OK) {
		printf("ERROR writing ENV-DATA register: %x", ret);
	}
}


/**
 * @brief		Read all variables: temperature, pressure, humidity, TVOC and eco2
 *
 * @param[in]		mode_number		:	(uint8_t)		Depending on the mode number, a delay time will be left for them to be sampled the data and the go to read it
 * @param[in]		*eco2			:	(uint8_t)		pointer to variable eco2 that will contain the value of eco2 (carbon dioxide)
 * @param[in]		*TVOC			:	(uint8_t)		pointer to TVOC variable that will contain the value of TVOC (particles in suspension)
 * @param[in]		*hum_comp		:	(float)			pointer to compensate humidity variable
 * @param[in]		*tempe_comp		:	(float)			pointer to compensate temperature variable
 * @param[in]		*press_comp		:	(float)			pointer to compensate pressure variable
 * @param[in]		*NVM_coef		:	(BME680_calib_t)	pointer to BME680_calib_t which stores the BME680 calibration data
 *
 */

int CCS811_read_all_variables(uint8_t mode_number, uint16_t *eco2, uint16_t *TVOC, float *hum_comp, float *tempe_comp, float *press_comp, BME680_calib_t *NVM_coef)
{
	uint8_t			buffer_out[4];
	esp_err_t		ret;

	CCS811_temperature_humidity_compensation(NVM_coef, hum_comp, tempe_comp, press_comp);

	switch(mode_number) {
		case 0:
			printf("It cant read in IDDLE MODE (0)...\n");

			break;
		case 1:
			vTaskDelay(1000/portTICK_RATE_MS);	// delay 1 segundo
			ret = CCS811_write_read_byte2(buffer_out, CCS811_REG_ALG_RESULT_DATA, 4);
			if(ret != ESP_OK) {
				printf("ERROR reading sensors!!\n");
				return -1;
			}

			break;
		case 2:
			vTaskDelay(10000/portTICK_RATE_MS);	// delay 10 segundos
			ret = CCS811_write_read_byte2(buffer_out, CCS811_REG_ALG_RESULT_DATA, 4);
			if(ret != ESP_OK) {
				printf("ERROR reading sensors!!\n");
				return -1;
			}

			break;
		case 3:
			vTaskDelay(60000/portTICK_RATE_MS);	// delay 60 segundos
			ret = CCS811_write_read_byte2(buffer_out, CCS811_REG_ALG_RESULT_DATA, 4);
			if(ret != ESP_OK) {
				printf("ERROR reading sensors!!\n");
				return -1;
			}

			break;
		case 4:
			vTaskDelay(250/portTICK_RATE_MS);	// delay 250 milisegundos
			ret = CCS811_write_read_byte2(buffer_out, CCS811_REG_ALG_RESULT_DATA, 4);
			if(ret != ESP_OK) {
				printf("ERROR reading sensors!!\n");
				return -1;
			}

			break;
		default: printf("Invalid mode...\n"); break;
	}

	printf("%d - %d - %d - %d\n",buffer_out[0],buffer_out[1],buffer_out[2],buffer_out[3]);

	*eco2 = ((uint16_t)buffer_out[0] << 8) | ((uint16_t)buffer_out[1] << 0);
	*TVOC = ((uint16_t)buffer_out[2] << 8) | ((uint16_t)buffer_out[3] << 0);

	return 0;
}








#ifdef __cplusplus
}
#endif

