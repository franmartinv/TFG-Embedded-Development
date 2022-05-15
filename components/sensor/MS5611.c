/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*	MS5611 sensor DRIVER writted in C
*	-------------------------------------------
*
*	***************************************************
*	*		  		FAST USER GUIDE			  	  	  *
*	***************************************************
*  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*
*			1. Master initialition with function "	hal_i2c_init();	".
*			2. MS5611 slave initialition with function "	MS5611_init();	".
*			3. Create FLOAT variables to read pressure and temperature
*			4. Direct temperature read with	:		"	MS5611_read_temperature();	"
*			5. Direct pressure read with	:		"	MS5611_read_pressure();		"
*
*  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*
*
*	It has 2 communicating protocols:
*		- SPI: 	PS pin == GND. 	Use	SDI, SDO, CSB pins.
*		- I2C: 	PS pin == Vcc. Use SDA and SDC pins.
*
*
*	PASOS DE USO DEL MS5611
*	------------------------
*		1. Reset
*		2. Read PROM (128 bits of calibration word)
*		3. D1 conversion (pressure)
*		4. D2 conversion (temperature)
*		5. ADC raw-data read (24 bits of pressure//temperature)
*
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifdef __cplusplus
extern "C" {
#endif


#include "MS5611.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sdkconfig.h"


/**
 * @brief Data writting in MS5611
 *
 * @param[in]		MS5611_command			: (uint8_t)		command of the register where we want to write
 *
 * @param[out]		ret						: (esp_err_t)	variable that indicates if there was a problem
 *
 */
esp_err_t MS5611_write_byte(uint8_t MS5611_command)
{
	esp_err_t 	ret;

	i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(i2c_cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, (MS5611_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, MS5611_command, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_stop(i2c_cmd));
	ret = i2c_master_cmd_begin(I2C_NUM_1, i2c_cmd, 100/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(i2c_cmd);

	return ret;
}


/**
 * @brief Data reading in MS5611
 *
 * @param[in]		seize			: 	(unsigned)		number of bytes that we are going to read
 * @param[in]		buffer_out		: 	(uint8_t)		array where I am going to save the data
 *
 * @param[out]		ret				:	(esp_err_t)		variable that indicates if there was a problem
 *
 */
esp_err_t MS5611_read(uint8_t * buffer_out, unsigned size)
{
	int 		i;
	esp_err_t 	ret;

	i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(i2c_cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, (MS5611_ADDRESS << 1) | I2C_MASTER_READ, I2C_MASTER_ACK));

	for(i = 0; i < (size - 1); i++) {
		ESP_ERROR_CHECK(i2c_master_read_byte(i2c_cmd, &buffer_out[i], I2C_MASTER_ACK));
	}

	ESP_ERROR_CHECK(i2c_master_read_byte(i2c_cmd, &buffer_out[size-1], I2C_MASTER_NACK));

	ESP_ERROR_CHECK(i2c_master_stop(i2c_cmd));
	ret = i2c_master_cmd_begin(I2C_NUM_1, i2c_cmd, 500/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(i2c_cmd);

	return ret;
}


/**
 * @brief Calibration coefficients obtained from the NVM memory
 * The coefficients that I obtained during the development of this driver was:
 * 		C1 = 54535
 * 		C2 = 54134
 * 		C3 = 34040
 * 		C4 = 30924
 * 		C5 = 30191
 * 		C6 = 28416
 *
 *  The registers where we can read the coefficients are
 * 	separated 2 by 2 bytes: 0xA2, 0xA4, 0xA6, 0xA8, 0xAA, 0xAC
 *
 * 	@param[in]	 	*C1			:	(uint16_t)	pointer to C1, Pressure sensitivity										| SENST1
 * 	@param[in]		*C2			:	(uint16_t)	pointer to C2, Pressure offset											| OFFT1
 * 	@param[in]		*C3			:	(uint16_t)	pointer to C3, Temperature coefficient of pressure sensitivity			| TCS
 * 	@param[in]		*C4			:	(uint16_t)	pointer to C4, Temperature coefficient of pressure offset				| TCO
 * 	@param[in]		*C5			:	(uint16_t)	pointer to C5, Reference temperature									| TREF
 * 	@param[in]		*C6			:	(uint16_t)	pointer to C6, Temperature coefficient of the temperature 				| TEMPSENS
 *
 */
int MS5611_coef(uint16_t *C1, uint16_t *C2, uint16_t *C3, uint16_t *C4, uint16_t *C5, uint16_t *C6)
{
	uint8_t 	buffer_out[2];
	uint16_t	coefs[6];
	esp_err_t 	ret;
	int			i;

	ret = MS5611_write_byte(MS5611_RESET);
	if(ret != ESP_OK) {
		printf("ERROR in RESET-coef\n");
		return -1;
	}

	//Coefficient read loop
	for(i = 0; i < 6; i++) {
		vTaskDelay(10/portTICK_PERIOD_MS);

		ret = MS5611_write_byte(0xA2 + (2 * i));
		if(ret != ESP_OK) {
			printf("ERROR writting coefficient %d\n",i);
			return -1;
		}

		vTaskDelay(10/portTICK_PERIOD_MS);

		ret = MS5611_read(buffer_out, 2);
		if(ret != ESP_OK) {
			printf("ERROR reading coefficient %d\n",i);
			return -1;
		}

		coefs[i] = ((uint16_t)buffer_out[0] << 8) | ((uint16_t)buffer_out[1]);
	}

	*C1 = coefs[0];
	*C2 = coefs[1];
	*C3 = coefs[2];
	*C4 = coefs[3];
	*C5 = coefs[4];
	*C6 = coefs[5];

	return 0;
}


/**
 * @brief	Slave initialition
 *
 */
void MS5611_init()
{
	esp_err_t	ret;

	int i2c_slave_port = I2C_SLAVE_NUM;
	i2c_config_t conf_slave;
	    conf_slave.sda_io_num = I2C_MASTER_SDA_IO;
	    conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
	    conf_slave.scl_io_num = I2C_MASTER_SCL_IO;
	    conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
	    conf_slave.mode = I2C_MODE_SLAVE;
	    conf_slave.slave.addr_10bit_en = 0;
	    conf_slave.slave.slave_addr = MS5611_ADDRESS;
	i2c_param_config(i2c_slave_port, &conf_slave);
	i2c_driver_install(i2c_slave_port, conf_slave.mode,
	                    I2C_SLAVE_RX_BUF_LEN,
	                    I2C_SLAVE_TX_BUF_LEN, 0);

	vTaskDelay(100/portTICK_PERIOD_MS);

	ret = MS5611_write_byte(MS5611_RESET);
	if(ret != ESP_OK) {
		printf("ERROR in RESET\n");
	}
}


/**
 * @brief	Temperature reading
 *
 * @param[in]	 	coef[5]				:	(uint16_t)		calibration coefficients array
 * @param[in]		*t_no_compensate	:	(long)			pointer to raw temperature data from the sensor
 *
 * @param[out]		aux					:	(long)			temperature in Celsius*100
 *
 */
int32_t MS5611_temperature(uint16_t coef[5], uint32_t* t_no_compensate)
{
	uint8_t 	buffer_out[3];
	uint16_t 	C5, C6;
	uint32_t 	temperature;
	uint32_t 	dT, aux, T2;
	esp_err_t 	ret;

	C5 = coef[4];
	C6 = coef[5];

	ret = MS5611_write_byte(MS5611_CONV_D2_OSR4096);
	if(ret != ESP_OK) {
		printf("ERROR connecting 1\n");
		return -1;
	}

	vTaskDelay(30/portTICK_RATE_MS);

	ret = MS5611_write_byte(MS5611_ADC_READ);
	if(ret != ESP_OK) {
		printf("ERROR connecting 2\n");
		return -1;
	}

	vTaskDelay(30/portTICK_RATE_MS);

	ret = MS5611_read(buffer_out, 3);
	if(ret != ESP_OK) {
		printf("ERROR connecting 3\n");
		return -1;
	}

	temperature 		= 	((buffer_out[0])*65536)+((buffer_out[1])*256)+(buffer_out[2]);
	*t_no_compensate 	= 	temperature;
	dT 					= 	temperature - (C5*256);
	aux 				= 	2000 + (dT*(C6/pow(2, 23))); //temperature

	if(temperature >= 2000) {
		return aux;
	}
	else {
		T2 		= (dT*dT)/2147483648;
		aux 	= aux - T2;
		return aux;
	}
}

/**
 * @brief	Pressure calibration
 *
 * @param[in]	 	temperature	:	(uint32_t)	raw temperature data directly from the sensor
 * @param[in] 		pressure	:	(long)		raw pressure data directly from the sensor
 * @param[in]	 	*C1			:	(uint16_t)	pointer to C1, Pressure sensitivity										| SENST1
 * @param[in]		*C2			:	(uint16_t)	pointer to C2, Pressure offset											| OFFT1
 * @param[in]		*C3			:	(uint16_t)	pointer to C3, Temperature coefficient of pressure sensitivity			| TCS
 * @param[in]		*C4			:	(uint16_t)	pointer to C4, Temperature coefficient of pressure offset				| TCO
 * @param[in]		*C5			:	(uint16_t)	pointer to C5, Reference temperature									| TREF
 * @param[in]		*C6			:	(uint16_t)	pointer to C6, Temperature coefficient of the temperature 				| TEMPSENS
 *
 * @param[out]	 	p			:	(uint32_t)	final pressure valor in mBar
 *
 */
int32_t press_calibrate(uint32_t temperature, int32_t pressure, uint16_t C1, uint16_t C2, uint16_t C3, uint16_t C4, uint16_t C5, uint16_t C6)
{
	uint32_t 	off, sens, p, dT;

	dT 		= 	temperature - (C5*pow(2,8));
	off 	= 	(C2*pow(2,16))+((C4*dT)/pow(2,7));
	sens 	= 	(C1*pow(2,15))+((C3*dT)/pow(2,8));

	p 		= 	((((pressure*sens) / pow(2,21)) - off) / pow(2,15));

	return p;
}

/**
 * @brief	Pressure calibration for 15 <= Temperature < 20
 *
 * @param[in]	  	temperature	:	(uint32_t)	raw temperature data directly from the sensor
 * @param[in]		pressure	:	(long)		raw pressure data directly from the sensor
 * @param[in]	 	*C1			:	(uint16_t)	pointer to C1, Pressure sensitivity										| SENST1
 * @param[in]		*C2			:	(uint16_t)	pointer to C2, Pressure offset											| OFFT1
 * @param[in]		*C3			:	(uint16_t)	pointer to C3, Temperature coefficient of pressure sensitivity			| TCS
 * @param[in]		*C4			:	(uint16_t)	pointer to C4, Temperature coefficient of pressure offset				| TCO
 * @param[in]		*C5			:	(uint16_t)	pointer to C5, Reference temperature									| TREF
 * @param[in]		*C6			:	(uint16_t)	pointer to C6, Temperature coefficient of the temperature 				| TEMPSENS
 *
 * @param[out]	 	p			:	(uint32_t)	final pressure valor in mBar
 *
 */
int32_t press_calibrate_2(uint32_t temperature, int32_t pressure, uint16_t C1, uint16_t C2, uint16_t C3, uint16_t C4, uint16_t C5, uint16_t C6)
{
	uint32_t 	off, sens, p, dT, off2, sens2;

	dT 		= 	temperature - (C5*pow(2,8));
	off 	= 	(C2*pow(2,16))+((C4*dT)/pow(2,7));
	sens 	= 	(C1*pow(2,15))+((C3*dT)/pow(2,8));

	off2 	= 	5*pow((temperature-2000),2)/2;
	sens2 	= 	5*pow((temperature-2000),2)/4;
	off 	=	off - off2;
	sens 	= 	sens - sens2;

	p 		= 	((((pressure*sens) / pow(2,21)) - off) / pow(2,15));

	return p;
}

/**
 * @brief	Pressure calibration for Temperature < 15
 *
 * @param[in]	 	temperature	:	(uint32_t)	raw temperature data directly from the sensor
 * @param[in]		pressure	:	(long)		raw pressure data directly from the sensor
 * @param[in]	 	*C1			:	(uint16_t)	pointer to C1, Pressure sensitivity										| SENST1
 * @param[in]		*C2			:	(uint16_t)	pointer to C2, Pressure offset											| OFFT1
 * @param[in]		*C3			:	(uint16_t)	pointer to C3, Temperature coefficient of pressure sensitivity			| TCS
 * @param[in]		*C4			:	(uint16_t)	pointer to C4, Temperature coefficient of pressure offset				| TCO
 * @param[in]		*C5			:	(uint16_t)	pointer to C5, Reference temperature									| TREF
 * @param[in]		*C6			:	(uint16_t)	pointer to C6, Temperature coefficient of the temperature 				| TEMPSENS
 *
 * @param[out]	 	p			:	(uint32_t)	final pressure valor in mBar
 *
 */
int32_t press_calibrate_3(uint32_t temperature, int32_t pressure, uint16_t C1, uint16_t C2, uint16_t C3, uint16_t C4, uint16_t C5, uint16_t C6)
{
	uint32_t 	off, sens, p, dT, off2, sens2, off3, sens3;

	dT 		= 	temperature - (C5*pow(2,8));
	off 	= 	(C2*pow(2,16))+((C4*dT)/pow(2,7));
	sens 	= 	(C1*pow(2,15))+((C3*dT)/pow(2,8));

	off2 	= 	5*pow((temperature-2000),2)/2;
	off3 	= 	off2 + (7*pow((temperature+1500),2));
	sens2 	= 	(5*pow((temperature-2000),2))/4;
	sens3 	=	sens2 + ((11*pow(temperature+1500,2))/2);
	off 	= 	off - off3;
	sens 	= 	sens - sens3;

	p 		= 	((((pressure*sens) / pow(2,21)) - off) / pow(2,15));

	return p;
}

/**
 * @brief	Pressure reading
 * [OUT]	- Pressure calibration functions
 * 				* 	press_calibrate 	->	20 Celsius 	<  	temperatura
 * 				*	press_calibrate2  	->	15 Celsius 	<  	temperatura 	<= 	20 Celsius
 * 				*	press_calibrate3	->	temperatura	<	15 Celsius
 *
 * @param[in]	  	coef[5]			:	(uint16_t)		calibration coefficients array
 * @param[in]	  	t_no_compensate	:	(uint32_t)		raw temperature data directly from the sensor
 *
 * @param[out]		pressure		:	(long)			final compensate pressure
 *
 */
int32_t MS5611_pressure(uint16_t coef[5],  uint32_t t_no_compensate)
{
	uint8_t 	buffer_out[3];
	uint16_t 	C1, C2, C3, C4, C5, C6;
	uint32_t 	temperature;
	int32_t		pressure;
	esp_err_t 	ret;

	C1 		= 	coef[0];
	C2 		= 	coef[1];
	C3 		= 	coef[2];
	C4 		= 	coef[3];
	C5 		= 	coef[4];
	C6 		= 	coef[5];

	ret 	= 	MS5611_write_byte(MS5611_CONV_D1_OSR4096);
	if(ret != ESP_OK) {
		printf("ERROR connecting 4\n");
		return -1;
	}

	vTaskDelay(30/portTICK_RATE_MS);

	ret = MS5611_write_byte(MS5611_ADC_READ);
	if(ret != ESP_OK) {
		printf("ERROR connecting 5\n");
		return -1;
	}

	vTaskDelay(30/portTICK_RATE_MS);

	ret = MS5611_read(buffer_out, 3);
	if(ret != ESP_OK) {
		printf("ERROR connecting 6\n");
		return -1;
	}

	pressure	=	((buffer_out[0]) * 65536) + ((buffer_out[1]) * 256) + (buffer_out[2]);
	temperature = 	t_no_compensate;

	if(temperature < 2000) {
		if(temperature < -1500) {
			return press_calibrate_3(temperature, pressure, C1, C2, C3, C4, C5, C6);
		}
		else {
			return press_calibrate_2(temperature, pressure, C1, C2, C3, C4, C5, C6);
		}
	}
	else {
		return press_calibrate(temperature, pressure, C1, C2, C3, C4, C5, C6);
	}
}


/**
 * @brief	Final pressure and temperature display function
 *
 * @param[in]	temperature		:	(long)		final temperature
 * @param[in]	pressure		:	(long)		final pressure
 */
void display_temp_press(long temperature, long pressure)
{
	float 	temp_final, press_final;

	temp_final 		= 	(float)temperature / 100;
	press_final 	= 	(float)abs(pressure) / 100;

	printf("The final values of temperature and pressure are:\n");
	printf("Temperature: %f C\n",temp_final);
	printf("Pressure: %f mBar\n\n",press_final);
}


/*
 * @brief	Final temperature valor function
 *
 * @param[out]	aux	:	(float)		temperature reading in [Celsius]
 *
 */
float MS5611_read_temperature()
{
	uint16_t 	C1, C2, C3, C4, C5, C6;
	uint16_t 	coef[6];
	uint32_t	t_no_compensate;
	int32_t		temperature;
	float		aux;

	vTaskDelay(30/portTICK_PERIOD_MS);
	MS5611_coef(&C1, &C2, &C3, &C4, &C5, &C6);

	coef[0] 	= 	C1;
	coef[1] 	= 	C2;
	coef[2] 	= 	C3;
	coef[3] 	= 	C4;
	coef[4] 	= 	C5;
	coef[5] 	= 	C6;

	printf("Coefficients: %d-%d-%d-%d-%d-%d\n\n", C1, C2, C3, C4, C5, C6);

	temperature = MS5611_temperature(coef, &t_no_compensate);
	aux = (float)temperature/100;

	return aux;
}


/**
 * @brief		Final pressure valor function
 *
 * @param[out]	aux	:	(float)		pressure reading in [mBar]
 *
 */
float MS5611_read_pressure()
{
	uint32_t 	t_no_compensate;
	int32_t		pressure;
	uint16_t 	C1, C2, C3, C4, C5, C6;
	uint16_t 	coef[6];
	float		aux;

	vTaskDelay(30/portTICK_PERIOD_MS);
	MS5611_coef(&C1, &C2, &C3, &C4, &C5, &C6);

	coef[0] 	= 	C1;
	coef[1] 	= 	C2;
	coef[2] 	= 	C3;
	coef[3] 	= 	C4;
	coef[4] 	= 	C5;
	coef[5] 	= 	C6;

	MS5611_temperature(coef, &t_no_compensate);

	pressure 	= MS5611_pressure(coef, t_no_compensate);
	aux 		= (float)abs(pressure / 100);

	return aux;
}


#ifdef __cplusplus
}
#endif



