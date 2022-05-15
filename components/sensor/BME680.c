/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*	BME680 sensor DRIVER
*	-------------------------------------------
*
*	***************************************************
*	*		  		FAST USER GUIDE 		  	  	  *
*	***************************************************
*  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*
*			1. Master initialition with function "	hal_i2c_init();	".
*			2. BME680 slave initialition with function "	BME680_init(mode_number);	".
*				2.1. Oversampling for all sensor is configured by default (in this driver) with x2 coefficient
*			3. Create a "BME680_calib_t" struct, where we are going to safe the calibration coefficients
*			4. Create some "float" variables for save the final temperature, humidity and pressure values
*			5. Direct read of the variables calling "BME680_get_data" function.
*
*  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*
*
*	Step by step to use BME680
*	--------------------------
*		1. First I2C read to ID-REGISTER (it always fail...).
*		2. Do a soft-reset.
*		3. Read hardware-ID. It will be 0x61.
*		4. Read calibration-data.
*		5. Selecting Forced-mode, where we can read temperature, pressure and humidity ones.
*		6. Setting sensor settings.
*			6.1. Switch to sleep-mode.
*			6.2. Configure IRR filter if we want to use it.
*			6.3. Oversampling configuration for pressure and temperature.
*			6.4. Oversampling configuration for humidity.
*		7. Set sensor mode to forced-mode again.
*		8. Before reading data from the resisters, we need to be sure that we are in forced-mode !!!
*		9. Compensate ADC raw-data.
*
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


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
 * @brief Data reading of BMP680
 *
 * @param[in]		* buffer_out	:	(uint8_t)	pointer to array buffer_out
 * @param[in]	  	BME680_command	:	(uint8_t)	command to where we want to read
 * @param[in]		size			:	(unsigned)	number of bytes that we need to read
 *
 * @param[out]		ret				:	(esp_err_t)	variable that indicates if there was a problem
 *
 */
esp_err_t BME680_read(uint8_t * buffer_out, uint8_t BME680_command, unsigned size)
{
	int 		i;
	esp_err_t 	ret;

	i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(i2c_cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, (BME680_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, BME680_command, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_start(i2c_cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, (BME680_I2C_ADDRESS << 1) | I2C_MASTER_READ, I2C_MASTER_ACK));

	for(i = 0; i < (size - 1); i++) {
		ESP_ERROR_CHECK(i2c_master_read_byte(i2c_cmd, &buffer_out[i], I2C_MASTER_ACK));
	}

	ESP_ERROR_CHECK(i2c_master_read_byte(i2c_cmd, &buffer_out[size-1], I2C_MASTER_NACK));

	ESP_ERROR_CHECK(i2c_master_stop(i2c_cmd));
	ret = i2c_master_cmd_begin(I2C_NUM_1, i2c_cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(i2c_cmd);

	return ret;
}


/**
 * @brief 			Data writing in BME680
 *
 * @param[in]		BME680_command	: 	(uint8_t)		command to where we want to write
 *
 * @param[out]		ret				:  	(esp_err_t)		variable that indicates if there was a problem
 *
 */
esp_err_t BME680_write_command(uint8_t BME680_command)
{
    esp_err_t	ret;

	i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, (BME680_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(i2c_cmd, BME680_command, I2C_MASTER_ACK);
    i2c_master_stop(i2c_cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_1, i2c_cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(i2c_cmd);

    return ret;
}


/**
 * @brief 			Data writing in BME680
 *
 * @param[in]		BME680_register			: (uint8_t)		command to where we want to write
 * @param[in]		BME680_register_value	: (uint8_t) 	value that we want to write in the register called BME680_register
 *
 * @param[out]		ret						: (esp_err_t)	variable that indicates if there was a problem
 *
 */
esp_err_t BME680_write_register(uint8_t BME680_register, uint8_t BME680_register_value)
{
	esp_err_t 	ret;

	i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(i2c_cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, (BME680_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, BME680_register, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, BME680_register_value, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_stop(i2c_cmd));
	ret = i2c_master_cmd_begin(I2C_NUM_1, i2c_cmd, 1000/portTICK_RATE_MS);
	i2c_cmd_link_delete(i2c_cmd);

	return ret;
}


/**
 * @brief			Calibration coefficients reading
 *
 * @param[in]		* NVM_coef		: (BME680_calib_t)	pointer to struct that contains all of the calibration coefficients
 *
 * @param[out]		ret				: (esp_err_t) 		variable that indicates if there was a problem
 *
 */
esp_err_t BME680_calibration_data(BME680_calib_t *NVM_coef)
{
	//int			i;
	uint8_t			buffer_out[BME680_COEFF_SIZE];
	esp_err_t		ret;

	ret = BME680_read(buffer_out, BME680_COEFF_ADDR1, BME680_COEFF_ADDR1_LEN);
	if(ret != ESP_OK) {
		printf("ERROR reading coefficients at address 1 (%x): %x\n", BME680_COEFF_ADDR1, ret);
	}

	vTaskDelay(100/portTICK_RATE_MS);

	ret = BME680_read(&buffer_out[BME680_COEFF_ADDR1_LEN], BME680_COEFF_ADDR2, BME680_COEFF_ADDR2_LEN);
	if(ret != ESP_OK) {
		printf("ERROR reading coefficients at address 2 (%x): %x\n", BME680_COEFF_ADDR2, ret);
	}


	// NVM temeperature coefficients
	(NVM_coef -> BME680_par_T1) = ((uint16_t) buffer_out[BME680_T1_MSB_REG] << 8) | ((uint16_t) buffer_out[BME680_T1_LSB_REG]);
	(NVM_coef -> BME680_par_T2) = ((uint16_t) buffer_out[BME680_T2_MSB_REG] << 8) | ((uint16_t) buffer_out[BME680_T2_LSB_REG]);
	(NVM_coef -> BME680_par_T3) = (int8_t) (buffer_out[BME680_T3_REG]);

	// NVM pressure coefficients
	(NVM_coef -> BME680_par_P1) = ((uint16_t) buffer_out[BME680_P1_MSB_REG] << 8) | ((uint16_t) buffer_out[BME680_P1_LSB_REG]);
	(NVM_coef -> BME680_par_P2) = ((uint16_t) buffer_out[BME680_P2_MSB_REG] << 8) | ((uint16_t) buffer_out[BME680_P2_LSB_REG]);
	(NVM_coef -> BME680_par_P3) = (int8_t) buffer_out[BME680_P3_REG];
	(NVM_coef -> BME680_par_P4) = ((uint16_t) buffer_out[BME680_P4_MSB_REG] << 8) | ((uint16_t) buffer_out[BME680_P4_LSB_REG]);
	(NVM_coef -> BME680_par_P5) = ((uint16_t) buffer_out[BME680_P5_MSB_REG] << 8) | ((uint16_t) buffer_out[BME680_P5_LSB_REG]);
	(NVM_coef -> BME680_par_P6) = (int8_t) (buffer_out[BME680_P6_REG]);
	(NVM_coef -> BME680_par_P7) = (int8_t) (buffer_out[BME680_P7_REG]);
	(NVM_coef -> BME680_par_P8) = ((uint16_t) buffer_out[BME680_P8_MSB_REG] << 8) | ((uint16_t) buffer_out[BME680_P8_LSB_REG]);
	(NVM_coef -> BME680_par_P9) = ((uint16_t) buffer_out[BME680_P9_MSB_REG] << 8) | ((uint16_t) buffer_out[BME680_P9_LSB_REG]);
	(NVM_coef -> BME680_par_P10) = (uint8_t) (buffer_out[BME680_P10_REG]);

	// NVM humidity coefficients
	(NVM_coef -> BME680_par_H1) = ((uint16_t) buffer_out[BME680_H1_MSB_REG] << 4) | ((uint16_t) buffer_out[BME680_H1_LSB_REG] & 0x0F);
	(NVM_coef -> BME680_par_H2) = ((uint16_t) buffer_out[BME680_H2_MSB_REG] << 4) | ((uint16_t) (buffer_out[BME680_H2_LSB_REG] >> 4));
	(NVM_coef -> BME680_par_H3) = (int8_t) buffer_out[BME680_H3_REG];
	(NVM_coef -> BME680_par_H4) = (int8_t) buffer_out[BME680_H4_REG];
	(NVM_coef -> BME680_par_H5) = (int8_t) buffer_out[BME680_H5_REG];
	(NVM_coef -> BME680_par_H6) = (uint8_t) buffer_out[BME680_H6_REG];
	(NVM_coef -> BME680_par_H7) = (int8_t) buffer_out[BME680_H7_REG];

	// NVM gas coefficients
	(NVM_coef -> BME680_par_gh1) = (int8_t) buffer_out[BME680_GH1_REG];
	(NVM_coef -> BME680_par_gh2) = ((uint16_t) buffer_out[BME680_H2_MSB_REG] << 8) | ((uint16_t) buffer_out[BME680_H2_LSB_REG]);
	(NVM_coef -> BME680_par_gh3) = (int8_t) buffer_out[BME680_GH3_REG];

	// Reading of other coefficients
	ret = BME680_read(buffer_out, BME680_ADDR_RES_HEAT_RANGE_ADDR, 1);
	if(ret != ESP_OK) {
		printf("ERROR reading temperature range: %x\n", ret);
		buffer_out[0] = 0;
	}
	(NVM_coef -> res_heat_range) = ((buffer_out[0] & 0x30) / 16);

	ret = BME680_read(buffer_out, BME680_ADDR_RES_HEAT_VAL_ADDR, 1);
	if(ret != ESP_OK) {
		printf("ERROR reading heater register values: %x\n", ret);
		buffer_out[0] = 0;
	}
	(NVM_coef -> res_heat_val) = (int8_t)buffer_out[0];

	(NVM_coef -> range_sw_err) = ((int8_t)buffer_out[0] & (int8_t)0xF0) / 16;

	//printf("%d\n", NVM_coef->BME680_par_T1);
	//printf("%d\n", NVM_coef->BME680_par_T2);
	//printf("%d\n", NVM_coef->BME680_par_T3);
	//printf("%d\n", NVM_coef->BME680_par_P1);
	//printf("%d\n", NVM_coef->BME680_par_P2);
	//printf("%d\n", NVM_coef->BME680_par_P3);
	//printf("%d\n", NVM_coef->BME680_par_P4);
	//printf("%d\n", NVM_coef->BME680_par_P5);
	//printf("%d\n", NVM_coef->BME680_par_P6);
	//printf("%d\n", NVM_coef->BME680_par_P7);
	//printf("%d\n", NVM_coef->BME680_par_P8);
	//printf("%d\n", NVM_coef->BME680_par_P9);
	//printf("%d\n", NVM_coef->BME680_par_P10);
	//printf("%d\n", NVM_coef->BME680_par_H1);
	//printf("%d\n", NVM_coef->BME680_par_H2);
	//printf("%d\n", NVM_coef->BME680_par_H3);
	//printf("%d\n", NVM_coef->BME680_par_H4);
	//printf("%d\n", NVM_coef->BME680_par_H5);
	//printf("%d\n", NVM_coef->BME680_par_H6);
	//printf("%d\n", NVM_coef->BME680_par_H7);
	//printf("%d\n", NVM_coef->BME680_par_gh1);
	//printf("%d\n", NVM_coef->BME680_par_gh2);
	//printf("%d\n", NVM_coef->BME680_par_gh3);

	return ret;
}


/**
 * @brief			Set sensor settings
 *
 * @param[in]		* NVM_coef		: (BME680_calib_t)	pointer to struct that contains all of the calibration coefficients
 *
 */
void BME680_settings(BME680_calib_t *NVM_coef)
{
	uint8_t		buffer_in, ovsamp_value;
	uint8_t		buffer_out[1];
	esp_err_t	ret;

	// Sleep mode selection
	ret = BME680_write_register(BME680_REG_CTRL_MEAS, BME680_set_sleep_mode);
	if(ret != ESP_OK) {
		printf("ERROR changing to sleep mode: %x\n", ret);
	}


	// IRR filter configuration
	ret = BME680_read(buffer_out, BME680_REG_IRR_filter, 1);
	if(ret != ESP_OK) {
		printf("ERROR reading IRR filter: %x\n", ret);
	}

	buffer_in = (buffer_out[0] & 0xE3) & (BME680_set_filter_coef_1 << 2);

	ret = BME680_write_register(BME680_REG_IRR_filter, buffer_in);
	if(ret != ESP_OK) {
		printf("ERROR writting IRR filter: %x\n", ret);
	}


	// Oversampling configuration of temperature and pressure
		// first I will adquire the register 0x74 where is the bits for temperature oversampling, pressure oversampling and mode settings
		// important to do a bit-mask because we cant change the mode (bits 0,1)!!!
	ret = BME680_read(buffer_out, BME680_REG_CTRL_MEAS, 1);
	if(ret != ESP_OK) {
		printf("ERROR reading at ctrl_meas register: %x\n", ret);
	}


	//			temperature_oversampling	pressure_oversampling		actual_mode
	ovsamp_value = (BME680_oversampling_x2 << 5) | (BME680_oversampling_x2 << 2) | (buffer_out[0] & 0x03);

	ret = BME680_write_register(BME680_REG_CTRL_MEAS, ovsamp_value);
	if(ret != ESP_OK) {
		printf("ERROR reading at ctrl_meas register: %x\n", ret);
	}

	ret = BME680_read(buffer_out, BME680_REG_CTRL_MEAS, 1);
	if(ret != ESP_OK) {
		printf("ERROR reading at ctrl_meas register: %x\n", ret);
	}
	else printf("Ctrl-meas register after oversampling read: %x\n", buffer_out[0]);


	// Oversampling configuration of humidity sensor
	ret = BME680_read(buffer_out, BME680_REG_CTRL_HUM, 1);
	if(ret != ESP_OK) {
		printf("ERROR reading at ctrl_humidity register: %x\n", ret);
	}

	buffer_in = (buffer_out[0] & 0xF8) | BME680_oversampling_x2;

	ret = BME680_write_register(BME680_REG_CTRL_HUM, buffer_in);
	if(ret != ESP_OK) {
		printf("ERROR reading at ctrl_humidity register: %x\n", ret);
	}

	ret = BME680_read(buffer_out, BME680_REG_CTRL_HUM, 1);
	if(ret != ESP_OK) {
		printf("ERROR reading at ctrl_humidity register: %x\n", ret);
	}
	else printf("Ctrl-humidity register after oversampling read: %x\n", buffer_out[0]);


	// Forced-mode regret
	ovsamp_value = (ovsamp_value & 0xFC) | BME680_set_forced_mode;
	ret = BME680_write_register(BME680_REG_CTRL_MEAS, ovsamp_value);
	if(ret != ESP_OK) {
		printf("ERROR selecting forced mode: %x\n", ret);
	}
	ret = BME680_read(buffer_out, BME680_REG_CTRL_MEAS, 1);
	if(ret != ESP_OK) {
		printf("ERROR reading at ctrl_meas register: %x\n", ret);
	}
	else printf("Ctrl-meas register after switching to forced-mode read: %x\n", buffer_out[0]);
}


/**
 * @brief	BME680 set sensor sleep mode
 *
 */
int BME680_set_mode()
{
	uint8_t		buffer_out[1];
	uint8_t		buffer_in;
	esp_err_t	ret;

	buffer_in = 0;

	do {
		ret = BME680_read(buffer_out, BME680_REG_CTRL_MEAS, 1);
		if(ret != ESP_OK) {
			printf("ERROR reading ctrl_meas register: %x\n", ret);
		}
		else {
			buffer_in = (buffer_out[0] & 0xFC) | BME680_set_sleep_mode;

			ret = BME680_write_register(BME680_REG_CTRL_MEAS, buffer_in);
			if(ret != ESP_OK) printf("ERROR writting sleep mode to ctrl-meas register: %x\n", ret);

			ret = BME680_read(buffer_out, BME680_REG_CTRL_MEAS, 1);
			if(ret != ESP_OK) {
				printf("ERROR reading ctrl_meas register: %x\n", ret);
			}
		}
	} while(BME680_set_sleep_mode != (buffer_out[0] & 0x03));

	ret = BME680_write_register(BME680_REG_CTRL_MEAS, buffer_in);
	if(ret != ESP_OK) printf("ERROR writting new power mode to ctrl-meas register: %x\n", ret);
	else printf("Setting sleep-mode OK!!\n");

	return 0;
}


/**
 * @brief			Reset sensor function
 *
 * @param[out]		ret		: (esp_err_t)	variable that indicates if there was a problem resetting the hardware
 *
 */
esp_err_t BME680_reset_function()
{
	uint8_t		buffer_out[1];
	esp_err_t	ret;

	ret = BME680_write_register(BME680_REG_SOFT_RESET, BME680_RESET_VALUE);
	if(ret != ESP_OK) {
		printf("ERROR writting at RESET register: %x\n", ret);
	}

	ret = BME680_read(buffer_out, BME680_REG_SOFT_RESET, 1);
	if(ret != ESP_OK) {
		printf("ERROR reading at RESET register: %x\n", ret);
	}
	else {
		printf("Software-reset read: %x\n", buffer_out[0]);
	}

	return ret;
}


/**
 * @brief			Slave initialition
 *
 * @param[in]		* NVM_coef		: (BME680_calib_t)	pointer to BME680_calib_t struct variable called NVM_coef
 *
 */
int BME680_init(BME680_calib_t *NVM_coef)
{
	uint8_t						buffer_out[1];
	esp_err_t 					ret;

	int i2c_slave_port = I2C_SLAVE_NUM;
	i2c_config_t conf_slave;
	memset(&conf_slave, 0, sizeof(i2c_config_t));
		conf_slave.sda_io_num = 21;
		conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
		conf_slave.scl_io_num = 22;
		conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
		conf_slave.mode = I2C_MODE_SLAVE;
		conf_slave.slave.slave_addr = BME680_I2C_ADDRESS;
	i2c_param_config(i2c_slave_port, &conf_slave);
	i2c_driver_install(i2c_slave_port, I2C_MODE_SLAVE,
						I2C_SLAVE_RX_BUF_LEN,
						I2C_SLAVE_TX_BUF_LEN, 0);

	vTaskDelay(1000/portTICK_RATE_MS);

	ret = BME680_write_command(BME680_REG_ID);
	if(ret != ESP_OK) {
		printf("ERROR at first iteration.\n");
	}

	vTaskDelay(100/portTICK_RATE_MS);

	// Soft-reset of the BME680
	BME680_reset_function();

	vTaskDelay(100/portTICK_RATE_MS);

	// Hardware-ID reading -> it should be 0x61 (HEX)
	ret = BME680_read(buffer_out, BME680_REG_ID, 1);
	if(ret != ESP_OK) {
		printf("ERROR reading hardware ID: %x\n", ret);
		return -1;
	}
	else if(buffer_out[0] != 97) {
		printf("ERROR writting hardware ID: %x - Error code: %x\n", buffer_out[0], ret);
		return -1;
	}
	else printf("Sensor hardware-ID is: %x\n", buffer_out[0]);

	vTaskDelay(100/portTICK_RATE_MS);


	// Calibration coefficients reading

	BME680_calibration_data(NVM_coef);

	vTaskDelay(100/portTICK_RATE_MS);

	// Forced mode selection
	ret = BME680_write_register(BME680_REG_CTRL_MEAS, BME680_set_forced_mode);
	if(ret != ESP_OK) {
		printf("ERROR selecting forced mode: %x\n", ret);
	}

	vTaskDelay(100/portTICK_RATE_MS);

	// Set sensor settings
	BME680_settings(NVM_coef);

	vTaskDelay(100/portTICK_RATE_MS);

	// Set sleep mode to the BME680 sensor
	BME680_set_mode();

	vTaskDelay(100/portTICK_RATE_MS);

	return 0;

}


/**
 * @brief			Pressure data compensation
 *
 * @param[in]		pres_adc		: (uint32_t)		raw pressure data read from the ADC
 * @param[in]		* NVM_coef		: (BME680_calib_t)	pointer to struct that contains all of the calibration coefficients
 *
 * @param[out]		pressure_comp	: (uint32_t)		valor of the pressure when its compensate
 *
 */
uint32_t BME680_pressure_compensation(uint32_t pres_adc , BME680_calib_t *NVM_coef)
{
	int32_t var1, var2, var3, pressure_comp;

	var1 = (((int32_t) NVM_coef->t_fine) >> 1) - 64000;
	var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t) NVM_coef->BME680_par_P6) >> 2;
	var2 = var2 + ((var1 * (int32_t) NVM_coef->BME680_par_P5) << 1);
	var2 = (var2 >> 2) + ((int32_t) NVM_coef->BME680_par_P4 << 16);
	var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) * ((int32_t) NVM_coef->BME680_par_P3 << 5)) >> 3) + (((int32_t) NVM_coef->BME680_par_P2 * var1) >> 1);
	var1 = var1 >> 18;
	var1 = ((32768 + var1) * (int32_t) NVM_coef->BME680_par_P1) >> 15;

	pressure_comp = 1048576 - pres_adc;
	pressure_comp = (int32_t)((pressure_comp - (var2 >> 12)) * ((uint32_t)3125));

	if (pressure_comp >= BME680_MAX_OVERFLOW_VAL) pressure_comp = ((pressure_comp / var1) << 1);
	else pressure_comp = ((pressure_comp << 1) / var1);

	var1 = ((int32_t) NVM_coef->BME680_par_P9 * (int32_t)(((pressure_comp >> 3) * (pressure_comp >> 3)) >> 13)) >> 12;
	var2 = ((int32_t)(pressure_comp >> 2) * (int32_t) NVM_coef->BME680_par_P8) >> 13;
	var3 = ((int32_t)(pressure_comp >> 8) * (int32_t)(pressure_comp >> 8) * (int32_t)(pressure_comp >> 8) * (int32_t) NVM_coef->BME680_par_P10) >> 17;

	pressure_comp = (int32_t)(pressure_comp) + ((var1 + var2 + var3 + ((int32_t) NVM_coef->BME680_par_P7 << 7)) >> 4);

	return (uint32_t)pressure_comp;
}


/**
 * @brief			Temperature data compensation
 *
 * @param[in]		temp_adc		: (uint32_t)		raw temperature data read from the ADC
 * @param[in]		* NVM_coef		: (BME680_calib_t)	pointer to struct that contains all of the calibration coefficients
 *
 * @param[out]		calc_temp		: (uint32_t)		valor of the temperature when its compensate
 *
 */
int16_t BME680_temperature_compensation(uint32_t temp_adc, BME680_calib_t *NVM_coef)
{
	int64_t var1, var2, var3;
	int16_t calc_temp;

	var1 = ((int32_t) temp_adc >> 3) - ((int32_t) NVM_coef->BME680_par_T1 << 1);
	var2 = (var1 * (int32_t) NVM_coef->BME680_par_T2) >> 11;
	var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
	var3 = ((var3) * ((int32_t) NVM_coef->BME680_par_T3 << 4)) >> 14;
	NVM_coef->t_fine = (int32_t) (var2 + var3);
	calc_temp = (int16_t) (((NVM_coef->t_fine * 5) + 128) >> 8);

	return calc_temp;
}


/**
 * @brief			Humidity data compensation
 *
 * @param[in]		hum_adc			: (uint32_t)		raw humidity data read from the ADC
 * @param[in]		* NVM_coef		: (BME680_calib_t)	pointer to struct that contains all of the calibration coefficients
 *
 * @param[out]		calc_hum		: (uint32_t)		valor of the humidity when its compensate
 *
 */
uint32_t BME680_humidity_compensation(uint16_t hum_adc, BME680_calib_t *NVM_coef)
{
	int32_t var1, var2, var3, var4, var5, var6, temp_scaled, calc_hum;

	temp_scaled = (((int32_t) NVM_coef->t_fine * 5) + 128) >> 8;
	var1 = (int32_t) (hum_adc - ((int32_t) ((int32_t) NVM_coef->BME680_par_H1 * 16))) - (((temp_scaled * (int32_t) NVM_coef->BME680_par_H3) / ((int32_t) 100)) >> 1);
	var2 = ((int32_t) NVM_coef->BME680_par_H2 * (((temp_scaled * (int32_t) NVM_coef->BME680_par_H4) / ((int32_t) 100)) + (((temp_scaled * ((temp_scaled * (int32_t) NVM_coef->BME680_par_H5) / ((int32_t) 100))) >> 6) / ((int32_t) 100)) + (int32_t) (1 << 14))) >> 10;
	var3 = var1 * var2;
	var4 = (int32_t) NVM_coef->BME680_par_H6 << 7;
	var4 = ((var4) + ((temp_scaled * (int32_t) NVM_coef->BME680_par_H7) / ((int32_t) 100))) >> 4;
	var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
	var6 = (var4 * var5) >> 1;
	calc_hum = (((var3 + var6) >> 10) * ((int32_t) 1000)) >> 12;

	if (calc_hum > 100000) /* Cap at 100%rH */
		calc_hum = 100000;
	else if (calc_hum < 0)
		calc_hum = 0;

	return (uint32_t) calc_hum;
}


/**
 * @brief			Getting sensor data and then doing the compensation
 *
 * @param[in]		* press_comp		: (float)			pointer to valor of the pressure compensate
 * @param[in]		* hum_comp			: (float)			pointer to valor of the humidity compensate
 * @param[in]		* tempe_comp		: (float)			pointer to valor of the temperature compensate
 * @param[in]		* NVM_coef			: (BME680_calib_t)	pointer to struct that contains all of the calibration coefficients
 *
 */
int BME680_get_data(float *press_comp, float *hum_comp, float *tempe_comp, BME680_calib_t *NVM_coef)
{
	uint8_t			measuring_flag, buffer_in;
	uint8_t			buffer_out[3];		// 3 bytes because we can read all data for each sensor with 3 bytes (pressure and temperature) or 2 bytes (humidity)
	uint32_t 		adc_temperature, adc_pres;
	uint16_t 		adc_hum;
	esp_err_t		ret;

	adc_temperature = 0;
	adc_pres = 0;
	adc_hum = 0;
	measuring_flag = 0;

	// We need to set forced-mode if we want to read ADC values... is because in forced-mode when we read ones, automatically sets to sleep-mode...
	ret = BME680_read(buffer_out, BME680_REG_CTRL_MEAS, 1);
	if(ret != ESP_OK) {
		printf("ERROR reading ctrl-meas register: %x\n", ret);
	}

	buffer_in = (buffer_out[0] & 0xFC) | BME680_set_forced_mode;

	ret = BME680_write_register(BME680_REG_CTRL_MEAS, buffer_in);
	if(ret != ESP_OK) {
		printf("ERROR selecting forced mode: %x\n", ret);
	}

	while (measuring_flag == 1) {
		ret = BME680_read(buffer_out, BME680_REG_STATUS, 1);
		if(ret != ESP_OK) {
			printf("ERROR reading status register: %x\n", ret);
		}

		measuring_flag = (buffer_out[0] >> 7) & 0x01;
	}

	// Pressure reading
	ret = BME680_read(buffer_out, BME680_REG_PRESS_MSB, 3);
	if(ret != ESP_OK) {
		printf("ERROR reading pressure data registers: %x\n", ret);
	}
	else adc_pres = (uint32_t) (((uint32_t) buffer_out[0] * 4096) | ((uint32_t) buffer_out[1] * 16) | ((uint32_t) buffer_out[2] / 16));

	vTaskDelay(100/portTICK_RATE_MS);

	// Temperature reading
	ret = BME680_read(buffer_out, BME680_REG_TEMP_MSB, 3);
	if(ret != ESP_OK) {
		printf("ERROR reading temperature data registers: %x\n", ret);
	}
	else adc_temperature = (uint32_t) (((uint32_t) buffer_out[0] * 4096) | ((uint32_t) buffer_out[1] * 16) | ((uint32_t) buffer_out[2] / 16));

	vTaskDelay(100/portTICK_RATE_MS);

	// Humidity reading
	ret = BME680_read(buffer_out, BME680_REG_HUM_MSB, 2);
	if(ret != ESP_OK) {
		printf("ERROR reading humidity data registers: %x\n", ret);
	}
	else adc_hum = (uint16_t) (((uint32_t) buffer_out[0] * 256) | (uint32_t) buffer_out[1]);


	*press_comp = (float) BME680_pressure_compensation(adc_pres, NVM_coef) / 100;
	*tempe_comp = (float) BME680_temperature_compensation(adc_temperature, NVM_coef) / 100;
	*hum_comp = (float) BME680_humidity_compensation(adc_hum, NVM_coef) / 1000;

	vTaskDelay(10/portTICK_RATE_MS);

	return 0;
}



#ifdef __cplusplus
}
#endif
