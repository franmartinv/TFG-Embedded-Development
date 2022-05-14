#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

#ifndef BME680_H_
#define BME680_H_

#ifdef __cplusplus
extern "C" {
#endif

// Parametros para conexion I2C
#define 	I2C_MASTER_FREQ_HZ          	100000
#define 	I2C_MASTER_TX_BUF_DISABLE   	0                          /*!< I2C master doesn't need buffer */
#define 	I2C_MASTER_RX_BUF_DISABLE   	0                          /*!< I2C master doesn't need buffer */
#define 	I2C_MASTER_TIMEOUT_MS       	1000
#define 	I2C_MASTER_SDA_IO 				21
#define 	I2C_MASTER_SCL_IO 				22
#define 	I2C_MASTER_NUM	 				I2C_NUM_1

#define 	DATA_LENGTH	  	        		512
#define 	I2C_SLAVE_NUM 					I2C_NUM_0    /*!<I2C port number for slave dev */
#define 	I2C_SLAVE_TX_BUF_LEN	  		(2*DATA_LENGTH) /*!<I2C slave tx buffer size */
#define 	I2C_SLAVE_RX_BUF_LEN  			(2*DATA_LENGTH) /*!<I2C slave rx buffer size */

// Direccion I2C del dispositivo
#define		BME680_I2C_ADDRESS				0x76
#define		BME680_I2C_ADDRESS2				0x77

// Registros del BME680
#define 	BME680_REG_TEMP_XLSB   			0x24					// bits: 7-4
#define 	BME680_REG_TEMP_LSB    			0x23
#define 	BME680_REG_TEMP_MSB    			0x22
#define 	BME680_REG_PRESS_XLSB  			0x21					// bits: 7-4
#define 	BME680_REG_PRESS_LSB   			0x20
#define	 	BME680_REG_PRESS_MSB   			0x1F
#define		BME680_REG_HUM_MSB				0x25
#define		BME680_REG_HUM_LSB				0x26
#define		BME680_REG_GAS_R_MSB			0x2A
#define		BME680_REG_GAS_R_LSB			0x2B	// bits: 7-6 -> LSB part gas resistance of raw gas resistance
													// bit 5: gas valid bit
													// bit 4: heater stability bit
													// bits: 3-0 -> ADC range of measured gas resistance

#define 	BME680_REG_CONFIG      			0xF5
#define 	BME680_REG_CTRL_MEAS      		0x74	// bits: 1-0 control the mode of the sensor
													// bits: 4-2 pressure oversampling
													// bits: 7-5 temperature oversampling
#define 	BME680_REG_STATUS      			0x1D	// bit 7: new data flag
													// bit 6: gas measuring status flag
													// bit 5: measuring status flag
													// bits 3-0: gas measurement index
#define 	BME680_REG_CTRL_HUM    			0x72
#define 	BME680_REG_ID          			0xD0
#define		BME680_REG_IRR_filter			0x75	// bits: 4-2 filter coefficient
#define		BME680_ctrl_gas_0				0x70	// bit 3 == 1; turn off current injected to heater setting bit to one
#define		BME680_ctrl_gas_1				0x71	// bits 3-0 indicates index of heater set point that will be used in force mode
													// bit 4 == 1; for running gas conversions in appropiate mode
#define		BME680_RESET_VALUE				0xB6
#define		BME680_REG_SOFT_RESET			0xE0


// Configuracion de oversampling
#define		BME680_oversampling_skip		0x00	// (000)
#define		BME680_oversampling_x1			0x01	// (001)
#define		BME680_oversampling_x2			0x02	// (010)
#define		BME680_oversampling_x4			0x03	// (011)
#define		BME680_oversampling_x8			0x04	// (100)
#define		BME680_oversampling_x16			0x07	// (111)


//	Configuracion de modo del sensor
#define		BME680_set_sleep_mode			0x00	// (00)
#define		BME680_set_forced_mode			0x01	// (01)


// Configuracion del filtro IRR
#define		BME680_set_filter_coef_0		0x00	// (000)
#define		BME680_set_filter_coef_1		0x01	// (001)
#define		BME680_set_filter_coef_3		0x02	// (010)
#define		BME680_set_filter_coef_7		0x03	// (011)
#define		BME680_set_filter_coef_15		0x04	// (100)
#define		BME680_set_filter_coef_31		0x05	// (101)
#define		BME680_set_filter_coef_63		0x06	// (110)
#define		BME680_set_filter_coef_127		0x07	// (111)


// Particular heaters set-point registers
#define		idac_hear_0						0x50
#define		idac_hear_1						0x51
#define		idac_hear_2						0x52
#define		idac_hear_3						0x53
#define		idac_hear_4						0x54
#define		idac_hear_5						0x55
#define		idac_hear_6						0x56
#define		idac_hear_7						0x57
#define		idac_hear_8						0x58
#define		idac_hear_9						0x59


// Target heater resistance
#define		res_wait_0						0x5A
#define		res_wait_1						0x5B
#define		res_wait_2						0x5C
#define		res_wait_3						0x5D
#define		res_wait_4						0x5E
#define		res_wait_5						0x5F
#define		res_wait_6						0x60
#define		res_wait_7						0x61
#define		res_wait_8						0x62
#define		res_wait_9						0x63


// Gas sensor wait-time multiplication factor
#define		gas_wait_factor_1				0x00		// (00)
#define		gas_wait_factor_4				0x01		// (01)
#define		gas_wait_factor_16				0x02		// (10)
#define		gas_wait_factor_64				0x03		// (11)


// Gas sensor wait time register
#define		gas_wait_0						0x64
#define		gas_wait_1						0x65
#define		gas_wait_2						0x66
#define		gas_wait_3						0x67
#define		gas_wait_4						0x68
#define		gas_wait_5						0x69
#define		gas_wait_6						0x6A
#define		gas_wait_7						0x6B
#define		gas_wait_8						0x6C
#define		gas_wait_9						0x6D


// Heater profile set-point
#define		BME680_heater_profile_0			0x00	// (0000)
#define		BME680_heater_profile_1			0x01	// (0001)
#define		BME680_heater_profile_2			0x02	// (0010)
#define		BME680_heater_profile_3			0x03	// (0011)
#define		BME680_heater_profile_4			0x04	// (0100)
#define		BME680_heater_profile_5			0x05	// (0101)
#define		BME680_heater_profile_6			0x06	// (0110)
#define		BME680_heater_profile_7			0x07	// (0111)
#define		BME680_heater_profile_8			0x08	// (1000)
#define		BME680_heater_profile_9			0x09	// (1001)


// Coefficient address
#define 	BME680_COEFF_ADDR1				0x89
#define 	BME680_COEFF_ADDR2				0xE1
#define 	BME680_COEFF_SIZE				41
#define 	BME680_COEFF_ADDR1_LEN			25
#define 	BME680_COEFF_ADDR2_LEN			16

// MSB and LSB position in coefficient's lecture array
#define 	BME680_T2_LSB_REG				(1)
#define 	BME680_T2_MSB_REG				(2)
#define 	BME680_T3_REG					(3)
#define 	BME680_P1_LSB_REG				(5)
#define 	BME680_P1_MSB_REG				(6)
#define 	BME680_P2_LSB_REG				(7)
#define 	BME680_P2_MSB_REG				(8)
#define 	BME680_P3_REG					(9)
#define 	BME680_P4_LSB_REG				(11)
#define 	BME680_P4_MSB_REG				(12)
#define 	BME680_P5_LSB_REG				(13)
#define	 	BME680_P5_MSB_REG				(14)
#define 	BME680_P7_REG					(15)
#define 	BME680_P6_REG					(16)
#define 	BME680_P8_LSB_REG				(19)
#define 	BME680_P8_MSB_REG				(20)
#define 	BME680_P9_LSB_REG				(21)
#define 	BME680_P9_MSB_REG				(22)
#define 	BME680_P10_REG					(23)
#define	 	BME680_H2_MSB_REG				(25)
#define	 	BME680_H2_LSB_REG				(26)
#define 	BME680_H1_LSB_REG				(26)
#define 	BME680_H1_MSB_REG				(27)
#define 	BME680_H3_REG					(28)
#define 	BME680_H4_REG					(29)
#define 	BME680_H5_REG					(30)
#define 	BME680_H6_REG					(31)
#define 	BME680_H7_REG					(32)
#define 	BME680_T1_LSB_REG				(33)
#define 	BME680_T1_MSB_REG				(34)
#define 	BME680_GH2_LSB_REG				(35)
#define 	BME680_GH2_MSB_REG				(36)
#define 	BME680_GH1_REG					(37)
#define 	BME680_GH3_REG					(38)


// Other coefficient address
#define		BME680_ADDR_RES_HEAT_VAL_ADDR		0x00
#define		BME680_ADDR_RES_HEAT_RANGE_ADDR		0x02
#define 	BME680_ADDR_RANGE_SW_ERR_ADDR		0x04
#define 	BME680_ADDR_SENS_CONF_START			0x5A
#define 	BME680_ADDR_GAS_CONF_START			0x64
#define 	LOW_TEMP							150
#define 	HIGH_TEMP 							350
#define 	HEATR_DUR							2000
#define 	N_MEAS								6
#define 	BME680_MAX_OVERFLOW_VAL      INT32_C(0x40000000)

typedef struct {
	// calibration coefficients of temperature
	uint16_t		BME680_par_T1;
	int16_t			BME680_par_T2;
	int8_t			BME680_par_T3;

	// calibration coefficients of pressure
	uint16_t		BME680_par_P1;
	int16_t			BME680_par_P2;
	int8_t			BME680_par_P3;
	int16_t			BME680_par_P4;
	int16_t			BME680_par_P5;
	int8_t			BME680_par_P6;
	int8_t			BME680_par_P7;
	int16_t			BME680_par_P8;
	int16_t			BME680_par_P9;
	uint8_t			BME680_par_P10;

	// calibration coefficients of humidity sensor
	uint16_t		BME680_par_H1;
	uint16_t		BME680_par_H2;
	int8_t			BME680_par_H3;
	int8_t			BME680_par_H4;
	int8_t			BME680_par_H5;
	uint8_t			BME680_par_H6;
	int8_t			BME680_par_H7;

	// calibration coefficients of gas sensor
	int8_t			BME680_par_gh1;
	int16_t 		BME680_par_gh2;
	int8_t 			BME680_par_gh3;

	// Other calibration coefficients
	int32_t			t_fine;
	uint8_t			res_heat_range;
	int8_t			res_heat_val;
	int8_t			range_sw_err;

} BME680_calib_t;

esp_err_t 	BME680_read(uint8_t * buffer_out, uint8_t BME680_command, unsigned size);
esp_err_t 	BME680_write_command(uint8_t BME680_command);
esp_err_t 	BME680_write_register(uint8_t BME680_register, uint8_t BME680_register_value);
esp_err_t 	BME680_calibration_data(BME680_calib_t *NVM_coef);
void 		BME680_settings(BME680_calib_t *NVM_coef);
int 		BME680_set_mode();
esp_err_t 	BME680_reset_function();
int 		BME680_init(BME680_calib_t *NVM_coef);
uint32_t 	BME680_pressure_compensation(uint32_t pres_adc , BME680_calib_t *NVM_coef);
int16_t 	BME680_temperature_compensation(uint32_t temp_adc, BME680_calib_t *NVM_coef);
uint32_t 	BME680_humidity_compensation(uint16_t hum_adc, BME680_calib_t *NVM_coef);
int 		BME680_get_data(float *press_comp, float *hum_comp, float *tempe_comp, BME680_calib_t *NVM_coef);

#ifdef __cplusplus
}
#endif
#endif


