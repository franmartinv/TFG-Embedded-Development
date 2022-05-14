#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "BME680.h"

#ifndef CCS811_H_
#define CCS811_H_

#ifdef __cplusplus
extern "C"
{
#endif

// Configuracion de I2C
#define 	I2C_MASTER_FREQ_HZ          		100000
#define 	I2C_MASTER_TX_BUF_DISABLE   		0                          /*!< I2C master doesn't need buffer */
#define 	I2C_MASTER_RX_BUF_DISABLE   		0                          /*!< I2C master doesn't need buffer */
#define 	I2C_MASTER_TIMEOUT_MS       		1000
#define 	I2C_MASTER_SDA_IO 					21
#define 	I2C_MASTER_SCL_IO 					22
#define 	I2C_MASTER_NUM	 					I2C_NUM_1

#define 	DATA_LENGTH	  	        			512
#define 	I2C_SLAVE_NUM 						I2C_NUM_0    /*!<I2C port number for slave dev */
#define 	I2C_SLAVE_TX_BUF_LEN	  			(2*DATA_LENGTH) /*!<I2C slave tx buffer size */
#define 	I2C_SLAVE_RX_BUF_LEN  				(2*DATA_LENGTH) /*!<I2C slave rx buffer size */


//	Direccion del dispositivo
#define 	CCS811_I2C_ADDRESS					0x5A
#define		CCS811_I2C_ADDRESS2					0x5B
#define		CCS811_HARDWARE_ID					0x81


//	Register map
#define		CCS811_REG_STATUS					0x00
#define		CCS811_REG_MEAS_MODE				0x01
#define		CCS811_REG_ALG_RESULT_DATA			0x02
#define		CCS811_REG_RAW_DATA					0x03
#define		CCS811_REG_ENV_DATA					0x05
#define		CCS811_REG_NTC						0x06
#define		CCS811_REG_THRESHOLDS				0x10
#define		CCS811_REG_BASELINE					0x11
#define		CCS811_REG_HW_ID					0x20
#define		CCS811_REG_HW_VERSION				0x21
#define		CCS811_REG_FW_BOOT_VERSION			0x23
#define		CCS811_REG_FW_APP_VERSION			0x24
#define		CCS811_REG_ERROR_ID					0xE0
#define 	CCS811_REG_APP_ERASE      		 	0xF1
#define 	CCS811_REG_APP_DATA        			0xF2
#define 	CCS811_REG_APP_VERIFY      			0xF3
#define 	CCS811_REG_APP_START       			0xF4
#define		CCS811_REG_SW_RESET					0xFF


//	Codigos de errores
#define		CCS811_ERR_BASE						0xA000

#define 	CCS811_ERR_BOOT_MODE      			(CCS811_ERR_BASE + 1) 		//	Firmware en bootmode
#define 	CCS811_ERR_NO_APP         			(CCS811_ERR_BASE + 2) 		//	No hay aplicacion firmware cargada
#define 	CCS811_ERR_NO_NEW_DATA    			(CCS811_ERR_BASE + 3) 		//	No hay nuevos datos muestreados
#define 	CCS811_ERR_NO_IAQ_DATA    			(CCS811_ERR_BASE + 4) 		//	No hay nuevos datos muestreados
#define 	CCS811_ERR_HW_ID          			(CCS811_ERR_BASE + 5) 		//	ID hardware invalida
#define 	CCS811_ERR_INV_SENS       			(CCS811_ERR_BASE + 6) 		//	ID del sensor invalida
#define 	CCS811_ERR_WR_REG_INV     			(CCS811_ERR_BASE + 7) 		//	Direccion del registro de escritura invalido
#define 	CCS811_ERR_RD_REG_INV     			(CCS811_ERR_BASE + 8) 		//	Direccion del registro de lectura invalido
#define 	CCS811_ERR_MM_INV         			(CCS811_ERR_BASE + 9) 		//	Modo del sensor invalido
#define 	CCS811_ERR_MAX_RESIST     			(CCS811_ERR_BASE + 10) 		//	Superada resistencia maxima del sensor
#define 	CCS811_ERR_HEAT_FAULT     			(CCS811_ERR_BASE + 11) 		//	Corriente del calentador fuera de rango
#define 	CCS811_ERR_HEAT_SUPPLY    			(CCS811_ERR_BASE + 12) 		//	Voltaje del calentador equivocado
#define 	CCS811_ERR_WRONG_MODE     			(CCS811_ERR_BASE + 13) 		//	Modo de medida equivocado
#define 	CCS811_ERR_RD_STAT_FAILED 			(CCS811_ERR_BASE + 14) 		//	Estado del registro de lectura fallido
#define 	CCS811_ERR_RD_DATA_FAILED 			(CCS811_ERR_BASE + 15) 		//	Lectura del sensor fallida
#define 	CCS811_ERR_APP_START_FAIL 			(CCS811_ERR_BASE + 16) 		//	Inicializacion del sensor fallida
#define 	CCS811_ERR_WRONG_PARAMS   			(CCS811_ERR_BASE + 17) 		//	Parametros mal usados


//	Rangos de medidas
#define		CCS_ECO2_RANGE_MIN					400
#define		CCS_ECO2_RANGE_MAX					8192
#define		CCS_TVOC_RANGE_MIN					0
#define		CCS_TVOC_RANGE_MAX					1187


//	Modos de funcionamiento
#define		CCS811_MODE_IDLE		0x00	//((0x00 << 4) & 0x7C)
#define		CCS811_MODE_1S			0x10	//((0x00 << 4) & 0x7C)
#define		CCS811_MODE_10S			0x20	//((0x02 << 4) & 0x7C)
#define		CCS811_MODE_60S			0x30	//((0x03 << 4) & 0x7C)
#define		CCS811_MODE_250MS		0x40	//((0x04 << 4) & 0x7C)



//	Bits del registro de estado
#define 	CCS811_STATUS_ERROR        			0x01  // error, consultar CCS811_REG_ERROR
#define 	CCS811_STATUS_DATA_RDY     			0x08  // Nuevo dato muestreado en ALG_RESULT_DATA
#define 	CCS811_STATUS_APP_VALID    			0x10  // Cargado nuevo firmware valido
#define 	CCS811_STATUS_FW_MODE      			0x80  // Firmware en modo aplicacion


//	Bits del registro de errores (hexadecimal, ver posiciones en binario)
#define		CCS811_MSG_INV						0x01
#define		CCS811_READ_REG_INV					0x02
#define		CCS811_MEAS_MODE_INV				0x04
#define		CCS811_MAX_RES						0x08
#define		CCS811_HEATER_FAIL					0x10
#define		CCS811_HEATER_SUPPLY				0x20


esp_err_t 	CCS811_write_register(uint8_t CCS811_register, uint8_t CCS811_register_value);
esp_err_t 	CCS811_write_byte(uint8_t CCS811_register, unsigned size);
esp_err_t 	CCS811_write_data_register(uint8_t CCS811_register, uint8_t *buffer_in, unsigned size);
esp_err_t 	CCS811_write_read_byte(uint8_t * buffer_out, uint8_t CCS811_register, unsigned size);
esp_err_t 	CCS811_write_read_byte2(uint8_t *buffer_out, uint8_t CCS811_register, unsigned size);
esp_err_t 	CCS811_software_reset();
int 		CCS811_mode(uint8_t mode_number);
void 		CCS811_configuring_baseline();
int 		CCS811_init(uint8_t mode_number);
void 		CCS811_temperature_humidity_compensation(BME680_calib_t *NVM_coef, float *hum_comp, float *tempe_comp, float *press_comp);
int 		CCS811_read_all_variables(uint8_t mode_number, uint16_t *eco2, uint16_t *TVOC, float *hum_comp, float *tempe_comp, float *press_comp, BME680_calib_t *NVM_coef);


#ifdef __cplusplus
}
#endif
#endif






