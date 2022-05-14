#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

//	********************************


esp_err_t 	MS5611_write_byte(uint8_t MS5611_command);
esp_err_t 	MS5611_read(uint8_t * buffer_out, unsigned size);
int 		MS5611_coef(uint16_t *C1, uint16_t *C2, uint16_t *C3, uint16_t *C4, uint16_t *C5, uint16_t *C6);
void 		MS5611_init();
int32_t 	MS5611_temperature(uint16_t coef[5], uint32_t* t_no_compensate);
int32_t 	press_calibrate(uint32_t temperature, int32_t pressure, uint16_t C1, uint16_t C2, uint16_t C3, uint16_t C4, uint16_t C5, uint16_t C6);
int32_t 	press_calibrate_2(uint32_t temperature, int32_t pressure, uint16_t C1, uint16_t C2, uint16_t C3, uint16_t C4, uint16_t C5, uint16_t C6);
int32_t 	press_calibrate_3(uint32_t temperature, int32_t pressure, uint16_t C1, uint16_t C2, uint16_t C3, uint16_t C4, uint16_t C5, uint16_t C6);
int32_t 	MS5611_pressure(uint16_t coef[5],  uint32_t t_no_compensate);
void 		display_temp_press(long temperature, long pressure);
float 		MS5611_read_temperature();
float 		MS5611_read_pressure();


//	********************************



//Macro general del dispositivo
#define MS5611_RESET				0x1E

//Macros para la medición de presión
#define MS5611_CONV_D1_OSR256		0x40
#define MS5611_CONV_D1_OSR512		0x42
#define MS5611_CONV_D1_OSR1024		0x44
#define MS5611_CONV_D1_OSR2048		0x46
#define MS5611_CONV_D1_OSR4096		0x48

//Macros para la medición de temperatura
#define MS5611_CONV_D2_OSR256		0x50
#define MS5611_CONV_D2_OSR512		0x52
#define MS5611_CONV_D2_OSR1024		0x54
#define MS5611_CONV_D2_OSR2048		0x56
#define MS5611_CONV_D2_OSR4096		0x58

//Macro para el ADC converter
#define MS5611_ADC_READ				0x00

//Defines para la conexión mediante I2C
#define I2C_MASTER_FREQ_HZ          100000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000
#define I2C_MASTER_SDA_IO 			21
#define I2C_MASTER_SCL_IO 			22
#define I2C_MASTER_NUM	 			I2C_NUM_1

#define DATA_LENGTH	  	        	512
#define I2C_SLAVE_NUM 				I2C_NUM_0    /*!<I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN	  	(2*DATA_LENGTH) /*!<I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN  		(2*DATA_LENGTH) /*!<I2C slave rx buffer size */

#define MS5611_ADDRESS				0x77
#define COEFF1_COMMAND				0xA2
#define COEFF2_COMMAND				0xA4
#define COEFF3_COMMAND				0xA6
#define COEFF4_COMMAND				0xA8
#define COEFF5_COMMAND				0xAA
#define COEFF6_COMMAND				0xAC

#define DIST_COEF					2

//Constantes del fabricante específicas de esta placa para la calibración del dispositivo
//#define C1	54535
//#define C2	54134
//#define C3	34040
//#define C4	30924
//#define C5	30191
//#define C6	28416


#ifdef __cplusplus
}
#endif
