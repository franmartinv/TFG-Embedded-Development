#ifdef __cplusplus
extern "C" {
#endif

#include "driver/gpio.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sdkconfig.h"
#include "CCS811.h"


/**
 * Master initialition
 *
 */
void hal_i2c_init()
{
	int i2c_master_port = I2C_MASTER_NUM;
	i2c_config_t conf;
	memset(&conf, 0, sizeof(i2c_config_t));
		conf.mode = I2C_MODE_MASTER;
	    conf.sda_io_num = 21;
	    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	    conf.scl_io_num = 22;
	    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	    conf.master.clk_speed = 100000;
	i2c_param_config(i2c_master_port, &conf);
	i2c_driver_install(i2c_master_port, conf.mode,
			I2C_MASTER_RX_BUF_DISABLE,
			I2C_MASTER_TX_BUF_DISABLE, 0);
}

#ifdef __cplusplus
}
#endif

