
#include <stdio.h> 
#include "esp_log.h"
#include "driver/i2c.h"


#ifndef _SHT20_H_ 
#define _SHT20_H_

#define I2C_MASTER_SCL_IO 22 
#define I2C_MASTER_SDA_IO 21 
#define I2C_MASTER_NUM 1
#define I2C_MASTER_FREQ_HZ 400000 
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS 1000


#define ACK_CHECK_EN 0x1 /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0
#define NACK_VAL 0x1
#define SHT20_SENSOR_ADDR 0x40
#define SHT20_SENSOR_TMEASUREMENT_NOHOLD_ADDR 0XF3 

#define SHT20_SENSOR_RHMEASUREMENT_NOHOLD_ADDR 0XF5 

//#define SHT20_SENSOR_USER_REG_ADDR 0XE7

esp_err_t i2c_master_init(void);
float SHT20_Get_Tempreture(void); 
float SHT20_Get_Humidity(void);
#endif 
/*_SHT20_H_*/


static const char *TAG = "i2c-simple-example";



esp_err_t i2c_master_init(void)
{
	int i2c_master_port = I2C_MASTER_NUM;
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = I2C_MASTER_SDA_IO,
		.scl_io_num = I2C_MASTER_SCL_IO,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_MASTER_FREQ_HZ,
};

i2c_param_config(i2c_master_port, &conf);

return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE,I2C_MASTER_TX_BUF_DISABLE,0);
}


float SHT20_Get_Tempreture (void)
{
	
	uint8_t data[2] = {0, 0};
	float Tempreture = 0;
	i2c_cmd_handle_t handle = i2c_cmd_link_create();
	
	i2c_master_start(handle);
	i2c_master_write_byte(handle, SHT20_SENSOR_ADDR << 1 | 0, ACK_CHECK_EN);
	i2c_master_write_byte(handle, SHT20_SENSOR_TMEASUREMENT_NOHOLD_ADDR, ACK_CHECK_EN); 
	vTaskDelay (0.02 / portTICK_PERIOD_MS);
	i2c_master_stop(handle);
	
	i2c_master_cmd_begin(I2C_MASTER_NUM, handle, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(handle);
	vTaskDelay(90 / portTICK_PERIOD_MS);
	
	handle = i2c_cmd_link_create();
	i2c_master_start(handle);
	i2c_master_write_byte(handle, SHT20_SENSOR_ADDR << 1 | 1, ACK_CHECK_EN); 
	i2c_master_read_byte(handle, &data[0], ACK_VAL); 
	i2c_master_read_byte(handle, &data[1], NACK_VAL);
	

	i2c_master_stop(handle);
	
	i2c_master_cmd_begin(I2C_MASTER_NUM, handle, 1000/ portTICK_PERIOD_MS); 
	i2c_cmd_link_delete(handle);
	Tempreture = (data[0] << 8) | (data[1] & 0xFC);
	Tempreture = 175.72* ((float) Tempreture / 65536) - 46.85;
	return Tempreture;
}



float SHT20_Get_Humidity(void)
{
	uint8_t data[2] = {0, 0};
	float Humidity = 0;
	i2c_cmd_handle_t handle = i2c_cmd_link_create();
	
	i2c_master_start(handle);
	i2c_master_write_byte(handle, SHT20_SENSOR_ADDR << 1 | 0, ACK_CHECK_EN);
	i2c_master_write_byte(handle, SHT20_SENSOR_RHMEASUREMENT_NOHOLD_ADDR, ACK_CHECK_EN); 
	vTaskDelay(0.02 / portTICK_PERIOD_MS);
	i2c_master_stop(handle);
	
	i2c_master_cmd_begin(I2C_MASTER_NUM, handle, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(handle); 
	vTaskDelay(35/portTICK_PERIOD_MS);
	
	handle = i2c_cmd_link_create();
	
	i2c_master_start(handle);
	i2c_master_write_byte(handle, SHT20_SENSOR_ADDR << 1 | 1, ACK_CHECK_EN);
	i2c_master_read_byte(handle, &data[0], ACK_VAL); 
	i2c_master_read_byte(handle, &data[1], NACK_VAL);
	
	
	i2c_master_stop(handle);
	
	i2c_master_cmd_begin(I2C_MASTER_NUM, handle, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(handle);
	
	Humidity = (data[0] << 8) | (data[1] & 0xFC);
	Humidity = 125 * ((float) Humidity / 65536) - 6; 
	return Humidity;
	
	
}
void app_main(void)
{
	ESP_ERROR_CHECK(i2c_master_init());
	ESP_LOGI(TAG, "I2C initialized successfully");
	
	while (1)
	{
		printf("Tempreture = %.2f°C \n", SHT20_Get_Tempreture());
		printf("Humidity = %.2f%% \n\n", SHT20_Get_Humidity());
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

