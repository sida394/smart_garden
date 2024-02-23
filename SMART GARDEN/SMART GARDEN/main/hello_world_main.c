#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define I2C_MASTER_SCL_IO_1    22          // GPIO number for I2C master clock of sensor 1
#define I2C_MASTER_SDA_IO_1    21          // GPIO number for I2C master data of sensor 1
#define I2C_MASTER_SCL_IO_2    18          // GPIO number for I2C master clock of sensor 2
#define I2C_MASTER_SDA_IO_2    19          // GPIO number for I2C master data of sensor 2
#define I2C_MASTER_SCL_IO_3    22          // GPIO number for I2C master clock of sensor 3 (SHT20)
#define I2C_MASTER_SDA_IO_3    21          // GPIO number for I2C master data of sensor 3 (SHT20)

#define I2C_MASTER_NUM_1       I2C_NUM_0   // I2C port number for master dev of sensor 1 (SHTC3)
#define I2C_MASTER_NUM_3       I2C_NUM_1   // I2C port number for master dev of sensor 3 (SHT20)

#define I2C_MASTER_TX_BUF_DISABLE 0        // I2C master do not need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0        // I2C master do not need buffer
#define I2C_MASTER_FREQ_HZ   100000        // I2C master clock frequency

#define SHTC3_SENSOR_ADDR      0x70        // I2C address of SHTC3 sensor
#define SHT20_SENSOR_ADDR      0x40        // I2C address of SHT20 sensor


#define ACK_CHECK_EN 0x1 /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0
#define NACK_VAL 0x1
#define SHT20_SENSOR_TMEASUREMENT_NOHOLD_ADDR 0XF3
#define SHT20_SENSOR_RHMEASUREMENT_NOHOLD_ADDR 0XF5

#define DEFAULT_VREF    1100        // Default ADC reference voltage (mV)
#define NO_OF_SAMPLES   64          // Number of samples for ADC averaging

esp_adc_cal_characteristics_t *adc_chars; // Declare adc_chars variable

void initialize_adc() {
    // Configure ADC characteristics
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);

    // Initialize ADC characteristics
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t)); // Allocate memory for adc_chars
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars); // Initialize adc_chars
}


void read_analog_voltage() {
    uint32_t adc_reading = 0;
    // Perform ADC sampling
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        adc_reading += adc1_get_raw(ADC1_CHANNEL_0);
    }
    adc_reading /= NO_OF_SAMPLES; // Calculate average ADC reading

    // Convert ADC reading to voltage
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    printf("ADC Reading: %u\tVoltage: %umV\n", (unsigned int)adc_reading, (unsigned int)voltage);

}

void i2c_master_init(i2c_port_t i2c_num, gpio_num_t sda_io_num, gpio_num_t scl_io_num) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_io_num,
        .scl_io_num = scl_io_num,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_num, &conf);
    i2c_driver_install(i2c_num, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

float SHTC3_Get_Temperature(i2c_port_t i2c_num) {
		uint8_t data[6];  // Buffer to store temperature and humidity data
	// Send command to measure temperature first and humidity second
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, 0x6458 >> 8, true);  // MSB of temperature command
        i2c_master_write_byte(cmd, 0x6458 & 0xFF, true);  // LSB of temperature command
        i2c_master_stop(cmd);
        i2c_master_cmd_begin( i2c_num, cmd, 1000 /  portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        // Wait for temperature measurement to complete
        vTaskDelay(15 / portTICK_PERIOD_MS);  // Wait for at least 15 ms (as per datasheet)

        // Read temperature data
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read_byte(cmd, &data[0], I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &data[1], I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &data[2], I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &data[3], I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &data[4], I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &data[5], I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_MASTER_NUM_1, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        // Process temperature data (data[0] is MSB, data[1] is LSB)
        float temperature_raw = (data[0] << 8) | data[1];
        float temperature = -45 + 175 * (temperature_raw / 65535.0);
		return temperature;
        // Delay before next measurement
        vTaskDelay(1000 / portTICK_PERIOD_MS);
}

float SHTC3_Get_Humidity(i2c_port_t i2c_num) {
		uint8_t data[6];  // Buffer to store temperature and humidity data
	// Send command to measure temperature first and humidity second
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, 0x6458 >> 8, true);  // MSB of temperature command
        i2c_master_write_byte(cmd, 0x6458 & 0xFF, true);  // LSB of temperature command
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        // Wait for temperature measurement to complete
        vTaskDelay(15 / portTICK_PERIOD_MS);  // Wait for at least 15 ms (as per datasheet)

        // Read temperature data
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read_byte(cmd, &data[0], I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &data[1], I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &data[2], I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &data[3], I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &data[4], I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &data[5], I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        // Process humidity data (data[3] is MSB, data[4] is LSB)
        float humidity_raw = (data[3] << 8) | data[4];
        float humidity = 100 * (humidity_raw / 65535.0);
		return humidity;

        // Delay before next measurement
        vTaskDelay(1000 / portTICK_PERIOD_MS);
}

float SHT20_Get_Temperature (void)
{

	uint8_t data[2] = {0, 0};
	float Tempreture = 0;
	i2c_cmd_handle_t handle = i2c_cmd_link_create();

	i2c_master_start(handle);
	i2c_master_write_byte(handle, SHT20_SENSOR_ADDR << 1 | 0, ACK_CHECK_EN);
	i2c_master_write_byte(handle, SHT20_SENSOR_TMEASUREMENT_NOHOLD_ADDR, ACK_CHECK_EN);
	vTaskDelay (0.02 / portTICK_PERIOD_MS);
	i2c_master_stop(handle);

	i2c_master_cmd_begin(I2C_MASTER_NUM_1, handle, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(handle);
	vTaskDelay(90 / portTICK_PERIOD_MS);

	handle = i2c_cmd_link_create();
	i2c_master_start(handle);
	i2c_master_write_byte(handle, SHT20_SENSOR_ADDR << 1 | 1, ACK_CHECK_EN);
	i2c_master_read_byte(handle, &data[0], ACK_VAL);
	i2c_master_read_byte(handle, &data[1], NACK_VAL);


	i2c_master_stop(handle);

	i2c_master_cmd_begin(I2C_MASTER_NUM_1, handle, 1000/ portTICK_PERIOD_MS);
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

	i2c_master_cmd_begin(I2C_MASTER_NUM_1, handle, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(handle);
	vTaskDelay(35/portTICK_PERIOD_MS);

	handle = i2c_cmd_link_create();

	i2c_master_start(handle);
	i2c_master_write_byte(handle, SHT20_SENSOR_ADDR << 1 | 1, ACK_CHECK_EN);
	i2c_master_read_byte(handle, &data[0], ACK_VAL);
	i2c_master_read_byte(handle, &data[1], NACK_VAL);


	i2c_master_stop(handle);

	i2c_master_cmd_begin(I2C_MASTER_NUM_1, handle, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(handle);

	Humidity = (data[0] << 8) | (data[1] & 0xFC);
	Humidity = 125 * ((float) Humidity / 65536) - 6;
	return Humidity;


}

void app_main(void) {
    i2c_master_init(I2C_MASTER_NUM_1, I2C_MASTER_SDA_IO_1, I2C_MASTER_SCL_IO_1);
    i2c_master_init(I2C_MASTER_NUM_3, I2C_MASTER_SDA_IO_2, I2C_MASTER_SCL_IO_2);
    initialize_adc(); // Initialize ADC

    while (1) {
        // Read data from SHTC3 sensor 1
        float temperature_1 = SHTC3_Get_Temperature(I2C_MASTER_NUM_1);
        float humidity_1 = SHTC3_Get_Humidity(I2C_MASTER_NUM_1);
        printf("SHTC3 Sensor 1 - Temperature: %.2f C, Humidity: %.2f%%\n", temperature_1, humidity_1);


        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // Read data from SHTC3 sensor 2
        float temperature_2 = SHTC3_Get_Temperature(I2C_MASTER_NUM_3);
        float humidity_2 = SHTC3_Get_Humidity(I2C_MASTER_NUM_3);
        printf("SHTC3 Sensor 2 - Temperature: %.2f C, Humidity: %.2f%%\n", temperature_2, humidity_2);

        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // Read data from SHT20 sensor
        float temperature_SHT20 = SHT20_Get_Temperature();
        float humidity_SHT20 = SHT20_Get_Humidity();
        printf("SHT20 Sensor   - Temperature: %.2f C, Humidity: %.2f%%\n", temperature_SHT20, humidity_SHT20);

        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // Read Battery Voltage
        read_analog_voltage(); // Read analog voltage
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second

        printf("%s\n", "//////////////////////////////////////////////////////////////////////");
        vTaskDelay(2500 / portTICK_PERIOD_MS);
    }
}
