// Minimal code to initialize and read a value from the ADC
// Based on Espressif example: 
// https://github.com/espressif/esp-idf/blob/f68c131e5603feca21659e92ad85f0c3369692fe/examples/peripherals/adc/oneshot_read/main/oneshot_read_main.c
// Steven Bell <sbell@ece.tufts.edu>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // Used for timer delay
#include "esp_adc/adc_oneshot.h"
#include "math.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "mqtt_client.h"
#include "minimal_wifi.h"
#include "driver/i2c_master.h"
#include "esp"


/* Pick your ADC channel here.  Channels 0-4 correspond to GPIO 0-4, and are
 * on ADC 1. ADC 2 doesn't work on ESP32-C3 due to a silicon bug, so GPIO 5
 * isn't available. */
#define ADC_CHANNEL ADC_CHANNEL_0

#define WIFI_SSID      "tufts_eecs"
#define WIFI_PASS      "foundedin1883"

#define BROKER_URI "mqtt://bell-mqtt.eecs.tufts.edu"


void app_main() {


    // Enable Flash (aka non-volatile storage, NVS)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    

i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = GPIO_NUM_8,
    .sda_io_num = GPIO_NUM_9,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

i2c_master_bus_handle_t bus_handle;
ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

i2c_device_config_t dev_cfg = {
.dev_addr_length = I2C_ADDR_BIT_LEN_7,
.device_address = 0x48,
.scl_speed_hz = 100000,
};

i2c_master_dev_handle_t dev_handle;
ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));


uint8_t set_config[20] = {0x01,0x60,0xA0};
uint8_t temp_addr=0x00;
ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, set_config, 1, -1));

uint8_t temp_bytes[2];

    while(1){
        // Sample the ADC and save the result
        ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, &temp_addr, 1, -1));
        i2c_master_receive(dev_handle, temp_bytes, 2, -1);
        int16_t e = temp_bytes[0];
        e= (e<<4)|(temp_bytes[1]>>4);
        if(e&0x800){
            e|=0xF000;
        }
        float temperature=0.0625*e;
        char str[40];
        // sprintf(str, "temp is %.2f\n", temperature);
        // printf(str);
        // // printf("BEANS");
        vTaskDelay(100);
    }

    // If we had other things to do with the ADC, we could release it with
    // adc_oneshot_del_unit(adc1_handle);

}