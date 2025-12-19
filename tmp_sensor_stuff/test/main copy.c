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
#include "esp_sleep.h"
#include "esp_sntp.h"
#include "esp_log.h"


#include "esp_timer.h"



/* Pick your ADC channel here.  Channels 0-4 correspond to GPIO 0-4, and are
 * on ADC 1. ADC 2 doesn't work on ESP32-C3 due to a silicon bug, so GPIO 5
 * isn't available. */
#define ADC_CHANNEL ADC_CHANNEL_0

#define WIFI_SSID      "tufts_eecs"
#define WIFI_PASS      "foundedin1883"

#define BROKER_URI "mqtt://bell-mqtt.eecs.tufts.edu"


void app_main() {
    ESP_LOGI("APP", "app_main started");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI("APP", "PAST HERE TOOO???");
    while (true)
    {
        // ESP_LOGI("APP", "IN LOOP");
        printf("HERE?\n");
        // esp_sleep_enable_timer_wakeup(1000000);
        vTaskDelay(100);
    }
    
    

    // Normally we'd need to initialize the event loop and the network stack,
    // but both of these will be done when we connect to WiFi.
    // printf("Connecting to WiFi...");

// i2c_master_bus_config_t i2c_mst_config = {
//     .clk_source = I2C_CLK_SRC_DEFAULT,
//     .i2c_port = I2C_NUM_0,
//     .scl_io_num = GPIO_NUM_3,
//     .sda_io_num = GPIO_NUM_2,
//     .glitch_ignore_cnt = 7,
//     .flags.enable_internal_pullup = true,
// };

// i2c_master_bus_handle_t bus_handle;
// ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

// i2c_device_config_t dev_cfg = {
// .dev_addr_length = I2C_ADDR_BIT_LEN_7,
// .device_address = 0x48,
// .scl_speed_hz = 100000,
// };
//     // printf("HERE????");
// i2c_master_dev_handle_t dev_handle;
// ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));


// uint8_t set_config[20] = {0x01,0x60,0xA0};
// uint8_t temp_addr=0xf00;
// ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, set_config, 1, -1));

// uint8_t temp_bytes[2];

//         // printf("weee?\n");
//         // Sample the ADC and save the result
//         // //ln(r(t1/r(t2)))
//     // printf("HERE????");
// ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, &temp_addr, 1, -1));
// i2c_master_receive(dev_handle, temp_bytes, 2, -1);
// int16_t e = temp_bytes[0];
//     e= (e<<4)|(temp_bytes[1]>>4);
//     if(e&0x800){
//         e|=0xF000;
//     }
//     float temperature2=0.0625*e;
//     char ostr[20];
//     arr[i]=esp_timer_get_time();
//     i++;
//     // printf("KILL");
    
//     wifi_connect(WIFI_SSID, WIFI_PASS);
//     arr[i]=esp_timer_get_time();
//     i++;
//     // Initialize the MQTT client
//     // Read the documentation for more information on what you can configure:
//     // https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/api-reference/protocols/mqtt.html
//     esp_mqtt_client_config_t mqtt_cfg = {
//         .broker.address.uri = BROKER_URI,
//     };
//     esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
//     esp_mqtt_client_start(client);
//     // printf("HERE????");
    

//     // sprintf(ostr, "%.2f", temperature2);

//     esp_mqtt_client_publish(client, "sgoss02/iteration1/ic_temp", ostr, 0, 0, 0);
//     arr[i]=esp_timer_get_time();
//     i++;
//     for (int t=0;t<5;t++){
//         arr[t]=arr[t]-t_us;
//         // printf("HERE?");
//         printf("%+" PRId64 "\n", arr[t]);
//         vTaskDelay(1);
//     }

//     printf("%+" PRId64 "\n",time_us2);
//     gettimeofday(&tv_now, NULL);
//     time_us2 = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
//     printf("%+" PRId64 "\n", time_us2);
//     esp_deep_sleep_start();//this should be a second. so roughly should 


    // Enable Flash (aka non-volatile storage, NVS)
    
    

    // If we had other things to do with the ADC, we could release it with
    // adc_oneshot_del_unit(adc1_handle);

}