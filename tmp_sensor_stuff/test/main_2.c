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
#include "driver/i2c_master.h"
#include "esp_sleep.h"
#include "driver/gpio.h"



/* Pick your ADC channel here.  Channels 0-4 correspond to GPIO 0-4, and are
 * on ADC 1. ADC 2 doesn't work on ESP32-C3 due to a silicon bug, so GPIO 5
 * isn't available. */
// #define ADC_CHANNEL ADC_CHANNEL_0
#define LED_PIN 2



void app_main() {

    // Enable Flash (aka non-volatile storage, NVS)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    // wifi_connect(WIFI_SSID,WIFI_PASS);
    // Normally we'd need to initialize the event loop and the network stack,
    // but both of these will be done when we connect to WiFi.
    gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << LED_PIN),      // Select GPIO 2
    .mode = GPIO_MODE_OUTPUT,            // Set as output
    .pull_up_en = GPIO_PULLUP_DISABLE,  // Disable pull-up
    .pull_down_en = GPIO_PULLDOWN_DISABLE,  // Disable pull-down
    .intr_type = GPIO_INTR_DISABLE             // Disable interrupts
  };

gpio_config(&io_conf);

    while(1){
        // printf("a\n");
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 0);
    }
    

    // If we had other things to do with the ADC, we could release it with
    // adc_oneshot_del_unit(adc1_handle);

}