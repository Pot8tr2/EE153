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
#include "driver/i2c.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_timer.h"





// I2C
#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_SDA_IO      2
#define I2C_MASTER_SCL_IO      3
#define I2C_MASTER_FREQ_HZ     100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

// TMP1075
#define TMP1075_ADDR           0x48
#define TMP1075_REG_TEMP       0x00



/* Pick your ADC channel here.  Channels 0-4 correspond to GPIO 0-4, and are
 * on ADC 1. ADC 2 doesn't work on ESP32-C3 due to a silicon bug, so GPIO 5
 * isn't available. */
#define ADC_CHANNEL ADC_CHANNEL_0

#define WIFI_SSID      "Tufts_Wireless"
#define WIFI_PASS      ""

#define BROKER_URI "mqtt://bell-mqtt.eecs.tufts.edu"



static esp_err_t tmp1075_read_raw(int16_t *raw)
{
    uint8_t data[2];

    // Write pointer to temperature register
    esp_err_t ret = i2c_master_write_read_device(
        I2C_MASTER_NUM,
        TMP1075_ADDR,
        (uint8_t[]){TMP1075_REG_TEMP},
        1,
        data,
        2,
        pdMS_TO_TICKS(50)
    );

    if (ret != ESP_OK) return ret;

    // TMP1075 12-bit temperature
    int16_t value = (data[0] << 8) | data[1];
    value >>= 4;

    // handle negative
    if (value & 0x800) {
        value |= 0xF000;
    }

    *raw = value;
    return ESP_OK;
}

static float tmp1075_read_celsius()
{
    int16_t raw;
    if (tmp1075_read_raw(&raw) != ESP_OK) {
        return NAN;
    }
    return raw * 0.0625f;
}

static float tmp1075_read_fahrenheit()
{
    float c = tmp1075_read_celsius();
    if (isnan(c)) return NAN;
    return (c * 9.0f / 5.0f) + 32.0f;
}

static void i2c_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    ESP_ERROR_CHECK(err);

    err = i2c_driver_install(
        I2C_MASTER_NUM,
        conf.mode,
        I2C_MASTER_RX_BUF_DISABLE,
        I2C_MASTER_TX_BUF_DISABLE,
        0
    );
    ESP_ERROR_CHECK(err);
}




void app_main() {

    // Enable Flash (aka non-volatile storage, NVS)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // calculate the amount of time you need to sleep and set esp sleep timer to that
    uint32_t sleep_time=(int)((double)60/((double)62)*3600);
    char output[100];
    sprintf(output, "sleep time is%ld\n",sleep_time);
    printf(output);
    uint64_t us_to_seconds=1000000;
    uint64_t total_time_sleep= (uint64_t)sleep_time*us_to_seconds;
    esp_sleep_enable_timer_wakeup(total_time_sleep);




    // Normally we'd need to initialize the event loop and the network stack,
    // but both of these will be done when we connect to WiFi.
    // printf("Connecting to WiFi...");
    wifi_connect(WIFI_SSID, WIFI_PASS);
    // try to connect to the server?
    // if it fails ToT

    // Initialize the MQTT client
    // Read the documentation for more information on what you can configure:
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/api-reference/protocols/mqtt.html

    // get the rssi and check that it is high enough. If not go to sleep for 5 mins. 
    // keeps wifi from being on too long.
    int rssi=0;
    esp_wifi_sta_get_rssi(&rssi);
    if(rssi<-80){
        esp_sleep_enable_timer_wakeup(1000000*300);
        esp_deep_sleep_start();
    }

    esp_mqtt_client_config_t mqtt_cfg = {
        
        .broker.address.uri = BROKER_URI,

    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    // do dns lookup to connect to the client. Can hardcode ip here, or can move to another bush with higher rssi strength. I did the later. 
    esp_mqtt_client_start(client);

    // ESP_LOGD("TAG", "Initializing I2C...");
    // printf("Initializing I2C...");

    //initilize i2c and read form the temperature sensor.
    i2c_init();
    float tempC = tmp1075_read_celsius();
    // format the string for publishing 
    sprintf(output, "{\"measurements\": [ [0, %.2f] ],  \"board_time\": 0, \"heartbeat\": {\"status\": 0, \"rssi\": %d}}",tempC,  rssi);
    // publish then go to sleep
    esp_mqtt_client_publish(client, "teamR/node0/update", output, 0, 0, 0);
    esp_deep_sleep_start();
    while(1){

    }

}