// Minimal code to initialize and read a value from the ADC
// Based on Espressif example:
// https://github.com/espressif/esp-idf/blob/f68c131e5603feca21659e92ad85f0c3369692fe/examples/peripherals/adc/oneshot_read/main/oneshot_read_main.c
// Steven Bell <sbell@ece.tufts.edu>


#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // Used for timer delay
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "driver/dac_oneshot.h"





/* Pick your ADC channel here.  Channels 0-4 correspond to GPIO 0-4, and are
* on ADC 1. ADC 2 doesn't work on ESP32-C3 due to a silicon bug, so GPIO 5
* isn't available. */
#define ADC_CHANNEL ADC_CHANNEL_0
#define BLINK_GPIO 1 // Change this to whatever GPIO pin you're using




void app_main() {
   int arr[500];
   int cur_val=0;
   int adc_raw=0;


   gpio_reset_pin(BLINK_GPIO);
   gpio_set_direction(BLINK_GPIO, GPIO_MODE_INPUT);



//    ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
    adc_cali_curve_fitting_config_t cali_config = {
    .unit_id = ADC_UNIT_1,
    .atten = ADC_ATTEN_DB_11,
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    // ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &handle));
    adc_cali_handle_t cali_handle = NULL;
    adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle);


   // Configure the ADC
   adc_oneshot_unit_init_cfg_t adc1_init_cfg = {
       .unit_id = ADC_UNIT_1
   };
   adc_oneshot_unit_handle_t adc1_handle;
   adc_oneshot_new_unit(&adc1_init_cfg, &adc1_handle);




   // Configure the channel within the ADC
   adc_oneshot_chan_cfg_t adc1_cfg = {
       .bitwidth = ADC_BITWIDTH_DEFAULT, // Default is 12 bits (max)
       .atten = ADC_ATTEN_DB_11
   };


   adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL, &adc1_cfg);


   // Skip calibration setup for now
   int cal_voltage=0;
   int prev_val=0;
   cur_val=0;
   int running=0;
   int x = gpio_get_level(GPIO_NUM_1);
   while(1){
    prev_val=x;
    x = gpio_get_level(GPIO_NUM_1);
    if(x==0 && prev_val==1){
           adc_oneshot_read(adc1_handle, ADC_CHANNEL, &adc_raw);
           adc_cali_raw_to_voltage(cali_handle,adc_raw,&cal_voltage);
        //    printf("%d\n",adc_raw);
           if(cur_val>0)break;
           arr[0]=adc_raw;
           running=!running;
           cur_val+=1;

        //    printf("HIT");
    }else if(cur_val <500){
        if(running==1){
            adc_oneshot_read(adc1_handle, ADC_CHANNEL, &adc_raw);
            adc_cali_raw_to_voltage(cali_handle,adc_raw,&cal_voltage);
            // printf("%d\n",cal_voltage);
            // printf("%d\n",cur_val);
        // printf("%i\n",cur_val);
        arr[cur_val]=adc_raw;
        cur_val+=1;
        }

        // printf(" , %d\n", cur_val);
    }else{
        // printf("%i\n",cur_val);
        // printf("ENDED!");

        break;
    }
       
       //else{
    //        adc_oneshot_read(adc1_handle, ADC_CHANNEL, &adc_raw);
    //        cur_val+=1;
    //        
    //        // arr[cur_val]=adc_raw;
    //        // 
    //    }
       // Sample the ADC and save the result
       // Skip calibration conversion
       vTaskDelay(1);
   }

   printf("current voltage measurments");
   printf("Voltage, ADC output\n");
   printf(" , %d\n", cur_val);
   for(int i=0; i<cur_val;i++){
    float voltage=((3.2*(((double)i/500))));
    //    printf("%f",voltage);
       printf("%f", voltage);
       int temp=arr[i]*2450/4095;
       printf(" , %d\n", temp);

       vTaskDelay(1);
   }
   printf(" done");
   while(1){
    vTaskDelay(1);
   }



   // If we had other things to do with the ADC, we could release it with
   // adc_oneshot_del_unit(adc1_handle);


}

