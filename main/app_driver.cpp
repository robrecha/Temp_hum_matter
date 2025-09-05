/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_log.h>
#include <stdlib.h>
#include <string.h>

#include <device.h>
#include <app_driver.h>
#include <sensor_data.h>
#include <my_screens/my_screens.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_err.h>

#include <cstdint>
#include <cmath>

#include <esp_matter.h>
#include "esp_matter_core.h"
#include <led_driver.h> 

#include <app/CommandHandler.h>
//#include <app/clusters/window-covering-server/window-covering-server.h>
//#include <lib/core/CHIPError.h>
//#include <app/clusters/window-covering-server/window-covering-delegate.h>
#include <app/server/Server.h>
#include <app_priv.h>

#include <app/server/OnboardingCodesUtil.h>
#include <credentials/examples/DeviceAttestationCredsExample.h>

static const char *TAG = "app_driver";

extern uint16_t temperature_sensor_endpoint_id;
extern uint16_t humidity_sensor_endpoint_id;

// Queues - Define here to remove duplicate definition error
QueueHandle_t q_commisioning_status;
QueueHandle_t q_I_led_pulse;
//QueueHandle_t q_RGB_led_pulse;
QueueHandle_t AHT0_sensor_queue = NULL;  // Definition
QueueHandle_t AHT1_sensor_queue = NULL;  // Definition

// AHT Temp Hum Sensor
#include <aht.h>

#ifdef CONFIG_TYPE_AHT1x
#define AHT_TYPE AHT_TYPE_AHT1x
#define AHT_TYPE AHT_TYPE_AHT20
#endif

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

void AHT10_0_TempHum_Task(void *pvParameters)
{
    static const char *AHT0_TAG = "AHT0";
    ESP_LOGI(AHT0_TAG, "Setup i2c");
    aht_t dev;
    dev.i2c_dev.port = I2C_NUM_0; // Use the appropriate I2C port
    dev.i2c_dev.addr = CONFIG_AHT0_I2C_ADDRESS;
    dev.i2c_dev.cfg.sda_io_num = gpio_num_t(CONFIG_AHT0_I2C_MASTER_SDA);
    dev.i2c_dev.cfg.scl_io_num = gpio_num_t(CONFIG_AHT0_I2C_MASTER_SCL);
    dev.i2c_dev.cfg.master.clk_speed = CONFIG_AHT0_I2C_MASTER_FREQ_HZ;
    dev.mode = AHT_MODE_NORMAL;
    dev.type = AHT_TYPE_AHT1x;

    ESP_LOGI(AHT0_TAG, "Init i2c");
    ESP_ERROR_CHECK(aht_init_desc(&dev, dev.i2c_dev.addr, dev.i2c_dev.port, dev.i2c_dev.cfg.sda_io_num, dev.i2c_dev.cfg.scl_io_num));
    //ESP_ERROR_CHECK(aht_init(&dev));

    bool calibrated;
    ESP_LOGI(AHT0_TAG, "get AHT status");
    ESP_ERROR_CHECK(aht_get_status(&dev, NULL, &calibrated));
    if (calibrated)
        ESP_LOGI(AHT0_TAG, " Sensor calibrated");
    else
        ESP_LOGW(AHT0_TAG, " Sensor not calibrated!");

    sensor_reading_t AHT_val;
    AHT_val.temperature[CURRENT] = 0;
    AHT_val.temperature[PREVIOUS] = 0;
    AHT_val.humidity[CURRENT] = 0;
    AHT_val.humidity[PREVIOUS] = 0;
    
    while (1)
    {
        esp_err_t res = aht_get_data(&dev, &AHT_val.temperature[CURRENT], &AHT_val.humidity[CURRENT]);
        if (res == ESP_OK) {
            ESP_LOGI(AHT0_TAG, " Temperature: %.1f°C, Humidity: %.2f%%", AHT_val.temperature[CURRENT], AHT_val.humidity[CURRENT]);
            if(AHT_val.temperature[CURRENT] != AHT_val.temperature[PREVIOUS] || AHT_val.humidity[CURRENT] != AHT_val.humidity[PREVIOUS]) {
                // Update Matter with new values
                updateMatterWithValues((u_int16_t)AHT_val.temperature[CURRENT] * 100, (u_int16_t)AHT_val.humidity[CURRENT] * 100);
                // Allocate update payload
                sensor_reading_t *AHT_update = (sensor_reading_t *)malloc(sizeof(sensor_reading_t));
                if (AHT_update) {
                    AHT_update->temperature[CURRENT] = AHT_val.temperature[CURRENT];
                    AHT_update->humidity[CURRENT]    = AHT_val.humidity[CURRENT];

                    // Schedule safe LVGL update
                    lv_async_call(update_temp_hum, AHT_update);
                }

//                if (xQueueSend(AHT0_sensor_queue, &AHT_val, pdMS_TO_TICKS(2)) != pdPASS) {        // For Screen update. Wait max 2ms if full
//                    ESP_LOGI("AHT0_sensor_queue", " q_full");
//                }
                AHT_val.temperature[PREVIOUS] = AHT_val.temperature[CURRENT];
                AHT_val.humidity[PREVIOUS] = AHT_val.humidity[CURRENT];
            }
        }
        else
            ESP_LOGE(AHT0_TAG, " Error reading data: %d (%s)", res, esp_err_to_name(res));

        vTaskDelay(CONFIG_AHT0_MEASURE_INTERVAL / portTICK_PERIOD_MS);
    }
}

void AHT10_1_TempHum_Task(void *pvParameters)
{
    static const char *AHT1_TAG = "AHT1";
    aht_t dev;
    dev.i2c_dev.port = I2C_NUM_0; // Use the appropriate I2C port
    dev.i2c_dev.addr = CONFIG_AHT1_I2C_ADDRESS;
    dev.i2c_dev.cfg.sda_io_num = gpio_num_t(CONFIG_AHT1_I2C_MASTER_SDA);
    dev.i2c_dev.cfg.scl_io_num = gpio_num_t(CONFIG_AHT1_I2C_MASTER_SCL);
    dev.i2c_dev.cfg.master.clk_speed = CONFIG_AHT1_I2C_MASTER_FREQ_HZ;
    dev.mode = AHT_MODE_NORMAL;
    dev.type = AHT_TYPE_AHT1x;

    ESP_ERROR_CHECK(aht_init_desc(&dev, dev.i2c_dev.addr, dev.i2c_dev.port, dev.i2c_dev.cfg.sda_io_num, dev.i2c_dev.cfg.scl_io_num));
    //ESP_ERROR_CHECK(aht_init(&dev));

    bool calibrated;
    ESP_ERROR_CHECK(aht_get_status(&dev, NULL, &calibrated));
    if (calibrated)
        ESP_LOGI(AHT1_TAG, " Sensor calibrated");
    else
        ESP_LOGW(AHT1_TAG, " Sensor not calibrated!");
    
    sensor_reading_t AHT_val;
    AHT_val.temperature[CURRENT] = 0;
    AHT_val.temperature[PREVIOUS] = 0;
    AHT_val.humidity[CURRENT] = 0;
    AHT_val.humidity[PREVIOUS] = 0;
    
    while (1)
    {
        esp_err_t res = aht_get_data(&dev, &AHT_val.temperature[CURRENT], &AHT_val.humidity[CURRENT]);
        if (res == ESP_OK) {
            ESP_LOGI(AHT1_TAG, " Temperature: %.1f°C, Humidity: %.2f%%", AHT_val.temperature[CURRENT], AHT_val.humidity[CURRENT]);
            if(AHT_val.temperature[CURRENT] != AHT_val.temperature[PREVIOUS] || AHT_val.humidity[CURRENT] != AHT_val.humidity[PREVIOUS]) {
                // Update Matter with new values
                updateMatterWithValues((u_int16_t)AHT_val.temperature[CURRENT] * 100, (u_int16_t)AHT_val.humidity[CURRENT] * 100);

//                if (xQueueSend(AHT1_sensor_queue, &AHT_val, pdMS_TO_TICKS(2)) != pdPASS) {        // For Screen update. Wait max 2ms if full
//                   ESP_LOGI("AHT1_sensor_queue", " q_full");
//                }
                AHT_val.temperature[PREVIOUS] = AHT_val.temperature[CURRENT];
                AHT_val.humidity[PREVIOUS] = AHT_val.humidity[CURRENT];
            }
        }
        else
            ESP_LOGE(AHT1_TAG, "Error reading data: %d (%s)", res, esp_err_to_name(res));

        vTaskDelay(CONFIG_AHT1_MEASURE_INTERVAL / portTICK_PERIOD_MS);
    }
}

/**
 * Update Matter values with temperature and humidity
 */
void updateMatterWithValues(u_int16_t temp, u_int16_t hum)
{
    // Update temperature values
    esp_matter_attr_val_t temperature_value;
    temperature_value = esp_matter_invalid(NULL);
    temperature_value.type = esp_matter_val_type_t::ESP_MATTER_VAL_TYPE_INT16;
    temperature_value.val.i16 = temp;
    esp_matter::attribute::update(temperature_sensor_endpoint_id, TemperatureMeasurement::Id, TemperatureMeasurement::Attributes::MeasuredValue::Id, &temperature_value);

    // Update humidity values
    esp_matter_attr_val_t humidity_value;
    humidity_value = esp_matter_invalid(NULL);
    humidity_value.type = esp_matter_val_type_t::ESP_MATTER_VAL_TYPE_UINT16;
    humidity_value.val.u16 = hum;
    esp_matter::attribute::update(humidity_sensor_endpoint_id, RelativeHumidityMeasurement::Id, RelativeHumidityMeasurement::Attributes::MeasuredValue::Id, &humidity_value);
}

// Example callback for temperature attribute change
esp_err_t temperature_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id, uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;

    if (type == POST_UPDATE)
    {
        ESP_LOGI(TAG, "Temperature attribute updated to %d", val->val.i16);
        // Add your logic here to use or display the temperature value
    }

    return err;
}

// // Example callback for humidity attribute change
esp_err_t humidity_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id, uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    if (type == POST_UPDATE)
    {
        ESP_LOGI(TAG, "Humidity attribute updated to %d", val->val.i16);
        // Add your logic here to use or display the humidity value
    }
    return ESP_OK;
}

esp_err_t sensor_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id, uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    // if (type == POST_UPDATE)
    if (type == PRE_UPDATE)
    {
        if (cluster_id == TemperatureMeasurement::Id)
        {
            ESP_LOGI(TAG, "Temperature attribute updated to %d", val->val.i16);
        }
        else if (cluster_id == RelativeHumidityMeasurement::Id)
        {
            ESP_LOGI(TAG, "Humidity attribute updated to %d", val->val.i16);
        }
    }
    return ESP_OK;
}

app_driver_handle_t app_driver_AHT_sensor_init(void)
{
    ESP_ERROR_CHECK(i2cdev_init());

    #ifdef CONFIG_USE_AHT0
    AHT0_sensor_queue = xQueueCreate(CONFIG_QUEUE_SIZE, sizeof(sensor_reading_t));      
    if (AHT0_sensor_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create AHT10 sensor queue");
        return NULL; // Return NULL if queue creation fails
    }

    ESP_LOGI(TAG, "Starting AHT10_0 Task");
    xTaskCreate(AHT10_0_TempHum_Task, "AHT10_0 Task", 4096, NULL, 1, NULL);
    #endif

    #ifdef CONFIG_USE_AHT1
    AHT1_sensor_queue = xQueueCreate(CONFIG_QUEUE_SIZE, sizeof(sensor_reading_t));
    if (AHT1_sensor_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create AHT10 sensor queue");
        return NULL; // Return NULL if queue creation fails
    }

    ESP_LOGI(TAG, "Starting AHT10_1 Task");
    xTaskCreate(AHT10_1_TempHum_Task, "AHT10_1 Task", 4096, NULL, 1, NULL);
    #endif
    // Return a placeholder value (you may need to adapt the return type)
    return (app_driver_handle_t)1;
}

// LEDC for front panel RGB buttons
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_timer.h"

void app_driver_config(void)
{
    app_driver_t config;
//        config.relay_gpio = (gpio_num_t)CONFIG_RELAY_GPIO;
        config.led_discovery_gpio = (gpio_num_t)CONFIG_IDENTITY_LED_PIN;
//        config.led_red_gpio = (gpio_num_t)CONFIG_BUTT0_R_LED_PIN;
//        config.led_green_gpio = (gpio_num_t)CONFIG_BUTT0_G_LED_PIN;
//        config.led_blue_gpio = (gpio_num_t)CONFIG_BUTT0_B_LED_PIN;
//        config.sensor_open_gpio = (gpio_num_t)CONFIG_BUTT0_B_LED_PIN;
//        config.sensor_mid_gpio = (gpio_num_t)CONFIG_BUTT0_B_LED_PIN;
//        config.sensor_closed_gpio = (gpio_num_t)CONFIG_BUTT0_B_LED_PIN;

            /* Initialize led */
    gpio_config_t io_conf = {};

    // --- Outputs: RELAY + LEDs ---
    io_conf.mode = GPIO_MODE_OUTPUT;    
    io_conf.pin_bit_mask =  
//                            (1ULL << config.relay_gpio) | 
                            (1ULL << config.led_discovery_gpio);
//                            (1ULL << config.led_red_gpio) |
//                            (1ULL << config.led_green_gpio) |
//                            (1ULL << config.led_blue_gpio);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

/*    
    // --- Inputs: Proximity Sensors ---
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask =  (1ULL << config.sensor_open_gpio) | 
                            (1ULL << config.sensor_mid_gpio) |
                            (1ULL << config.sensor_closed_gpio);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
*/
    // --- Initialize outputs to OFF state ---    
//    gpio_set_level(config.relay_gpio, 0);
    gpio_set_level(config.led_discovery_gpio, 1);
//    gpio_set_level(config.led_red_gpio, 1);
//    gpio_set_level(config.led_green_gpio, 1);
//    gpio_set_level(config.led_blue_gpio, 1);

    q_commisioning_status = xQueueCreate(CONFIG_QUEUE_SIZE, sizeof(int));
    if (q_commisioning_status == NULL) {
        ESP_LOGE("q_commisioning_status", "Fail");
    }

    q_I_led_pulse = xQueueCreate(CONFIG_QUEUE_SIZE, sizeof(led_pulse_t));
    if (q_I_led_pulse == NULL) {
        ESP_LOGE("q_I_led_pulse", "Fail");
    }

    ledc_init();
    ESP_LOGI(TAG, "Temp Humidity Driver initialized -Start all task");
    //xTaskCreate(identity_led_update, "identity_led_update", 8192, NULL, TASK_ID_LED_UPDATE_PRIORITY, NULL);
    //xTaskCreate(identityLedPulseTask, "identity_Led_pulse", 4096, NULL, TASK_ID_LED_PULSE_PRIORITY, NULL);
}

void app_driver_update_commissioning_status(int commm_status)
{
    ESP_LOGI(TAG, "Comm Status updated to %d", commm_status);
    if (xQueueSend(q_commisioning_status, &commm_status, pdMS_TO_TICKS(5)) != pdPASS) {        // Send update to Led Task
        ESP_LOGI("Comm_sta", "q_full");
    }
}

// Initialize the LEDC timer and channels for multiple LEDs
void ledc_init(void)
{
    static const char *LEDC_INIT = "LEDC_Init";
    // Prepare and set configuration of timers
   
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,          // Timer mode
        .duty_resolution = LEDC_DUTY_RES, // Resolution of PWM duty
        .timer_num = LEDC_TIMER,          // Timer index
        .freq_hz = 5000,                  // Frequency of PWM signal
        .clk_cfg = LEDC_AUTO_CLK          // Auto select the source clock
    };

    ledc_timer_config(&ledc_timer);

    esp_err_t err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) {
        ESP_LOGE(LEDC_INIT, "ledc_timer_config failed: %s", esp_err_to_name(err));
    }

    // Initialize all LEDC channels
    for (int i = 0; i < led_gpio_count; i++) {
        ledc_channel_config_t ledc_channel = {
            .gpio_num   = led_gpio_pins[i],
            .speed_mode = LEDC_MODE,
            .channel    = ledc_channels[i],
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = LEDC_TIMER,
            .duty       = 0,              // Initial duty cycle
            .hpoint     = 0,
            .flags      = {0}
        };
        esp_err_t err = ledc_channel_config(&ledc_channel);
        if (err != ESP_OK) {
            ESP_LOGE(LEDC_INIT, "ledc_timer_config [i:%d] failed: %s", i, esp_err_to_name(err));
            return;
        } else {
            ESP_LOGI(LEDC_INIT, "ledc_timer_config [i:%d] ok - pin:%d spd:%d chan:%d, intr:%d, tmr:%d", 
                                i, ledc_channel.gpio_num, ledc_channel.speed_mode, ledc_channel.channel, ledc_channel.intr_type, ledc_channel.timer_sel);
        }
    }

    // Initialize fade service
    ledc_fade_func_install(0);
    clear_all_RGB_leds();
}

static void identity_led_update(void *pvParameters)
{
    static const char *RGB_TAG = "identityLedUpdate";
//    ESP_LOGI(RGB_TAG, "Starting");
    // Initialize LEDC
    int delay = 200, inCommissioning[2] = {-1,-1};
    led_pulse_t s_outILedPulse = {
        .ledChan = LEDC_CHANNEL_0,
        .speed = CONFIG_LEDC_FADE_TIME_SLOW,
        .run = false,
        .state = false,
        .changed = false
    };

    while (1)
    {
        xQueueReceive(q_commisioning_status, &inCommissioning[CURRENT], pdMS_TO_TICKS(10));

        if (inCommissioning[PREVIOUS] != inCommissioning[CURRENT]) {      // if commissioning state changed - Impacts Identity LED ONLY
            switch(inCommissioning[CURRENT]) {

                case COMMISSSIONING_STARTED:
                    ESP_LOGI(RGB_TAG, "c-start");
                    s_outILedPulse.ledChan = LEDC_CHANNEL_3;
                    s_outILedPulse.speed = CONFIG_LEDC_FADE_TIME_SLOW;
                    s_outILedPulse.run = true;
                    s_outILedPulse.state = true;
                    s_outILedPulse.changed = true;   // Trigger change
                    break;

                case COMMISSSIONING_FAILURE:
                    ESP_LOGI(RGB_TAG, "c-fail");
                    s_outILedPulse.ledChan = LEDC_CHANNEL_3;
                    s_outILedPulse.speed = CONFIG_LEDC_FADE_TIME_FAST;
                    s_outILedPulse.run = true;
                    s_outILedPulse.state = true;
                    s_outILedPulse.changed = true;   // Trigger change
                    break;

                case COMMISSSIONING_SUCCESS:
                    ESP_LOGI(RGB_TAG, "c-success");
                    s_outILedPulse.ledChan = LEDC_CHANNEL_3;
                    s_outILedPulse.speed = CONFIG_LEDC_FADE_TIME_FAST;
                    s_outILedPulse.run = false;
                    s_outILedPulse.state = true;
                    s_outILedPulse.changed = true;   // Trigger change
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_3, LEDC_DUTY); // Identity 100% duty (LED on)
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_3);
                    break;

                case COMMISSSIONING_PRE_SUCCESS:
                    ESP_LOGI(RGB_TAG, "c-pre success");
                    s_outILedPulse.ledChan = LEDC_CHANNEL_3;
                    s_outILedPulse.speed = CONFIG_LEDC_FADE_TIME_FAST;
                    s_outILedPulse.run = false;
                    s_outILedPulse.state = true;
                    s_outILedPulse.changed = true;   // Trigger change
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_3, LEDC_DUTY); // Identity 100% duty (LED on)
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_3);
                    break;

                default:
                    ESP_LOGI(RGB_TAG, "c-Default");
                    s_outILedPulse.ledChan = LEDC_CHANNEL_3;
                    s_outILedPulse.speed = CONFIG_LEDC_FADE_TIME_FAST;
                    s_outILedPulse.run = true;
                    s_outILedPulse.state = true;
                    s_outILedPulse.changed = true;   // Trigger change
                    ESP_LOGI(RGB_TAG, "Identity Led Default");
                    break;
            }
            inCommissioning[PREVIOUS] = inCommissioning[CURRENT];
            if(s_outILedPulse.changed) {
                ESP_LOGI(RGB_TAG, "s_outILedPulse changed");
                if (xQueueSend(q_I_led_pulse, &s_outILedPulse, pdMS_TO_TICKS(5)) != pdPASS) {        // Send update to Led Task
                    ESP_LOGI(RGB_TAG, "OLPq_full");
                } else {
                    ESP_LOGI(RGB_TAG, "q_I_led_pulse send success");
                }
                s_outILedPulse.changed = true;   // Trigger change
            } else {
                ESP_LOGI(RGB_TAG, "s_outILedPulse NOT changed");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(delay));       // Wait for the fade-out to complete
    }
}
// End Identity Led Update

static void identityLedPulseTask(void *pvParameters)
{
    static const char *ILP_TAG = "I_LedPulse";
    ESP_LOGI(ILP_TAG, "Starting");
    bool ledOn = 0;
    int delay = 100;
    led_pulse_t ledPulseIn = {
        .ledChan = LEDC_CHANNEL_0,
        .speed = CONFIG_LEDC_FADE_TIME_SLOW,
        .run = false,
        .state = false,
        .changed = 0
    };

    while(1) {
//        ESP_LOGI(ILP_TAG, "RUNNING!!!");
        xQueueReceive(q_I_led_pulse, &ledPulseIn, pdMS_TO_TICKS(10));      // 5ms
        if(ledPulseIn.run) {
            if(ledOn) {     // Pulse
                ledc_set_fade_time_and_start(LEDC_MODE, ledPulseIn.ledChan, LEDC_DUTY, ledPulseIn.speed, LEDC_FADE_NO_WAIT);  
            } else {
                ledc_set_fade_time_and_start(LEDC_MODE, ledPulseIn.ledChan, 0, ledPulseIn.speed, LEDC_FADE_NO_WAIT);
            }
            ledOn = !ledOn; 
            // Wait for the fade-in to complete
        }
        else if(ledPulseIn.changed) {
            if(ledPulseIn.state) {
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_3, LEDC_DUTY); // Identity LED on
                ESP_LOGI(ILP_TAG, "ID Led FULL ON");
            } else {
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_3, 0); // Identity LED off
                ESP_LOGI(ILP_TAG, "ID Led FULL OFF");
            }
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_3);
            ledPulseIn.changed = 0;
        }
        delay = ledPulseIn.speed / portTICK_PERIOD_MS;  
        vTaskDelay(pdMS_TO_TICKS(delay));
    }
}

void clear_all_RGB_leds(void) {
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, LEDC_DUTY); // B0 R 100% duty (LED off)
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, LEDC_DUTY); // B0 G 100% duty (LED off)
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, LEDC_DUTY); // B0 B 100% duty (LED off)
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_2);
}

