// app_driver.h

#pragma once

#include "driver/gpio.h"
// LEDC for front panel RGB buttons
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_timer.h"
//#include <app/clusters/window-covering-server/window-covering-server.h>
//#include <app/clusters/window-covering-server/window-covering-delegate.h>
#include <app/util/attribute-storage.h>
#include <app-common/zap-generated/attributes/Accessors.h>

#define TASK_ID_LED_UPDATE_PRIORITY        2
#define TASK_ID_LED_PULSE_PRIORITY         2

typedef struct {
    ledc_channel_t ledChan;
    int speed;
    bool run;
    bool state;
    bool changed;
} led_pulse_t;

typedef struct {
//    gpio_num_t relay_gpio;
    gpio_num_t led_discovery_gpio;
//    gpio_num_t led_red_gpio;
//    gpio_num_t led_green_gpio;
//   gpio_num_t led_blue_gpio;
} app_driver_t;

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (8191)            // Max duty (2^13 - 1)

// Array of GPIO pins for the LEDs
static const int led_gpio_pins[] = {    CONFIG_BUTT0_R_LED_PIN, CONFIG_BUTT0_G_LED_PIN, CONFIG_BUTT0_B_LED_PIN,
                                        CONFIG_IDENTITY_LED_PIN};
const int led_gpio_count = sizeof(led_gpio_pins) / sizeof(led_gpio_pins[0]);
// Array of LEDC channels corresponding to the LEDs
static const ledc_channel_t ledc_channels[led_gpio_count] = { LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3};

// Task declarations
static void identity_led_update(void *pvParameters);
//static TaskHandle_t s_RGB_pulse_led_task_handle = NULL;
static void identityLedPulseTask(void *pvParameters);
//static TaskHandle_t s_l_pulse_led_task_handle = NULL;
void AHT10_0_TempHum_Task(void *pvParameters);
void AHT10_1_TempHum_Task(void *pvParameters);

// Function declarations
void ledc_init(void);
void app_driver_update_commissioning_status(int commm_status);
void clear_all_RGB_leds(void);
void updateMatterWithValues(u_int16_t temp, u_int16_t hum);
int read_input(gpio_num_t gpio);
