/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_matter.h>
#include <esp_matter_core.h>
#include "esp_matter_endpoint.h"
#include <esp_matter_console.h>
#include <esp_matter_ota.h>
#include <common_macros.h>

#include <app/CommandHandler.h>
#include <app_priv.h>
#include <app_driver.h>
#include "driver/gpio.h"
#include <app_reset.h>

#include <lib/core/CHIPCore.h>  // For chip::CharSpan

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>

#ifdef CONFIG_ENABLE_SET_CERT_DECLARATION_API
#include <esp_matter_providers.h>
#include <lib/support/Span.h>
#ifdef CONFIG_SEC_CERT_DAC_PROVIDER
#include <platform/ESP32/ESP32SecureCertDACProvider.h>
#elif defined(CONFIG_FACTORY_PARTITION_DAC_PROVIDER)
#include <platform/ESP32/ESP32FactoryDataProvider.h>
#endif  
using namespace chip::DeviceLayer;
#endif

// For Oled
//#include "esp_lcd_panel_io.h"
//#include "esp_lcd_panel_ops.h"
//#include "driver/i2c_master.h"
//#include "esp_lvgl_port.h"
//#include "lvgl.h"
//#include "esp_lcd_panel_vendor.h"
#include "i2c_oled_display.h"
//#include "SSD1306_display.h"

static const char *TAG = "app_main";

// -- Variables
int dir = 0, count = 0, lastCount = 0, maxCount, newCount = 0, newMaxCount = 0, lastSendPercentage = 0, lastTargetPosition = 0, bufferedPercentage = 0, filteredPercentage = 0, newPercentage = 0;
const int filterDuration = 1500;
float percentage = 0;
bool maxCountReloaded = false, newPercentageReceived = false, bufferedValueSaved = false, calButton = false, remote = false, posCertain = false, calMode = false, calUp = false, calDown = false, lastSendPercentageInit = false, newCountInit = false, maxCountInit = false, counted = false, posChange = false,posRunUp = false, posRunDown = false;
unsigned long lastUpdateTime = 0;

//uint16_t button_endpoint_id = 0;
uint16_t temperature_sensor_endpoint_id = 0;
uint16_t humidity_sensor_endpoint_id = 0;
uint16_t err_state_endpoint_id = 0;
//attribute_t *attribute_ref_target;
//attribute_t *attribute_ref_1;
//attribute_t *attribute_err_state;

//const char *label_str = "Garage Sensor Error";

// === Window Covering Attribute List ===
//uint32_t err_state_attribute_id = 0xF001;
//uint16_t err_state_default_value = NO_FAULT;

constexpr auto k_timeout_seconds = 300;

#ifdef CONFIG_ENABLE_SET_CERT_DECLARATION_API
extern const uint8_t cd_start[] asm("_binary_certification_declaration_der_start");
extern const uint8_t cd_end[] asm("_binary_certification_declaration_der_end");

const chip::ByteSpan cdSpan(cd_start, static_cast<size_t>(cd_end - cd_start));
#endif // CONFIG_ENABLE_SET_CERT_DECLARATION_API

#if CONFIG_ENABLE_ENCRYPTED_OTA
extern const char decryption_key_start[] asm("_binary_esp_image_encryption_key_pem_start");
extern const char decryption_key_end[] asm("_binary_esp_image_encryption_key_pem_end");

static const char *s_decryption_key = decryption_key_start;
static const uint16_t s_decryption_key_len = decryption_key_end - decryption_key_start;
#endif // CONFIG_ENABLE_ENCRYPTED_OTA

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    static const char *AE_TAG = "app_event_cb";
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(AE_TAG, "Interface IP Address changed");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(AE_TAG, "Commissioning complete");
        app_driver_update_commissioning_status(COMMISSSIONING_SUCCESS);
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(AE_TAG, "Commissioning failed, fail safe timer expired");
        app_driver_update_commissioning_status(COMMISSSIONING_FAILURE);
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(AE_TAG, "Commissioning session started");
        app_driver_update_commissioning_status(COMMISSSIONING_STARTED);
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(AE_TAG, "Commissioning session stopped");
        app_driver_update_commissioning_status(COMMISSSIONING_STOPPED);
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(AE_TAG, "Commissioning window opened");
        app_driver_update_commissioning_status(COMMISSSIONING_STARTED);
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(AE_TAG, "Commissioning window closed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        {
            ESP_LOGI(AE_TAG, "Fabric removed successfully");
            if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0)
            {
                chip::CommissioningWindowManager & commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
                constexpr auto kTimeoutSeconds = chip::System::Clock::Seconds16(k_timeout_seconds);
                if (!commissionMgr.IsCommissioningWindowOpen())
                {
                    /* After removing last fabric, this example does not remove the Wi-Fi credentials
                     * and still has IP connectivity so, only advertising on DNS-SD.
                     */
                    CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(kTimeoutSeconds,
                                                    chip::CommissioningWindowAdvertisement::kDnssdOnly);
                    if (err != CHIP_NO_ERROR)
                    {
                        ESP_LOGE(AE_TAG, "Failed to open commissioning window, err:%" CHIP_ERROR_FORMAT, err.Format());
                    }
                }
            }
        break;
        }

    case chip::DeviceLayer::DeviceEventType::kFabricWillBeRemoved:
        ESP_LOGI(AE_TAG, "Fabric will be removed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricUpdated:
        ESP_LOGI(AE_TAG, "Fabric is updated");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricCommitted:
        ESP_LOGI(AE_TAG, "Fabric is committed");
        break;

    case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
        ESP_LOGI(AE_TAG, "BLE deinitialized and memory reclaimed");
        break;

    default:
        break;
    }
}

// This callback is invoked when clients interact with the Identify Cluster.
// In the callback implementation, an endpoint can identify itself. (e.g., by flashing an LED or light).
static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
    uint8_t effect_variant, void *priv_data)
{
ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
return ESP_OK;
}

// This callback is called for every attribute update. The callback implementation shall
// handle the desired attributes and return an appropriate error code. If the attribute
// is not of your interest, please do not return an error code and strictly return ESP_OK.
static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;

    ESP_LOGI(TAG, " ========  app_attribute_update_cb  ======== \n");

    // if (type == POST_UPDATE) // or READ?
    // {
    // }

    return err;
}

extern "C" void app_main()
{
    esp_err_t err = ESP_OK;

    /* Initialize the ESP NVS layer */
    nvs_flash_init();

    app_driver_config();     // Start all tasks AFTER Matter begins.

    // Creation of custom tasks - Rob Recinella 5 Sep 2024
    //xTaskCreate(actionMQTTIn, "MQTT In", 8196, NULL, 1, NULL);
    //xTaskCreate(changeDetection, "Change Detect", 8196, NULL, 1, NULL);
   
    // Initialize driver
    //app_driver_garage_init();
    //app_driver_handle_t garage_handle = app_driver_garage_init();
    //app_driver_handle_t garage_handle = app_driver_garage_init();
//    app_driver_handle_t button0_handle = app_driver_button0_init();
//    app_reset_button_register(button0_handle);
    /* Initialize driver */
    app_driver_handle_t temperature_sensor_handle = app_driver_AHT_sensor_init();
    // app_driver_handle_t humidity_sensor_handle = app_driver_DHT_sensor_init();
   
    //Create a Matter node and add the mandatory Root Node device type on endpoint 0
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));
         {
        // Create endpoint with temperature measurement
        temperature_sensor::config_t temperature_sensor_config;
        temperature_sensor_config.temperature_measurement.measured_value = (uint16_t)CONFIG_AHT_DEFAULT_TEMPERATURE_VALUE;
        endpoint_t *temperature_sensor_endpoint = temperature_sensor::create(node, &temperature_sensor_config, ENDPOINT_FLAG_NONE, temperature_sensor_handle);

        if (temperature_sensor_endpoint)
        {
            ESP_LOGI(TAG, "Created temperature endpoint");
            temperature_sensor_endpoint_id = endpoint::get_id(temperature_sensor_endpoint);
            ESP_LOGI(TAG, "Temperature endpoint created with endpoint_id %d", temperature_sensor_endpoint_id);

            // // Register temperature attribute callback
            // esp_matter::attribute::set_callback(temperature_attribute_update_cb);
            esp_matter::attribute::set_callback(sensor_attribute_update_cb);
        }

        // Create endpoint with humidity measurement
        humidity_sensor::config_t humidity_sensor_config;
        humidity_sensor_config.relative_humidity_measurement.measured_value = (uint16_t)CONFIG_AHT_DEFAULT_HUMIDITY_VALUE;
        endpoint_t *humidity_sensor_endpoint = humidity_sensor::create(node, &humidity_sensor_config, ENDPOINT_FLAG_NONE, NULL);

        if (humidity_sensor_endpoint)
        {
            ESP_LOGI(TAG, "Created humidity endpoint");

            humidity_sensor_endpoint_id = endpoint::get_id(humidity_sensor_endpoint);
            ESP_LOGI(TAG, "Humidity endpoint created with endpoint_id %d", humidity_sensor_endpoint_id);

            // // Register humidity attribute callback
            // esp_matter::attribute::set_callback(humidity_attribute_update_cb);
            esp_matter::attribute::set_callback(sensor_attribute_update_cb);
        }

        /* These node and endpoint handles can be used to create/add other endpoints and clusters. */
        if (!node || !temperature_sensor_endpoint || !humidity_sensor_endpoint)
        {
            ESP_LOGE(TAG, "Matter node creation failed");
        }

    }

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD && CHIP_DEVICE_CONFIG_ENABLE_WIFI_STATION
    // Enable secondary network interface
    secondary_network_interface::config_t secondary_network_interface_config;
    endpoint = endpoint::secondary_network_interface::create(node, &secondary_network_interface_config, ENDPOINT_FLAG_NONE, nullptr);
    ABORT_APP_ON_FAILURE(endpoint != nullptr, ESP_LOGE(TAG, "Failed to create secondary network interface endpoint"));
#endif


#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

#ifdef CONFIG_ENABLE_SET_CERT_DECLARATION_API
    auto * dac_provider = get_dac_provider();
#ifdef CONFIG_SEC_CERT_DAC_PROVIDER
    static_cast<ESP32SecureCertDACProvider *>(dac_provider)->SetCertificationDeclaration(cdSpan);
#elif defined(CONFIG_FACTORY_PARTITION_DAC_PROVIDER)
    static_cast<ESP32FactoryDataProvider *>(dac_provider)->SetCertificationDeclaration(cdSpan);
#endif
#endif // CONFIG_ENABLE_SET_CERT_DECLARATION_API

    // Matter start
    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));

#if CONFIG_ENABLE_ENCRYPTED_OTA
    err = esp_matter_ota_requestor_encrypted_init(s_decryption_key, s_decryption_key_len);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to initialized the encrypted OTA, err: %d", err));
#endif // CONFIG_ENABLE_ENCRYPTED_OTA

#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
    esp_matter::console::factoryreset_register_commands();
#if CONFIG_OPENTHREAD_CLI
    esp_matter::console::otcli_register_commands();
#endif
    esp_matter::console::init();
#endif

start_lvgl_task();
//init_buttons(); // <-- Add this
ESP_LOGI(TAG, "Leaving Main");

}
