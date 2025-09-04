/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#pragma once

#include <esp_err.h>
#include <esp_matter.h>
#include "driver/gpio.h"
#include <app/server/OnboardingCodesUtil.h>

#include <credentials/examples/DeviceAttestationCredsExample.h>
/* FOR DELETION #######
#include <app/clusters/window-covering-server/window-covering-server.h>
#include <app/clusters/window-covering-server/window-covering-delegate.h>
*/

typedef enum {
    SENSOR_ACTIVE,
    SENSOR_INACTIVE,
    SENSOR_UNKNOWN
} sensor_status_t;

typedef enum {
    TRIG_NONE = 0,
    TRIG_HANDLER,
    TRIG_MOVEMENT,
    TRIG_POSITION,
    TRIG_MOV_POS,
    TRIG_UNKNOWN
} trig_vals_t;

typedef enum {
    COMMISSSIONING_STARTED,
    COMMISSSIONING_STOPPED,
    COMMISSSIONING_SUCCESS,
    COMMISSSIONING_PRE_SUCCESS,
    COMMISSSIONING_FAILURE
} commisioning_status_t;

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include "esp_openthread_types.h"
#endif

using namespace chip;
using namespace chip::app::Clusters;
//using namespace chip::app::Clusters::WindowCovering;
using namespace chip::app::Clusters::TemperatureMeasurement;
using namespace chip::app::Clusters::RelativeHumidityMeasurement;

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;

using namespace esp_matter::cluster;

// Cluster for Temp & Humidity
const uint32_t TEMPERATURE_SENSE_ATTRIBUTE_ID = TemperatureMeasurement::Attributes::MeasuredValue::Id;
const uint32_t HUMIDITY_SENSE_ATTRIBUTE_ID = RelativeHumidityMeasurement::Attributes::MeasuredValue::Id;

// DHT22 Values



typedef void *app_driver_handle_t;
// Cluster and attribute ID used by Matter WindowCovering Device
/* FOR DELETION ********************
constexpr uint16_t ATTRIBUTE_ID_NODE_LABEL = 0x0005; // NodeLabel attribute in Basic Information cluster
constexpr uint32_t BASIC_INFORMATION_CLUSTER_ID = 0x00000028;

const uint32_t WINDOW_COVERING_CLUSTER_ID = WindowCovering::Id;
const uint32_t WINDOW_COVERING_ATTRIBUTE_PER100THS_ID_TARGET = WindowCovering::Attributes::TargetPositionLiftPercent100ths::Id;
const uint32_t WINDOW_COVERING_ATTRIBUTE_PER100THS_ID_CURRENT = WindowCovering::Attributes::CurrentPositionLiftPercent100ths::Id;
//const uint32_t WINDOW_COVERING_ATTRIBUTE_ID_TARGET = WindowCovering::Attributes::TargetPositionLiftPercent100ths::Id;
const uint32_t WINDOW_COVERING_ATTRIBUTE_ID_CURRENT = WindowCovering::Attributes::CurrentPositionLiftPercentage::Id;
const uint32_t WINDOW_COVERING_ATTRIBUTE_ID_OPERATE = WindowCovering::Attributes::OperationalStatus::Id;
const uint32_t WINDOW_COVERING_ATTRIBUTE_ID_ENDTYPE = WindowCovering::Attributes::EndProductType::Id;
const uint32_t WINDOW_COVERING_ATTRIBUTE_ID_TYPE = WindowCovering::Attributes::Type::Id;
const uint32_t WINDOW_COVERING_ATTRIBUTE_ID_CONFIG = WindowCovering::Attributes::ConfigStatus::Id;
const uint32_t WINDOW_COVERING_ATTRIBUTE_ID_MODE = WindowCovering::Attributes::Mode::Id;
const uint32_t WINDOW_COVERING_ATTRIBUTE_ID_SAFETY = WindowCovering::Attributes::SafetyStatus::Id;

// Cluster for CalibrationSwitch
const uint32_t ONOFF_CLUSTER_ID = OnOff::Id;
const uint32_t ONOFF_ATTRIBUTE_ID = OnOff::Attributes::OnOff::Id;


const int MAX_GARAGE_POSITION = 10000;
const int MIN_GARAGE_POSITION = 0;

// Custom attribute ID
constexpr uint32_t kAttrGarageFaultStatus = 0xF001;

// Possible Error state values
typedef enum {
    NO_FAULT        = 0,
    SENSOR_FAULT    = 1,
    POS_UNKNOWN     = 2
} GarageFaultStatus;

#define WINDOW_COVERING_FEATURES (1 << 0)  // Just Lift, no PositionAware

extern uint16_t garage_endpoint_id;
extern uint16_t switch_endpoint_id_1;
extern uint16_t err_state_endpoint_id;
extern attribute_t *attribute_ref_target;
extern attribute_t *attribute_ref_1;

typedef void *app_driver_handle_t;

extern QueueHandle_t q_trigger_pos_attr_update  ;

app_driver_handle_t app_driver_button0_init(void);
app_driver_handle_t app_driver_sensor_closed_init(void);
app_driver_handle_t app_driver_sensor_mid_init(void);
app_driver_handle_t app_driver_sensor_open_init(void);

typedef enum {
    GARAGE_STATE_IDLE,
    GARAGE_STATE_OPENING,
    GARAGE_STATE_CLOSING,
    GARAGE_STATE_OPENED,
    GARAGE_STATE_CLOSED,
    GARAGE_STATE_TIMEOUT,
    GARAGE_STATE_ERROR
} garage_state_t;

void app_driver_update_movement(bool moving_up, bool moving_down);
void setCurrentDirection(garage_state_t movement);
void set_operational_state(chip::EndpointId endpoint, chip::app::Clusters::WindowCovering::OperationalState newState);

void sendCurrentPosition(int pos);
void set_window_covering_position(esp_matter_attr_val_t *current);

void updateErrorState(GarageFaultStatus err_state);
void set_window_covering_err_state(esp_matter_attr_val_t *err_state);

void set_window_covering_unknown_position(void);
*/
app_driver_handle_t app_driver_AHT_sensor_init(void);
void app_driver_config(void);

/** Initialize the garage driver
 *
 * This initializes the garage driver associated with the selected board.
 *
 * @return Handle on success.
 * @return NULL in case of failure.
 */
void app_driver_garage_init(void);

/** Initialize the button driver
 *
 * This initializes the button driver associated with the selected board.
 *
 * @return Handle on success.
 * @return NULL in case of failure.
 */
void app_driver_button_init();

/** Driver Update
 *
 * This API should be called to update the driver for the attribute being updated.
 * This is usually called from the common `app_attribute_update_cb()`.
 *
 * @param[in] endpoint_id Endpoint ID of the attribute.
 * @param[in] cluster_id Cluster ID of the attribute.
 * @param[in] attribute_id Attribute ID of the attribute.
 * @param[in] val Pointer to `esp_matter_attr_val_t`. Use appropriate elements as per the value type.
 *
 * @return ESP_OK on success.
 * @return error in case of failure.
 */
esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val);

int16_t app_driver_read_temperature(uint16_t endpoint_id);
uint16_t app_driver_read_humidity(uint16_t endpoint_id);
esp_err_t sensor_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id, uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data);

/** Set defaults for Garage driver
 *
 * Set the attribute drivers to their default values from the created data model.
 *
 * @param[in] endpoint_id Endpoint ID of the driver.
 *
 * @return ESP_OK on success.
 * @return error in case of failure.
 */

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#define ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG()                                           \
    {                                                                                   \
        .radio_mode = RADIO_MODE_NATIVE,                                                \
    }

#define ESP_OPENTHREAD_DEFAULT_HOST_CONFIG()                                            \
    {                                                                                   \
        .host_connection_mode = HOST_CONNECTION_MODE_NONE,                              \
    }

#define ESP_OPENTHREAD_DEFAULT_PORT_CONFIG()                                            \
    {                                                                                   \
        .storage_partition_name = "nvs", .netif_queue_size = 10, .task_queue_size = 10, \
    }
#endif

