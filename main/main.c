#include "esp_check.h"
#include "esp_err.h"
#include "string.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "main.h"
#include "esp_ieee802154.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include <nvs_flash.h>

#define TOTAL_BUTTONS  5
#define TOTAL_LIGHTS 5
#define RESET_GPIO_PIN CONFIG_DALIZB_RESET_PIN
#define RESET_GPIO_PIN2 CONFIG_DALIZB_RESET_PIN2

#include "dali/dali.h"
#include "driver/gptimer.h"
#include "esp_timer.h"
#include "esp_pm.h"

#include "driver/ledc.h"

#include "scenes.h"
#include "ota.h"
#include "wifi.h"

static const char *TAG = "ZB_DALI";

// Design Light
static char modelid[] = {22, 'D', 'e', 's', 'i', 'g', 'n', ' ', 'L', 'i', 'g', 'h', 't', ' ', 'F', 'i', 'v', 'e', ' ', 'F', 'o', 'l', 'd'};
static char manufname[] = {9, 'E', 's', 'p', 'r', 'e', 's', 's', 'i', 'f'};

// Internal button state
static bool zigbee_initialized = false;
static light_state last_light_state[TOTAL_LIGHTS];
static int32_t last_effect = -1, effect = 0;

static dali *dali_ = NULL;

static bool factory_reset = false;
static bool driver_is_inited = false;
static int dali_bus_timeouting = 0;

static TaskHandle_t xZigbeeTask = NULL;
static TaskHandle_t xDaliTask = NULL;
static SemaphoreHandle_t lightMutex;

static nvs_handle_t nvs_h;

#define LED_MAX_DUTY 4000
#define LED_DUTY_DIVIDER (LED_MAX_DUTY/256)
ledc_channel_config_t ledc_channel[TOTAL_LIGHTS] = {
    {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 0,
        .gpio_num   = CONFIG_DALIZB_LED_1_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_1,
        .flags.output_invert = 0
    },
    {
        .channel    = LEDC_CHANNEL_1,
        .duty       = 0,
        .gpio_num   = CONFIG_DALIZB_LED_2_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_1,
        .flags.output_invert = 0
    },
    {
        .channel    = LEDC_CHANNEL_2,
        .duty       = 0,
        .gpio_num   = CONFIG_DALIZB_LED_3_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_1,
        .flags.output_invert = 0
    },
    {
        .channel    = LEDC_CHANNEL_3,
        .duty       = 0,
        .gpio_num   = CONFIG_DALIZB_LED_4_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_1,
        .flags.output_invert = 0
    },
    {
        .channel    = LEDC_CHANNEL_4,
        .duty       = 0,
        .gpio_num   = CONFIG_DALIZB_LED_5_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_1,
        .flags.output_invert = 0
    },
};

static const float gamma_correction_lut[101] = {
    0.000000, 0.000006, 0.000038, 0.000110, 0.000232, 0.000414, 0.000666, 0.000994, 0.001406, 0.001910,
    0.002512, 0.003218, 0.004035, 0.004969, 0.006025, 0.007208, 0.008525, 0.009981, 0.011580, 0.013328,
    0.015229, 0.017289, 0.019512, 0.021902, 0.024465, 0.027205, 0.030125, 0.033231, 0.036527, 0.040016,
    0.043703, 0.047593, 0.051688, 0.055993, 0.060513, 0.065249, 0.070208, 0.075392, 0.080805, 0.086451,
    0.092333, 0.098455, 0.104821, 0.111434, 0.118298, 0.125416, 0.132792, 0.140428, 0.148329, 0.156498,
    0.164938, 0.173653, 0.182645, 0.191919, 0.201476, 0.211321, 0.221457, 0.231886, 0.242612, 0.253639,
    0.264968, 0.276603, 0.288548, 0.300805, 0.313378, 0.326268, 0.339480, 0.353016, 0.366879, 0.381073,
    0.395599, 0.410461, 0.425662, 0.441204, 0.457091, 0.473325, 0.489909, 0.506846, 0.524138, 0.541789,
    0.559801, 0.578177, 0.596920, 0.616032, 0.635515, 0.655374, 0.675610, 0.696226, 0.717224, 0.738608,
    0.760380, 0.782542, 0.805097, 0.828048, 0.851398, 0.875148, 0.899301, 0.923861, 0.948829, 0.974208,
    1.000000,
};

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK,, TAG, "Failed to start Zigbee bdb commissioning");
}

void init_dali_bus()
{
    ESP_LOGI(TAG, "Initializing DALI bus...");
    dali_ = dali_begin(CONFIG_DALIZB_TX_PIN, CONFIG_DALIZB_RX_PIN, true);
    assert(dali_ != NULL);
}

void init_led_emulator()
{
    ESP_LOGI(TAG, "Configuring LED control timer");
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 4000,                      // frequency of PWM signal
        .speed_mode = LEDC_LOW_SPEED_MODE,           // timer mode
        .timer_num = LEDC_TIMER_1,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };

    ledc_timer_config(&ledc_timer);

    // Set LED Controller with previously prepared configuration
    for (int i = 0; i < TOTAL_LIGHTS; i++) {
        ESP_LOGI(TAG, "Configuring LED channel: %d", i);
        ledc_channel_config(&ledc_channel[i]);
    }

    for (int i = 0; i < TOTAL_LIGHTS; i++) {
        ledc_set_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel, 0);
        ledc_update_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel);
    }
}

static esp_err_t deferred_driver_init(void)
{
    if (!driver_is_inited) {
#ifndef CONFIG_DALIZB_LED_EMULATOR
        init_dali_bus();
        driver_is_inited = true;
#else
        init_led_emulator();
        driver_is_inited = true;
#endif
    }
    return driver_is_inited ? ESP_OK : ESP_FAIL;
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    esp_zb_zdo_signal_leave_params_t *leave_params = NULL;
    esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = NULL;

    switch (sig_type) {
    case ESP_ZB_BDB_SIGNAL_FINDING_AND_BINDING_TARGET_FINISHED:
        ESP_LOGI(TAG, "Zigbee: ESP_ZB_BDB_SIGNAL_FINDING_AND_BINDING_TARGET_FINISHED");
        break;
    case ESP_ZB_BDB_SIGNAL_FINDING_AND_BINDING_INITIATOR_FINISHED:
        ESP_LOGI(TAG, "ESP_ZB_BDB_SIGNAL_FINDING_AND_BINDING_INITIATOR_FINISHED");
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING_CANCELLED:
        ESP_LOGI(TAG, "ESP_ZB_BDB_SIGNAL_STEERING_CANCELLED");
        break;
    case ESP_ZB_BDB_SIGNAL_FORMATION_CANCELLED:
        ESP_LOGI(TAG, "ESP_ZB_BDB_SIGNAL_FORMATION_CANCELLED");
        break;
    case ESP_ZB_BDB_SIGNAL_TC_REJOIN_DONE:
        ESP_LOGI(TAG, "ESP_ZB_BDB_SIGNAL_TC_REJOIN_DONE");
        break;
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        leave_params = (esp_zb_zdo_signal_leave_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        if (leave_params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET) {
            ESP_LOGI(TAG, "Reset device");
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGI(TAG, "New device commissioned or rejoined (short: 0x%04hx)", dev_annce_params->device_short_addr);
        break;
    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (err_status == ESP_OK) {
            if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p)) {
                ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
            } else {
                ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
            }
        }
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in%s factory-reset mode", factory_reset ? "" : " non");
            if (factory_reset)
            {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
            else
            {
                ESP_LOGI(TAG, "Device rebooted");
                zigbee_initialized = true;
            }
        } else {
            ESP_LOGW(TAG, "%s failed with status: %s, retrying", esp_zb_zdo_signal_to_string(sig_type),
                     esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            if (xDaliTask != NULL)
                xTaskNotifyGive(xDaliTask);
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static esp_err_t zb_attribute_reporting_handler(const esp_zb_zcl_report_attr_message_t *message)
{
    ESP_LOGI(TAG, "zb_attribute_reporting_handler");
    return ESP_OK;
}

static esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;
    while (variable) {
        ESP_LOGI(TAG, "Read attribute response: status(%d), cluster(0x%x), attribute(0x%x), type(0x%x), value(%d)", variable->status,
                 message->info.cluster, variable->attribute.id, variable->attribute.data.type,
                 variable->attribute.data.value ? * (uint8_t *)variable->attribute.data.value : 0);
        variable = variable->next;
    }

    return ESP_OK;
}

static esp_err_t zb_configure_report_resp_handler(const esp_zb_zcl_cmd_config_report_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    esp_zb_zcl_config_report_resp_variable_t *variable = message->variables;
    while (variable) {
        ESP_LOGI(TAG, "Configure report response: status(%d), cluster(0x%x), attribute(0x%x)", message->info.status, message->info.cluster,
                 variable->attribute_id);
        variable = variable->next;
    }

    return ESP_OK;
}

// This is just an informational function
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    ESP_LOGI(TAG, "zb_action_handler(%d)", callback_id);

    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        if (message != NULL) {
            xSemaphoreTake(lightMutex, portMAX_DELAY);

            ESP_LOGI(TAG, "Set attribute value message");
            esp_zb_zcl_set_attr_value_message_t *msg = (esp_zb_zcl_set_attr_value_message_t *)message;
            ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d), data(%d)",
                msg->info.dst_endpoint, msg->info.cluster, msg->attribute.id, msg->attribute.data.size, *(bool *)msg->attribute.data.value);
            ESP_LOGI(TAG, "Endpoint: %d Cluster: %d Attribute ID: %d", msg->info.dst_endpoint, msg->info.cluster, msg->attribute.id);
            if (msg->info.dst_endpoint == 10) {
                ESP_LOGI(TAG, "Set attribute value message");
                if (msg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
                    if (msg->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && msg->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
                        bool light_state = msg->attribute.data.value ? *(bool *)msg->attribute.data.value : false;
                        ESP_LOGI(TAG, "Light state changed to: %s", (light_state ? "on" : "off"));
                        if (light_state) {
                            effect = 0; // fade in
                        } else {
                            effect = 1; // fade out
                        }
                    }
                } else if (msg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL) {
                    uint8_t level = *(uint8_t *)msg->attribute.data.value;
                    
                    ESP_LOGI(TAG, "Setting brightness to: %d", level);
                    if (level > 0) {
                        for (int i = 0; i < TOTAL_EFFECTS; i++) {
                            light_effects[i].max_brightness = level;
                            for (int o = 0; o < TOTAL_LIGHTS; o++) {
                                if (light_effects[i].light_state[o].level > level) {
                                    light_effects[i].light_state[o].level = level;
                                }
                            } 
                        }
                    }
                } else if (msg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT) {
                    float analog_output_new_value = *(float *)msg->attribute.data.value;
                    ESP_LOGI(TAG, "Analog output set to %f", analog_output_new_value);
                } else if (msg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_MULTI_VALUE) {
                    uint16_t multistate_value_new_value = *(uint16_t *)msg->attribute.data.value;
                    if (multistate_value_new_value < TOTAL_EFFECTS) {
                        ESP_LOGI(TAG, "Activating effect %d", multistate_value_new_value);
                        effect = (int32_t)multistate_value_new_value;
                    } else {
                        ESP_LOGW(TAG, "Invalid effect specified: %d", multistate_value_new_value);
                    }
                }
            }

            xSemaphoreGive(lightMutex);
        } else {
            ESP_LOGW(TAG, "Received Zigbee set attribute value with empty message.");
        }
        break;
    case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback: report attr", callback_id);
        ret = zb_attribute_reporting_handler((esp_zb_zcl_report_attr_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback: read attr", callback_id);
        ret = zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback: config response", callback_id);
        ret = zb_configure_report_resp_handler((esp_zb_zcl_cmd_config_report_resp_message_t *)message);
        break;
    case ESP_ZB_CORE_IDENTIFY_EFFECT_CB_ID:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback: identify effect", callback_id);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

// Force reset in case reset button is pressed
static void reset_interrupt_handler(void *args)
{
    esp_restart();
}

// Checks if the reset GPIO is bridged and performs a factory
// reset
void check_reset_gpio(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << RESET_GPIO_PIN);
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_config_t io_out_conf = {};
    io_out_conf.intr_type = GPIO_INTR_DISABLE;
    io_out_conf.mode = GPIO_MODE_OUTPUT;
    io_out_conf.pin_bit_mask = (1ULL << RESET_GPIO_PIN2);
    io_out_conf.pull_down_en = 0;
    io_out_conf.pull_up_en = 1;
    ESP_ERROR_CHECK(gpio_config(&io_out_conf));

    ESP_LOGI(TAG, "Setting pin %d to high.", RESET_GPIO_PIN2);
    gpio_set_level(RESET_GPIO_PIN2, 1);

    for (int i = 0; i < 5; i++) {
        int level = gpio_get_level(RESET_GPIO_PIN);
        if (level == 1) {
            if (i == 4)
            {
                ESP_LOGW(TAG, "Pin %d is high, performing factory reset", RESET_GPIO_PIN);
                nvs_flash_erase();
                esp_zb_factory_reset();
            }
            else
            {
                ESP_LOGI(TAG, "Reset pin is high, waiting for %d seconds to factory reset...", 5 - i);
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else
        {
            ESP_LOGI(TAG, "Reset pin is low, normal startup.");
            break;
        }
    }
    if (!factory_reset) {
        factory_reset = esp_zb_bdb_is_factory_new();
    }
    ESP_LOGI(TAG, "Factory resetting (factory reset status is %s), continuing startup...", (factory_reset ? "true" : "false"));

    ESP_LOGI(TAG, "Enabling reboot interrupt on reset button.");
    gpio_install_isr_service(0);
    gpio_set_intr_type(RESET_GPIO_PIN, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(RESET_GPIO_PIN, reset_interrupt_handler, (void *)NULL);
    gpio_intr_enable(RESET_GPIO_PIN);
}

// Main Zigbee setup
static void esp_zb_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Initializing Zigbee stack...");

    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    uint8_t zcl_version, null_values;
    zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE;
    null_values = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE;

    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();

    // Basic cluster
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &zcl_version));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &null_values));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, &modelid[0]));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, &manufname[0]));

    // Identify cluster
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    ESP_ERROR_CHECK(esp_zb_identify_cluster_add_attr(esp_zb_identify_cluster, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &null_values));

    // On-off cluster
    esp_zb_on_off_cluster_cfg_t on_off_cfg;
    on_off_cfg.on_off = ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE;
    esp_zb_attribute_list_t *esp_zb_on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);

    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    // On-off clusters are in server role, though we won't accept attribute updates.
    // From Zigbee cluster specifications:
    // Conversely, the command that facilitates dynamic attribute reporting, i.e., the report attribute command
    // is (typically) sent from the server device (as typically this is where the attribute data itself is stored)
    // and sent to the client device that has been bound to the server device

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    // Add level cluster
    esp_zb_level_cluster_cfg_t level_cfg;
    level_cfg.current_level = 254;

    esp_zb_attribute_list_t *esp_zb_level_cluster = esp_zb_level_cluster_create(&level_cfg);

    uint16_t remaining_time = ESP_ZB_ZCL_LEVEL_CONTROL_REMAINING_TIME_DEFAULT_VALUE;
    uint8_t min_level = MIN_BRIGHTNESS;
    uint8_t max_level = MAX_BRIGHTNESS;

    ESP_ERROR_CHECK(esp_zb_level_cluster_add_attr(esp_zb_level_cluster, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_REMAINING_TIME_ID, &remaining_time));
    ESP_ERROR_CHECK(esp_zb_level_cluster_add_attr(esp_zb_level_cluster, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_MIN_LEVEL_ID, &min_level));
    ESP_ERROR_CHECK(esp_zb_level_cluster_add_attr(esp_zb_level_cluster, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_MAX_LEVEL_ID, &max_level));

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_level_cluster(esp_zb_cluster_list, esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

#if 0
    for (int i = 0; i < TOTAL_LIGHTS; i++)
    {
        // Add level clusters for each light for debugging
        esp_zb_level_cluster_cfg_t light_level_cfg;
        light_level_cfg.current_level = 254;

        esp_zb_attribute_list_t *esp_zb_level_cluster_light = esp_zb_level_cluster_create(&light_level_cfg);

        uint16_t remaining_time = ESP_ZB_ZCL_LEVEL_CONTROL_REMAINING_TIME_DEFAULT_VALUE;
        uint8_t min_level = MIN_BRIGHTNESS;
        uint8_t max_level = MAX_BRIGHTNESS;

        ESP_ERROR_CHECK(esp_zb_level_cluster_add_attr(esp_zb_level_cluster_light, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_REMAINING_TIME_ID, &remaining_time));
        ESP_ERROR_CHECK(esp_zb_level_cluster_add_attr(esp_zb_level_cluster_light, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_MIN_LEVEL_ID, &min_level));
        ESP_ERROR_CHECK(esp_zb_level_cluster_add_attr(esp_zb_level_cluster_light, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_MAX_LEVEL_ID, &max_level));

        ESP_ERROR_CHECK(esp_zb_cluster_list_add_level_cluster(esp_zb_cluster_list, esp_zb_level_cluster_light, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    }
#endif

    esp_zb_multistate_value_cluster_cfg_t multistate_value_cfg;
    multistate_value_cfg.out_of_service = false;
    multistate_value_cfg.present_value = 0;
    multistate_value_cfg.status_flags = ESP_ZB_ZCL_MULTI_VALUE_STATUS_FLAGS_DEFAULT_VALUE;
    multistate_value_cfg.number_of_states = sizeof(light_effects) / sizeof(light_effects[0]);
    esp_zb_attribute_list_t *esp_zb_multistate_value_cluster = esp_zb_multistate_value_cluster_create(&multistate_value_cfg);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_multistate_value_cluster(esp_zb_cluster_list, esp_zb_multistate_value_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

#if 0
    esp_zb_attribute_list_t *multi_value_cluster = esp_zb_cluster_list_get_cluster(esp_zb_cluster_list, ESP_ZB_ZCL_CLUSTER_ID_MULTI_VALUE, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_attribute_list_t *attr = multi_value_cluster;
    while (attr) {
        if (attr->attribute.id == ESP_ZB_ZCL_ATTR_MULTI_VALUE_PRESENT_VALUE_ID) {
            ESP_LOGI(TAG, "Enabling attribute reporting for multi-value cluster");
            attr->attribute.access = multi_value_cluster->attribute.access | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING;
            break;
        }
        attr = attr->next;
    }
#endif
 
#if 0
    // Add analog value cluster for effect setting
    esp_zb_analog_output_cluster_cfg_t analog_output_cfg;
    analog_output_cfg.out_of_service = 0;
    analog_output_cfg.present_value = 0;
    
    esp_zb_attribute_list_t *esp_zb_analog_output_cluster = esp_zb_analog_output_cluster_create(&analog_output_cfg);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_output_cluster(esp_zb_cluster_list, esp_zb_analog_output_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
#endif

    esp_zb_endpoint_config_t endpoint_cfg = {
        .endpoint = 10,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_LEVEL_CONTROL_SWITCH_DEVICE_ID,
        .app_device_version = 1,
    };
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_cfg));

    esp_zb_device_register(esp_zb_ep_list);

    esp_zb_zcl_reporting_info_t on_off_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = 10,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,
        .u.send_info.max_interval = 0,
        .u.send_info.def_min_interval = 1,
        .u.send_info.def_max_interval = 0,
        .u.send_info.delta.u16 = 1000,
        .attr_id = ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_zb_zcl_update_reporting_info(&on_off_reporting_info);

#if 0
    esp_zb_zcl_reporting_info_t level_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = 10,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,
        .u.send_info.max_interval = 0,
        .u.send_info.def_min_interval = 1,
        .u.send_info.def_max_interval = 0,
        .u.send_info.delta.u16 = 1000,
        .attr_id = ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_zb_zcl_update_reporting_info(&level_reporting_info);
#endif

    /* Config the reporting info for multi-value  */
    esp_zb_zcl_reporting_info_t multi_value_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = 10,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_MULTI_VALUE,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,
        .u.send_info.max_interval = 0,
        .u.send_info.def_min_interval = 1,
        .u.send_info.def_max_interval = 0,
        .u.send_info.delta.u16 = 1000,
        .attr_id = ESP_ZB_ZCL_ATTR_MULTI_VALUE_PRESENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_zb_zcl_update_reporting_info(&multi_value_reporting_info);

    esp_zb_core_action_handler_register(zb_action_handler);

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    // esp_zb_set_secondary_network_channel_set(ESP_ZB_SECONDARY_CHANNEL_MASK);

    ESP_LOGI(TAG, "Starting Zigbee now...");
    ESP_ERROR_CHECK(esp_zb_start(true));

    esp_zb_stack_main_loop();
}

int gamma_correct_led_duty(uint8_t level)
{
    int idx = (int)((float)level / (MAX_BRIGHTNESS / 100.0));
    int ret = (int)((float)LED_MAX_DUTY * gamma_correction_lut[idx]);
    if (ret > LED_MAX_DUTY) {
        ESP_LOGW(TAG, "Gamma correct failed for level=%d (idx=%d, ret=%d)", level, idx, ret);
    }
    return ret;
}

int dali_command(uint8_t address, dali_cmd_t command, dali_address_type_t addr_type)
{
    int resp = 0;

#ifdef DALI_DEBUG
    int64_t start_time = 0, end_time = 0, isr_start = 0;
    ESP_LOGI(TAG, "Sending command %d to %d", address, command);
    isr_start = dali_->bus->isr_count;
    start_time = esp_timer_get_time() / 1000;
#endif

    resp = dali_send_cmd_wait(dali_, address, command, addr_type, 100);
    if (resp == DALI_READY_TIMEOUT)
    {
        dali_bus_timeouting += 1;
    }
    else
    {
        dali_bus_timeouting = 0;
    }

#ifdef DALI_DEBUG
    end_time = esp_timer_get_time() / 1000;
    ESP_LOGI(TAG, "Response was: %d (took %llu ms with %llu ISRs)", resp, end_time - start_time, dali_->bus->isr_count - isr_start);
#endif
    return resp;
}

int dali_arc(uint8_t address, uint8_t value, dali_address_type_t addr_type)
{
    dali_ret_t resp = 0;

#ifdef DALI_DEBUG
    int64_t start_time = 0, end_time = 0, isr_start = 0;
    ESP_LOGI(TAG, "Sending ARC %d to %d", address, value);
    isr_start = dali_->bus->isr_count;
    start_time = esp_timer_get_time() / 1000;
#endif

    resp = dali_send_arc_wait(dali_, address, value, addr_type, 50);
    if (resp == DALI_READY_TIMEOUT)
    {
        dali_bus_timeouting += 1;
    }
    else
    {
        dali_bus_timeouting = 0;
    }
#ifdef DALI_DEBUG
    end_time = esp_timer_get_time() / 1000;
    ESP_LOGI(TAG, "Response was: %d (took %llu ms with %llu ISRs)", resp, end_time - start_time, dali_->bus->isr_count - isr_start);
#endif
    return (int)resp;
}

// Main Dali task
static void esp_dali_task(void *pvParameters)
{
    uint32_t fps_sleep = 0, frame = 0;
    uint64_t frame_start_time = 0, frame_end_time = 0;
    uint64_t effect_start_time = 0, dali_commands = 0, last_dali_commands = 0;
    int64_t  frame_diff = 0;

    ESP_LOGI(TAG, "Waiting for driver initialization...");

    uint32_t ulNotificationValue;
    ulNotificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100000));
    if (ulNotificationValue == 1) {
        ESP_LOGI(TAG, "Drivers are initialized.");
    } else {
        ESP_LOGE(TAG, "Driver initialization timed out.");
    }
    light_effect *light_effect = NULL;
    effect_start_time = esp_timer_get_time() / 1000;

    while (true) {
        frame_start_time = esp_timer_get_time() / 1000;

        xSemaphoreTake(lightMutex, portMAX_DELAY);
        if (last_effect != effect) {
            ESP_LOGI(TAG, "Last effect: %ld", last_effect);
            if (last_effect > -1 && light_effects[last_effect].free_user_data && light_effects[last_effect].user_data != NULL)
            {
                free(light_effects[last_effect].user_data);
                light_effects[last_effect].user_data = NULL;
            }
            last_effect = effect;

            esp_zb_lock_acquire(portMAX_DELAY);
            uint16_t mstate_value = (uint16_t)effect;
            uint16_t light_state = (uint16_t)(effect == 1 ? false : true);
            esp_zb_zcl_set_attribute_val(10,
                                         ESP_ZB_ZCL_CLUSTER_ID_MULTI_VALUE, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                         ESP_ZB_ZCL_ATTR_MULTI_VALUE_PRESENT_VALUE_ID, &mstate_value, false);
            esp_zb_zcl_set_attribute_val(10,
                                         ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                         ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, &light_state, false);
            esp_zb_lock_release();

            // Save last effect to NVS
            esp_err_t nvs_err = nvs_set_i32(nvs_h, "effect_index", effect);
            ESP_ERROR_CHECK(nvs_err);
            nvs_err = nvs_commit(nvs_h);
            ESP_ERROR_CHECK(nvs_err);

            light_effect = &light_effects[effect];
            fps_sleep = 1000 / light_effect->fps;
            memcpy(light_effect->light_state, last_light_state, sizeof(light_state) * light_effect->total_lights);
            effect_start_time = esp_timer_get_time() / 1000000;
            ESP_LOGI(TAG, "Effect %ld active, FPS sleep: %lu ms per frame", effect, fps_sleep);
        }
        light_effect->render((void *)light_effect, frame);
        frame++;

        if (frame % 60 == 0) {
            char light_status[256] = { '\0' };
            snprintf(light_status, sizeof(light_status), "Frame %-8lu ", frame);
            for (int i = 0; i < light_effect->total_lights; i++) {
                char buf[32] = { '\0' };
                snprintf(buf, sizeof(buf), "| %-6d", light_effect->light_state[i].level);
                strcat(light_status, buf);
            }
            strcat(light_status, "|");
            ESP_LOGI(TAG, "%s -- %lld cmds/60frames", light_status, (dali_commands - last_dali_commands));
            last_dali_commands = dali_commands;
        }
        bool light_state_updated = false;

        for (int i = 0; i < light_effect->total_lights; i++) {
            if (light_effect->light_state[i].level != last_light_state[i].level) {
                if (light_effect->light_state[i].level == light_effect->min_brightness) {
                    // Turn light off
#ifndef CONFIG_DALIZB_LED_EMULATOR
                    // ESP_LOGI(TAG, "Setting DALI address %u to off", i);
                    dali_command(i + 1, DALI_CMD_OFF, DALI_SHORT_ADDRESS);
                    dali_commands += 1;
                    light_state_updated = true;
#else
                    if (driver_is_inited) {
                        ledc_set_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel, 0);
                        ledc_update_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel);
                    }
#endif
                } else {
                    // Send level directly
#ifndef CONFIG_DALIZB_LED_EMULATOR
                    // ESP_LOGI(TAG, "Setting DALI address %u to %u", i, light_effect->light_state[i].level);
                    int light_diff = abs((int)light_effect->light_state[i].level - (int)last_light_state[i].level);
                    if (light_diff >= 16 || (light_diff != 0 && light_effect->light_state[i].level == light_effect->max_brightness))
                    {
                        dali_arc(i + 1, light_effect->light_state[i].level, DALI_SHORT_ADDRESS);
                        dali_commands += 1;
                        light_state_updated = true;
                    }
#else
                    // ESP_LOGI(TAG,   "Set LED %d to %d", i, LED_DUTY_DIVIDER*light_effect->light_state[i].level);
                    if (driver_is_inited) {
                        ledc_set_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel, gamma_correct_led_duty(light_effect->light_state[i].level));
                        ledc_update_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel);
                    }
#endif
                }
            }
        }
        // Store last state
        if (light_state_updated)
        {
            memcpy(last_light_state, light_effect->light_state, sizeof(light_state) * light_effect->total_lights);
        }
        frame_end_time = esp_timer_get_time() / 1000;
        frame_diff = frame_end_time - frame_start_time;
        xSemaphoreGive(lightMutex);

        if (frame_diff < fps_sleep) {
            vTaskDelay(pdMS_TO_TICKS((TickType_t)(fps_sleep - frame_diff)));
        }

        if (dali_bus_timeouting > 10)
        {
            if (dali_bus_timeouting == 1000)
            {
                ESP_LOGW(TAG, "DALI bus is timeouting, putting long pauses in.");
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

// Main function
void app_main(void)
{
    uint8_t ieeeMac[8] = { 0 };
    esp_read_mac(ieeeMac, ESP_MAC_IEEE802154);
    ESP_LOGI(TAG, "Zigbee MAC: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", ieeeMac[0], ieeeMac[1], ieeeMac[2], ieeeMac[3], ieeeMac[4], ieeeMac[5], ieeeMac[6], ieeeMac[7]);

    ESP_LOGI(TAG, "Checking for factory reset...");
    check_reset_gpio();

    ESP_LOGI(TAG, "nvs_flash_init()");
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs_h));
    nvs_err = nvs_get_i32(nvs_h, "effect_index", &effect);
    switch (nvs_err)
    {
    case ESP_OK:
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        effect = 0;
        break;
    default:
        ESP_ERROR_CHECK(nvs_err);
    }

#if 0
#ifdef CONFIG_DALIZB_WIFI_PROVISION
    over_the_air_update(true);
#else
#ifdef CONFIG_DALIZB_WIFI
    over_the_air_update(false);
#endif
#endif
#endif

    esp_pm_lock_handle_t freq_lock;
    esp_pm_lock_handle_t light_sleep_lock;
    esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "max_freq", &freq_lock);
    esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "light_sleep", &light_sleep_lock);

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    lightMutex = xSemaphoreCreateMutex();

    xTaskCreate(esp_dali_task, "DALI", 4096, NULL, 5, &xDaliTask);
    xTaskCreate(esp_zb_task, "Zigbee", 4096, NULL, 10, &xZigbeeTask);
}
