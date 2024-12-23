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

#define TOTAL_BUTTONS  5
#define RESET_GPIO_PIN 6
#define INT_GPIO_PIN   4
#define RST_GPIO_PIN   5

#include "dali/dali.h"
#include "driver/gptimer.h"
#include "esp_timer.h"
#include "esp_pm.h"

#include "driver/ledc.h"

#include "scenes.h"
#include "ota.h"

static const char *TAG = "ZB_DALI";

// Design Light
static char modelid[] = {22, 'D', 'e', 's', 'i', 'g', 'n', ' ', 'L', 'i', 'g', 'h', 't', ' ', 'F', 'i', 'v', 'e', ' ', 'F', 'o', 'l', 'd'};
static char manufname[] = {9, 'E', 's', 'p', 'r', 'e', 's', 's', 'i', 'f'};

// Internal button state
static uint8_t button_state = 0, previous_state = 0;
static uint8_t button_attrs[TOTAL_BUTTONS];
static uint8_t button_attrs_previous[TOTAL_BUTTONS];
static bool zigbee_initialized = false;

static light_state last_light_state[TOTAL_LIGHTS];
static uint8_t dali_address[TOTAL_LIGHTS] = { (uint8_t) 0 };

static dali *dali_ = NULL;

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

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p     = signal_struct->p_app_signal;
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
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status != ESP_OK) {
            ESP_LOGW(TAG, "Stack %s failure with %s status, steering", esp_zb_zdo_signal_to_string(sig_type), esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        } else {
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
                // Just send an update, in case coordinator is out of sync
                zigbee_initialized = true;
                // esp_zb_scheduler_alarm((esp_zb_callback_t)send_reports, (uint8_t)true, 1000);
            }
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGI(TAG, "New device commissioned or rejoined (short: 0x%04hx)", dev_annce_params->device_short_addr);
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Network steering started");
        }
        break;
    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            zigbee_initialized = true;
            // esp_zb_scheduler_alarm((esp_zb_callback_t)send_reports, (uint8_t)true, 1000);
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
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
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        break;
    }
}

static esp_err_t zb_attribute_reporting_handler(const esp_zb_zcl_report_attr_message_t *message)
{
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
    switch (callback_id) {
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
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
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

    for (int i = 0; i < 5; i++) {
        int level = gpio_get_level(RESET_GPIO_PIN);
        if (level == 1) {
            ESP_LOGW(TAG, "Pin %d is high, performing factory reset...", RESET_GPIO_PIN);
            if (!esp_zb_bdb_is_factory_new()) {
                esp_zb_factory_reset();
            } else {
                ESP_LOGI(TAG, "Skipping factory reset, since we are already factory reset.");
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(120));
    }
    ESP_LOGI(TAG, "Not factory resetting, continuing startup...");
}

// Main Zigbee setup
static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
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

    // Add scenes cluster
#if 0
    esp_zb_scenes_cluster_cfg_t scenes_cfg;
    scenes_cfg.scenes_count = sizeof(light_effects) / sizeof(light_effects[0]);
    ESP_LOGI(TAG, "Available scenes: %d", scenes_cfg.scenes_count);
    scenes_cfg.current_scene = 0;
    scenes_cfg.scene_valid = true;

    esp_zb_attribute_list_t *esp_zb_scenes_cluster = esp_zb_scenes_cluster_create(&scenes_cfg);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_scenes_cluster(esp_zb_cluster_list, esp_zb_scenes_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
#endif

    esp_zb_endpoint_config_t endpoint_cfg = {
        .endpoint = 10,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_LEVEL_CONTROL_SWITCH_DEVICE_ID,
        .app_device_version = 1,
    };
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_cfg));

    esp_zb_device_register(esp_zb_ep_list);

    esp_zb_core_action_handler_register(zb_action_handler);

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    // esp_zb_set_secondary_network_channel_set(ESP_ZB_SECONDARY_CHANNEL_MASK);

    ESP_ERROR_CHECK(esp_zb_start(true));

    check_reset_gpio();

    esp_zb_stack_main_loop();
}

void init_dali_bus()
{
    dali_ = dali_begin(RST_GPIO_PIN, INT_GPIO_PIN, true);
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

    resp = dali_send_arc_wait(dali_, address, value, addr_type, 100);
#ifdef DALI_DEBUG
    end_time = esp_timer_get_time() / 1000;
    ESP_LOGI(TAG, "Response was: %d (took %llu ms with %llu ISRs)", resp, end_time - start_time, dali_->bus->isr_count - isr_start);
#endif
    return (int)resp;
}

// Main Dali task
static void esp_dali_task(void *pvParameters)
{
    uint8_t counter;
    int     last_effect = -1, effect = 4;
    uint32_t fps_sleep = 0, frame = 0;
    uint64_t frame_start_time = 0, frame_end_time = 0;
    uint64_t effect_start_time = 0;
    int64_t  frame_diff = 0;

    light_effect *light_effect = NULL;
    while (true) {
        frame_start_time = esp_timer_get_time() / 1000;

        if (last_effect != effect) {
            last_effect = effect;
            light_effect = &light_effects[effect];
            fps_sleep = 1000 / light_effect->fps;
            effect_start_time = esp_timer_get_time() / 1000000;
            ESP_LOGI(TAG, "Effect %d active, FPS sleep: %lu", effect, fps_sleep);
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
            ESP_LOGI(TAG, "%s", light_status);
        }

        for (int i = 0; i < light_effect->total_lights; i++) {
            if (light_effect->light_state[i].level != last_light_state[i].level) {
                if (light_effect->light_state[i].level == light_effect->min_brightness) {
                    // Turn light off
#ifndef CONFIG_DALIZB_LED_EMULATOR
                    ESP_LOGI(TAG, "Setting DALI address %u to off", i);
                    dali_command(i, DALI_CMD_OFF, DALI_SHORT_ADDRESS);
#else
                    ledc_set_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel, 0);
                    ledc_update_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel);
#endif
                } else {
                    // Send level directly
#ifndef CONFIG_DALIZB_LED_EMULATOR
                    ESP_LOGI(TAG, "Setting DALI address %u to %u", i, light_effect->light_state[i].level);
                    dali_arc(i, light_effect->light_state[i].level, DALI_SHORT_ADDRESS);
#else
                    // ESP_LOGI(TAG,   "Set LED %d to %d", i, LED_DUTY_DIVIDER*light_effect->light_state[i].level);
                    ledc_set_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel, gamma_correct_led_duty(light_effect->light_state[i].level));
                    ledc_update_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel);
#endif
                }
            }
        }
        frame_end_time = esp_timer_get_time() / 1000;
        frame_diff = frame_end_time - frame_start_time;
        if (frame_diff < fps_sleep) {
            vTaskDelay(pdMS_TO_TICKS(fps_sleep - frame_diff));
        }
        // Store last state
        memcpy(last_light_state, light_effect->light_state, sizeof(light_state) * light_effect->total_lights);
    }
}

// Main function
void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };

    uint8_t ieeeMac[8] = { 0 };
    esp_read_mac(ieeeMac, ESP_MAC_IEEE802154);
    ESP_LOGI(TAG, "Zigbee MAC: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", ieeeMac[0], ieeeMac[1], ieeeMac[2], ieeeMac[3], ieeeMac[4], ieeeMac[5], ieeeMac[6], ieeeMac[7]);

    ESP_LOGI(TAG, "Hello World from OTA update! Version 4!");

    over_the_air_update();

    // ESP_LOGI(TAG, "nvs_flash_init()");
    // ESP_ERROR_CHECK(nvs_flash_init());

    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    esp_pm_lock_handle_t freq_lock;
    esp_pm_lock_handle_t light_sleep_lock;
    esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "max_freq", &freq_lock);
    esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "light_sleep", &light_sleep_lock);

#ifndef CONFIG_DALIZB_LED_EMULATOR
    init_dali_bus();
#else
    init_led_emulator();
#endif

    // xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
    xTaskCreate(esp_dali_task, "DALI", 4096, NULL, configMAX_PRIORITIES - 1, NULL);
}
