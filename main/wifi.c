#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "wifi.h"
#include <esp_event.h>

#include "wifi_manager.h"

static const char *TAG = "WiFi";
static esp_netif_t *s_dali_sta_netif = NULL;
static SemaphoreHandle_t s_semph_get_ip_addrs = NULL;

static int s_retry_num = 0;

static TaskHandle_t *xTaskToNotify = NULL;

static void
dali_handler_on_wifi_disconnect(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    s_retry_num++;
    if (s_retry_num > CONFIG_DALIZB_MAXIMUM_RETRY) {
        ESP_LOGI(TAG, "WiFi Connect failed %d times, stop reconnect.", s_retry_num);
        /* let DALIZB_wifi_sta_do_connect() return */
        if (s_semph_get_ip_addrs) {
            xSemaphoreGive(s_semph_get_ip_addrs);
        }
        return;
    }
    ESP_LOGI(TAG, "Wi-Fi disconnected, trying to reconnect...");
    esp_err_t err = esp_wifi_connect();
    if (err == ESP_ERR_WIFI_NOT_STARTED) {
        return;
    }
    ESP_ERROR_CHECK(err);
}

static void dali_handler_on_wifi_connect(void *esp_netif, esp_event_base_t event_base,
                                         int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "Wi-Fi connected.");
}

bool dali_is_our_netif(const char *prefix, esp_netif_t *netif)
{
    return strncmp(prefix, esp_netif_get_desc(netif), strlen(prefix) - 1) == 0;
}

static void dali_handler_on_sta_got_ip(void *arg, esp_event_base_t event_base,
                                       int32_t event_id, void *event_data)
{
    s_retry_num = 0;
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    if (!dali_is_our_netif(DALIZB_NETIF_DESC_STA, event->esp_netif)) {
        ESP_LOGI(TAG, "Got something, not our Netif");
        return;
    }
    ESP_LOGI(TAG, "Got IPv4 event: Interface \"%s\" address: " IPSTR, esp_netif_get_desc(event->esp_netif), IP2STR(&event->ip_info.ip));
    if (s_semph_get_ip_addrs) {
        xSemaphoreGive(s_semph_get_ip_addrs);
    } else {
        ESP_LOGI(TAG, "- IPv4 address: " IPSTR ",", IP2STR(&event->ip_info.ip));
    }
}

void dali_wifi_start(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
    // Warning: the interface desc is used in tests to capture actual connection details (IP, gw, mask)
    esp_netif_config.if_desc = DALIZB_NETIF_DESC_STA;
    esp_netif_config.route_prio = 128;
    s_dali_sta_netif = esp_netif_create_wifi(WIFI_IF_STA, &esp_netif_config);
    esp_wifi_set_default_wifi_sta_handlers();

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void dali_wifi_stop(void)
{
    esp_err_t err = esp_wifi_stop();
    if (err == ESP_ERR_WIFI_NOT_INIT) {
        return;
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_wifi_deinit());
    ESP_ERROR_CHECK(esp_wifi_clear_default_wifi_driver_and_handlers(s_dali_sta_netif));
    esp_netif_destroy(s_dali_sta_netif);
    s_dali_sta_netif = NULL;
}

esp_err_t dali_wifi_sta_do_connect(wifi_config_t wifi_config, bool wait)
{
    if (wait) {
        s_semph_get_ip_addrs = xSemaphoreCreateBinary();
        if (s_semph_get_ip_addrs == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    s_retry_num = 0;
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &dali_handler_on_wifi_disconnect, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &dali_handler_on_sta_got_ip, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &dali_handler_on_wifi_connect, s_dali_sta_netif));

    ESP_LOGI(TAG, "Connecting to %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    esp_err_t ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi connect failed! ret:%x", ret);
        return ret;
    }
    if (wait) {
        ESP_LOGI(TAG, "Waiting for IP(s)");
        xSemaphoreTake(s_semph_get_ip_addrs, portMAX_DELAY);
        if (s_retry_num > CONFIG_DALIZB_MAXIMUM_RETRY) {
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

esp_err_t dali_wifi_sta_do_disconnect(void)
{
#ifndef CONFIG_DALIZB_WIFI_PROVISION
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &dali_handler_on_wifi_disconnect));
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &dali_handler_on_sta_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &dali_handler_on_wifi_connect));
    if (s_semph_get_ip_addrs) {
        vSemaphoreDelete(s_semph_get_ip_addrs);
    }
    return esp_wifi_disconnect();
#else
    return ESP_OK;
#endif
}

void dali_wifi_shutdown(void)
{
#ifndef CONFIG_DALIZB_WIFI_PROVISION
    dali_wifi_sta_do_disconnect();
    dali_wifi_stop();
#endif
}

esp_err_t dali_wifi_connect_now()
{
    ESP_LOGI(TAG, "Start WiFi connect.");
    esp_log_level_set("wifi", ESP_LOG_WARN);

    dali_wifi_start();
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_DALIZB_WIFI_SSID,
            .password = CONFIG_DALIZB_WIFI_PASSWORD,
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .sort_method = WIFI_CONNECT_AP_BY_SECURITY,
            .threshold.rssi = -127,
            .threshold.authmode = DALIZB_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    return dali_wifi_sta_do_connect(wifi_config, true);
}

esp_err_t dali_wifi_connect()
{
    if (dali_wifi_connect_now() != ESP_OK)
    {
        return ESP_FAIL;
    }
    ESP_ERROR_CHECK(esp_register_shutdown_handler(&dali_wifi_shutdown));

    return ESP_OK;
}

void dali_wifi_provisioning_monitoring_task(void *pvParameter)
{
    for (;;)
    {
        ESP_LOGI(TAG, "Free heap: %lu KB", esp_get_free_heap_size() / 1024);
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

/**
 * @brief this is an exemple of a callback that you can setup in your own app to get notified of wifi manager event.
 */
void dali_wifi_provisioning_cb_connection_ok(void *pvParameter)
{
    ip_event_got_ip_t *param = (ip_event_got_ip_t *)pvParameter;

    /* transform IP to human readable string */
    char str_ip[16];
    esp_ip4addr_ntoa(&param->ip_info.ip, str_ip, IP4ADDR_STRLEN_MAX);

    ESP_LOGI(TAG, "I have a connection and my IP is %s!", str_ip);
    if (xTaskToNotify != NULL)
    {
        xTaskNotifyGive(*xTaskToNotify);
        ESP_LOGI(TAG, "Stopping WiFi manager...");
        wifi_manager_stop();
    }
}

void dali_wifi_provision(TaskHandle_t *xOtaTask)
{
    xTaskToNotify = xOtaTask;

    /* start the wifi manager */
    wifi_manager_init();
    wifi_manager_start();

    /* register a callback as an example to how you can integrate your code with the wifi manager */
    wifi_manager_set_callback(WM_EVENT_STA_GOT_IP, &dali_wifi_provisioning_cb_connection_ok);

    /* Your code should go here. Here we simply create a task on core 2 that monitors free heap memory */
    xTaskCreate(&dali_wifi_provisioning_monitoring_task, "monitoring_task", 2048, NULL, 1, NULL);
}