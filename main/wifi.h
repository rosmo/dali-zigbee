#ifndef _WIFI_H
#define _WIFI_H

#include "esp_wifi.h"
#include <nvs_flash.h>

#define DALIZB_NETIF_DESC_STA "dalizb_netif_sta"

#if CONFIG_DALIZB_WIFI_AUTH_OPEN
#define DALIZB_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_DALIZB_WIFI_AUTH_WEP
#define DALIZB_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_DALIZB_WIFI_AUTH_WPA_PSK
#define DALIZB_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_DALIZB_WIFI_AUTH_WPA2_PSK
#define DALIZB_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_DALIZB_WIFI_AUTH_WPA_WPA2_PSK
#define DALIZB_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_DALIZB_WIFI_AUTH_WPA2_ENTERPRISE
#define DALIZB_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_ENTERPRISE
#elif CONFIG_DALIZB_WIFI_AUTH_WPA3_PSK
#define DALIZB_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_DALIZB_WIFI_AUTH_WPA2_WPA3_PSK
#define DALIZB_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_DALIZB_WIFI_AUTH_WAPI_PSK
#define DALIZB_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

esp_err_t dali_wifi_connect();
void dali_wifi_shutdown(void);
void dali_wifi_provision();
#endif