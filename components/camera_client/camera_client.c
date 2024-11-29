#include "camera_client.h"


#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK

#define WIFI_EVENT_CONNECTED    ((EventBits_t) 0x01UL)
#define WIFI_EVENT_MAX_ATTEMPTS ((EventBits_t) 0x02UL)

static const char * const TAG = "CAM";

static EventGroupHandle_t wifi_connection_event_group;

static esp_err_t wifi_init_station(void);
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

esp_err_t camera_client_init(void)
{
    esp_err_t status;

    status = nvs_flash_init();

    if (ESP_ERR_NVS_NO_FREE_PAGES == status || ESP_ERR_NVS_NEW_VERSION_FOUND == status)
    {
        status = nvs_flash_erase();

        if (ESP_OK == status)
        {
            status = nvs_flash_init();
        }
    }

    if (ESP_OK == status)
    {
        status = wifi_init_station();
    }

    return status;
}

esp_err_t wifi_init_station(void)
{
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t wifi_config;
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    EventBits_t wifi_init_event_bits;
    esp_err_t status;

    status = ESP_OK;

    wifi_connection_event_group = xEventGroupCreate();

    if (NULL == wifi_connection_event_group)
    {
        status = ESP_ERR_NO_MEM;
    }

    if (ESP_OK == status)
    {
        status = esp_netif_init();
    }

    if (ESP_OK == status)
    {
        status = esp_event_loop_create_default();
    }

    if (ESP_OK == status)
    {
        esp_netif_create_default_wifi_sta();
        status = esp_wifi_init(&wifi_init_config);
    }

    if (ESP_OK == status)
    {
        status = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id);
    }

    if (ESP_OK == status)
    {
        status = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip);
    }

    if (ESP_OK == status)
    {
        wifi_config = (wifi_config_t) {
            .sta = {
                .ssid = CONFIG_WIFI_AP_SSID,
                .password = CONFIG_WIFI_AP_PASSWORD,
                .threshold.authmode = WIFI_SCAN_AUTH_MODE_THRESHOLD,
                .sae_pwe_h2e = WPA3_SAE_PWE_HUNT_AND_PECK,
                .sae_h2e_identifier = "",
            },
        };

        status = esp_wifi_set_mode(WIFI_MODE_STA);
    }

    if (ESP_OK == status)
    {
        status = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    }

    if (ESP_OK == status)
    {
        status = esp_wifi_start();
    }

    if (ESP_OK == status)
    {
        wifi_init_event_bits = xEventGroupWaitBits(
            wifi_connection_event_group, 
            WIFI_EVENT_CONNECTED | WIFI_EVENT_MAX_ATTEMPTS,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY
        );

        if (WIFI_EVENT_CONNECTED & wifi_init_event_bits)
        {
            ESP_LOGI(TAG, "Connected to AP with SSID = %s", CONFIG_WIFI_AP_SSID);
        }
        else if (WIFI_EVENT_MAX_ATTEMPTS & wifi_init_event_bits)
        {
            ESP_LOGW(TAG, "Connection to %s failed", CONFIG_WIFI_AP_SSID);
        }
        else
        {
            ESP_LOGE(TAG, "Unhandled WIFI event.");
        }
    }

    return status;
}

void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    static int32_t connection_attempts_remaining = CONFIG_WIFI_AP_MAX_CONNECTION_ATTEMPTS;
    ip_event_got_ip_t * event; 

    if (WIFI_EVENT == event_base && WIFI_EVENT_STA_START == event_id)
    {
        esp_wifi_connect();
    }
    else if (WIFI_EVENT == event_base && WIFI_EVENT_STA_DISCONNECTED == event_id)
    {
        if (0 < connection_attempts_remaining)
        {
            esp_wifi_connect();
            connection_attempts_remaining--;
            ESP_LOGI(TAG, "Attempting connection to AP again");
        }
        else
        {
            xEventGroupSetBits(wifi_connection_event_group, WIFI_EVENT_MAX_ATTEMPTS);
        }
        ESP_LOGI(TAG, "Connection to AP failed");
    }
    else if (IP_EVENT == event_base && IP_EVENT_STA_GOT_IP)
    {
        event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        connection_attempts_remaining = CONFIG_WIFI_AP_MAX_CONNECTION_ATTEMPTS;
        xEventGroupSetBits(wifi_connection_event_group, WIFI_EVENT_CONNECTED);
    }
}