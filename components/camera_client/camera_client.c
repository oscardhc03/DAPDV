#include "camera_client.h"


#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK

#define WIFI_EVENT_CONNECTED    ((EventBits_t) 0x01UL)
#define WIFI_EVENT_MAX_ATTEMPTS ((EventBits_t) 0x02UL)

#define HTTP_STATUS_OK ((int32_t) 200)

#define MAX_HTTP_RESPONSE_BUFFER_LEN    (512)

static const char * const TAG = "CAM";

static EventGroupHandle_t wifi_connection_event_group;

static char * http_response_buffer = NULL;

static esp_err_t wifi_init_station(void);
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

static esp_err_t http_event_handler(esp_http_client_event_t * event);
static esp_err_t http_on_data_event_handler(esp_http_client_event_t * event, char * output_buffer, size_t * const output_len);
static esp_err_t http_on_finish_event_handler(esp_http_client_event_t * event, char * output_buffer, size_t * const output_len);
static esp_err_t http_on_disconnect_event_handler(esp_http_client_event_t * event, char * output_buffer, size_t * const output_len);

static esp_err_t parse_response_json(const char * const json_buffer, int64_t len, cam_detect_objects_t * const parsed_results);


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

esp_err_t camera_client_fetch_results(cam_detect_objects_t * const results)
{
    static esp_http_client_config_t http_client_config = {
        .url = CONFIG_CAMERA_SERVER_URL,
        .event_handler = http_event_handler,
        .user_data = NULL,
        .disable_auto_redirect = true,
        .timeout_ms = 5000,
    };

    esp_http_client_handle_t http_client;
    int64_t content_length;
    int32_t response_status_code;
    esp_err_t status;

    status = ESP_OK;

    if (NULL == results)
    {
        status = ESP_ERR_INVALID_ARG;
    }

    if (NULL == http_response_buffer) 
    {
        http_response_buffer = (char *) malloc(MAX_HTTP_RESPONSE_BUFFER_LEN * sizeof(char));

        if (NULL == http_response_buffer)
        {
            status = ESP_ERR_NO_MEM;
        }
    }

    http_client = NULL;
    if (ESP_OK == status)
    {
        http_client_config.user_data = http_response_buffer;
        content_length = 0LL;
        response_status_code = -1;

        http_client = esp_http_client_init(&http_client_config);

        if (NULL == http_client)
        {
            status = ESP_FAIL;
        }
    }

    if (ESP_OK == status)
    {
        ESP_LOGI(TAG, "Making request");
        status = esp_http_client_perform(http_client);
    }

    if (ESP_OK == status) 
    {
        response_status_code = esp_http_client_get_status_code(http_client);
        content_length = esp_http_client_get_content_length(http_client);

        ESP_LOGI(TAG, "HTTP GET status = %ld, content length = %llu", response_status_code, content_length);
    } 
    else 
    {
        ESP_LOGE(TAG, "HTTP GET request failed (%s)", esp_err_to_name(status));
    }

    if (ESP_OK == status && HTTP_STATUS_OK == response_status_code && 0 < content_length) 
    {
        status = parse_response_json(http_response_buffer, content_length, results);
    } 
    else 
    {
        ESP_LOGW(TAG, "HTTP request success, but response was unexpected.");
        status = ESP_FAIL;
    }

    if (NULL != http_client)
    {
        esp_http_client_cleanup(http_client);
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
        ESP_LOGI(TAG, "Connection to AP failed (%ld attempts)", connection_attempts_remaining);
    }
    else if (IP_EVENT == event_base && IP_EVENT_STA_GOT_IP == event_id)
    {
        event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        connection_attempts_remaining = CONFIG_WIFI_AP_MAX_CONNECTION_ATTEMPTS;
        xEventGroupSetBits(wifi_connection_event_group, WIFI_EVENT_CONNECTED);
    }
}

esp_err_t http_event_handler(esp_http_client_event_t * event)
{
    esp_err_t status = ESP_OK;
    static char* output_buffer;
    static size_t output_len;

    switch(event->event_id)
    {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", event->header_key, event->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            status = http_on_data_event_handler(event, output_buffer, &output_len);
            break;
        case HTTP_EVENT_ON_FINISH: 
            status = http_on_finish_event_handler(event, output_buffer, &output_len);
            break;
        case HTTP_EVENT_DISCONNECTED:
            status = http_on_disconnect_event_handler(event, output_buffer, &output_len);
            break;
        case HTTP_EVENT_REDIRECT:
        default:
            break;
    }

    return status;
}

esp_err_t http_on_data_event_handler(esp_http_client_event_t * event, char * output_buffer, size_t * const output_len)
{
    if (NULL == event || NULL == output_len) 
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, length = %d", event->data_len);

    if (!esp_http_client_is_chunked_response(event->client))
    {
        if (NULL != event->user_data) 
        {
            memcpy(event->user_data + (*output_len), event->data, event->data_len);
        } else {
            if (NULL == output_buffer)
            {
                output_buffer = (char *) malloc(esp_http_client_get_content_length(event->client));
                *output_len = 0;

                if (NULL == output_buffer) 
                {
                    ESP_LOGE(TAG, "Failed to allocate memory for HTTP client output_buffer");
                    return ESP_ERR_NO_MEM;
                }
            }

            memcpy(output_buffer + (*output_len), event->data,  event->data_len);
        }

        (*output_len) += event->data_len;
    }

    return ESP_OK;
}

esp_err_t http_on_finish_event_handler(esp_http_client_event_t * event, char * output_buffer, size_t * const output_len)
{
    esp_err_t status = ESP_OK;

    if (NULL == event || NULL == output_len)
    {
        status = ESP_ERR_INVALID_ARG;
        return status;
    }

    ESP_LOGI(TAG, "HTTP HTTP_EVENT_ON_FINISH");
    *output_len = 0;

    if (NULL != output_buffer) 
    {
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, output_buffer, *output_len, ESP_LOG_DEBUG);
        free(output_buffer);
        output_buffer = NULL;
    }

    return status;
}

esp_err_t http_on_disconnect_event_handler(esp_http_client_event_t * event, char * output_buffer, size_t * const output_len)
{
    ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
    int mbedtls_err = 0;
    esp_err_t status = esp_tls_get_and_clear_last_error(event->data, &mbedtls_err, NULL);

    if (ESP_OK != status)
    {
        if (output_buffer != NULL)
        {
            free(output_buffer);
            output_buffer = NULL;
        }

        *output_len = 0;
        ESP_LOGW(TAG, "Last esp error code (0x%x)", status);
        ESP_LOGW(TAG, "Last mbedtls failure (0x%x)", mbedtls_err);
    }

    return status;
}

esp_err_t parse_response_json(const char * const json_buffer, int64_t len, cam_detect_objects_t * const parsed_results)
{
    esp_err_t status;
    cJSON * parsed_json;

    const cJSON * elemento_json;

    status = ESP_OK;

    if (NULL == json_buffer || NULL == parsed_results) 
    {
        status = ESP_ERR_INVALID_ARG;
    }

    const char* parse_end = "";

    parsed_json = cJSON_ParseWithLengthOpts(json_buffer, (len + 1), &parse_end, cJSON_False);

    if (NULL == parsed_json)
    {
        if (NULL != parse_end)
        {
            ESP_LOGW(TAG, "JSON parse error before: %s", parse_end);
        }

        status = ESP_FAIL;
    }

    if (ESP_OK == status)
    {
        elemento_json = cJSON_GetObjectItemCaseSensitive(parsed_json, "0 Persona");
        if (cJSON_IsNumber(elemento_json))
        {
            parsed_results->persona = ((float) elemento_json->valuedouble);
        }

        elemento_json = cJSON_GetObjectItemCaseSensitive(parsed_json, "1 Escaleras");
        if (cJSON_IsNumber(elemento_json))
        {
            parsed_results->escaleras = ((float) elemento_json->valuedouble);
        }

        elemento_json = cJSON_GetObjectItemCaseSensitive(parsed_json, "2 Entorno");
        if (cJSON_IsNumber(elemento_json))
        {
            parsed_results->entorno = ((float) elemento_json->valuedouble);
        }

        elemento_json = cJSON_GetObjectItemCaseSensitive(parsed_json, "3 Puerta");
        if (cJSON_IsNumber(elemento_json))
        {
            parsed_results->puerta = ((float) elemento_json->valuedouble);
        }
    }

    cJSON_Delete(parsed_json);

    return status;
}