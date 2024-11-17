#include <stdio.h>

#include <esp_log.h>

#include <driver/gpio.h>

#include <freertos/FreeRTOS.h>
// #include <freertos/task.h>
#include <freertos/queue.h>

#include <IQmathLib.h>

#include "gps.h"
#include "tf_mini_parser.h"

static const char * const TAG = "MAIN";

static const size_t TF_MINI_DATA_QUEUE_LEN = 20;
static const uint16_t TF_MINI_DIST_MIN_CM = 10u;
static const uint16_t TF_MINI_DIST_MAX_CM = 1200u;

static const uint8_t DIST_SENSOR_VALID_FG = 0x01u;

static const gps_config_t gps_config = {
    .uart = {
        .port = UART_NUM_1,
        .bit_rate = 9600,
        .rx_pin = GPIO_NUM_17,
        .word_length = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
    },
    .event_queue_length = 20,
};

static QueueHandle_t tf_mini_data_queue;
static QueueHandle_t gps_event_queue;

static void test_consumer_task(void * pvParameters);

void app_main(void)
{
    esp_err_t status;
    BaseType_t result;
    tf_mini_handle_t tf_mini_handle;
    gps_handle_t gps_handle;
    tf_mini_parser_config_t tf_mini_cfg;

    status = ESP_OK;

    if (ESP_OK == status)
    {
        tf_mini_data_queue = xQueueCreate(TF_MINI_DATA_QUEUE_LEN, sizeof(tf_mini_df_t * ));

        if (NULL == tf_mini_data_queue)
        {
            ESP_LOGW(TAG, "Create TF MINI data queue fail, no memory");
            status = ESP_ERR_NO_MEM;
        }
    }

    if (ESP_OK == status)
    {
        tf_mini_cfg = (tf_mini_parser_config_t) {
            .uart = {
                .port = UART_NUM_2,
                .bit_rate = 115200,
                .rx_pin = GPIO_NUM_16,
                .word_length = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_1,
            },
            .event_queue_length = 20,
            .data_queue_handle = tf_mini_data_queue
        };

        status = tf_mini_parser_init(&tf_mini_handle, &tf_mini_cfg);

        if (ESP_OK == status)
        {
            ESP_LOGI(TAG, "TF MINI parser init OK");
        }
        else
        {
            ESP_LOGE(TAG, "TF MINI parser init error (%s)", esp_err_to_name(status));
        }
    }

    if (ESP_OK == status)
    {
        status = gps_init(&gps_config, &gps_handle, &gps_event_queue);

        if (ESP_OK == status && NULL != gps_event_queue)
        {
            ESP_LOGI(TAG, "GPS event parser init OK");
        }
        else
        {
            ESP_LOGE(TAG, "GPS parser init error (%s)", esp_err_to_name(status));
        }
    }

    if (ESP_OK == status)
    {
        result = xTaskCreate(test_consumer_task, "consumer_task", 2048, NULL, 5, NULL);

        if (pdPASS == result)
        {
            ESP_LOGI(TAG, "TF mini consumer task init OK");
        }
        else
        {
            status = ESP_FAIL;
            ESP_LOGE(TAG, "TF mini consumer task init error (%s)", esp_err_to_name(status));
        }
    }
}

void test_consumer_task(void * pvParameters)
{
    BaseType_t queue_result;
    tf_mini_df_t * data_frame;
    gps_event_t gps_event;
    gps_position_t * gps_position;
    gps_speed_t * gps_speed;
    uint8_t valid_data_frames;
    char lat_str[16];
    char lon_str[16];
    char alt_str[16];

    gps_position = NULL;
    gps_speed = NULL;

    while (NULL != tf_mini_data_queue && NULL != gps_event_queue)
    {
        valid_data_frames = 0u;
        queue_result = xQueueReceive(tf_mini_data_queue, (void *) &data_frame, pdMS_TO_TICKS(100));
    
        if (pdPASS == queue_result && NULL != data_frame)
        {
            switch (data_frame->event_id)
            {
            case TF_MINI_OK:
                if (TF_MINI_DIST_MIN_CM > data_frame->distance_cm)
                {
                    ESP_LOGW(TAG, "TF mini (TOO CLOSE, dist = %hu cm)", data_frame->distance_cm);
                }
                else if (TF_MINI_DIST_MAX_CM < data_frame->distance_cm)
                {
                    ESP_LOGW(TAG, "TF mini (TOO FAR, dist = %hu cm)", data_frame->distance_cm);
                }
                else
                {
                    valid_data_frames |= DIST_SENSOR_VALID_FG;
                }
                break;
            case TF_MINI_ERR_LOW_STRENGTH:
                ESP_LOGW(TAG, "TF mini abnormal value (LOW SIGNAL STRENGTH)");
                break;
            case TF_MINI_ERR_STRENGTH_SATURATION:
                ESP_LOGW(TAG, "TF mini abnormal value (SIGNAL STRENGTH SAT)");
                break;
            case TF_MINI_ERR_AMB_LIGHT_SATURATION:
                ESP_LOGW(TAG, "TF mini abnormal value (AMB LIGHT SAT)");
                break;
            case TF_MINI_ERR_TEMPERATURE:
                ESP_LOGW(TAG, "TF mini abnormal value (TEMPERATURE)");
                break;
            default:
                ESP_LOGE(TAG, "TF mini event not handled");
                break;
            }
        }
        else
        {
            ESP_LOGE(TAG, "TF mini NO DATA");
        }

        queue_result = xQueueReceive(gps_event_queue, (void *) &gps_event, pdMS_TO_TICKS(10));

        if (pdPASS == queue_result && NULL != gps_event.data)
        {
            if (GPS_EVENT_POSITION == gps_event.id)
            {
                gps_position = (gps_position_t *) gps_event.data;

                _IQ16toa(lat_str, "%3.6f", gps_position->latitude);
                _IQ16toa(lon_str, "%5.6f", gps_position->longitude);
                _IQ18toa(alt_str, "%4.6f", gps_position->altitude);

                ESP_LOGI(
                    TAG, 
                    "GPS position { lat = %s, lon = %s, satellites = %hhu, alt = %s}", 
                    lat_str,
                    lon_str,
                    gps_position->num_satellites,
                    alt_str
                );
            }
            else if (GPS_EVENT_SPEED == gps_event.id)
            {
                gps_speed = (gps_speed_t *) gps_event.data;
                 _IQ23toa(alt_str, "%3.6f", gps_speed->ground_speed);
                ESP_LOGI(TAG, "GPS ground speed = %s", alt_str);
            }
        }

        if (DIST_SENSOR_VALID_FG & valid_data_frames)
        {
            ESP_LOGI(TAG, "TF mini DF { dist = %hu cm, strength = %hu, temp = %.2f deg C}", data_frame->distance_cm, data_frame->signal_strength, data_frame->temperature_deg_c);
        }

        if (NULL != data_frame)
        {
            free(data_frame);
            data_frame = NULL;
        }

        if (NULL != gps_position)
        {
            free(gps_position);
            gps_position = NULL;
        }

        if (NULL != gps_speed)
        {
            free(gps_speed);
            gps_speed = NULL;
        }

        data_frame = NULL;
    }

    vTaskDelete(NULL);
}
