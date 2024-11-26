#include <stdio.h>

#include <esp_log.h>

#include <driver/gpio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <IQmathLib.h>

#include "tf_mini_parser.h"

static const char * const TAG = "MAIN";

static const size_t TF_MINI_DATA_QUEUE_LEN = 20;
static const _iq15 TF_MINI_DIST_MIN_M = _IQ15(0.1f);
static const _iq15 TF_MINI_DIST_MAX_M = _IQ15(12);

static const uint8_t DIST_SENSOR_VALID_FG = 0x01u;

static void distance_sensor_task(void * pvParameters);

void app_main(void)
{
    esp_err_t status;
    BaseType_t result;
    QueueHandle_t distance_sensor_data_queue;
    tf_mini_handle_t tf_mini_handle;
    tf_mini_parser_config_t tf_mini_cfg;

    status = ESP_OK;

    if (ESP_OK == status)
    {
        distance_sensor_data_queue = xQueueCreate(TF_MINI_DATA_QUEUE_LEN, sizeof(tf_mini_df_t *));

        if (NULL == distance_sensor_data_queue)
        {
            ESP_LOGW(TAG, "Create distance sensor data queue fail, no memory");
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
            .data_queue_handle = distance_sensor_data_queue
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
        result = xTaskCreate(distance_sensor_task, "consumer_task", 2048, (void *) &distance_sensor_data_queue, 5, NULL);

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

void distance_sensor_task(void * pvParameters)
{
    QueueHandle_t distance_sensor_data_queue;
    BaseType_t queue_result;
    tf_mini_df_t * data_frame;
    uint8_t valid_data_frames;

    char dist_str[12u];
    char temperature_str[12u];

    if (NULL != pvParameters)
    {
        distance_sensor_data_queue = *((QueueHandle_t *) pvParameters);
    }
    else
    {
        distance_sensor_data_queue = NULL;
    }

    while (1)
    {
        valid_data_frames = 0u;
        queue_result = xQueueReceive(distance_sensor_data_queue, (void *) &data_frame, (TickType_t) pdMS_TO_TICKS(100));
    
        if (pdPASS == queue_result && NULL != data_frame)
        {
            switch (data_frame->event_id)
            {
            case TF_MINI_OK:
                _IQ15toa(dist_str, "%2.2f", data_frame->distance_meters);
                _IQ15toa(temperature_str, "%2.1f", data_frame->temperature_deg_c);

                if (TF_MINI_DIST_MIN_M > data_frame->distance_meters)
                {
                    ESP_LOGW(TAG, "TF mini (TOO CLOSE, dist = %s m)", dist_str);
                }
                else if (TF_MINI_DIST_MAX_M < data_frame->distance_meters)
                {
                    ESP_LOGW(TAG, "TF mini (TOO FAR, dist = %s m)", dist_str);
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

        if (DIST_SENSOR_VALID_FG & valid_data_frames)
        {
            ESP_LOGI(TAG, "TF mini DF { dist = %s m, strength = %hu, temp = %s deg C}", dist_str, data_frame->signal_strength, temperature_str);
        }

        if (NULL != data_frame)
        {
            free(data_frame);
            data_frame = NULL;
        }
    }

    vTaskDelete(NULL);
}
