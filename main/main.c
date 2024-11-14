#include <stdio.h>

#include <freertos/FreeRTOS.h>
// #include <freertos/task.h>
#include <freertos/queue.h>

#include <esp_log.h>

#include "tf_mini_parser.h"

static const char * const TAG = "MAIN";

static const size_t TF_MINI_DATA_QUEUE_LEN = 20;
static const uint16_t TF_MINI_DIST_MIN_CM = 10u;
static const uint16_t TF_MINI_DIST_MAX_CM = 1200u;

static const uint8_t DIST_SENSOR_VALID_FG = 0x01u;

static QueueHandle_t tf_mini_data_queue;

static void test_consumer_task(void * pvParameters);

void app_main(void)
{
    esp_err_t status;
    BaseType_t result;
    tf_mini_handle_t tf_mini_handle;
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
                .rx_pin = 16,
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
    uint8_t valid_data_frames;

    while (1)
    {
        valid_data_frames = 0u;
        queue_result = xQueueReceive(tf_mini_data_queue, (void *) &data_frame, (TickType_t) pdMS_TO_TICKS(100));
    
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

        if (DIST_SENSOR_VALID_FG & valid_data_frames)
        {
            ESP_LOGI(TAG, "TF mini DF { dist = %hu cm, strength = %hu, temp = %.2f deg C}", data_frame->distance_cm, data_frame->signal_strength, data_frame->temperature_deg_c);
        }

        if (NULL != data_frame)
        {
            free(data_frame);
        }

        data_frame = NULL;
    }

    vTaskDelete(NULL);
}
