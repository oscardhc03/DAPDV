#include <stdio.h>

#include <esp_err.h>
#include <esp_log.h>

#include <driver/gpio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <IQmathLib.h>

#include "hmi.h"
#include "tf_mini_parser.h"

typedef struct _proximity_range_update_t {
    int8_t index_change;
    _iq15 distance_meters;
} proximity_range_update_t;

static const char * const TAG = "MAIN";

static const UBaseType_t TF_MINI_DATA_QUEUE_LEN = 20;

static const BaseType_t PROXIMITY_SENSOR_ERROR = LONG_MAX;
static const UBaseType_t PROXIMITY_UPDATE_QUEUE_LEN = 3;
static const int8_t PROXIMITY_RANGE_INTERVAL_COUNT = 9;

static tf_mini_parser_config_t distance_sensor_config = (tf_mini_parser_config_t) {
    .uart = {
        .port = UART_NUM_2,
        .bit_rate = 115200,
        .rx_pin = GPIO_NUM_16,
        .word_length = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
    },
    .event_queue_length = 20,
    .data_queue_handle = NULL,
};

static void distance_sensor_task(void * pvParameters);
static void proximity_task(void * pvParameters);
static void hmi_task(void * pvParameters);

static esp_err_t feedback_event_send(feedback_event_id_t event_id, feedback_source_t source, feedback_priority_t priority, TickType_t timeout_ticks);

static QueueHandle_t feedback_event_queue;

void app_main(void)
{
    esp_err_t status;
    BaseType_t result;
    QueueHandle_t proximity_update_queue;

    status = ESP_OK;

    if (ESP_OK == status)
    {
        feedback_event_queue = xQueueCreate(CONFIG_HMI_FEEDBACK_EVENT_QUEUE_LENGTH, sizeof(feedback_event_t *));

        if (NULL == feedback_event_queue)
        {
            status = ESP_ERR_NO_MEM;
            ESP_LOGW(TAG, "Create feedback event queue fail (%s)", esp_err_to_name(status));
        }
    }

    if (ESP_OK == status)
    {
        proximity_update_queue = xQueueCreate(PROXIMITY_UPDATE_QUEUE_LEN, sizeof(proximity_range_update_t *));

        if (NULL == proximity_update_queue)
        {
            status = ESP_ERR_NO_MEM;
            ESP_LOGW(TAG, "Create proximity update queue fail (%s)", esp_err_to_name(status));
        }
    }

    if (ESP_OK == status)
    {
        result = xTaskCreate(proximity_task, "proximity_task", 2048, (void *) proximity_update_queue, 5, NULL);

        if (pdPASS != result)
        {
            status = ESP_FAIL;
            ESP_LOGE(TAG, "Proximity task init error (%s)", esp_err_to_name(status));
        }
    }

    if (ESP_OK == status)
    {
        result = xTaskCreate(distance_sensor_task, "dist_sensor_task", 2048, (void *) proximity_update_queue, 5, NULL);

        if (pdPASS != result)
        {
            status = ESP_FAIL;
            ESP_LOGE(TAG, "Distance sensor task init error (%s)", esp_err_to_name(status));
        }
    }

    if (ESP_OK == status)
    {
        result = xTaskCreate(hmi_task, "hmi_task", 2048, NULL, 6, NULL);

        if (pdPASS != result)
        {
            status = ESP_FAIL;
            ESP_LOGE(TAG, "HMI feedback task init error (%s)", esp_err_to_name(status));
        }
    }

    if (ESP_OK == status)
    {
        ESP_LOGI(TAG, "Initialization complete");
    }
    else
    {
        ESP_LOGE(TAG, "Main initialization error (%s)", esp_err_to_name(status));
    }
}

void hmi_task(void * pvParameters)
{
    static const TickType_t FEEDBACK_RECEIVE_TIMEOUT_TICKS = pdMS_TO_TICKS(1000u);

    BaseType_t result;
    feedback_event_t * feedback_event;

    if (NULL == feedback_event_queue)
    {
        ESP_LOGE(TAG, "HMI task feedback event queue not initialized");
        vTaskDelete(NULL);
    }

    while (1)
    {
        result = xQueueReceive(feedback_event_queue, (void * const) &feedback_event, FEEDBACK_RECEIVE_TIMEOUT_TICKS);

        if (pdTRUE == result)
        {
            if (NULL != feedback_event)
            {
                ESP_LOGI(TAG, "HMI feedback event - ID = %hhu, src = %hhu, priority = %hhu", (uint8_t) feedback_event->id, (uint8_t) feedback_event->source, (uint8_t) feedback_event->priority);

                //TODO: Drive feedback device according to feedback event.

                free(feedback_event);
                feedback_event = NULL;
            }
            else
            {
                ESP_LOGW(TAG, "HMI task received feedback event with no payload");
            }
        }
        else
        {
            ESP_LOGD(TAG, "HMI task received no feedback events");
        }
    }

    vTaskDelete(NULL);
}

void proximity_task(void * pvParameters)
{
    static const TickType_t RECEIVE_RANGE_UPDATE_TIMEOUT = pdMS_TO_TICKS(100);
    static const TickType_t SEND_FEEDBACK_EVENT_TIMEOUT_TICKS = pdMS_TO_TICKS(10);
    static const _iq15 FAR_COLLISION_ALERT_DISTANCE_M = _IQ15(1.0f);
    static const _iq15 NEAR_COLLISION_ALERT_DISTANCE_M = _IQ15(0.5f);

    QueueHandle_t proximity_range_update_queue;
    proximity_range_update_t * proximity_range_update;

    int8_t cursor;
    _iq15 * proximity_range;

    esp_err_t status;
    BaseType_t queue_result;
    UBaseType_t notify_level;

    status = ESP_OK;
    cursor = 4u;

    if (NULL != pvParameters)
    {
        proximity_range_update_queue = (QueueHandle_t) pvParameters;
    }
    else
    {
        proximity_range_update_queue = NULL;
        status = ESP_ERR_INVALID_ARG;
    }

    if (ESP_OK == status)
    {
        proximity_range = (_iq15 *) malloc(PROXIMITY_RANGE_INTERVAL_COUNT * sizeof(_iq15));

        if (NULL == proximity_range)
        {
            status = ESP_ERR_NO_MEM;
        }
    }

    if (ESP_OK != status)
    {
        ESP_LOGE(TAG, "Proximity task initialization failed (%s)", esp_err_to_name(status));
        vTaskDelete(NULL);
    }

    notify_level = 0u;

    while (1)
    {
        queue_result = xQueueReceive(proximity_range_update_queue, (void *) &proximity_range_update, RECEIVE_RANGE_UPDATE_TIMEOUT);

        if (pdPASS == queue_result && NULL != proximity_range_update)
        {
            cursor += proximity_range_update->index_change;

            if (0 > cursor)
            {
                cursor = 0;
            }
            else if (PROXIMITY_RANGE_INTERVAL_COUNT <= cursor)
            {
                cursor = PROXIMITY_RANGE_INTERVAL_COUNT - 1;
            }

            proximity_range[cursor] = proximity_range_update->distance_meters;

            free(proximity_range_update);
            proximity_range_update = NULL;

            if (PROXIMITY_SENSOR_ERROR != proximity_range[cursor])
            {
                if (FAR_COLLISION_ALERT_DISTANCE_M >= proximity_range[cursor])
                {
                    if (NEAR_COLLISION_ALERT_DISTANCE_M >= proximity_range[cursor])
                    {
                        if (2u > notify_level)
                        {
                            status = feedback_event_send(FEEDBACK_EVENT_STOP, FEEDBACK_SOURCE_PROXIMITY, FEEDBACK_PRIORITY_HIGH, SEND_FEEDBACK_EVENT_TIMEOUT_TICKS);

                            if (ESP_OK == status)
                            {
                                notify_level = 2u;
                            }
                            else
                            {
                                ESP_LOGW(TAG, "Send feedback event fail (%s)", esp_err_to_name(status));
                            }
                        }
                    }
                    else if (2u == notify_level)
                    {
                        notify_level = 1u;
                    }
                    else if (1u > notify_level)
                    {
                        status = feedback_event_send(FEEDBACK_EVENT_STOP, FEEDBACK_SOURCE_PROXIMITY, FEEDBACK_PRIORITY_NORMAL, SEND_FEEDBACK_EVENT_TIMEOUT_TICKS);

                        if (ESP_OK == status)
                        {
                            notify_level = 1u;
                        }
                        else
                        {
                            ESP_LOGW(TAG, "Send feedback event fail (%s)", esp_err_to_name(status));
                        }
                    }
                }
                else
                {
                    notify_level = 0u;
                }
            }
            else
            {
                status = feedback_event_send(FEEDBACK_EVENT_SENSOR_ERROR, FEEDBACK_SOURCE_PROXIMITY, FEEDBACK_PRIORITY_NORMAL, SEND_FEEDBACK_EVENT_TIMEOUT_TICKS);

                if (ESP_OK != status)
                {
                    ESP_LOGW(TAG, "Send feedback event fail (%s)", esp_err_to_name(status));
                }
            }
        }
    }

    vTaskDelete(NULL);
}

void distance_sensor_task(void * pvParameters)
{
    static const TickType_t SEND_PROXIMITY_UPDATE_TIMEOUT_TICKS = pdMS_TO_TICKS(10);
    static const _iq15 TF_MINI_DIST_MIN_M = _IQ15(0.1f);
    static const _iq15 TF_MINI_DIST_MAX_M = _IQ15(12);

    static const uint8_t DIST_SENSOR_VALID_FG = 0x01u;

    esp_err_t status;
    BaseType_t queue_result;

    QueueHandle_t distance_sensor_data_queue;
    QueueHandle_t proximity_range_update_queue;

    tf_mini_handle_t tf_mini_handle;
    tf_mini_df_t * data_frame;
    _iq15 previous_distance_meters;
    uint8_t valid_data_frames;
    uint8_t previous_valid_data_frames;

    proximity_range_update_t * proximity_range_update;
    
    char dist_str[12u];
    char temperature_str[12u];

    status = ESP_OK;

    if (NULL != pvParameters)
    {
        proximity_range_update_queue = (QueueHandle_t) pvParameters;
    }
    else
    {
        status = ESP_ERR_INVALID_ARG;
        proximity_range_update_queue = NULL;
    }

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
        distance_sensor_config.data_queue_handle = distance_sensor_data_queue;
        status = tf_mini_parser_init(&tf_mini_handle, &distance_sensor_config);
    }

    if (ESP_OK != status)
    {
        ESP_LOGE(TAG, "Distance sensor task initialization failed (%s)", esp_err_to_name(status));
        vTaskDelete(NULL);
    }

    previous_distance_meters = _IQ15(0);
    previous_valid_data_frames = 0u;

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

        if (NULL != data_frame)
        {
            if (DIST_SENSOR_VALID_FG & valid_data_frames)
            {
                ESP_LOGD(TAG, "TF mini DF { dist = %s m, strength = %hu, temp = %s deg C}", dist_str, data_frame->signal_strength, temperature_str);
            }

            //TODO: Only send update when one of these conditions is met: 1) change of distance 2) change of angle 3) sensor error.
            if (previous_distance_meters != data_frame->distance_meters || previous_valid_data_frames != valid_data_frames)
            {
                proximity_range_update = (proximity_range_update_t *) malloc(sizeof(proximity_range_update_t));

                if (NULL != proximity_range_update)
                {
                    proximity_range_update->index_change = 0;

                    if (DIST_SENSOR_VALID_FG & valid_data_frames)
                    {
                        proximity_range_update->distance_meters = data_frame->distance_meters;
                    }
                    else
                    {
                        proximity_range_update->distance_meters = (_iq15) PROXIMITY_SENSOR_ERROR;
                    }

                    queue_result = xQueueSendToBack(proximity_range_update_queue, (void *) &proximity_range_update, SEND_PROXIMITY_UPDATE_TIMEOUT_TICKS);
                    proximity_range_update = NULL;

                    if (pdPASS == queue_result)
                    {
                        previous_distance_meters = data_frame->distance_meters;
                        previous_valid_data_frames = valid_data_frames;
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Send proximity update fail (QUEUE_FULL)");
                    }
                }
                else
                {
                    ESP_LOGW(TAG,"Proximity range update allocation error (ESP_ERR_NO_MEM)");
                }
            }
        }

        if (NULL != data_frame)
        {
            free(data_frame);
            data_frame = NULL;
        }
    }

    vTaskDelete(NULL);
}

esp_err_t feedback_event_send(feedback_event_id_t event_id, feedback_source_t source, feedback_priority_t priority, TickType_t timeout_ticks)
{
    esp_err_t status;
    BaseType_t result;
    feedback_event_t * feedback_event;

    if (NULL != feedback_event_queue)
    {
        status = ESP_OK;
    }
    else
    {
        status = ESP_ERR_INVALID_STATE;
    }

    if (status == ESP_OK)
    {
        feedback_event = (feedback_event_t *) malloc(sizeof(feedback_event_t));

        if (NULL == feedback_event)
        {
            status = ESP_ERR_NO_MEM;
        }
    }

    if (ESP_OK == status)
    {
        *feedback_event = (feedback_event_t) {
            .id = event_id,
            .source = source,
            .priority = priority,
        };

        result = xQueueSendToBack(feedback_event_queue, (void *) &feedback_event, timeout_ticks);

        if (pdFAIL == result)
        {
            status = ESP_FAIL;
        }
    }

    return status;
}
