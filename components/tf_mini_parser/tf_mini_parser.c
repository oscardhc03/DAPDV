#include "tf_mini_parser.h"

static const char * const TAG = "TF_MINI";

static const uint8_t TF_MINI_HEADER_SIZE = 2u;
static const char DATA_FRAME_HEADER_PATTERN_CHAR = 0x59;
static const uint8_t DATA_FRAME_PAYLOAD_SIZE = 7u;

// Valores anormales de distancia.
static const uint16_t LOW_STRENGTH_DISTANCE = 0xFFFFu;
static const uint16_t SAT_STRENGTH_DISTANCE = 0xFFFEu;
static const uint16_t AMB_LIGHT_SAT_DISTANCE = 0xFFFCu;

// Umbrales para identificar la calidad de la señal del sensor.
static const uint16_t LOW_STRENGTH_THRESHOLD = 100u;
static const uint16_t SAT_STRENGTH_THRESHOLD = 0xFFFFu;

// Umbrales de temperatura de operación.
static const uint16_t OP_TEMPERATURE_MIN_C = 0u;
static const uint16_t OP_TEMPERATURE_MAX_C = 70u;

typedef struct {
    uart_port_t uart_port;
    TaskHandle_t task_handle;
    QueueHandle_t event_queue;
    QueueHandle_t data_queue;
} tf_mini_t;

/**
 * Declaraciones de funciones internas del componente.
 */
static void tf_mini_parser_task(void * pvParameters);

static tf_mini_df_t * parse_tf_mini_df(const uint8_t * const header, const uint8_t * const buf);

/**
 * Definiciones de funciones públicas.
 */
esp_err_t tf_mini_parser_init(tf_mini_handle_t * const out_tf_min_handle, const tf_mini_parser_config_t * const config)
{
    esp_err_t status = ESP_OK;
    BaseType_t task_create_result;
    uart_config_t uart_config;
    tf_mini_t * tf_mini;

    if (NULL == out_tf_min_handle || NULL == config || NULL == config->data_queue_handle)
    {
        status = ESP_ERR_INVALID_ARG;
    }

    if (ESP_OK == status)
    {
        tf_mini = calloc(1, sizeof(tf_mini_t));

        if (NULL != tf_mini)
        {
            tf_mini->uart_port = config->uart.port;
            tf_mini->data_queue = config->data_queue_handle;
        }
        else
        {
            status = ESP_ERR_NO_MEM;
        }
    }

    if (ESP_OK == status)
    {
        uart_config = (uart_config_t) {
            .baud_rate = config->uart.bit_rate,
            .data_bits = config->uart.word_length,
            .parity = config->uart.parity,
            .stop_bits = config->uart.stop_bits,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };

        status = uart_param_config(config->uart.port, &uart_config);
    }

    if (ESP_OK == status)
    {
        status = uart_set_pin(config->uart.port, UART_PIN_NO_CHANGE, config->uart.rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }

    if (ESP_OK == status)
    {
        status = uart_driver_install(config->uart.port, CONFIG_TF_MINI_PARSER_BUFFER_SIZE, 0, config->event_queue_length, &tf_mini->event_queue, 0);
    }

    if (ESP_OK == status)
    {
        status = uart_enable_pattern_det_baud_intr(config->uart.port, DATA_FRAME_HEADER_PATTERN_CHAR, TF_MINI_HEADER_SIZE, 9, 0, 0);
        uart_pattern_queue_reset(config->uart.port, config->event_queue_length);
        uart_flush(config->uart.port);
    }
    else
    {
        uart_driver_delete(config->uart.port);
    }

    if (ESP_OK == status)
    {
        task_create_result = xTaskCreate(
                tf_mini_parser_task, 
                "tf_mini", 
                CONFIG_TF_MINI_PARSER_TASK_STACK_DEPTH, 
                tf_mini, 
                CONFIG_TF_MINI_PARSER_TASK_PRIORITY, 
                &tf_mini->task_handle
        );

        if (pdFAIL == task_create_result)
        {
            ESP_LOGW(TAG, "Create parser task error");
        }
    }

    return status;
}

esp_err_t tf_mini_parser_deinit(tf_mini_handle_t handle)
{
    esp_err_t status;
    tf_mini_t * tf_mini;

    if (NULL != handle)
    {
        tf_mini = (tf_mini_t *) handle;
        status = ESP_OK;
    }
    else
    {
        tf_mini = NULL;
        status = ESP_ERR_INVALID_ARG;
    }

    if (ESP_OK == status && NULL != tf_mini->task_handle)
    {
        vTaskDelete(tf_mini->task_handle);
        tf_mini->task_handle = NULL;
    }

    if (ESP_OK == status)
    {
        //TODO: Investigar si uart_driver_delete también borra la queue de FreeRTOS que usa para eventos de UART.
        status = uart_driver_delete(tf_mini->uart_port);
    }

    return status;   
}

/**
 * Tarea de FreeRTOS que recibe los datos de UART.
 * 
 * TODO: Enviar comandos por UART Tx para configurar el sensor.
 */
void tf_mini_parser_task(void * pvParameters)
{
    tf_mini_t * tf_mini;
    esp_err_t status;
    BaseType_t queue_status;
    uart_event_t event;
    uint8_t pattern_buf[TF_MINI_HEADER_SIZE + 1];
    int pattern_position;
    int num_bytes_read;
    size_t buffered_size;
    uint8_t * dataframe_buf;
    tf_mini_df_t * parsed_data_frame;

    status = ESP_OK;

    if (NULL != pvParameters)
    {
        tf_mini = (tf_mini_t *) pvParameters;
    }
    else
    {
        tf_mini = NULL;
        status = ESP_ERR_INVALID_ARG;
    }

    if (ESP_OK == status)
    {
        dataframe_buf = (uint8_t *) malloc(CONFIG_TF_MINI_PARSER_BUFFER_SIZE);

        if (NULL == dataframe_buf)
        {
            status = ESP_ERR_NO_MEM;
        }
    }
    else
    {
        dataframe_buf = NULL;
    }

    if (ESP_OK == status && (NULL == tf_mini->event_queue || NULL == tf_mini->data_queue))
    {
        status = ESP_ERR_INVALID_ARG;
    }

    while (ESP_OK == status)
    {
        queue_status = xQueueReceive(tf_mini->event_queue, (void *) &event, portMAX_DELAY);

        if (pdPASS == queue_status)
        {
            ESP_LOGD(TAG, "uart[%d] event", tf_mini->uart_port);

            switch (event.type)
            {
            case UART_DATA:
                ESP_LOGD(TAG, "UART DATA: %d", event.size);

                num_bytes_read = uart_read_bytes(tf_mini->uart_port, (void *) dataframe_buf, event.size, pdMS_TO_TICKS(100));

                if (DATA_FRAME_HEADER_PATTERN_CHAR == pattern_buf[0] && DATA_FRAME_HEADER_PATTERN_CHAR == pattern_buf[1] && DATA_FRAME_PAYLOAD_SIZE == num_bytes_read)
                {
                    parsed_data_frame = parse_tf_mini_df(pattern_buf, dataframe_buf);

                    if (NULL != parsed_data_frame)
                    {
                        queue_status = xQueueSendToBack(tf_mini->data_queue, (void *) &parsed_data_frame, (TickType_t) 0);

                        if (pdFAIL == queue_status)
                        {
                            ESP_LOGW(TAG, "send data to back of queue failed (QUEUE_FULL)");
                        }
                    }
                }
                else
                {
                    ESP_LOGW(TAG, "Dataframe format error.");
                }
                break;
            case UART_PATTERN_DET:
                uart_get_buffered_data_len(tf_mini->uart_port, &buffered_size);
                pattern_position = uart_pattern_pop_pos(tf_mini->uart_port);

                ESP_LOGD(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pattern_position, buffered_size);
                
                if (pattern_position == -1)
                {
                    uart_flush_input(tf_mini->uart_port);
                }
                else
                {   
                    num_bytes_read = uart_read_bytes(tf_mini->uart_port, (void *) dataframe_buf, buffered_size - 2, pdMS_TO_TICKS(100));
                    
                    uart_read_bytes(tf_mini->uart_port, (void *) pattern_buf, TF_MINI_HEADER_SIZE, pdMS_TO_TICKS(100));

                    ESP_LOGD(TAG, "read pattern: %hhX, %hhX", pattern_buf[0], pattern_buf[1]);
                    ESP_LOGD(TAG, "Read %d extra bytes", num_bytes_read);
                }
                break;
            default:
                ESP_LOGD(TAG, "uart event type: %d", event.type);
                break; 
            }
        }
    }

    if (NULL != dataframe_buf)
    {
        free(dataframe_buf);
    }

    dataframe_buf = NULL;

    vTaskDelete(NULL);
}

tf_mini_df_t * parse_tf_mini_df(const uint8_t * const header, const uint8_t * const buf)
{
    tf_mini_df_t * parsed_df;
    uint8_t i;
    uint8_t checksum;

    parsed_df = NULL;

    if (NULL != header && NULL != buf) 
    {
        i = DATA_FRAME_PAYLOAD_SIZE - 1;
        checksum = header[0] + header[1];

        while (i--)
        {
            checksum += buf[i];
        }

        if (buf[DATA_FRAME_PAYLOAD_SIZE - 1] == checksum)
        {
            parsed_df = (tf_mini_df_t *) malloc(sizeof(tf_mini_df_t));

            if (NULL != parsed_df)
            {
                *parsed_df = (tf_mini_df_t) {
                    .distance_cm = (((uint16_t) buf[1]) << 8u) | (uint16_t) buf[0],
                    .signal_strength = (((uint16_t) buf[3]) << 8u) | (uint16_t) buf[2],
                    .temperature_deg_c = (((((uint16_t) buf[5]) << 8u) | (uint16_t) buf[4]) / 8) - 256,
                    .event_id = TF_MINI_OK,
                };

                // Identificar si alguno de los valores es anormal.
                if (LOW_STRENGTH_DISTANCE == parsed_df->distance_cm || LOW_STRENGTH_THRESHOLD > parsed_df->signal_strength)
                {
                    // La fuerza de la señal es baja.
                    parsed_df->event_id = TF_MINI_ERR_LOW_STRENGTH;
                }
                else if (SAT_STRENGTH_DISTANCE == parsed_df->distance_cm || SAT_STRENGTH_THRESHOLD == parsed_df->signal_strength)
                {
                    // Fuerza de la señal está saturada.
                    parsed_df->event_id = TF_MINI_ERR_STRENGTH_SATURATION;
                }
                else if (AMB_LIGHT_SAT_DISTANCE == parsed_df->distance_cm)
                {
                    // Saturación a causa de la luz en el entorno.
                    parsed_df->event_id = TF_MINI_ERR_AMB_LIGHT_SATURATION;
                }
                else if (OP_TEMPERATURE_MIN_C > parsed_df->temperature_deg_c || OP_TEMPERATURE_MAX_C < parsed_df->temperature_deg_c)
                {
                    // La temperatura del sensor está fuera del rango de operación.
                    parsed_df->event_id = TF_MINI_ERR_TEMPERATURE;
                }
            }
            else
            {
                ESP_LOGW(TAG, "Dataframe parse error (NO_MEM)");
            }
        }
        else
        {
            ESP_LOGW(TAG, "Dataframe checksum is incorrect (%hhu != %hhu)", checksum, buf[DATA_FRAME_PAYLOAD_SIZE - 1]);
        }
    }

    return parsed_df;
}
