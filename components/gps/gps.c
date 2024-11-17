#include "gps.h"


#define NMEA_MAX_ITEM_LENGTH ((uint8_t) 16u)

typedef enum _gps_statement_t {
    GPS_STATEMENT_UNKNOWN,
    GPS_STATEMENT_GGA,
    GPS_STATEMENT_VTG,
} gps_statement_t;

typedef struct {
    uart_port_t uart_port;
    TaskHandle_t task_handle;
    QueueHandle_t event_queue;
    QueueHandle_t data_queue;
    uint8_t * line_buffer;
    uint8_t item_number;                /* <! The number of the parsed item in the statement. */
    uint8_t item_index;                 /* <! The index of the next character in the item */
    uint8_t item_str[NMEA_MAX_ITEM_LENGTH];
} gps_t;

static const char * const TAG = "GPS";

static const char NMEA_END_OF_LINE_CHARACTER = '\n';
static const uint8_t NMEA_END_OF_LINE_LENGTH = 1u;

static const char * const STATEMENT_GGA_STR = "GPGGA";
static const char * const STATEMENT_VTG_STR = "GPVTG";

static const size_t NMEA_LINE_BUFFER_SIZE = CONFIG_GPS_PARSER_BUFFER_SIZE;

static const char NMEA_START_OF_STATEMENT = '$';
static const char NMEA_ITEM_SEPARATOR = ',';
static const char NMEA_END_OF_STATEMENT = '\r';

static const float KNOTS_TO_METERS_PER_SECOND = _IQ23(1.852f);
static const float KILOMETERS_PER_SECOND_TO_METERS_PER_SECOND = _IQ23(3.6f);

/**
 * Declaraciones de funciones internas del componente.
 */
static void gps_parser_task(void * pvParameters);

static void handle_end_of_line(gps_t * const gps);
static esp_err_t gps_line_decode(gps_t * const gps);
static _iq16 parse_coordinate(const uint8_t * const item_str);

/**
 * Definiciones de funciones públicas.
 */
esp_err_t gps_init(const gps_config_t * const config, gps_handle_t * const out_gps_handle, QueueHandle_t * const out_data_queue_handle)
{
    esp_err_t status = ESP_OK;
    BaseType_t os_create_result;
    uart_config_t uart_config;
    gps_t * gps;

    if (NULL == config || NULL == out_gps_handle || NULL == out_data_queue_handle)
    {
        status = ESP_ERR_INVALID_ARG;
    }

    if (ESP_OK == status)
    {
        gps = (gps_t *) calloc(1, sizeof(gps_t));

        if (NULL != gps)
        {
            gps->uart_port = config->uart.port;
        }
        else
        {
            status = ESP_ERR_NO_MEM;
        }
    }

    if (ESP_OK == status)
    {
        gps->line_buffer = (uint8_t *) calloc(NMEA_LINE_BUFFER_SIZE, sizeof(uint8_t));

        if (NULL == gps->line_buffer)
        {
            ESP_LOGW(TAG, "Line buffer allocation error");
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
        status = uart_driver_install(config->uart.port, CONFIG_GPS_PARSER_BUFFER_SIZE, 0, config->event_queue_length, &gps->event_queue, 0);
    }

    if (ESP_OK == status)
    {
        status = uart_enable_pattern_det_baud_intr(config->uart.port, NMEA_END_OF_LINE_CHARACTER, NMEA_END_OF_LINE_LENGTH, 9, 0, 0);
        uart_pattern_queue_reset(config->uart.port, config->event_queue_length);
        uart_flush(config->uart.port);
    }
    else
    {
        uart_driver_delete(config->uart.port);
    }

    if (ESP_OK == status)
    {
        gps->data_queue = xQueueCreate(CONFIG_GPS_PARSER_EVENT_QUEUE_LENGTH, sizeof(gps_event_t));

        if (NULL == gps->data_queue)
        {
            ESP_LOGW(TAG, "Data event queue create error");
            status = ESP_ERR_NO_MEM;
        }
    }

    if (ESP_OK == status)
    {
        os_create_result = xTaskCreate(
                gps_parser_task, 
                "gps", 
                CONFIG_GPS_PARSER_TASK_STACK_DEPTH, 
                gps, 
                CONFIG_GPS_PARSER_TASK_PRIORITY, 
                &gps->task_handle
        );

        if (pdFAIL == os_create_result)
        {
            status = ESP_FAIL;
            ESP_LOGW(TAG, "Create parser task error");
        }
    }

    if (ESP_OK == status)
    {
        *out_gps_handle = (gps_handle_t) gps;
        *out_data_queue_handle = gps->data_queue;
    }

    return status;
}

esp_err_t gps_deinit(const gps_handle_t gps_handle)
{
    /*TODO: Implementar deinit. */
    return ESP_FAIL;
}

void gps_parser_task(void * pvParameters) 
{
    esp_err_t status;
    BaseType_t queue_result;
    gps_t * gps;
    uart_event_t uart_event;

    if (NULL != pvParameters)
    {
        gps = (gps_t *) pvParameters;
        status = ESP_OK;
    } 
    else
    {
        gps = NULL;
        status = ESP_ERR_INVALID_ARG;
    }

    if (ESP_OK == status)
    {
        while (1)
        {
            queue_result = xQueueReceive(gps->event_queue, (void *) &uart_event, portMAX_DELAY);

            if (pdPASS == queue_result)
            {
                ESP_LOGD(TAG, "uart[%d] event", gps->uart_port);

                switch (uart_event.type)
                {
                case UART_DATA:
                    // Intencionalmente no maneja los datos en el evento UART_DATA.
                    break;
                case UART_PATTERN_DET:
                    handle_end_of_line(gps);
                    break;
                default:
                    ESP_LOGI(TAG, "uart event type = %d", uart_event.type);
                    break;
                }
            }
        }
    }

    vTaskDelete(NULL);
}

void handle_end_of_line(gps_t * const gps)
{
    esp_err_t status;
    int pattern_position;
    int num_bytes_in_line;

    pattern_position = uart_pattern_pop_pos(gps->uart_port);

    if (-1 != pattern_position)
    {
        num_bytes_in_line = uart_read_bytes(gps->uart_port, gps->line_buffer, pattern_position + 1, pdMS_TO_TICKS(100));
        
        if (0 <= num_bytes_in_line)
        {
            gps->line_buffer[num_bytes_in_line] = '\0';

            status = gps_line_decode(gps);

            if (ESP_OK != status)
            {
                ESP_LOGW(TAG, "Decode failed, ignoring line");
            }
        }
    }
    else
    {
        ESP_LOGW(TAG, "uart pattern queue size exceeded");
        uart_flush(gps->uart_port);
    }
}

esp_err_t gps_line_decode(gps_t * const gps)
{
    esp_err_t status;
    gps_event_t gps_event;
    gps_position_t * gps_position_data;
    gps_speed_t * gps_speed_data;
    BaseType_t event_result;
    gps_statement_t statement;
    const uint8_t * cursor;

    cursor = gps->line_buffer;
    status = ESP_OK;
    statement = GPS_STATEMENT_UNKNOWN;
    gps_position_data = NULL;
    gps_speed_data = NULL;

    while (*cursor && ESP_OK == status)
    {
        if (NMEA_START_OF_STATEMENT == *cursor)
        {
            statement = GPS_STATEMENT_UNKNOWN;
            gps->item_index = 0u;
            gps->item_number = 0u;
            gps->item_str[0] = '\0';
        }
        else if (NMEA_ITEM_SEPARATOR == *cursor)
        {
            switch (statement)
            {
            case GPS_STATEMENT_UNKNOWN:
                if (0 == strcmp(STATEMENT_GGA_STR, (const char *) gps->item_str))
                {
                    statement = GPS_STATEMENT_GGA;
                    gps_position_data = (gps_position_t *) malloc(sizeof(gps_position_t));

                    if (NULL == gps_position_data)
                    {
                        ESP_LOGE(TAG, "Position data allocation error (ESP_ERR_NO_MEM)");
                        status = ESP_ERR_NO_MEM;
                    } 
                }
                else if (0 == strcmp(STATEMENT_VTG_STR, (const char *) gps->item_str))
                {
                    statement = GPS_STATEMENT_VTG;
                    gps_speed_data = (gps_speed_t *) malloc(sizeof(gps_speed_t));

                    if (NULL == gps_speed_data)
                    {
                        ESP_LOGE(TAG, "Speed data allocation error (ESP_ERR_NO_MEM)");
                        status = ESP_ERR_NO_MEM;
                    } 
                }
                else 
                {
                    // Ignorar todas las demás sentencias.
                    return status;
                }
                break;
            case GPS_STATEMENT_GGA:
                switch (gps->item_number)
                {
                case 2:
                    gps_position_data->latitude = parse_coordinate(gps->item_str);
                    break;
                case 3:
                    if ('S' == gps->item_str[0] || 's' == gps->item_str[0])
                    {
                        gps_position_data->latitude = _IQ16mpy(gps_position_data->latitude, _IQ16(-1));
                    }
                    break;
                case 4:
                    gps_position_data->longitude = parse_coordinate(gps->item_str);
                    break;
                case 5:
                    if ('W' == gps->item_str[0] || 'w' == gps->item_str[0])
                    {
                        gps_position_data->longitude = _IQ16mpy(gps_position_data->longitude, _IQ16(-1));
                    }
                    break;
                case 7:
                    gps_position_data->num_satellites = (uint8_t) strtol((const char *) gps->item_str, NULL, 10);
                    break;
                case 9:
                    gps_position_data->altitude = _atoIQ18((const char *) gps->item_str);
                    break;
                default:
                    break;
                }
                break;
            case GPS_STATEMENT_VTG:
                switch (gps->item_number)
                {
                case 5:
                    gps_speed_data->ground_speed = _IQ23div(_atoIQ23((const char *) gps->item_str), KNOTS_TO_METERS_PER_SECOND);
                    break;
                case 7:
                    gps_speed_data->ground_speed = _IQ23div(_atoIQ23((const char *) gps->item_str), KILOMETERS_PER_SECOND_TO_METERS_PER_SECOND);
                    break;
                default:
                    break;
                }
                break;
            }

            gps->item_index = 0u;
            gps->item_str[0] = '\0';
            gps->item_number++;
        }
        else if (NMEA_END_OF_STATEMENT == *cursor)
        {
            gps_event.id = GPS_EVENT_NONE;
            if (GPS_STATEMENT_GGA == statement && NULL != gps_position_data)
            {
                gps_event.id = GPS_EVENT_POSITION;
                gps_event.data = (void *) gps_position_data;
            }
            else if (GPS_STATEMENT_VTG == statement && NULL != gps_speed_data)
            {
                gps_event.id = GPS_EVENT_SPEED;
                gps_event.data = (void *) gps_speed_data;
            }

            if (GPS_EVENT_NONE != gps_event.id)
            {
                // Send GPS data event.
                event_result = xQueueSendToBack(gps->data_queue, (void *) &gps_event, pdMS_TO_TICKS(10));

                if (pdFAIL == event_result)
                {
                    ESP_LOGW(TAG, "Send data event to queue fail (QUEUE_FULL)");
                    status = ESP_FAIL;
                }
            }
        }
        else
        {
            gps->item_str[gps->item_index] = *cursor;
            gps->item_index++;
            gps->item_str[gps->item_index] = '\0';
        }

        cursor++;
    }

    return status;
}

_iq16 parse_coordinate(const uint8_t * const item_str)
{
    _iq16 coordinate;
    _iq16 degrees;
    _iq16 minutes;

    ESP_LOGI(TAG, "%s", item_str);
    coordinate = _atoIQ16((char *) item_str);
    degrees = _IQ16(_IQ16int(_IQ16div(coordinate, _IQ16(100.0f))));
    minutes = coordinate - _IQ16mpy(degrees, _IQ16(100.0f));
    coordinate = degrees + _IQ16div(minutes, _IQ16(60.0f)); // 20.666657, 103.422001

    return coordinate;
}
