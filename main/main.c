#include <stdio.h>
#include <math.h>

#include <esp_log.h>

#include <driver/gpio.h>

#include <freertos/FreeRTOS.h>
// #include <freertos/task.h>
#include <freertos/queue.h>

#include "gps.h"
#include "tf_mini_parser.h"

#define GPS_WAYPOINT_NO_ROUTE_INDEX         ((uint8_t) 0u)
#define GPS_WAYPOINT_LOAD_NEXT_ROUTE_FLAG   ((uint32_t) 0x01u)

#define NAV_NOTIFY_BEFORE_WAYPOINT          ((uint8_t) 0x01u)
#define NAV_NOTIFY_OUT_OF_BOUNDS            ((uint8_t) 0x02u)

#define FEEDBACK_EVENT_PRIORITY_LOW         ((uint8_t) 0u)
#define FEEDBACK_EVENT_PRIORITY_NORMAL      ((uint8_t) 1u)
#define FEEDBACK_EVENT_PRIORITY_HIGH        ((uint8_t) 2u)

typedef enum _feedback_event_id_t {
    FEEDBACK_EVENT_NONE,
    FEEDBACK_EVENT_STRAIGHT,
    FEEDBACK_EVENT_LEFT,
    FEEDBACK_EVENT_RIGHT,
    FEEDBACK_EVENT_U_TURN,
    FEEDBACK_EVENT_STOP,
    FEEDBACK_EVENT_SENSOR_ERROR,
    FEEDBACK_EVENT_LOW_BATTERY,
    FEEDBACK_EVENT_PWR_SAVE_ENTER,
    FEEDBACK_EVENT_PWR_SAVE_EXIT,
    FEEDBACK_EVENT_MAX,
} feedback_event_id_t;

typedef enum _feedback_source_t {
    FEEDBACK_SOURCE_PROXIMITY,
    FEEDBACK_SOURCE_NAVIGATION,
    FEEDBACK_SOURCE_OBJECT_DETECT,
    FEEDBACK_SOURCE_MAX,
} feedback_source_t;

typedef struct _feedback_event_t {
    feedback_event_id_t id;
    feedback_source_t source;
    uint8_t priority;
} feedback_event_t;

static const char * const TAG = "MAIN";

/* Proximity sensor configuration. */
static const size_t TF_MINI_DATA_QUEUE_LEN = 20;
static const uint16_t TF_MINI_DIST_MIN_CM = 10u;
static const uint16_t TF_MINI_DIST_MAX_CM = 1200u;

static const uint8_t DIST_SENSOR_VALID_FG = 0x01u;

static const float SAFE_AREA_RADIUS_M = ((float) CONFIG_NAV_SAFE_AREA_RADIUS_CM) / 100.0f;

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

static QueueHandle_t feedback_event_queue;

static QueueHandle_t tf_mini_data_queue;
static QueueHandle_t gps_event_queue;

static TaskHandle_t gps_waypoint_loader_handle;


static void test_consumer_task(void * pvParameters);
static void gps_waypoint_loader_task(void * pvParameters);
static void gps_navigation_task(void * pvParameters);
static void hmi_task(void * pvParameters);

static feedback_event_id_t next_nav_feedback(const gps_position_t * const current_waypoint, const gps_position_t * const next_waypoint, float course_over_ground);

/**
 * FreeRTOS tasks.
 */
void app_main(void)
{
    esp_err_t status;
    BaseType_t result;
    tf_mini_handle_t tf_mini_handle;
    gps_handle_t gps_handle;
    tf_mini_parser_config_t tf_mini_cfg;
    TaskHandle_t gps_navigation_task_handle;

    status = ESP_OK;

    if (ESP_OK == status)
    {
        feedback_event_queue = xQueueCreate(CONFIG_FEEDBACK_EVENT_QUEUE_LENGTH, sizeof(feedback_event_t *));

        if (NULL == feedback_event_queue)
        {
            ESP_LOGW(TAG, "Create feedback event queue fail, not enough free memory.");
            status = ESP_ERR_NO_MEM;
        }
    }

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
        result = xTaskCreate(hmi_task, "hmi_task", 2048, NULL, 6, NULL);

        if (pdPASS != result)
        {
            status = ESP_FAIL;
            ESP_LOGE(TAG, "HMI task init error (%s)", esp_err_to_name(status));
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

    if (ESP_OK == status)
    {
        result = xTaskCreate(gps_navigation_task, "gps_nav", 2048, NULL, 3, &gps_navigation_task_handle);

        if (pdPASS == result)
        {
            ESP_LOGI(TAG, "GPS nav task init OK");
        }
        else
        {
            status = ESP_FAIL;
            ESP_LOGE(TAG, "GPS navigation task init error (%s)", esp_err_to_name(status));
        }
    }

    if (ESP_OK == status)
    {
        result = xTaskCreate(gps_waypoint_loader_task, "waypoint_ld", 2048, (void *) gps_navigation_task_handle, 2u, &gps_waypoint_loader_handle);

        if (pdPASS == result)
        {
            ESP_LOGI(TAG, "Waypoint task init OK");
        }
        else
        {
            status = ESP_FAIL;
            ESP_LOGE(TAG, "Waypoint task init error (%s)", esp_err_to_name(status));
        }
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

void test_consumer_task(void * pvParameters)
{
    BaseType_t queue_result;
    tf_mini_df_t * data_frame;
    uint8_t valid_data_frames;

    while (NULL != tf_mini_data_queue)
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

        if (DIST_SENSOR_VALID_FG & valid_data_frames)
        {
            ESP_LOGI(TAG, "TF mini DF { dist = %hu cm, strength = %hu, temp = %.2f deg C}", data_frame->distance_cm, data_frame->signal_strength, data_frame->temperature_deg_c);
        }

        if (NULL != data_frame)
        {
            free(data_frame);
            data_frame = NULL;
        }
    }

    vTaskDelete(NULL);
}

void gps_waypoint_loader_task(void * pvParameters)
{
    uint32_t notify_value;
    BaseType_t did_notify_route_change;
    uint8_t current_route_index;
    uint8_t route_count;
    TaskHandle_t gps_navigation_task_handle;

    gps_route_t * route;

    current_route_index = GPS_WAYPOINT_NO_ROUTE_INDEX;
    route_count = 1;

    // Obtener la referencia a la tarea de navegación GPS.
    gps_navigation_task_handle = NULL;
    if (NULL != pvParameters)
    {
        gps_navigation_task_handle = *((TaskHandle_t *) pvParameters);
    }

    //TODO: cargar los waypoints reales, quitar este placeholder.
    // INICIO PLACEHOLDER
    // route = (gps_route_t) {
    //     .waypoints = NULL,
    //     .waypoint_count = 0UL,
    // };

    route = (gps_route_t *) malloc(sizeof(gps_route_t));

    if (NULL != route)
    {
        route->waypoints = (gps_position_t *) malloc(3 * sizeof(gps_position_t));
        route->waypoint_count = 3UL;

        if (NULL != route->waypoints)
        {
            route->waypoints[0] = (gps_position_t) {
                .latitude = 0.0f,
                .longitude = 0.0f,
                .altitude = 1000u,
            };
        }
    }

    if (NULL == route || NULL == route->waypoints)
    {
        ESP_LOGE(TAG, "Not enough free memory for waypoints");
        vTaskDelete(NULL);
    }
    // FIN PLACEHOLDER

    while (1)
    {
        did_notify_route_change = xTaskNotifyWait(0UL, ULONG_MAX, &notify_value, pdMS_TO_TICKS(10000));

        if (did_notify_route_change && 0 != (GPS_WAYPOINT_LOAD_NEXT_ROUTE_FLAG & notify_value))
        {
            current_route_index += 1u;
            if (route_count <= current_route_index)
            {
                current_route_index = GPS_WAYPOINT_NO_ROUTE_INDEX;
            }
            
            ESP_LOGI(TAG, "GPS waypoint loader route changed to %hhu", current_route_index);

            if (GPS_WAYPOINT_NO_ROUTE_INDEX != current_route_index)
            {
                //TODO: Cargar los waypoints de la ruta seleccionada.
            }

            // Notificar a la tarea de navegación GPS de la nueva ruta.
            if (NULL != gps_navigation_task_handle)
            {
                xTaskNotify(gps_navigation_task_handle, (uint32_t) route, eSetValueWithOverwrite);
                ESP_LOGI(TAG, "Notified GPS navigation task with new route");
            }
        }
        else
        {
            ESP_LOGI(TAG, "GPS waypoint loader route unchanged.");
        }
    }

    vTaskDelete(NULL);
}

void gps_navigation_task(void * pvParameters)
{
    static const TickType_t NAV_FEEDBACK_EVT_POST_TIMEOUT_TICKS = pdMS_TO_TICKS(CONFIG_NAV_FEEDBACK_POST_TIMEOUT_MS);

    BaseType_t result;
    uint8_t notification_state;

    const gps_route_t * route;
    gps_event_t gps_event;
    gps_positioning_data_t * gps_positioning;
    gps_speed_t * gps_speed;

    uint32_t index_of_current_waypoint;
    uint32_t index_of_closest_waypoint;
    
    float distance_to_position;
    float distance_to_closest_waypoint;
    gps_position_t * intersect_position;

    feedback_event_t * nav_feedback_evt;

    uint32_t i;

    gps_positioning = NULL;
    gps_speed = NULL;
    index_of_current_waypoint = ULONG_MAX;
    notification_state = 0u;

    while (NULL != gps_event_queue)
    {
        result = xTaskNotifyWait(0UL, ULONG_MAX, (uint32_t *) &route, (TickType_t) 0);

        if (pdPASS == result && NULL != route)
        {
            // Loaded waypoints for a new route.
            index_of_current_waypoint = ULONG_MAX;
            notification_state = NAV_NOTIFY_OUT_OF_BOUNDS;

            ESP_LOGI(TAG, "Navigation: new route");
        }

        result = xQueueReceive(gps_event_queue, (void *) &gps_event, pdMS_TO_TICKS(1000));

        if (pdPASS == result && NULL != gps_event.data)
        {
            if (GPS_EVENT_POSITION == gps_event.id)
            {
                if (NULL != gps_positioning)
                {
                    free(gps_positioning);
                    gps_positioning = NULL;
                }

                gps_positioning = (gps_positioning_data_t *) gps_event.data;\
            }
            else if (GPS_EVENT_SPEED == gps_event.id)
            {
                if (NULL != gps_speed)
                {
                    free(gps_speed);
                    gps_speed = NULL;
                }

                gps_speed = (gps_speed_t *) gps_event.data;
            }
        }
        else
        {
            gps_event.id = GPS_EVENT_NONE;
            gps_event.data = NULL;
            ESP_LOGW(TAG, "GPS no data event");
        }

        if (GPS_EVENT_NONE != gps_event.id)
        {
            ESP_LOGI(
                TAG, 
                "GPS position { lat = %.6f, lon = %.6f, satellites = %hhu, alt = %hu m}", 
                gps_positioning->position.latitude,
                gps_positioning->position.longitude,
                gps_positioning->num_satellites,
                gps_positioning->position.altitude
            );
            ESP_LOGI(TAG, "GPS ground course = %.1f, speed = %.3f", gps_speed->course_degrees, gps_speed->ground_speed);

            // Si hay una ruta, evaluar si posición está en el área y guiar al siguiente waypoint.
            if (NULL != route && 0u != route->waypoint_count)
            {
                // Encontrar los dos waypoints más cercanos, si los hay.
                index_of_closest_waypoint = ULONG_MAX;
                distance_to_closest_waypoint = -1.0f;
                
                i = route->waypoint_count;
                while (i--)
                {
                    distance_to_position = ((gps_positioning->position.latitude - route->waypoints[i].latitude) * (gps_positioning->position.latitude - route->waypoints[i].latitude)) + 
                                                ((gps_positioning->position.longitude - route->waypoints[i].longitude) * (gps_positioning->position.longitude - route->waypoints[i].longitude)); 

                    if (distance_to_position < distance_to_closest_waypoint || 0.0f > distance_to_closest_waypoint)
                    {
                        index_of_closest_waypoint = i;
                        distance_to_closest_waypoint = distance_to_position;
                    }
                }

                if (ULONG_MAX == index_of_current_waypoint && ULONG_MAX != index_of_closest_waypoint)
                {
                    // Si la ruta es nueva, comenzar a guiar hacia el punto más cercano al usuario.
                    index_of_current_waypoint = index_of_closest_waypoint;
                }

                if (ULONG_MAX != index_of_current_waypoint)
                {
                    if (0u != index_of_current_waypoint)
                    {
                        // Puede determinar si está dentro del área delimitada por 2 waypoints.
                        get_position_intersecting_line(
                            &gps_positioning->position, 
                            &(route->waypoints[index_of_current_waypoint]),
                            &(route->waypoints[index_of_current_waypoint - 1]),
                            intersect_position
                        );

                        distance_to_position = distance_in_meters_between_positions(&gps_positioning->position, intersect_position);

                        ESP_LOGI(TAG, "Distance between position and waypoint line (%lu - %lu) = %.1f m", index_of_current_waypoint - 1, index_of_current_waypoint, distance_to_position);
                    }
                    else
                    {
                        // Sólo puede evaluar si está cerca del waypoint más cercano.
                        distance_to_position = distance_in_meters_between_positions(&gps_positioning->position, &(route->waypoints[index_of_closest_waypoint]));

                        ESP_LOGI(TAG, "Distance between position and closest waypoint (%lu) = %.1f m", index_of_closest_waypoint, distance_to_position);
                    }

                    if (SAFE_AREA_RADIUS_M < distance_to_position)
                    {
                        ESP_LOGI(TAG, "Position is out of bounds");
                        if (0u == (NAV_NOTIFY_OUT_OF_BOUNDS & notification_state))
                        {
                            notification_state |= NAV_NOTIFY_OUT_OF_BOUNDS;
                            nav_feedback_evt = (feedback_event_t *) malloc(sizeof(feedback_event_t));

                            if (NULL != nav_feedback_evt)
                            {
                                *nav_feedback_evt = (feedback_event_t) {
                                    .id = FEEDBACK_EVENT_STOP,
                                    .source = FEEDBACK_SOURCE_NAVIGATION,
                                    .priority = FEEDBACK_EVENT_PRIORITY_NORMAL
                                };

                                result = xQueueSendToBack(feedback_event_queue, (void *) &nav_feedback_evt, NAV_FEEDBACK_EVT_POST_TIMEOUT_TICKS);

                                if (pdFALSE == result)
                                {
                                    free(nav_feedback_evt);
                                    ESP_LOGI(TAG, "Navigation feedback send fail (QUEUE_FULL)");
                                }

                                nav_feedback_evt = NULL;
                            }
                            else
                            {
                                ESP_LOGW(TAG, "Navigation feedback event allocation error (ESP_ERR_NO_MEM)");
                            }
                        }
                    }
                    else
                    {
                        notification_state &= ~notification_state;
                    }

                    // Navigation when route isn't already complete.
                    if (route->waypoint_count != index_of_current_waypoint)
                    {
                        distance_to_position = distance_in_meters_between_positions(&gps_positioning->position, &(route->waypoints[index_of_current_waypoint]));
                        ESP_LOGI(TAG, "Distance to next waypoint = %.2f m", distance_to_position);

                        if (route->waypoint_count > index_of_current_waypoint + 1u)
                        {
                            if (CONFIG_NAV_WAYPOINT_VISIT_DISTANCE_CM > distance_to_position)
                            {
                                notification_state &= ~NAV_NOTIFY_BEFORE_WAYPOINT;
                                ESP_LOGI(TAG, "Reached waypoint %lu", index_of_current_waypoint);

                                nav_feedback_evt = (feedback_event_t *) malloc(sizeof(feedback_event_t));

                                if (NULL != nav_feedback_evt)
                                {
                                    nav_feedback_evt->id = next_nav_feedback(
                                        &(route->waypoints[index_of_current_waypoint]), 
                                        &(route->waypoints[index_of_current_waypoint + 1u]),
                                        gps_speed->course_degrees
                                    );   
                                }
                                else
                                {
                                    ESP_LOGW(TAG, "Navigation feedback event allocation error (ESP_ERR_NO_MEM)");
                                }

                                index_of_current_waypoint++;
                            }
                            else if (CONFIG_NAV_NOTIFY_BEFORE_WAYPOINT_DISTANCE_CM > distance_to_position && 0u == (NAV_NOTIFY_BEFORE_WAYPOINT & notification_state))
                            {
                                nav_feedback_evt = (feedback_event_t *) malloc(sizeof(feedback_event_t));

                                if (NULL != nav_feedback_evt)
                                {
                                    nav_feedback_evt->id = next_nav_feedback(
                                        &(route->waypoints[index_of_current_waypoint]), 
                                        &(route->waypoints[index_of_current_waypoint + 1u]),
                                        gps_speed->course_degrees
                                    );   
                                }
                                else
                                {
                                    ESP_LOGW(TAG, "Navigation feedback event allocation error (ESP_ERR_NO_MEM)");
                                }

                                notification_state |= NAV_NOTIFY_BEFORE_WAYPOINT;                            
                            }

                            if (NULL != nav_feedback_evt)
                            {
                                // Hay un evento de feedback de navegación pendiente.
                                nav_feedback_evt->source = FEEDBACK_SOURCE_NAVIGATION;
                                nav_feedback_evt->priority = FEEDBACK_EVENT_PRIORITY_LOW;
                                result = xQueueSendToBack(feedback_event_queue, (void *) &nav_feedback_evt, NAV_FEEDBACK_EVT_POST_TIMEOUT_TICKS);

                                if (pdFALSE == result)
                                {
                                    free(nav_feedback_evt);
                                    ESP_LOGW(TAG, "Navigation feedback send fail (QUEUE_FULL)");
                                }

                                nav_feedback_evt = NULL;
                            }
                        }
                    }
                }
            }
        }
    }

    vTaskDelete(NULL);
}

/**
 * Helper functions.
 */
feedback_event_id_t next_nav_feedback(const gps_position_t * const current_waypoint, const gps_position_t * const next_waypoint, float course_over_ground)
{
    feedback_event_id_t feedback_event;
    float target_heading;

    feedback_event = FEEDBACK_EVENT_NONE;

    if (fabsf(current_waypoint->latitude - next_waypoint->latitude) > fabsf(current_waypoint->longitude - next_waypoint->longitude))
    {
        if (current_waypoint->latitude < next_waypoint->latitude)
        {
            target_heading = 0.0f;
        }
        else
        {
            target_heading = 180.0f;
        }
    }
    else
    {
        if (current_waypoint->longitude < next_waypoint->longitude)
        {
            target_heading = 90.0f;
        }
        else
        {
            target_heading = 270.0f;
        }
    }

    target_heading -= course_over_ground;

    if (0.0f == target_heading)
    {
        feedback_event = FEEDBACK_EVENT_STRAIGHT;
    }
    else if (-90.0f == target_heading)
    {
        feedback_event = FEEDBACK_EVENT_LEFT;
    }
    else if (90.0f == target_heading)
    {
        feedback_event = FEEDBACK_EVENT_RIGHT;
    }
    else if (1.0f < fabsf(target_heading - 180.0f))
    {
        feedback_event = FEEDBACK_EVENT_U_TURN;
    }

    return feedback_event;
}
