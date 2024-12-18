#ifndef HMI_H_
#define HMI_H_

#include <stdint.h>

#include <esp_err.h>
#include <esp_log.h>

#include <driver/gpio.h>
#include <driver/ledc.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "power_save.h"

/**
 * Public structures and enumerations.
 */
typedef enum _feedback_priority_t {
    FEEDBACK_PRIORITY_LOW,
    FEEDBACK_PRIORITY_NORMAL,
    FEEDBACK_PRIORITY_HIGH,
    FEEDBACK_PRIORITY_MAX,
} feedback_priority_t;

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
    FEEDBACK_SOURCE_SYSTEM,
    FEEDBACK_SOURCE_MAX,
} feedback_source_t;

typedef struct _feedback_event_t {
    feedback_event_id_t id;             /*!< Identificador del tipo de evento de retroalimentación al usuario. */
    feedback_source_t source;           /*!< Fuente que origina el evento de retroalimentación. */
    feedback_priority_t priority;       /*!< Prioridad (LOW, NORMAL, HIGH) del evento. */
} feedback_event_t;

/**
 * @brief Inicializa los botones y el buzzer del HMI.
 * 
 * @note Para que las interrupciones de GPIO funcionen, la aplicación debe invocar gpio_install_isr_service() antes de invocar hmi_init().
 * 
 * @param feedback_event_queue_handle referencia a la queue que usará HMI para transmitir eventos de retroalimentación.
 * 
 * @return - ESP_OK HMI fue inicializada correctamente.
 * @return - ESP_ERR_INVALID_ARG si feedback_event_queue_handle es NULL.
 * @return - ESP_FAIL error durante la inicialización de HMI.
 */
esp_err_t hmi_init(QueueHandle_t feedback_event_queue_handle);


/**
 * @brief Activa el buzzer siguiendo la secuencia de tonos correspondiente al tipo de evento.
 * 
 * @note Esta función bloquea la tarea de FreeRTOS que la invoca.
 * 
 * @param event_id Identificador del evento que selecciona la secuencia.
 * @param tiemout_ticks El número máximo de ticks que espera a que el buzzer esté libre.
 * 
 * @return ESP_OK si la secuencia se reprodució correctamente.
 */
esp_err_t hmi_buzzer_play_feedback_sequence(feedback_event_id_t event_id, TickType_t timeout_ticks);

/**
 * @brief Envía un evento de retroalimentación para que HMI lo comunique al usuario.
 * 
 * @param event_id identificador del evento de retroalimentación (FEEDBACK_EVENT_*).
 * @param source fuente de la retroalimentación (FEEDBACK_SOURCE_*).
 * @param priority prioridad de la retroalimentación (FEEDBACK_PRIORITY_*).
 * @param timeout_ticks el número máximo de ticks que bloquea a la tarea si no hay espacio en la queue de eventos.
 * 
 * @return - ESP_OK si envió el evento de retroalimentación correctamente.
 * @return - ESP_ERR_INVALID_STATE si HMI no está inicializado.
 * @return - ESP_ERR_NO_MEM si no hay suficiente memoria para enviar el evento.
 * @return - ESP_FAIL si no hay espacio en la queue de eventos. La tarea puede haber sido bloqueada hasta timeout_ticks.
 */
esp_err_t hmi_feedback_event_send(feedback_event_id_t event_id, feedback_source_t source, feedback_priority_t priority, TickType_t timeout_ticks);

#endif /* HMI_H_ */
