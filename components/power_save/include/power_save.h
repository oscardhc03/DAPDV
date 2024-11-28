#ifndef POWER_SAVE_H_
#define POWER_SAVE_H_

#include <esp_log.h>
#include <esp_err.h>

#include <esp_sleep.h>
#include <driver/gpio.h>
#include <driver/rtc_io.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/timers.h>

#define POWER_SAVE_EVT_ENTER            ((EventBits_t) (1u << 0u))
#define POWER_SAVE_EVT_WOKE_FROM_SLEEP  ((EventBits_t) (1u << 1u))
#define POWER_SAVE_EVT_LOW_BATTERY      ((EventBits_t) (1u << 2u))
#define POWER_SAVE_EVT_USER_REQUEST     ((EventBits_t) (1u << 3u))
#define POWER_SAVE_EVT_SENSOR_1_IDLE    ((EventBits_t) (1u << 4u))
#define POWER_SAVE_EVT_SENSOR_2_IDLE    ((EventBits_t) (1u << 5u))
#define POWER_SAVE_EVT_SENSOR_3_IDLE    ((EventBits_t) (1u << 6u))
#define POWER_SAVE_EVT_SENSOR_4_IDLE    ((EventBits_t) (1u << 7u))


/**
 * @brief Inicializa el módulo de ahorro de energía, determina la causa del wakeup.
 * 
 * Debe invocarse power_save_init() antes de usar cualquier otra función del módulo
 * de ahorro de energía.
 * 
 * @return - ESP_OK inicializó correctamente el módulo.
 * @return - ESP_ERR_NO_MEM no hay memoria libre suficiente para inicializar el módulo.
 * @return - ESP_FAIL en caso de algún otro error.
 */
esp_err_t power_save_init(void);

/**
 * @brief De-inicializa y libera los recursos usados por el módulo de ahorro de energía.
 * 
 * @return - ESP_OK liberó los recursos y desactivó el módulo de ahorro de energía.
 * @return - ESP_ERR_INVALID_STATE el módulo de ahorro de energía no estaba inicializado.
 * @return - ESP_FAIL en caso de algún otro error. 
 */
esp_err_t power_save_deinit(void);

/**
 * @brief Registra el número del GPIO que sirve para wakeup externo desde sueño profundo.
 * 
 * @param wakeup_pin el número de GPIO que despertará al mcu cuando el pin cambie a estado bajo.
 * 
 * @note En ESP32 base, wakeup_pin debe ser ESP32: 0, 2, 4, 12-15, 25-27, 32-39.
 * 
 * @return - ESP_OK si configuró correctamente el GPIO para despertar de sueño profundo.
 * @return - ESP_FAIL en caso de algún otro error. 
 */
esp_err_t power_save_register_ext0_wakeup(gpio_num_t wakeup_pin);

/**
 * @brief Recupera el estado actual de los eventos del módulo de ahorro de energía.
 * 
 * No bloquea la tarea para esperar a un conjunto de bits específico.
 * 
 * @note No usar esta función desde una interrupción.
 * 
 * @param out_events referencia en donde almacenará el estado de los eventos.
 * 
 * @return - ESP_OK recuperó correctamente el estado de los eventos de ahorro de energía.
 * @return - ESP_ERR_INVALID_STATE el módulo de ahorro de energía no estaba inicializado.
 * @return - ESP_ERR_INVALID_ARG out_events es NULL.
 * @return - ESP_FAIL en caso de algún otro error.
 */
esp_err_t power_save_events_pending(EventBits_t * const out_events);

/**
 * @brief Publica conjunto de eventos POWER_SAVE_*.
 * 
 * No bloquea la tarea.
 * 
 * @param event el conjunto de eventos de ahorro de energía.
 * 
 * @return - ESP_OK si actualizó correctamente el estado de los eventos de ahorro de energía.
 * @return - ESP_ERR_INVALID_STATE el módulo de ahorro de energía no estaba inicializado.
 * @return - ESP_FAIL en caso de algún otro error.
 */
esp_err_t power_save_post_event(const EventBits_t event);

#endif /* POWER_SAVE_H_ */
