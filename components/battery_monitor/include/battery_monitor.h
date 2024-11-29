#ifndef BATTERY_MONITOR_H_
#define BATTERY_MONITOR_H_

#include <esp_log.h>
#include <esp_err.h>

#include <hal/adc_types.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>

#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <freertos/queue.h>

#include "hmi.h"


/**
 * @brief Inicializa el monitor de estado de carga de la batería.
 * 
 * Cuando el estado de carga de la batería pasa por los umbrales configurados, el monitor
 * de batería envía eventos de retroalimentación al usuario y de ahorro de energía (POWER_SAVE_EVT_BATTERY_CRITICAL). 
 * 
 * @note Este módulo usa ADC1 como una forma básica de detectar el voltaje actual de la batería. 
 * 
 * @param feedback_event_queue_handle referencia a la queue de eventos de retroalimentación.
 * 
 * @return - ESP_OK cuando inicializó correctamente el monitor de batería.
 * @return - ESP_ERR_NO_MEM cuando no hay memoria suficiente memoria heap libre para los recursos del módulo.
 * @return - ESP_ERR_INVALID_ARG cuando la configuración en kconfig tiene valores incorrectos.
 * @return - ESP_FAIL cuando no pudo iniciar el software timer que controla el muestreo de la batería.
 */
esp_err_t battery_monitor_init(const QueueHandle_t feedback_event_queue_handle);


/**
 * @brief De-inicializa el monitor de estado de carga de la batería y libera los recursos asociados a él.
 * 
 * @return - ESP_OK cuando de-inicializó correctamente al módulo.
 * @return - ESP_ERR_INVALID_STATE cuando el módulo estaba inicializado.
 * @return - ESP_FAIL en cualquier otro error.
 */
esp_err_t battery_monitor_deinit(void);

#endif /* BATTERY_MONITOR_H_ */
