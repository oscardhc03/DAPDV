#ifndef CAMERA_CLIENT_H_
#define CAMERA_CLIENT_H_

#include <esp_err.h>
#include <esp_log.h>

#include <esp_event.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <esp_http_client.h>
#include <esp_tls.h>

#include <cJSON.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>


/**
 * Resultado de la inferencia en la imagen enviada por la cámara al servidor.
 */
typedef struct _cam_detect_objects_t {
    float persona;
    float escaleras;
    float entorno;
    float puerta;
} cam_detect_objects_t;

/**
 * @brief Inicializa el cliente HTTP que obtiene los resultados de inferencia de la cámara.
 * 
 * Esta funcionalidad activa Wifi para conectar el cliente al servidor.
 * 
 * La SSID y contraseña del AP se definen en menuconfig.
 * 
 * @return - ESP_OK si inicializó correctamente el cliente HTTP.
 * @return - ESP_ERR_INVALID_ARG si uno de los valores configurados en kconfig no es válido.
 * @return - ESP_FAIL si ocurrió algún otro problema.
 */
esp_err_t camera_client_init(void);

/**
 * @brief Obtiene los resultados de detección de objetos desde un servidor intermedio.
 * 
 * Para obtener los resultados, usa un cliente HTTP para hacer una petición GET
 * al endpoint "predict" del servicio. Luego interpreta la respuesta con JSON.
 * 
 * @param results referencia a la estructura donde almacenará los resultados.
 * 
 * @return - ESP_OK obtuvo correctamente los resultados de la cámara.
 * @return - ESP_ERR_INVALID_ARG si results es NULL.
 * @return - ESP_ERR_INVALID_STATE si no estaba inicializado el cliente con camera_client_init().
 * @return - ESP_FAIL si ocurrió algún otro error.
 */
esp_err_t camera_client_fetch_results(cam_detect_objects_t * const results);

#endif /* CAMERA_CLIENT_H_ */
