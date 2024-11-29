#ifndef CAMERA_CLIENT_H_
#define CAMERA_CLIENT_H_

#include <esp_err.h>
#include <esp_log.h>

#include <esp_event.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <nvs_flash.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

/**
 * @brief Inicializa el cliente HTTP que obtiene los resultados de inferencia de la cámara.
 * 
 * Esta funcionalidad activa Wifi para conectar el cliente al servidor.
 * 
 * La SSID y contraseña del AP se definen en menuconfig.
 * 
 * @return ESP_OK si inicializó correctamente el cliente HTTP.
 * @return ESP_ERR_INVALID_ARG si uno de los valores configurados en kconfig no es válido.
 * @return ESP_FAIL si ocurrió algún otro problema.
 */
esp_err_t camera_client_init(void);

#endif /* CAMERA_CLIENT_H_ */
