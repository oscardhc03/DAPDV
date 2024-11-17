#ifndef GPS_H_
#define GPS_H_

#include <stdlib.h> 
#include <string.h>

#include <esp_err.h>
#include <esp_log.h>

#include <driver/gpio.h>
#include <driver/uart.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <IQmathLib.h>

typedef struct _gps_position_t {
    _iq16 latitude;
    _iq16 longitude;
    _iq18 altitude;
    uint8_t num_satellites;
} gps_position_t;

typedef struct _gps_speed_t {
    _iq23 ground_speed;
} gps_speed_t;

typedef enum _gps_event_id_t {
    GPS_EVENT_NONE,
    GPS_EVENT_POSITION,
    GPS_EVENT_SPEED,
} gps_event_id_t;

typedef struct _gps_event_t {
    gps_event_id_t id;
    void * data;
} gps_event_t;

typedef struct _gps_config_t {
    struct {
        uart_port_t port;               /*!< Puerto de UART para la comunicación. */
        gpio_num_t rx_pin;              /*!< Número de pin asignado a RX en el puerto de UART. */
        uint32_t bit_rate;              /*!< Velocidad de la comunicación UART (bits por segundo). */
        uart_word_length_t word_length; /*!< Número de bits en cada palabra de UART. */
        uart_parity_t parity;           /*!< Paridad de UART (desactivada, 1 bit o 2 bits). */
        uart_stop_bits_t stop_bits;     /*!< Número de stop bits. */
    } uart;
    uint32_t event_queue_length;        /*!< Longitud de la queue interna para detectar patrones en UART. */
} gps_config_t;

typedef void * gps_handle_t;

/**
 * @brief Inicializa el parser de sentencias NMEA enviadas por un módulo GPS a través de UART.
 * 
 * @param config configuración de comunicación para el driver.
 * @param out_gps_handle referencia a la instancia inicializada del driver.
 * @param out_data_queue_handle referencia a la queue FreeRTOS usada por el driver para enviar eventos de GPS.
 * 
 * @return - ESP_OK cuando inicializó correctamente el driver.        
 * @return - ESP_ERR_NO_MEM cuando no hay suficiente heap disponible para la instancia del driver.
 * @return - ESP_ERR_INVALID_ARG uno o más de los argumentos es nulo.
 * @return - ESP_FAIL en cualquier otro error.
 */
esp_err_t gps_init(const gps_config_t * const config, gps_handle_t * const out_gps_handle, QueueHandle_t * const out_data_queue_handle);

esp_err_t gps_deinit(const gps_handle_t gps_handle);

#endif /* GPS_H_ */
