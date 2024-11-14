#ifndef TF_MINI_PARSER_H_
#define TF_MINI_PARSER_H_

#include <esp_err.h>
#include <esp_log.h>
#include <esp_event.h>

#include <driver/gpio.h>
#include <driver/uart.h>

typedef enum _tf_mini_event_id_t {
    TF_MINI_OK,
    TF_MINI_ERR_LOW_STRENGTH,
    TF_MINI_ERR_STRENGTH_SATURATION,
    TF_MINI_ERR_AMB_LIGHT_SATURATION,
    TF_MINI_ERR_TEMPERATURE,
    TF_MINI_ERR_MAX,
} tf_mini_event_id_t;

typedef struct _tf_mini_df_t {
    uint16_t distance_cm;               /*!< Distancia detectada en centímetros. */
    uint16_t signal_strength;           /*!< Fuerza de la señal detectada. Si está fuera de rango, distance_cm tiene valor anormal. */
    float temperature_deg_c;            /*!< Temperatura interna del sensor en grados celsius. */
    tf_mini_event_id_t event_id;        /*!< Clasificación del dataframe según los valores de los campos. */
} tf_mini_df_t;

typedef struct _tf_mini_parser_config_t {
    struct {
        uart_port_t port;               /*!< Puerto de UART para la comunicación. */
        gpio_num_t rx_pin;              /*!< Número de pin asignado a RX en el puerto de UART. */
        uint32_t bit_rate;              /*!< Velocidad de la comunicación UART (bits por segundo). */
        uart_word_length_t word_length; /*!< Número de bits en cada palabra de UART. */
        uart_parity_t parity;           /*!< Paridad de UART (desactivada, 1 bit o 2 bits). */
        uart_stop_bits_t stop_bits;     /*!< Número de stop bits. */
    } uart;
    uint32_t event_queue_length;        /*!< Longitud de la queue interna para detectar patrones en UART. */
    QueueHandle_t data_queue_handle;    /*!< Referencia a la queue de datos que usará el driver. */
} tf_mini_parser_config_t;

typedef void * tf_mini_handle_t;

/**
 * @brief Inicializa el receptor de datos del sensor TF MINI.
 * 
 * @param out_tf_mini_handle referencia a la instancia del driver.
 * @param config configuración de comunicación y muestreo del sensor.
 * 
 * @return - ESP_OK cuando inicializó correctamente el sensor.        
 * @return - ESP_ERR_NO_MEM cuando no hay suficiente heap disponible para la instancia del driver.
 * @return - ESP_ERR_INVALID_ARG uno de los argumentos es nulo.
 * @return - ESP_FAIL en cualquier otro error.
 */
esp_err_t tf_mini_parser_init(tf_mini_handle_t * const out_tf_mini_handle, const tf_mini_parser_config_t * const config);

/**
 * @brief De-inicializa el receptor de datos del sensor TF MINI.
 * 
 * @param handle referencia a la instancia del driver.
 *
 * @return - ESP_OK cuando de-inicializó correctamente el sensor.   
 * @return - ESP_FAIL en cualquier otro error.
 */
esp_err_t tf_mini_parser_deinit(tf_mini_handle_t handle);

#endif /* TF_MINI_PARSER_H_ */
