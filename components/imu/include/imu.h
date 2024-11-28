#ifndef IMU_H_
#define IMU_H_

#include <esp_err.h>
#include <esp_log.h>

#include <driver/i2c.h>

#include <mpu6050.h>


typedef struct _imu_value_t {
    struct {
        float x;                /*<! Aceleración en el eje x */
        float y;                /*<! Aceleración en el eje y */
        float z;                /*<! Aceleración en el eje z */
    } acceleration;             /*<! Componentes de la aceleración sensada por la IMU */
    struct {
        float x;                /*<! Velocidad angular en el eje x */
        float y;                /*<! Velocidad angular en el eje y */
        float z;                /*<! Velocidad angular en el eje z */
    } gyro;
    float temperature;          /*<! Temperatura de la IMU en grados Celsius */
} imu_value_t;

typedef mpu6050_handle_t imu_handle_t;
typedef mpu6050_isr_t imu_isr_t;

/**
 * @brief Inicializa el bus de comunicación conectado a la IMU y aplica la configuración especificada.
 * 
 * @param out_handle referencia a la instancia del driver de IMU inicializado.
 * @param imu_isr una función ISR que el driver invoca cuando la IMU genera una interrupción.
 * 
 * @return - ESP_OK cuando inicializó correctamente la IMU.
 * @return - ESP_ERR_NO_MEM cuando no hay memoria heap suficiente para inicializar el driver.
 * @return - ESP_ERR_INVALID_ARG si out_handle es NULL.
 * @return - ESP_FAIL en caso de algún otro error.
 */
esp_err_t imu_init(imu_handle_t * const out_handle, const imu_isr_t imu_isr);

/**
 * @brief De-inicializa una instancia del driver, libera los recursos asociados.
 * 
 * @param handle referencia a la instancia del driver de IMU inicializado.
 * 
 * @return - ESP_OK cuando liberó correctamente la instancia del driver.
 * @return - ESP_ERR_INVALID_ARG si handle es NULL.
 * @return - ESP_FAIL en caso de algún otro error.
 */
esp_err_t imu_deinit(imu_handle_t handle);

/**
 * @brief Lee los registros de datos de la IMU y los regresa en forma de imu_value_t.
 * 
 * @note Si la interrupción DATA_READY está activada, imu_read() primero revisa si
 *       la IMU ha generado esa interrupción. Si no es así, la lectura falla, indicando 
 *       que no hay datos nuevos en los registros.
 * 
 * @param handle referencia a la instancia del driver de IMU.
 * @param out_value referencia en la que almacenará los valores de los registros de la IMU.
 * 
 * @return - ESP_OK cuando leyó y almacenó correctamente los valores.
 * @return - ESP_ERR_TIMEOUT cuando no hay datos nuevos disponibles.
 * @return - ESP_ERR_INVALID_ARG cuando handle u out_value es NULL.
 * @return - ESP_FAIL en caso de algún otro error.
 */
esp_err_t imu_read(imu_handle_t handle, imu_value_t * const out_value);

#endif /* IMU_H_ */
