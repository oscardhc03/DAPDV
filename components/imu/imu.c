#include "imu.h"


/* Configuración estándar de IMU */
#define IMU_I2C_NUM I2C_NUM_0
#define IMU_ACCE_FS ACCE_FS_4G
#define IMU_GYRO_FS GYRO_FS_250DPS

/* Configuración para que la IMU genere interrupciones */
#define IMU_INTERRUPT_SOURCES MPU6050_DATA_RDY_INT_BIT
#define IMU_INTERRUPT_POLARITY INTERRUPT_PIN_ACTIVE_HIGH
#define IMU_INTERRUPT_GPIO_CONFIG INTERRUPT_PIN_PUSH_PULL
#define IMU_INTERRUPT_LATCH INTERRUPT_LATCH_50US
#define IMU_INTERRUPT_CLEAR_BEHAVIOR INTERRUPT_CLEAR_ON_STATUS_READ

static esp_err_t i2c_bus_init(void);

static const char * const TAG = "IMU";

static const mpu6050_int_config_t imu_intr_config = {
    .interrupt_pin              = CONFIG_IMU_INTERRUPT_GPIO_NUM,
    .active_level               = IMU_INTERRUPT_POLARITY,
    .pin_mode                   = IMU_INTERRUPT_GPIO_CONFIG,
    .interrupt_latch            = IMU_INTERRUPT_LATCH,
    .interrupt_clear_behavior   = IMU_INTERRUPT_CLEAR_BEHAVIOR,
};

esp_err_t imu_init(imu_handle_t * const out_handle, const imu_isr_t imu_isr)
{
    esp_err_t status;
    mpu6050_handle_t mpu6050_handle;
    uint8_t mpu6050_device_id;

    if (NULL != out_handle)
    {
        status = ESP_OK;
    }
    else
    {
        status = ESP_ERR_INVALID_ARG;
    }

    if (ESP_OK == status)
    {
        status = i2c_bus_init();

        if (ESP_OK != status)
        {
            ESP_LOGE(TAG, "IMU i2c bus initialization error (%s)", esp_err_to_name(status));
        }
    }

    if (ESP_OK == status)
    {
        mpu6050_handle = (imu_handle_t) mpu6050_create(IMU_I2C_NUM, MPU6050_I2C_ADDRESS);

        if (NULL == mpu6050_handle)
        {
            status = ESP_ERR_NO_MEM;
        }
    }

    if (ESP_OK == status)
    {
        status = mpu6050_get_deviceid(mpu6050_handle, &mpu6050_device_id);

        if (ESP_OK != status || MPU6050_I2C_ADDRESS != mpu6050_device_id)
        {
            ESP_LOGE(TAG, "IMU WHOAMI test failed (%s)", esp_err_to_name(status));
        }  
    }

    if (ESP_OK == status)
    {
        status = mpu6050_config(mpu6050_handle, IMU_ACCE_FS, IMU_GYRO_FS);
    }

    if (ESP_OK == status)
    {
        status = mpu6050_wake_up(mpu6050_handle);
    }

    if (NULL != imu_isr && ESP_OK == status)
    {
        status = mpu6050_config_interrupts(mpu6050_handle, &imu_intr_config);
        
        if (ESP_OK == status)
        {
            status = mpu6050_enable_interrupts(mpu6050_handle, IMU_INTERRUPT_SOURCES);
        }

        if (ESP_OK == status)
        {
            status = mpu6050_register_isr(mpu6050_handle, imu_isr);
        }

        if (ESP_OK != status)
        {
            ESP_LOGI(TAG, "IMU isr configuration failed (%s)", esp_err_to_name(status));
        }
    }

    if (ESP_OK == status)
    {
        *out_handle = mpu6050_handle;
    }

    return status;
}

esp_err_t imu_deinit(imu_handle_t handle)
{
    esp_err_t status;

    //TODO
    status = ESP_OK;

    return status;
}

esp_err_t imu_read(imu_handle_t handle, imu_value_t * const out_value)
{
    mpu6050_acce_value_t mpu6050_acce;
    mpu6050_gyro_value_t mpu6050_gyro;
    mpu6050_temp_value_t mpu6050_temp;

    esp_err_t status;
#if IMU_INTERRUPT_SOURCES == MPU6050_DATA_RDY_INT_BIT
    uint8_t mpu6050_interrupt_status;
#endif

    if (NULL != handle && NULL != out_value)
    {
        status = ESP_OK;
    }
    else
    {
        status = ESP_ERR_INVALID_ARG;
    }

#if IMU_INTERRUPT_SOURCES == MPU6050_DATA_RDY_INT_BIT
    if (ESP_OK == status)
    {
        status = mpu6050_get_interrupt_status((mpu6050_handle_t) handle, &mpu6050_interrupt_status);

        if (ESP_OK == status && 0 == mpu6050_is_data_ready_interrupt(mpu6050_interrupt_status))
        {
            // MPU6050 no ha generado la interrupción DATA_READY. Suponer que los valores de los registros son viejos.
            status = ESP_ERR_TIMEOUT;
        }
    }
#endif

    if (ESP_OK == status)
    {
        status = mpu6050_get_acce((mpu6050_handle_t) handle, &mpu6050_acce);
    }

    if (ESP_OK == status)
    {
        status = mpu6050_get_gyro((mpu6050_handle_t) handle, &mpu6050_gyro);
    }

    if (ESP_OK == status)
    {
        status = mpu6050_get_temp((mpu6050_handle_t) handle, &mpu6050_temp);
    }

    if (ESP_OK == status)
    {
        out_value->acceleration.x = mpu6050_acce.acce_x;
        out_value->acceleration.y = mpu6050_acce.acce_y;
        out_value->acceleration.z = mpu6050_acce.acce_z;

        out_value->gyro.x = mpu6050_gyro.gyro_x;
        out_value->gyro.y = mpu6050_gyro.gyro_y;
        out_value->gyro.z = mpu6050_gyro.gyro_z;

        out_value->temperature = mpu6050_temp.temp;
    }

    return status;
}

esp_err_t i2c_bus_init()
{
    static const i2c_config_t i2c_bus_config = {
        .mode = I2C_MODE_MASTER,
        .master.clk_speed = (uint32_t) CONFIG_IMU_I2C_BUS_FREQ_HZ,
        .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL,
        .sda_io_num = (gpio_num_t) CONFIG_IMU_SDA_GPIO_NUM,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = (gpio_num_t) CONFIG_IMU_SCL_GPIO_NUM,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    };

    esp_err_t status;

    status = i2c_param_config(IMU_I2C_NUM, &i2c_bus_config);

    if (ESP_OK == status)
    {
        status = i2c_driver_install(IMU_I2C_NUM, i2c_bus_config.mode, 0, 0, 0);
    }

    return status;
}
