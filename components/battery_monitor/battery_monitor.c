#include "battery_monitor.h"

#define CAN_ENTER_DEEP_SLEEP_ON_LOW_BATTERY (0)

#define BATTERY_MONITOR_ADC_CONVERSION_MODE ADC_CONV_SINGLE_UNIT_1
#define BATTERY_MONITOR_ADC_OUTPUT_FORMAT ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define BATTERY_MONITOR_ADC_ATTENUATION ADC_ATTEN_DB_0
#define BATTERY_MONITOR_ADC_BIT_WIDTH SOC_ADC_DIGI_MAX_BITWIDTH

static void battery_monitor_timer_callback(TimerHandle_t timer); 

static bool adc_calibration_init(adc_unit_t unit_id, adc_channel_t channel, adc_atten_t attenuation, adc_cali_handle_t * out_calibration_handle);

static const char * const TAG = "BATTERY_MON";

static bool adc_is_calibrated;
static adc_oneshot_unit_handle_t adc_handle; 
static adc_channel_t battery_monitor_adc_channel;
static adc_cali_handle_t battery_monitor_adc_calibration_handle;

static TimerHandle_t battery_monitor_timer;

static QueueHandle_t feedback_event_queue;

esp_err_t battery_monitor_init(const QueueHandle_t feedback_event_queue_handle)
{
    static const adc_oneshot_chan_cfg_t battery_monitor_adc_config = {
        .atten = BATTERY_MONITOR_ADC_ATTENUATION,
        .bitwidth = BATTERY_MONITOR_ADC_BIT_WIDTH,
    };

    esp_err_t status;
    BaseType_t result;
    adc_oneshot_unit_init_cfg_t adc1_init_config;

    status = ESP_OK;
    adc_handle = NULL;

    if (ESP_OK == status)
    {
        status = adc_oneshot_io_to_channel(CONFIG_BATTERY_MON_ADC_GPIO_NUM, &adc1_init_config.unit_id, &battery_monitor_adc_channel);

        if (ESP_ERR_NOT_FOUND == status)
        {
            ESP_LOGE(TAG, "ADC IO to channel conversion fail (%s)", esp_err_to_name(status));
        }
    }

    if (ESP_OK == status)
    {
        adc1_init_config.clk_src = 0;
        status = adc_oneshot_new_unit(&adc1_init_config, &adc_handle);
    }

    if (ESP_OK == status)
    {
        ESP_LOGI(TAG, "ADC pattern attenuation = %hhu", battery_monitor_adc_config.atten);
        ESP_LOGI(TAG, "ADC bitwidth = %hhu", battery_monitor_adc_config.bitwidth);
        ESP_LOGI(TAG, "ADC pattern unit = %hhu", adc1_init_config.unit_id);
        ESP_LOGI(TAG, "ADC pattern channel = %hhu (GPIO NUMBER = %hhu)", battery_monitor_adc_channel, CONFIG_BATTERY_MON_ADC_GPIO_NUM);

        status = adc_oneshot_config_channel(adc_handle, battery_monitor_adc_channel, &battery_monitor_adc_config);
    }

    if (ESP_OK == status)
    {
        adc_is_calibrated = adc_calibration_init(
            adc1_init_config.unit_id, 
            battery_monitor_adc_channel, 
            battery_monitor_adc_config.atten, 
            &battery_monitor_adc_calibration_handle
        );
    }

    if (ESP_OK == status)
    {
        feedback_event_queue = feedback_event_queue_handle;

        battery_monitor_timer = xTimerCreate(
            "bat_mon",
            pdMS_TO_TICKS(CONFIG_BATTERY_MONITOR_SAMPLE_PERIOD_MS),
            pdTRUE,
            NULL,
            battery_monitor_timer_callback
        );

        if (NULL != battery_monitor_timer)
        {
            result = xTimerStart(battery_monitor_timer, pdMS_TO_TICKS(10));

            if (pdFAIL == result)
            {
                ESP_LOGE(TAG, "Battery monitor timer start fail");
                status = ESP_FAIL;
            }
        }
        else
        {
            status = ESP_ERR_NO_MEM;
        }
    }

    return status;
}

esp_err_t battery_monitor_deinit(void)
{
    esp_err_t status;

    status = ESP_OK;

    return status;
}

void battery_monitor_timer_callback(TimerHandle_t timer)
{
    esp_err_t status;
    static int32_t previous_value_mv = LONG_MAX;
    int32_t raw_conversion_value;
    int32_t value_mv;

    if (NULL != adc_handle)
    {
        status = adc_oneshot_read(adc_handle, battery_monitor_adc_channel, (int *) &raw_conversion_value);

        if (ESP_OK == status)
        {
            ESP_LOGD(TAG, "ADC raw data = %ld", raw_conversion_value);

            if (adc_is_calibrated)
            {
                status = adc_cali_raw_to_voltage(battery_monitor_adc_calibration_handle, raw_conversion_value, (int *) &value_mv);

                if (ESP_OK == status)
                {
                    ESP_LOGD(TAG, "ADC calibrated and converted voltage = %ld mV", value_mv);

                    if (CAN_ENTER_DEEP_SLEEP_ON_LOW_BATTERY && LONG_MAX != previous_value_mv)
                    {
                        if (CONFIG_BATTERY_MON_LOW_BATTERY_THRESHOLD_MV <= previous_value_mv && CONFIG_BATTERY_MON_LOW_BATTERY_THRESHOLD_MV > value_mv)
                        {
                            // El voltaje de la batería cayó de CONFIG_BATTERY_MON_LOW_BATTERY_THRESHOLD_MV.
                            status = hmi_feedback_event_send(FEEDBACK_EVENT_LOW_BATTERY, FEEDBACK_SOURCE_SYSTEM, FEEDBACK_PRIORITY_NORMAL, (TickType_t) 0);

                            if (ESP_OK != status)
                            {
                                ESP_LOGW(TAG, "send LOW BATTERY feedabck fail (%s)", esp_err_to_name(status));
                            }
                        }
                        
                        if (((int32_t) CONFIG_BATTERY_MON_CRITICAL_BATTERY_THRESHOLD_MV) > previous_value_mv && ((int32_t) CONFIG_BATTERY_MON_CRITICAL_BATTERY_THRESHOLD_MV) > value_mv)
                        {
                            // El voltaje de la batería cayó a través de CONFIG_BATTERY_MON_CRITICAL_BATTERY_THRESHOLD_MV.
                            status = power_save_post_event(POWER_SAVE_EVT_BATTERY_CRITICAL, pdFALSE);

                            if (ESP_OK != status)
                            {
                                ESP_LOGW(TAG, "send BATTERY CRITICAL event fail (%s)", esp_err_to_name(status));
                            }
                        }
                        // else if (CONFIG_BATTERY_MON_CRITICAL_BATTERY_THRESHOLD_MV > previous_value_mv && CONFIG_BATTERY_MON_CRITICAL_BATTERY_THRESHOLD_MV <= value_mv)
                        // {
                        //     // El estado de carga de la batería ya no es crítico. 
                        //     // El voltaje de la batería subió de CONFIG_BATTERY_MON_CRITICAL_BATTERY_THRESHOLD_MV.
                        //     // Limpiar el evento de batería crítica.
                        //     status = power_save_post_event(POWER_SAVE_EVT_BATTERY_CRITICAL, pdTRUE);

                        //     if (ESP_OK != status)
                        //     {
                        //         ESP_LOGW(TAG, "send BATTERY NO LONGER CRITICAL event fail (%s)", esp_err_to_name(status));
                        //     }
                        // }
                    }

                    previous_value_mv = value_mv;
                }
            }
        }
        else
        {
            ESP_LOGW(TAG, "ADC read fail (%s)", esp_err_to_name(status));
        }
    }
}

bool adc_calibration_init(adc_unit_t unit_id, adc_channel_t channel, adc_atten_t attenuation, adc_cali_handle_t * out_calibration_handle)
{
    adc_cali_handle_t handle;
    esp_err_t status;
    bool calibrated;

    calibrated = false;
    status = ESP_OK;

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t line_fitting_config;
#endif

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated)
    {

    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version = LINE_FITTING");
        line_fitting_config = (adc_cali_line_fitting_config_t) {
            .unit_id = unit_id,
            .atten = attenuation,
            .bitwidth = BATTERY_MONITOR_ADC_BIT_WIDTH
        };

        status = adc_cali_create_scheme_line_fitting(&line_fitting_config, &handle);

        if (ESP_OK == status)
        {
            calibrated = true;
        }
    }
#endif

    if (ESP_OK == status)
    {
        *out_calibration_handle = handle;
        ESP_LOGI(TAG, "Calibration success");
    }
    else if (ESP_ERR_NOT_SUPPORTED == status || !calibrated)
    {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    }
    else
    {
        ESP_LOGE(TAG, "Calibration fail (invalid argument or not enough memory)");
    }

    return calibrated;
}
