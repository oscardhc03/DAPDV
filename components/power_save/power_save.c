#include <time.h>
#include <sys/time.h>

#include "power_save.h"

static const char * const TAG = "PWR_SAVE";

static const TickType_t WAIT_BEFORE_DEEP_SLEEP_ENTER_TICKS = pdMS_TO_TICKS(CONFIG_WAIT_BEFORE_DEEP_SLEEP_ENTER_MS);
static const EventBits_t POWER_SAVE_EVT_ALL_SENSORS_IDLE = (
    POWER_SAVE_EVT_SENSOR_1_IDLE | POWER_SAVE_EVT_SENSOR_2_IDLE | 
    POWER_SAVE_EVT_SENSOR_3_IDLE | POWER_SAVE_EVT_SENSOR_4_IDLE
);

static EventGroupHandle_t power_save_event_group;
static TimerHandle_t sleep_activation_timer;


#if SOC_RTC_FAST_MEM_SUPPORTED
static RTC_DATA_ATTR struct timeval sleep_enter_time;
#else
static struct timeval sleep_enter_time;
#endif

static gpio_num_t ext0_wakeup_gpio = GPIO_NUM_MAX;

static void sleep_enter_timer_callback(TimerHandle_t timer);

esp_err_t power_save_init(void)
{
    esp_err_t status;
    struct timeval now;
    esp_sleep_wakeup_cause_t wakeup_cause;
    int ms_spent_in_sleep;

    status = ESP_OK;
    gettimeofday(&now, NULL);
    ms_spent_in_sleep = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    // Inicializar recursos.
    if (ESP_OK == status)
    {
        power_save_event_group = xEventGroupCreate();

        if (NULL == power_save_event_group)
        {
            status = ESP_ERR_NO_MEM;
        }
    }

    // Determinar la causa de reset (WAKEUP u otro tipo de reset)
    wakeup_cause = esp_sleep_get_wakeup_cause();

    switch (wakeup_cause)
    {
    case ESP_SLEEP_WAKEUP_EXT0:
        ESP_LOGI(TAG, "Wake up from ext0, spent %d ms in deep sleep", ms_spent_in_sleep);
        break;
    case ESP_SLEEP_WAKEUP_UNDEFINED:
        ESP_LOGI(TAG, "Not a deep sleep wakeup");
        break;
    default:
        ESP_LOGI(TAG, "Unhandled wakeup cause: %u", wakeup_cause);
        break;
    }

    if (ESP_OK == status)
    {
        sleep_activation_timer = NULL;
        xEventGroupSetBits(power_save_event_group, POWER_SAVE_EVT_WOKE_FROM_SLEEP);
    }

    return status;
}

esp_err_t power_save_deinit(void)
{
    esp_err_t status;

    status = ESP_OK;
    if (NULL == power_save_event_group)
    {
        status = ESP_ERR_INVALID_STATE;
    }

    if (ESP_OK == status)
    {
        vEventGroupDelete(power_save_event_group);
    }

    return status;
}

esp_err_t power_save_register_ext0_wakeup(gpio_num_t wakeup_pin)
{
    esp_err_t status;

    status = rtc_gpio_deinit(wakeup_pin);

    if (ESP_OK == status)
    {
        ext0_wakeup_gpio = wakeup_pin;
    }

    return status;
}

esp_err_t power_save_events_pending(EventBits_t * const out_events)
{
    esp_err_t status;

    status = ESP_OK;

    if (NULL == out_events)
    {
        status = ESP_ERR_INVALID_ARG;
    }

    if (ESP_OK == status && NULL == power_save_event_group)
    {
        status = ESP_ERR_INVALID_STATE;
    }

    if (ESP_OK == status)
    {
        *out_events = xEventGroupGetBits(power_save_event_group);
    }

    return status;
}

esp_err_t power_save_post_event(const EventBits_t event, BaseType_t clear_bits)
{
    EventBits_t power_save_event_bits;
    BaseType_t should_enter_sleep;

    esp_err_t status;
    BaseType_t timer_modify_result;

    status = ESP_OK;

    if (NULL == power_save_event_group)
    {
        status = ESP_ERR_INVALID_STATE;
    }

    if (ESP_OK == status)
    {
        if (pdTRUE == clear_bits)
        {
            power_save_event_bits = xEventGroupClearBits(power_save_event_group, event);
        }
        else
        {
            power_save_event_bits = xEventGroupSetBits(power_save_event_group, event);
        }

        should_enter_sleep = (
            POWER_SAVE_EVT_BATTERY_CRITICAL & power_save_event_bits || 
            POWER_SAVE_EVT_USER_REQUEST & power_save_event_bits || 
            POWER_SAVE_EVT_ALL_SENSORS_IDLE == (POWER_SAVE_EVT_ALL_SENSORS_IDLE & power_save_event_bits)
        );

        if (should_enter_sleep && NULL == sleep_activation_timer)
        {
            sleep_activation_timer = xTimerCreate(
                "sleep", 
                WAIT_BEFORE_DEEP_SLEEP_ENTER_TICKS, 
                pdFALSE, 
                NULL, 
                sleep_enter_timer_callback
            );

            if (NULL != sleep_activation_timer)
            {
                timer_modify_result = xTimerStart(sleep_activation_timer, pdMS_TO_TICKS(10));

                if (pdPASS == timer_modify_result)
                {
                    xEventGroupSetBits(power_save_event_group, POWER_SAVE_EVT_ENTER);
                }
                else
                {
                    ESP_LOGE(TAG, "Sleep timer start fail");
                }
            }
            else
            {
                status = ESP_ERR_NO_MEM;
            }
        }
        else if (!should_enter_sleep && NULL != sleep_activation_timer)
        {
            timer_modify_result = xTimerStop(sleep_activation_timer, pdMS_TO_TICKS(10));

            if (pdPASS == timer_modify_result)
            {
                timer_modify_result = xTimerDelete(sleep_activation_timer, pdMS_TO_TICKS(10));

                if (pdPASS == timer_modify_result)
                {
                    sleep_activation_timer = NULL;
                    xEventGroupClearBits(power_save_event_group, POWER_SAVE_EVT_ENTER);
                }
                else
                {
                    ESP_LOGE(TAG, "Sleep timer delete fail");
                }
            }
            else
            {
                ESP_LOGE(TAG, "Sleep timer stop fail");
            }
        }
    }

    return status;
}

esp_err_t power_save_post_event_from_isr(const EventBits_t event, BaseType_t clear_bits, BaseType_t * const pxHigherPriorityTaskWoken)
{
    EventBits_t power_save_event_bits;
    BaseType_t should_enter_sleep;

    esp_err_t status;
    BaseType_t result;

    status = ESP_OK;

    if (NULL == pxHigherPriorityTaskWoken)
    {
        status = ESP_ERR_INVALID_ARG;
    }

    if (NULL == power_save_event_group)
    {
        status = ESP_ERR_INVALID_STATE;
    }

    if (ESP_OK == status)
    {
        if (pdTRUE == clear_bits)
        {
            result = xEventGroupClearBitsFromISR(power_save_event_group, event);

            if (pdPASS == result)
            {
                power_save_event_bits = xEventGroupGetBitsFromISR(power_save_event_group) & ~event;
            }
        }
        else
        {
            result = xEventGroupSetBitsFromISR(power_save_event_group, event, pxHigherPriorityTaskWoken);

            if (pdPASS == result)
            {
                power_save_event_bits = xEventGroupGetBitsFromISR(power_save_event_group) | event;
            }
        }

        if (pdPASS == result)
        {
            should_enter_sleep = (
                POWER_SAVE_EVT_BATTERY_CRITICAL & power_save_event_bits || 
                POWER_SAVE_EVT_USER_REQUEST & power_save_event_bits || 
                POWER_SAVE_EVT_ALL_SENSORS_IDLE == (POWER_SAVE_EVT_ALL_SENSORS_IDLE & power_save_event_bits)
            );

            if (should_enter_sleep && NULL == sleep_activation_timer)
            {
                sleep_activation_timer = xTimerCreate(
                    "sleep", 
                    WAIT_BEFORE_DEEP_SLEEP_ENTER_TICKS, 
                    pdFALSE, 
                    NULL, 
                    sleep_enter_timer_callback
                );

                if (NULL != sleep_activation_timer)
                {
                    result = xTimerStart(sleep_activation_timer, pdMS_TO_TICKS(100));

                    if (pdPASS == result)
                    {
                        xEventGroupSetBitsFromISR(power_save_event_group, POWER_SAVE_EVT_ENTER, pxHigherPriorityTaskWoken);
                    }
                }
                else
                {
                    status = ESP_ERR_NO_MEM;
                }
            }
            else if (!should_enter_sleep && NULL != sleep_activation_timer)
            {
                result = xTimerStop(sleep_activation_timer, pdMS_TO_TICKS(10));

                if (pdPASS == result)
                {
                    result = xTimerDelete(sleep_activation_timer, pdMS_TO_TICKS(10));

                    if (pdPASS == result)
                    {
                        sleep_activation_timer = NULL;
                        xEventGroupClearBitsFromISR(power_save_event_group, POWER_SAVE_EVT_ENTER);
                    }
                }
            }
        }
    }

    return status;
}

void sleep_enter_timer_callback(TimerHandle_t timer)
{
    EventBits_t power_save_event_bits;
    BaseType_t should_enter_sleep;
    esp_err_t status;

    ESP_LOGI(TAG, "Sleep enter timer callback");

    if (NULL != power_save_event_group)
    {
        status = ESP_OK;
        // Revisar si todavía se cumple al menos una de las tres condiciones para entrar en sueño profundo.
        power_save_event_bits = xEventGroupGetBits(power_save_event_group);
        should_enter_sleep = (
            POWER_SAVE_EVT_BATTERY_CRITICAL & power_save_event_bits || 
            POWER_SAVE_EVT_USER_REQUEST & power_save_event_bits || 
            POWER_SAVE_EVT_ALL_SENSORS_IDLE == (POWER_SAVE_EVT_ALL_SENSORS_IDLE & power_save_event_bits)
        );

        if (should_enter_sleep)
        {
            if (GPIO_NUM_MAX > ext0_wakeup_gpio)
            {
                status = esp_sleep_enable_ext0_wakeup(ext0_wakeup_gpio, 0);
                
                ESP_LOGI(TAG, "Enable EXT0 wakeup on GPIO %d (%s)", ext0_wakeup_gpio, esp_err_to_name(status));

                if (ESP_OK == status)
                {
                    // ESP_ERROR_CHECK(rtc_gpio_pulldown_dis(ext0_wakeup_gpio));
                    // ESP_ERROR_CHECK(rtc_gpio_pullup_en(ext0_wakeup_gpio));
                }
            }

            if (ESP_OK == status)
            {
                ESP_LOGI(TAG, "Entering deep sleep");
                gettimeofday(&sleep_enter_time, NULL);
                esp_deep_sleep_start();
            }
            else
            {
                ESP_LOGW(TAG, "Enter deep sleep cancel (%s)", esp_err_to_name(status));
            }
        }
        else
        {
            xEventGroupClearBits(power_save_event_group, POWER_SAVE_EVT_ENTER);
        }
    }
    else
    {
        ESP_LOGW(TAG, "Did not enter sleep, power save event group is NULL");
    }
}
