#include "hmi.h"

#define BUZZER_BASE_FREQUENCY_HZ ((uint32_t) 1000)

#define BUZZER_LEDC_SPEED_MODE  (LEDC_LOW_SPEED_MODE)
#define BUZZER_LEDC_CHANNEL     (LEDC_CHANNEL_0)
#define BUZZER_LEDC_TIMER       (LEDC_TIMER_0)

#define NUM_NOTES_PER_SEQUENCE  ((uint8_t) 2u)

#define DUTY_NOTE_OFF           ((uint32_t) 0u)
#define DUTY_NOTE_ON            ((uint32_t) 512u)  /* 50 % duty cycle */

#define NOTE_C4 ((uint32_t) 262u)
#define NOTE_D4 ((uint32_t) 294u)
#define NOTE_E4 ((uint32_t) 330u)

/* Configuración para controlar un LED muy raro, que emite sonido en vez de luz. */
static const ledc_timer_config_t buzzer_pwm_config = {
    .speed_mode = BUZZER_LEDC_SPEED_MODE,
    .timer_num = BUZZER_LEDC_TIMER,
    .freq_hz = BUZZER_BASE_FREQUENCY_HZ,
    .duty_resolution = LEDC_TIMER_10_BIT,
    .clk_cfg = LEDC_USE_RC_FAST_CLK,
};

static const ledc_channel_config_t buzzer_pwm_channel_config = {
    .speed_mode = BUZZER_LEDC_SPEED_MODE,
    .channel = BUZZER_LEDC_CHANNEL,
    .gpio_num = (gpio_num_t) CONFIG_HMI_BUZZER_PIN_NUMBER,
    .timer_sel = BUZZER_LEDC_TIMER,
    .intr_type = LEDC_INTR_DISABLE,
    .duty = 0,
    .hpoint = 0,
};

static const char * const TAG = "HMI";

static const uint32_t note_sequences[FEEDBACK_EVENT_MAX - 1][NUM_NOTES_PER_SEQUENCE] = {
    { NOTE_C4, NOTE_C4 },
    { BUZZER_BASE_FREQUENCY_HZ, BUZZER_BASE_FREQUENCY_HZ },
    { BUZZER_BASE_FREQUENCY_HZ, BUZZER_BASE_FREQUENCY_HZ },
    { BUZZER_BASE_FREQUENCY_HZ, BUZZER_BASE_FREQUENCY_HZ },
    { NOTE_D4, NOTE_D4 },
    { NOTE_E4, NOTE_E4 },
    { BUZZER_BASE_FREQUENCY_HZ, BUZZER_BASE_FREQUENCY_HZ },
    { BUZZER_BASE_FREQUENCY_HZ, BUZZER_BASE_FREQUENCY_HZ },
    { BUZZER_BASE_FREQUENCY_HZ, BUZZER_BASE_FREQUENCY_HZ },
};

static const TickType_t NOTE_DURATION_TICKS = pdMS_TO_TICKS(250);
static const TickType_t NOTE_PAUSE_TICKS = pdMS_TO_TICKS(250);

// Semáforo para controlar el acceso al buzzer. Sólo puede tomarlo una tarea cuando el buzzer no está reproduciendo un sonido.
static SemaphoreHandle_t buzzer_semaphore;

esp_err_t hmi_init(void)
{
    esp_err_t status;

    status = ESP_OK;

    buzzer_semaphore = xSemaphoreCreateBinary();

    if (NULL != buzzer_semaphore)
    {
        if (pdFALSE == xSemaphoreGive(buzzer_semaphore))
        {
            ESP_LOGE(TAG, "Failed to initially give buzzer semaphore");
        }
    }
    else
    {
        status = ESP_ERR_NO_MEM;
    }

    if (ESP_OK == status)
    {
        status = ledc_timer_config(&buzzer_pwm_config);

        if (ESP_OK != status)
        {
            ESP_LOGE(TAG, "Buzzer LEDC timer config error (%s)", esp_err_to_name(status));
        }
    }

    if (ESP_OK == status)
    {
        status = ledc_channel_config(&buzzer_pwm_channel_config);

        if (ESP_OK != status)
        {
            ESP_LOGE(TAG, "Buzzer LEDC channel config error (%s)", esp_err_to_name(status));
        }
    }

    return status;
}

//TODO: Considerar implementar la secuencia de buzzer usando software timers para no bloquear la tarea que invoca.
esp_err_t hmi_buzzer_play_feedback_sequence(feedback_event_id_t event_id, TickType_t timeout_ticks)
{
    esp_err_t status;
    uint32_t i;

    status = ESP_OK;

    if (FEEDBACK_EVENT_NONE == event_id || FEEDBACK_EVENT_MAX <= event_id)
    {
        status = ESP_ERR_INVALID_ARG;
    }
    else if (NULL == buzzer_semaphore)
    {
        status = ESP_ERR_INVALID_STATE;
    }

    if (ESP_OK == status && pdTRUE == xSemaphoreTake(buzzer_semaphore, timeout_ticks))
    {
        ESP_LOGI(TAG, "Using buzzer");
        i = NUM_NOTES_PER_SEQUENCE;
        while (i--)
        {
            if (BUZZER_BASE_FREQUENCY_HZ != note_sequences[((uint8_t) event_id) - 1u][i])
            {
                status = ledc_set_freq(BUZZER_LEDC_SPEED_MODE, BUZZER_LEDC_TIMER, note_sequences[((uint8_t) event_id) - 1u][i]);

                if (ESP_OK == status)
                {
                    status = ledc_set_duty(BUZZER_LEDC_SPEED_MODE, BUZZER_LEDC_CHANNEL, DUTY_NOTE_ON);
                }

                if (ESP_OK == status)
                {
                    status = ledc_update_duty(BUZZER_LEDC_SPEED_MODE, BUZZER_LEDC_CHANNEL);
                }
            }

            vTaskDelay(NOTE_DURATION_TICKS);

            status = ledc_set_duty(BUZZER_LEDC_SPEED_MODE, BUZZER_LEDC_CHANNEL, DUTY_NOTE_OFF);

            if (ESP_OK == status)
            {
                status = ledc_update_duty(BUZZER_LEDC_SPEED_MODE, BUZZER_LEDC_CHANNEL);
            }

            vTaskDelay(NOTE_PAUSE_TICKS);
        }

        // Regresar el semáforo cuando termina de sonar el buzzer.
        if (pdFALSE == xSemaphoreGive(buzzer_semaphore))
        {
            ESP_LOGE(TAG, "Buzzer xSemaphoreGive failed even when xSemaphoreTake had succeeded");
        }

        ESP_LOGI(TAG, "Buzzer is free again");
    }
    else
    {
        ESP_LOGE(TAG, "Buzzer access denied (%s)", esp_err_to_name(status));
    }

    if (ESP_OK != status)
    {
        ESP_LOGW(TAG, "HMI buzzer control error (%s)", esp_err_to_name(status));
    }

    return status;
}
