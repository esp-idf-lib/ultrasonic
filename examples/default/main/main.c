#include <inttypes.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <ultrasonic.h>

#define INTERVAL_MS CONFIG_EXAMPLE_MEASURE_INTERVAL_MS

static SemaphoreHandle_t result_mutex;
static ultrasonic_result_t last_result;
static bool result_ready;

static const char *result_name(esp_err_t err)
{
    switch (err)
    {
        case ESP_ERR_ULTRASONIC_NO_ECHO:
            return "no echo";
        case ESP_ERR_ULTRASONIC_BUSY:
            return "busy";
        case ESP_ERR_ULTRASONIC_EMPTY:
            return "no result";
        case ESP_ERR_ULTRASONIC_CANCELED:
            return "canceled";
        default:
            return esp_err_to_name(err);
    }
}

static void on_measurement(ultrasonic_sensor_t sensor, const ultrasonic_result_t *result, void *user_ctx)
{
    (void)sensor;
    (void)user_ctx;

    if (xSemaphoreTake(result_mutex, 0) != pdTRUE)
        return;

    last_result = *result;
    result_ready = true;
    xSemaphoreGive(result_mutex);
}

void app_main(void)
{
    result_mutex = xSemaphoreCreateMutex();
    configASSERT(result_mutex);

    ultrasonic_config_t cfg =
    {
        .group = {
            .trigger_gpio = CONFIG_EXAMPLE_TRIGGER_GPIO,
        },
        .sensor = {
            .echo_gpio = CONFIG_EXAMPLE_ECHO_GPIO,
            .max_echo_us = 38000,
            .done_cb = on_measurement,
        },
    };

    ultrasonic_sensor_t sensor;
    ESP_ERROR_CHECK(ultrasonic_init(&cfg, &sensor));

    while (1)
    {
        if (!ultrasonic_is_busy(sensor))
            ESP_ERROR_CHECK(ultrasonic_start(sensor));

        ultrasonic_result_t res;
        if (xSemaphoreTake(result_mutex, portMAX_DELAY) == pdTRUE)
        {
            if (result_ready)
            {
                res = last_result;
                result_ready = false;
                if (res.status == ESP_OK)
                    printf("Distance: %" PRIu32 " mm (%.3f m), echo %" PRIu32 " us\n",
                           res.distance_mm, res.distance_m, res.echo_us);
                else
                    printf("Measurement failed: %s\n", result_name(res.status));
            }
            xSemaphoreGive(result_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(INTERVAL_MS));
    }
}
