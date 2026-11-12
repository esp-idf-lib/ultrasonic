#include <inttypes.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <ultrasonic.h>

#define INTERVAL_MS CONFIG_EXAMPLE_MEASURE_INTERVAL_MS
#define SENSOR_COUNT 2

static SemaphoreHandle_t result_mutex;
static ultrasonic_result_t results[SENSOR_COUNT];
static bool result_ready[SENSOR_COUNT];

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

    int index = (int)(intptr_t)user_ctx;
    if (index < 0 || index >= SENSOR_COUNT)
        return;

    if (xSemaphoreTake(result_mutex, 0) != pdTRUE)
        return;

    results[index] = *result;
    result_ready[index] = true;
    xSemaphoreGive(result_mutex);
}

void app_main(void)
{
    result_mutex = xSemaphoreCreateMutex();
    configASSERT(result_mutex);

    ultrasonic_group_config_t group_cfg =
    {
        .trigger_gpio = CONFIG_EXAMPLE_TRIGGER_GPIO,
    };

    ultrasonic_group_t group;
    ESP_ERROR_CHECK(ultrasonic_group_init(&group_cfg, &group));

    const gpio_num_t echo_pins[SENSOR_COUNT] =
    {
        CONFIG_EXAMPLE_ECHO_GPIO_1,
        CONFIG_EXAMPLE_ECHO_GPIO_2,
    };

    ultrasonic_sensor_t sensors[SENSOR_COUNT];
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        ultrasonic_sensor_config_t sensor_cfg =
        {
            .echo_gpio = echo_pins[i],
            .max_echo_us = 38000,
            .done_cb = on_measurement,
            .user_ctx = (void *)(intptr_t)i,
        };
        ESP_ERROR_CHECK(ultrasonic_group_add_sensor(group, &sensor_cfg, &sensors[i]));
    }

    while (1)
    {
        esp_err_t res = ultrasonic_group_start(group);
        if (res != ESP_OK)
            printf("group_start: %s\n", result_name(res));

        vTaskDelay(pdMS_TO_TICKS(INTERVAL_MS));

        if (xSemaphoreTake(result_mutex, portMAX_DELAY) == pdTRUE)
        {
            for (int i = 0; i < SENSOR_COUNT; i++)
            {
                if (!result_ready[i])
                    continue;

                result_ready[i] = false;
                ultrasonic_result_t r = results[i];

                if (r.status == ESP_OK)
                    printf("Sensor %d (GPIO%d): %" PRIu32 " mm (%.3f m), echo %" PRIu32 " us\n",
                           i + 1, echo_pins[i], r.distance_mm, r.distance_m, r.echo_us);
                else
                    printf("Sensor %d (GPIO%d): %s\n", i + 1, echo_pins[i], result_name(r.status));
            }
            xSemaphoreGive(result_mutex);
        }
    }
}
