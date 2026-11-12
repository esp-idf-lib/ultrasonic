/*
 * Copyright (c) 2026 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file ultrasonic.c
 *
 * ESP-IDF driver for ultrasonic range meters, e.g. HC-SR04, HY-SRF05 and the like
 *
 * Copyright (c) 2026 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <stdlib.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_rom_sys.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "ultrasonic.h"

#define DEFAULT_PULSE_US                 10
#define DEFAULT_MAX_ECHO_US              38000

// Added to the maximal echo duration so that an echo at the very border of the
// range is not cut off, and to cover the delay between the trigger pulse and
// the beginning of the echo.
#define TIMEOUT_MARGIN_US                2000

// Speed of sound in mm/s: v(T) = 331300 + 606 * T
#define SPEED_OF_SOUND_AT_0C_MM_S        331300
#define SPEED_OF_SOUND_PER_C_MM_S        606

static const char *TAG = "ultrasonic";

// The trigger line is shared by a group, and two groups on one line would
// pulse each other's sensors, so the pin is reserved for the lifetime of
// the group that owns it
static portMUX_TYPE trigger_mux = portMUX_INITIALIZER_UNLOCKED;
static bool isr_service_ready;
static bool trigger_in_use[GPIO_NUM_MAX];

static bool trigger_acquire(gpio_num_t pin)
{
    bool acquired = false;

    portENTER_CRITICAL(&trigger_mux);
    if (!trigger_in_use[pin])
    {
        trigger_in_use[pin] = true;
        acquired = true;
    }
    portEXIT_CRITICAL(&trigger_mux);

    return acquired;
}

static void trigger_release(gpio_num_t pin)
{
    portENTER_CRITICAL(&trigger_mux);
    trigger_in_use[pin] = false;
    portEXIT_CRITICAL(&trigger_mux);
}

typedef enum
{
    USS_IDLE,
    USS_ARMED,
    USS_ECHO_STARTED,
} ultrasonic_state_t;

typedef struct
{
    ultrasonic_group_t group;

    gpio_num_t echo_gpio;
    uint32_t max_echo_us;
    bool temperature_compensation;
    int16_t ambient_temperature_c;
    bool owns_group;
    ultrasonic_done_cb_t done_cb;
    void *user_ctx;

    SemaphoreHandle_t done;
    esp_timer_handle_t timeout;

    portMUX_TYPE mux;
    volatile ultrasonic_state_t state;
    int64_t echo_start_us;

    ultrasonic_result_t result;
} ultrasonic_sensor_impl_t;

typedef struct
{
    gpio_num_t trigger_gpio;
    uint16_t pulse_us;

    SemaphoreHandle_t mutex;

    ultrasonic_sensor_impl_t **sensors;
    size_t count;
    size_t capacity;
} ultrasonic_group_impl_t;

/**
 * esp_timer_delete() refuses to delete a timer whose callback is running, so
 * wait for the callback to finish instead of freeing the object it uses
 */
static void delete_timer(esp_timer_handle_t timer)
{
    for (int i = 0; i < 100; i++)
    {
        if (esp_timer_delete(timer) == ESP_OK)
            return;
        vTaskDelay(1);
    }
    ESP_LOGE(TAG, "Could not delete the timer, its callback never finished");
}

static void convert_distance(ultrasonic_result_t *res, bool compensation, int16_t temperature_c)
{
    int32_t speed_mm_s = SPEED_OF_SOUND_AT_0C_MM_S;
    if (compensation)
        speed_mm_s += SPEED_OF_SOUND_PER_C_MM_S * temperature_c;

    // The echo travels to the object and back, hence the factor of two
    res->distance_mm = (uint32_t)(((uint64_t)res->echo_us * (uint64_t)speed_mm_s) / 2000000ULL);
    res->distance_m = (float)((double)res->echo_us * speed_mm_s / 2000000000.0);
}

/**
 * Runs in the FreeRTOS timer service task: the ISR and the esp_timer callback
 * only pend this function, so the user callback never runs in an ISR.
 */
static void notify_done(void *arg1, uint32_t arg2)
{
    ultrasonic_sensor_impl_t *sensor = arg1;
    (void)arg2;

    if (!sensor->done_cb)
        return;

    portENTER_CRITICAL(&sensor->mux);
    ultrasonic_result_t res = sensor->result;
    bool compensation = sensor->temperature_compensation;
    int16_t temperature_c = sensor->ambient_temperature_c;
    portEXIT_CRITICAL(&sensor->mux);

    if (res.status == ESP_OK)
        convert_distance(&res, compensation, temperature_c);

    sensor->done_cb(sensor, &res, sensor->user_ctx);
}

#if INCLUDE_xTimerPendFunctionCall
static void pend_notify_from_isr(ultrasonic_sensor_impl_t *sensor, BaseType_t *woken)
{
    if (!sensor->done_cb)
        return;

    if (xTimerPendFunctionCallFromISR(notify_done, sensor, 0, woken) != pdPASS)
        ESP_EARLY_LOGW(TAG, "Timer queue is full, the callback is lost");
}

static void pend_notify(ultrasonic_sensor_impl_t *sensor)
{
    if (!sensor->done_cb)
        return;

    if (xTimerPendFunctionCall(notify_done, sensor, 0, 0) != pdPASS)
        ESP_LOGW(TAG, "Timer queue is full, the callback is lost");
}
#else
static void pend_notify_from_isr(ultrasonic_sensor_impl_t *sensor, BaseType_t *woken)
{
    (void)sensor;
    (void)woken;
}

static void pend_notify(ultrasonic_sensor_impl_t *sensor)
{
    (void)sensor;
}
#endif

/**
 * GPIO interrupt handler, called on both edges of the ECHO line.
 *
 * The first edge starts the measurement, the second one finishes it. Edges
 * outside a measurement are ignored, so the interrupt stays enabled all the
 * time and no interrupt is ever masked by the driver.
 */
static void IRAM_ATTR echo_isr(void *arg)
{
    ultrasonic_sensor_impl_t *sensor = arg;
    int64_t now = esp_timer_get_time();
    bool notify = false;

    portENTER_CRITICAL_ISR(&sensor->mux);
    if (sensor->state == USS_ARMED)
    {
        sensor->echo_start_us = now;
        sensor->state = USS_ECHO_STARTED;
    }
    else if (sensor->state == USS_ECHO_STARTED)
    {
        sensor->result.status = ESP_OK;
        sensor->result.echo_us = (uint32_t)(now - sensor->echo_start_us);
        sensor->state = USS_IDLE;
        notify = true;
    }
    portEXIT_CRITICAL_ISR(&sensor->mux);

    if (notify)
    {
        BaseType_t woken = pdFALSE;
        xSemaphoreGiveFromISR(sensor->done, &woken);
        pend_notify_from_isr(sensor, &woken);
        if (woken)
            portYIELD_FROM_ISR();
    }
}

/**
 * One-shot timer callback, runs in the esp_timer task.
 *
 * The deadline means either that there was no echo at all, or that the echo is
 * longer than max_echo_us, i.e. the object is out of range.
 */
static void timeout_cb(void *arg)
{
    ultrasonic_sensor_impl_t *sensor = arg;
    bool notify = false;

    portENTER_CRITICAL(&sensor->mux);
    if (sensor->state != USS_IDLE)
    {
        sensor->result.status = ESP_ERR_ULTRASONIC_NO_ECHO;
        sensor->result.echo_us = 0;
        sensor->state = USS_IDLE;
        notify = true;
    }
    portEXIT_CRITICAL(&sensor->mux);

    if (notify)
    {
        xSemaphoreGive(sensor->done);
        pend_notify(sensor);
    }
}

static void trigger_pulse(const ultrasonic_group_impl_t *group)
{
    gpio_set_level(group->trigger_gpio, 1);
    esp_rom_delay_us(group->pulse_us);
    gpio_set_level(group->trigger_gpio, 0);
}

static esp_err_t sensor_arm(ultrasonic_sensor_impl_t *sensor)
{
    portENTER_CRITICAL(&sensor->mux);
    bool idle = sensor->state == USS_IDLE;
    if (idle)
    {
        sensor->state = USS_ARMED;
        sensor->result.status = ESP_OK;
        sensor->result.echo_us = 0;
        sensor->result.distance_mm = 0;
        sensor->result.distance_m = 0;
    }
    portEXIT_CRITICAL(&sensor->mux);

    if (!idle)
        return ESP_ERR_ULTRASONIC_BUSY;

    // Drop a result nobody collected, and re-arm the deadline
    xSemaphoreTake(sensor->done, 0);
    esp_timer_stop(sensor->timeout);

    esp_err_t res = esp_timer_start_once(sensor->timeout, sensor->max_echo_us + TIMEOUT_MARGIN_US);
    if (res != ESP_OK)
    {
        portENTER_CRITICAL(&sensor->mux);
        sensor->state = USS_IDLE;
        portEXIT_CRITICAL(&sensor->mux);
    }

    return res;
}

esp_err_t ultrasonic_group_init(const ultrasonic_group_config_t *cfg, ultrasonic_group_t *out)
{
    if (!cfg || !out || cfg->trigger_gpio < 0 || cfg->trigger_gpio >= GPIO_NUM_MAX)
        return ESP_ERR_INVALID_ARG;

    // The ISR service is global, so it is installed once and never uninstalled
    // here: other components may use it as well
    esp_err_t res = ESP_OK;
    if (!isr_service_ready)
    {
        res = gpio_install_isr_service(0);
        if (res == ESP_OK || res == ESP_ERR_INVALID_STATE)
        {
            // ESP_ERR_INVALID_STATE means the service is already installed
            isr_service_ready = true;
            res = ESP_OK;
        }
        else
            ESP_LOGE(TAG, "Could not install the GPIO ISR service: %s", esp_err_to_name(res));
    }
    if (res != ESP_OK)
        return res;

    if (!trigger_acquire(cfg->trigger_gpio))
    {
        ESP_LOGE(TAG, "GPIO%d is already used as a trigger by another group", cfg->trigger_gpio);
        return ESP_ERR_INVALID_STATE;
    }

    gpio_config_t io_conf =
    {
        .pin_bit_mask = 1ULL << cfg->trigger_gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    res = gpio_config(&io_conf);
    if (res != ESP_OK)
        goto fail_release;

    res = gpio_set_level(cfg->trigger_gpio, 0);
    if (res != ESP_OK)
        goto fail_release;

    ultrasonic_group_impl_t *group = calloc(1, sizeof(ultrasonic_group_impl_t));
    if (!group)
    {
        res = ESP_ERR_NO_MEM;
        goto fail_release;
    }

    group->trigger_gpio = cfg->trigger_gpio;
    group->pulse_us = cfg->pulse_us ? cfg->pulse_us : DEFAULT_PULSE_US;
    group->mutex = xSemaphoreCreateMutex();
    if (!group->mutex)
    {
        free(group);
        res = ESP_ERR_NO_MEM;
        goto fail_release;
    }

    *out = group;
    return ESP_OK;

fail_release:
    gpio_reset_pin(cfg->trigger_gpio);
    trigger_release(cfg->trigger_gpio);
    return res;
}

esp_err_t ultrasonic_group_add_sensor(ultrasonic_group_t group, const ultrasonic_sensor_config_t *cfg,
                                      ultrasonic_sensor_t *out)
{
    ultrasonic_group_impl_t *g = group;

    if (!g || !cfg || !out || cfg->echo_gpio < 0)
        return ESP_ERR_INVALID_ARG;

    if (cfg->echo_gpio == g->trigger_gpio)
    {
        ESP_LOGE(TAG, "ECHO and TRIG share the same GPIO %d", cfg->echo_gpio);
        return ESP_ERR_INVALID_ARG;
    }

    ultrasonic_sensor_impl_t *sensor = calloc(1, sizeof(ultrasonic_sensor_impl_t));
    if (!sensor)
        return ESP_ERR_NO_MEM;

    sensor->group = group;
    sensor->echo_gpio = cfg->echo_gpio;
    sensor->max_echo_us = cfg->max_echo_us ? cfg->max_echo_us : DEFAULT_MAX_ECHO_US;
    sensor->temperature_compensation = cfg->temperature_compensation;
    sensor->ambient_temperature_c = cfg->ambient_temperature_c;
    sensor->done_cb = cfg->done_cb;
    sensor->user_ctx = cfg->user_ctx;
    sensor->mux = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
    sensor->state = USS_IDLE;

    gpio_config_t io_conf =
    {
        .pin_bit_mask = 1ULL << cfg->echo_gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = cfg->pull_down ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    esp_err_t res = gpio_config(&io_conf);
    if (res != ESP_OK)
        goto fail;

    sensor->done = xSemaphoreCreateBinary();
    if (!sensor->done)
    {
        res = ESP_ERR_NO_MEM;
        goto fail;
    }

    esp_timer_create_args_t timer_args =
    {
        .callback = timeout_cb,
        .arg = sensor,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "ultrasonic",
    };
    res = esp_timer_create(&timer_args, &sensor->timeout);
    if (res != ESP_OK)
        goto fail;

    res = gpio_isr_handler_add(cfg->echo_gpio, echo_isr, sensor);
    if (res != ESP_OK)
        goto fail;

    if (xSemaphoreTake(g->mutex, portMAX_DELAY) != pdTRUE)
    {
        res = ESP_ERR_TIMEOUT;
        goto fail_isr;
    }

    if (g->count == g->capacity)
    {
        size_t capacity = g->capacity ? g->capacity * 2 : 4;
        ultrasonic_sensor_impl_t **sensors = realloc(g->sensors, capacity * sizeof(ultrasonic_sensor_impl_t *));
        if (!sensors)
        {
            xSemaphoreGive(g->mutex);
            res = ESP_ERR_NO_MEM;
            goto fail_isr;
        }
        g->sensors = sensors;
        g->capacity = capacity;
    }

    g->sensors[g->count++] = sensor;
    xSemaphoreGive(g->mutex);

    *out = sensor;
    return ESP_OK;

fail_isr:
    gpio_isr_handler_remove(cfg->echo_gpio);
fail:
    if (sensor->timeout)
        esp_timer_delete(sensor->timeout);
    if (sensor->done)
        vSemaphoreDelete(sensor->done);
    free(sensor);
    return res;
}

static esp_err_t collect_result(ultrasonic_sensor_impl_t *sensor, ultrasonic_result_t *res, TickType_t wait)
{
    if (xSemaphoreTake(sensor->done, wait) != pdTRUE)
        return wait ? ESP_ERR_TIMEOUT : ESP_ERR_ULTRASONIC_EMPTY;

    bool compensation;
    int16_t temperature_c;

    portENTER_CRITICAL(&sensor->mux);
    *res = sensor->result;
    compensation = sensor->temperature_compensation;
    temperature_c = sensor->ambient_temperature_c;
    portEXIT_CRITICAL(&sensor->mux);

    if (res->status == ESP_OK)
        convert_distance(res, compensation, temperature_c);

    return ESP_OK;
}

esp_err_t ultrasonic_group_start(ultrasonic_group_t group)
{
    ultrasonic_group_impl_t *g = group;

    if (!g)
        return ESP_ERR_INVALID_ARG;

    if (xSemaphoreTake(g->mutex, portMAX_DELAY) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    esp_err_t res = g->count ? ESP_OK : ESP_ERR_ULTRASONIC_EMPTY;

    // Check the whole group first: a half-armed group would wait for the
    // deadline on the sensors that were already started
    for (size_t i = 0; res == ESP_OK && i < g->count; i++)
    {
        if (ultrasonic_is_busy(g->sensors[i]))
            res = ESP_ERR_ULTRASONIC_BUSY;
    }

    for (size_t i = 0; res == ESP_OK && i < g->count; i++)
        res = sensor_arm(g->sensors[i]);

    if (res == ESP_OK)
        trigger_pulse(g);

    xSemaphoreGive(g->mutex);
    return res;
}

esp_err_t ultrasonic_group_delete(ultrasonic_group_t group)
{
    ultrasonic_group_impl_t *g = group;

    if (!g)
        return ESP_ERR_INVALID_ARG;

    if (xSemaphoreTake(g->mutex, portMAX_DELAY) == pdTRUE)
    {
        for (size_t i = 0; i < g->count; i++)
        {
            ultrasonic_sensor_impl_t *sensor = g->sensors[i];

            gpio_isr_handler_remove(sensor->echo_gpio);
            gpio_set_intr_type(sensor->echo_gpio, GPIO_INTR_DISABLE);

            esp_timer_stop(sensor->timeout);
            delete_timer(sensor->timeout);

            vSemaphoreDelete(sensor->done);
            free(sensor);
        }

        free(g->sensors);
        g->sensors = NULL;
        g->count = 0;
        g->capacity = 0;

        xSemaphoreGive(g->mutex);
        vSemaphoreDelete(g->mutex);
    }
    else
    {
        ESP_LOGE(TAG, "Could not take the group mutex, sensors are leaked");
    }

    gpio_set_level(g->trigger_gpio, 0);
    gpio_reset_pin(g->trigger_gpio);
    trigger_release(g->trigger_gpio);
    free(g);
    return ESP_OK;
}

esp_err_t ultrasonic_init(const ultrasonic_config_t *cfg, ultrasonic_sensor_t *out)
{
    if (!cfg || !out)
        return ESP_ERR_INVALID_ARG;

    ultrasonic_group_t group;
    esp_err_t res = ultrasonic_group_init(&cfg->group, &group);
    if (res != ESP_OK)
        return res;

    res = ultrasonic_group_add_sensor(group, &cfg->sensor, out);
    if (res != ESP_OK)
    {
        ultrasonic_group_delete(group);
        return res;
    }

    // The sensor owns the group it was just created with, so that
    // ultrasonic_delete() removes both
    ((ultrasonic_sensor_impl_t *)*out)->owns_group = true;

    return ESP_OK;
}

esp_err_t ultrasonic_delete(ultrasonic_sensor_t sensor)
{
    ultrasonic_sensor_impl_t *s = sensor;

    if (!s)
        return ESP_ERR_INVALID_ARG;

    if (!s->owns_group)
        return ESP_ERR_INVALID_STATE;

    return ultrasonic_group_delete(s->group);
}

esp_err_t ultrasonic_start(ultrasonic_sensor_t sensor)
{
    ultrasonic_sensor_impl_t *s = sensor;

    if (!s)
        return ESP_ERR_INVALID_ARG;

    // A sensor of a multi-sensor group is started together with its group,
    // a pulse from a single sensor would start all of them
    if (!s->owns_group)
        return ESP_ERR_INVALID_STATE;

    ultrasonic_group_impl_t *g = s->group;

    if (xSemaphoreTake(g->mutex, portMAX_DELAY) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    esp_err_t res = sensor_arm(s);
    if (res == ESP_OK)
        trigger_pulse(g);

    xSemaphoreGive(g->mutex);
    return res;
}

esp_err_t ultrasonic_measure(ultrasonic_sensor_t sensor, ultrasonic_result_t *res)
{
    if (!sensor || !res)
        return ESP_ERR_INVALID_ARG;

    esp_err_t err = ultrasonic_start(sensor);
    if (err != ESP_OK)
        return err;

    // The deadline timer always completes the measurement, so an infinite wait
    // here is safe
    return ultrasonic_get_result(sensor, res, portMAX_DELAY);
}

bool ultrasonic_is_busy(ultrasonic_sensor_t sensor)
{
    ultrasonic_sensor_impl_t *s = sensor;

    if (!s)
        return false;

    portENTER_CRITICAL(&s->mux);
    bool busy = s->state != USS_IDLE;
    portEXIT_CRITICAL(&s->mux);

    return busy;
}

esp_err_t ultrasonic_get_result(ultrasonic_sensor_t sensor, ultrasonic_result_t *res, TickType_t wait)
{
    ultrasonic_sensor_impl_t *s = sensor;

    if (!s || !res)
        return ESP_ERR_INVALID_ARG;

    return collect_result(s, res, wait);
}

esp_err_t ultrasonic_cancel(ultrasonic_sensor_t sensor)
{
    ultrasonic_sensor_impl_t *s = sensor;
    if (!s)
        return ESP_ERR_INVALID_ARG;

    esp_timer_stop(s->timeout);

    bool notify = false;

    portENTER_CRITICAL(&s->mux);
    if (s->state != USS_IDLE)
    {
        s->result.status = ESP_ERR_ULTRASONIC_CANCELED;
        s->result.echo_us = 0;
        s->state = USS_IDLE;
        notify = true;
    }
    portEXIT_CRITICAL(&s->mux);

    if (notify)
    {
        xSemaphoreTake(s->done, 0);
        xSemaphoreGive(s->done);
        pend_notify(s);
    }

    return ESP_OK;
}

esp_err_t ultrasonic_set_ambient_temperature(ultrasonic_sensor_t sensor, int16_t temperature_c)
{
    ultrasonic_sensor_impl_t *s = sensor;

    if (!s)
        return ESP_ERR_INVALID_ARG;

    portENTER_CRITICAL(&s->mux);
    s->ambient_temperature_c = temperature_c;
    portEXIT_CRITICAL(&s->mux);

    return ESP_OK;
}
