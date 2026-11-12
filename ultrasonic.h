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
 * @file ultrasonic.h
 * @defgroup ultrasonic ultrasonic
 * @{
 *
 * ESP-IDF driver for ultrasonic range meters, e.g. HC-SR04, HY-SRF05 and the like
 *
 * Copyright (c) 2026 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __ULTRASONIC_H__
#define __ULTRASONIC_H__

#include <stdint.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <driver/gpio.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_ERR_ULTRASONIC_NO_ECHO      0x200   //!< No echo received (nothing in front of the sensor or out of range)
#define ESP_ERR_ULTRASONIC_BUSY         0x201   //!< A measurement is already running
#define ESP_ERR_ULTRASONIC_EMPTY        0x202   //!< The group has no sensors
#define ESP_ERR_ULTRASONIC_CANCELED     0x203   //!< The measurement was canceled by the caller

/**
 * @brief Group of sensors sharing a trigger line
 *
 * Created by @ref ultrasonic_group_init() and destroyed by
 * @ref ultrasonic_group_delete(). The handle is opaque.
 */
typedef void *ultrasonic_group_t;

/**
 * @brief Sensor handle
 *
 * Created by @ref ultrasonic_init() or @ref ultrasonic_group_add_sensor() and
 * destroyed by @ref ultrasonic_delete() or @ref ultrasonic_group_delete()
 * respectively. The handle is opaque.
 */
typedef void *ultrasonic_sensor_t;

/**
 * @brief Group configuration
 */
typedef struct
{
    gpio_num_t trigger_gpio;    //!< GPIO connected to the TRIG line of all sensors in the group
    uint16_t pulse_us;          //!< Trigger pulse width, microseconds. 0 means 10
} ultrasonic_group_config_t;

/**
 * @brief Measurement result
 */
typedef struct
{
    esp_err_t status;           //!< @ref ESP_OK or one of @ref ESP_ERR_ULTRASONIC_NO_ECHO /
    //!< @ref ESP_ERR_ULTRASONIC_CANCELED
    uint32_t echo_us;           //!< Echo pulse duration, microseconds
    uint32_t distance_mm;       //!< Distance to the object in millimeters
    float distance_m;           //!< Distance to the object in meters
} ultrasonic_result_t;

/**
 * @brief Measurement completion callback
 *
 * Called when a measurement finishes: the echo arrived, the deadline expired,
 * or the measurement was canceled. Runs in a task, never in an interrupt, so
 * ordinary FreeRTOS APIs may be used, but the callback should stay short.
 *
 * With `CONFIG_FREERTOS_USE_TIMERS` the callback runs in the FreeRTOS timer
 * service task, otherwise in the task the driver creates for the group. The
 * result is valid only for the duration of the call.
 *
 * @param sensor Sensor handle
 * @param result Measurement result
 * @param user_ctx Value of @ref ultrasonic_sensor_config_t::user_ctx
 */
typedef void (*ultrasonic_done_cb_t)(ultrasonic_sensor_t sensor, const ultrasonic_result_t *result, void *user_ctx);

/**
 * @brief Sensor configuration
 */
typedef struct
{
    gpio_num_t echo_gpio;               //!< GPIO connected to the ECHO line of this sensor
    uint32_t max_echo_us;               //!< Longest echo to wait for, microseconds. 0 means 38000.
    //!< A longer echo is reported as @ref ESP_ERR_ULTRASONIC_NO_ECHO
    bool pull_down;                     //!< Enable the internal pull-down on the ECHO line
    bool temperature_compensation;      //!< Apply the ambient temperature correction to the speed of sound
    int16_t ambient_temperature_c;      //!< Ambient temperature used when compensation is enabled
    ultrasonic_done_cb_t done_cb;       //!< Optional completion callback, NULL to poll instead
    void *user_ctx;                     //!< Passed to done_cb as is
} ultrasonic_sensor_config_t;

/**
 * @brief Configuration of a sensor with its own trigger line
 */
typedef struct
{
    ultrasonic_group_config_t group;    //!< Trigger line of this sensor
    ultrasonic_sensor_config_t sensor;  //!< Echo line and measurement parameters
} ultrasonic_config_t;

/**
 * A single sensor is created with @ref ultrasonic_init(), started with
 * @ref ultrasonic_start() and read with @ref ultrasonic_get_result(). The result
 * can be collected in three ways:
 *
 * - blocking: @ref ultrasonic_measure() starts a measurement and waits for it;
 * - callback: @ref ultrasonic_sensor_config_t::done_cb is called by the driver
 *   when the result is ready, no polling and no blocking;
 * - polling: @ref ultrasonic_is_busy() tells whether the measurement is still
 *   running, @ref ultrasonic_get_result() collects the result.
 *
 * @code{.c}
 * static void on_result(ultrasonic_sensor_t sensor, const ultrasonic_result_t *result, void *user_ctx)
 * {
 *     if (result->status == ESP_OK)
 *         printf("%" PRIu32 " mm\n", result->distance_mm);
 *     else
 *         printf("measurement failed: %s\n", esp_err_to_name(result->status));
 * }
 *
 * void app_main(void)
 * {
 *     ultrasonic_config_t cfg = {
 *         .group = { .trigger_gpio = GPIO_NUM_17 },
 *         .sensor = { .echo_gpio = GPIO_NUM_16, .done_cb = on_result },
 *     };
 *     ultrasonic_sensor_t sensor;
 *
 *     ESP_ERROR_CHECK(ultrasonic_init(&cfg, &sensor));
 *     ESP_ERROR_CHECK(ultrasonic_start(sensor));
 * }
 * @endcode
 *
 * @ref ultrasonic_measure() blocks until the echo arrives, which never takes
 * longer than `max_echo_us` plus 2 ms: the deadline is armed by a one-shot
 * timer, so a missing echo ends the measurement with
 * @ref ESP_ERR_ULTRASONIC_NO_ECHO instead of hanging.
 *
 * Neither @ref ultrasonic_start() nor @ref ultrasonic_get_result() keep any
 * interrupt disabled and neither busy-waits: the ECHO line is handled by a GPIO
 * interrupt, so the driver is safe to use on a busy system.
 *
 * Several sensors wired to a common TRIG line are handled with the group API:
 * @ref ultrasonic_group_init(), @ref ultrasonic_group_add_sensor(),
 * @ref ultrasonic_group_start() and @ref ultrasonic_group_delete(). The group
 * produces one pulse for all its sensors, so they measure simultaneously; the
 * results are collected per sensor with the same functions as above.
 */

/**
 * @brief Create a single sensor with its own trigger line
 *
 * Installs the global GPIO ISR service if it is not installed yet.
 *
 * @param cfg Sensor configuration
 * @param[out] out Sensor handle
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if cfg or out is NULL, the trigger GPIO is negative,
 *        the echo GPIO is negative, or the echo GPIO is the trigger GPIO
 *      - ESP_ERR_INVALID_STATE if the ISR service or the ISR handler can not be set up
 *      - ESP_ERR_NO_MEM if there is no memory
 */
esp_err_t ultrasonic_init(const ultrasonic_config_t *cfg, ultrasonic_sensor_t *out);

/**
 * @brief Destroy a single sensor
 *
 * Stops a pending measurement, removes the ECHO interrupt and frees the handle.
 * Use @ref ultrasonic_group_delete() for a sensor added to a group.
 *
 * @param sensor Sensor handle
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if sensor is NULL
 *      - ESP_ERR_INVALID_STATE if the sensor belongs to a group
 */
esp_err_t ultrasonic_delete(ultrasonic_sensor_t sensor);

/**
 * @brief Start a measurement on a single sensor
 *
 * Returns immediately after the trigger pulse, the ECHO line is measured by the
 * interrupt handler.
 *
 * @param sensor Sensor handle
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if sensor is NULL
 *      - ESP_ERR_ULTRASONIC_BUSY if a measurement is already running
 */
esp_err_t ultrasonic_start(ultrasonic_sensor_t sensor);

/**
 * @brief Start a measurement and wait for its result
 *
 * Blocking one-call version of @ref ultrasonic_start() followed by
 * @ref ultrasonic_get_result() with an infinite wait. The wait is bounded:
 * `max_echo_us` plus about 2 ms after the trigger pulse the measurement is
 * finished by the deadline timer, so this never hangs.
 *
 * @param sensor Sensor handle
 * @param[out] res Result, see @ref ultrasonic_get_result()
 * @return
 *      - ESP_OK if a result was collected, check `res->status`
 *      - ESP_ERR_INVALID_ARG if sensor or res is NULL
 *      - ESP_ERR_ULTRASONIC_BUSY if a measurement is already running
 */
esp_err_t ultrasonic_measure(ultrasonic_sensor_t sensor, ultrasonic_result_t *res);

/**
 * @brief Check whether a measurement is running
 *
 * @param sensor Sensor handle
 * @return true if a measurement was started and no result is available yet
 */
bool ultrasonic_is_busy(ultrasonic_sensor_t sensor);

/**
 * @brief Get the result of the measurement
 *
 * @param sensor Sensor handle
 * @param[out] res Result. `res->status` is @ref ESP_OK when the measurement
 *             succeeded, @ref ESP_ERR_ULTRASONIC_NO_ECHO when the echo did not
 *             arrive within `max_echo_us`, and @ref ESP_ERR_ULTRASONIC_CANCELED
 *             when the measurement was canceled. Distances are recalculated on
 *             every call, so the ambient temperature can be changed before
 *             collecting a result.
 * @param wait Ticks to wait for the result. 0 polls without blocking
 * @return
 *      - ESP_OK if a result was collected, check `res->status`
 *      - ESP_ERR_INVALID_ARG if sensor or res is NULL
 *      - ESP_ERR_TIMEOUT if wait == 0 and the measurement is still running,
 *        or the result was not ready within `wait`
 */
esp_err_t ultrasonic_get_result(ultrasonic_sensor_t sensor, ultrasonic_result_t *res, TickType_t wait);

/**
 * @brief Cancel a running measurement
 *
 * Stops the timeout timer and makes the pending result
 * @ref ESP_ERR_ULTRASONIC_CANCELED. Does nothing if no measurement is running.
 *
 * @param sensor Sensor handle
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if sensor is NULL
 */
esp_err_t ultrasonic_cancel(ultrasonic_sensor_t sensor);

/**
 * @brief Set the ambient temperature used for the temperature correction
 *
 * @param sensor Sensor handle
 * @param temperature_c Ambient temperature, degrees Celsius
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if sensor is NULL
 */
esp_err_t ultrasonic_set_ambient_temperature(ultrasonic_sensor_t sensor, int16_t temperature_c);

/**
 * @brief Create a group of sensors sharing one trigger line
 *
 * Installs the global GPIO ISR service if it is not installed yet.
 *
 * @param cfg Group configuration
 * @param[out] out Group handle
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if cfg or out is NULL, or the trigger GPIO is negative
 *      - ESP_ERR_INVALID_STATE if the ISR service can not be installed
 *      - ESP_ERR_NO_MEM if there is no memory
 */
esp_err_t ultrasonic_group_init(const ultrasonic_group_config_t *cfg, ultrasonic_group_t *out);

/**
 * @brief Add a sensor to the group
 *
 * Configures the ECHO line as an input with interrupts on both edges.
 *
 * @param group Group handle
 * @param cfg Sensor configuration
 * @param[out] out Sensor handle
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if group or out is NULL, the ECHO GPIO is negative,
 *        or the ECHO GPIO is the group trigger GPIO
 *      - ESP_ERR_INVALID_STATE if the ISR handler can not be added
 *      - ESP_ERR_NO_MEM if there is no memory
 */
esp_err_t ultrasonic_group_add_sensor(ultrasonic_group_t group, const ultrasonic_sensor_config_t *cfg,
                                      ultrasonic_sensor_t *out);

/**
 * @brief Arm all sensors of the group and produce one trigger pulse
 *
 * Returns immediately after the pulse. The results are collected later with
 * @ref ultrasonic_get_result() or delivered via
 * @ref ultrasonic_sensor_config_t::done_cb.
 *
 * If any sensor of the group is still measuring, nothing is started and
 * @ref ESP_ERR_ULTRASONIC_BUSY is returned, so that a group is never left
 * half-armed.
 *
 * @param group Group handle
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if group is NULL
 *      - ESP_ERR_ULTRASONIC_EMPTY if the group has no sensors
 *      - ESP_ERR_ULTRASONIC_BUSY if a measurement is running
 *      - ESP_ERR_TIMEOUT if the group mutex can not be taken
 */
esp_err_t ultrasonic_group_start(ultrasonic_group_t group);

/**
 * @brief Destroy the group and all its sensors
 *
 * Stops pending measurements, removes the ECHO interrupts and frees every
 * sensor handle. Sensor handles become invalid.
 *
 * @param group Group handle
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if group is NULL
 */
esp_err_t ultrasonic_group_delete(ultrasonic_group_t group);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __ULTRASONIC_H__ */
