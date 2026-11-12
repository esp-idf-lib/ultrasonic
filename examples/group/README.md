# Example for `ultrasonic` component: a group of sensors

## What it does

Several ultrasonic sensors can share one `TRIGGER` line: a group starts all of
them with a single pulse, so the sensors do not hear each other's trigger and
the measurements are taken at the same moment.

The example starts a group of two sensors every 100 ms and prints both results:

```
Sensor 1 (GPIO16): 252 mm (0.252 m), echo 1524 us
Sensor 2 (GPIO18): ESP_ERR_ULTRASONIC_NO_ECHO
```

Results come from the per-sensor callback, the main loop only prints them.

## Wiring

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_TRIGGER_GPIO` | GPIO number connected to the `TRIGGER` pins of all sensors | 17 |
| `CONFIG_EXAMPLE_ECHO_GPIO_1` | GPIO number connected to the `ECHO` pin of the first sensor | 16 |
| `CONFIG_EXAMPLE_ECHO_GPIO_2` | GPIO number connected to the `ECHO` pin of the second sensor | 18 |

The second sensor is optional: without it the example reports
`ESP_ERR_ULTRASONIC_NO_ECHO` for the second entry, which is the expected
behaviour for a missing echo.
