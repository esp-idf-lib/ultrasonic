# Example for `ultrasonic` component

## What it does

Measures the distance to an obstacle in a loop and prints it into the console:

```
Distance: 252 mm (0.252 m), echo 1524 us
```

The measurement itself does not block a task: `ultrasonic_start()` only sends
the trigger pulse, the result is printed by the main loop when the sensor
reports it. The example is compatible with any ultrasonic module with
`TRIGGER` and `ECHO` pins.

## Wiring

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_TRIGGER_GPIO` | GPIO number connected to the `TRIGGER` pin | 17 |
| `CONFIG_EXAMPLE_ECHO_GPIO` | GPIO number connected to the `ECHO` pin | 16 |
