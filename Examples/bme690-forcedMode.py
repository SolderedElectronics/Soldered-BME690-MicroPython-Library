# FILE: bme690-forcedMode.py
# AUTHOR: Josip Simun Kuci @ Soldered
# BRIEF: Read temperature, pressure, humidity and gas resistance from the
#        BME690 in forced mode, one measurement at a time
# WORKS WITH: BME690 breakout board: solde.red/SKU
# LAST UPDATED: 2026-09-16

from bme690 import (
    BME690,
    BME69X_ERROR,
    BME69X_FILTER_SIZE_3,
    BME69X_FORCED_MODE,
)
import time

# Start the sensor on the default I2C address (0x76). Use
# BME690(address=BME69X_I2C_ADDR_HIGH) if the address jumper is soldered.
sensor = BME690()

# Oversampling for temperature, pressure and humidity.
sensor.set_tph()

# IIR filter for the temperature and pressure readings.
sensor.set_filter(BME69X_FILTER_SIZE_3)

# Heat the gas sensor plate to 300 degrees Celsius for 100 milliseconds.
sensor.set_heater_prof(300, 100)

if sensor.check_status() == BME69X_ERROR:
    raise Exception("BME690 setup failed: " + sensor.status_string())

print(
    "Timestamp(ms), Temperature(C), Pressure(Pa), Humidity(%), "
    "Gas resistance(Ohm), Status"
)

while True:
    # Trigger a single measurement.
    sensor.set_op_mode(BME69X_FORCED_MODE)

    # Wait for the measurement to finish, the heater duration included.
    time.sleep_us(sensor.get_meas_dur())

    if sensor.fetch_data():
        data, _ = sensor.get_data()

        print(
            "{}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, 0x{:02x}".format(
                time.ticks_ms(),
                data.temperature,
                data.pressure,
                data.humidity,
                data.gas_resistance,
                data.status,
            )
        )

    time.sleep(1)
