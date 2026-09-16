# FILE: bme690-sequentialMode.py
# AUTHOR: Josip Simun Kuci @ Soldered
# BRIEF: Run the BME690 in sequential mode, where the sensor steps through a
#        heater profile on its own, sleeping between the measurements
# WORKS WITH: BME690 breakout board: solde.red/SKU
# LAST UPDATED: 2026-09-16

from bme690 import (
    BME690,
    BME69X_ERROR,
    BME69X_ODR_250_MS,
    BME69X_SEQUENTIAL_MODE,
)
import time

# Heater temperature profile in degrees Celsius.
HEATER_TEMP = [320, 100, 100, 100, 200, 200, 200, 320, 320, 320]

# Heating duration profile in milliseconds.
HEATER_DUR = [150, 150, 150, 150, 150, 150, 150, 150, 150, 150]

sensor = BME690()

sensor.set_tph()

# Sleep duration between two measurements of the profile.
sensor.set_seq_sleep(BME69X_ODR_250_MS)

sensor.set_heater_prof(HEATER_TEMP, HEATER_DUR)
sensor.set_op_mode(BME69X_SEQUENTIAL_MODE)

if sensor.check_status() == BME69X_ERROR:
    raise Exception("BME690 setup failed: " + sensor.status_string())

print(
    "Timestamp(ms), Temperature(C), Pressure(Pa), Humidity(%), "
    "Gas resistance(Ohm), Status, Gas index"
)

while True:
    if sensor.fetch_data():
        fields_left = 1

        while fields_left:
            data, fields_left = sensor.get_data()

            print(
                "{}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, 0x{:02x}, {}".format(
                    time.ticks_ms(),
                    data.temperature,
                    data.pressure,
                    data.humidity,
                    data.gas_resistance,
                    data.status,
                    data.gas_index,
                )
            )

    time.sleep_ms(100)
