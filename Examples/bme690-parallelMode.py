# FILE: bme690-parallelMode.py
# AUTHOR: Josip Simun Kuci @ Soldered
# BRIEF: Run the BME690 in parallel mode, where the gas sensor sweeps through
#        a heater profile while temperature, pressure and humidity are
#        measured continuously
# WORKS WITH: BME690 breakout board: solde.red/SKU
# LAST UPDATED: 2026-09-16

from bme690 import (
    BME690,
    BME69X_ERROR,
    BME69X_GASM_VALID_MSK,
    BME69X_HEAT_STAB_MSK,
    BME69X_NEW_DATA_MSK,
    BME69X_PARALLEL_MODE,
)
import time

# New data, gas measurement valid and heater stable, all at once.
BME690_VALID_DATA = (
    BME69X_NEW_DATA_MSK | BME69X_GASM_VALID_MSK | BME69X_HEAT_STAB_MSK
)

# Heater temperature profile in degrees Celsius.
HEATER_TEMP = [320, 100, 100, 100, 200, 200, 200, 320, 320, 320]

# Multipliers of the shared heater duration, one per profile step.
HEATER_MUL = [5, 2, 10, 30, 5, 5, 5, 5, 5, 5]

sensor = BME690()

sensor.set_tph()

# The shared heating duration is the total measurement duration minus the
# time needed for the temperature, pressure and humidity measurement.
shared_heatr_dur = 140 - sensor.get_meas_dur(BME69X_PARALLEL_MODE) // 1000

sensor.set_heater_prof(HEATER_TEMP, HEATER_MUL, shared_heatr_dur)
sensor.set_op_mode(BME69X_PARALLEL_MODE)

if sensor.check_status() == BME69X_ERROR:
    raise Exception("BME690 setup failed: " + sensor.status_string())

print(
    "Timestamp(ms), Temperature(C), Pressure(Pa), Humidity(%), "
    "Gas resistance(Ohm), Status, Gas index, Meas index"
)

while True:
    if sensor.fetch_data():
        fields_left = 1

        while fields_left:
            data, fields_left = sensor.get_data()

            # Skip the fields which hold no valid measurement.
            if data.status != BME690_VALID_DATA:
                continue

            print(
                "{}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, 0x{:02x}, {}, "
                "{}".format(
                    time.ticks_ms(),
                    data.temperature,
                    data.pressure,
                    data.humidity,
                    data.gas_resistance,
                    data.status,
                    data.gas_index,
                    data.meas_index,
                )
            )
