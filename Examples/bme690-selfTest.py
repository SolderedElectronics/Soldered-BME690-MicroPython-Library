# FILE: bme690-selfTest.py
# AUTHOR: Josip Simun Kuci @ Soldered
# BRIEF: Run the built-in self test of the BME690 and print the result
# WORKS WITH: BME690 breakout board: solde.red/SKU
# LAST UPDATED: 2026-09-16

from bme690 import BME690, BME69X_OK

sensor = BME690()

print("Unique ID: 0x{:08x}".format(sensor.get_unique_id()))
print("Running the self test, this takes a few seconds...")

# The sensor is left in sleep mode with its own configuration afterwards,
# so it has to be set up again before it is used for measurements.
if sensor.self_test() == BME69X_OK:
    print("Self test passed.")
else:
    print("Self test failed: " + sensor.status_string())
