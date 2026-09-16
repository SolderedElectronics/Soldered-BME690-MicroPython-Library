# FILE: bme690-aiStudioLogger.py
# AUTHOR: Josip Simun Kuci @ Soldered
# BRIEF: Record BME690 raw data into a .bmerawdata file on an SD card, in the
#        Bosch BME AI-Studio Raw Data Format, so that it can be imported into
#        BME AI-Studio and used to train a gas classification algorithm
# WORKS WITH: BME690 breakout board: solde.red/SKU
# LAST UPDATED: 2026-09-16
#
# The example runs the sensor in parallel mode with a ten step heater profile,
# logs every valid gas measurement for LOG_DURATION_MS and then closes the
# file.
#
# AI-Studio splits a recording into Specimens wherever the label tag changes,
# and it measures the duration of a Specimen from its first to its last point.
# A recording whose label tag never changes therefore imports as a Specimen of
# zero length. The recording starts already tagged, and the button on
# LABEL_BUTTON_PIN switches the tag while it runs, which marks the moment as
# the start of a new Specimen. That is the same thing buttons S1 and S2 do on
# the BME688 Development Kit.
#
# Needed hardware:
# - an ESP32 board (WiFi and NTP are used for the real time clock)
# - a BME690 breakout on the I2C pins, or connected via Qwiic
# - an SD card module on the SPI pins, chip select on SD_CS
#
# The AI-Studio workflow is described here:
# https://www.bosch-sensortec.com/software/bme/docs/overview/getting-started.html

from bme690 import (
    BME690,
    BME69X_ERROR,
    BME69X_GASM_VALID_MSK,
    BME69X_HEAT_STAB_MSK,
    BME69X_NEW_DATA_MSK,
    BME69X_PARALLEL_MODE,
    BME69X_WARNING,
)
from machine import Pin, SDCard
import json
import network
import ntptime
import os
import random
import time

# WiFi credentials, only used to get the current time over NTP.
SSID = "YOUR_SSID_HERE"
PASSWORD = "YOUR_PASSWORD_HERE"

# Pins of the SD card module, wired to the SPI bus.
SD_SCK = 18
SD_MISO = 19
SD_MOSI = 23
SD_CS = 5

# Button which marks the start of a new Specimen while recording. It is
# active low, so the internal pull up is enough. Set it to None to record the
# whole session as a single Specimen.
LABEL_BUTTON_PIN = 0

# Debounce time of that button.
LABEL_BUTTON_DEBOUNCE_MS = 250

# Total duration of one measurement session.
LOG_DURATION_MS = 10 * 60000

# Duration of a single heater profile step in milliseconds. The Raw Data
# Format expects the heater profile time base to be 140 ms.
MEAS_DUR = 140

# New data, gas measurement valid and heater stable, all at once.
BME690_VALID_DATA = (
    BME69X_NEW_DATA_MSK | BME69X_GASM_VALID_MSK | BME69X_HEAT_STAB_MSK
)

# Identifier of the board which recorded the data. AI-Studio uses it to tell
# recordings of different boards apart, so give every board its own value.
BOARD_ID = "E0E2E69BA804"

# Unique id of the sensor element, used to trace a row back to one sensor.
SENSOR_ID = 1903381786

# Index of the sensor on the board. A single BME690 breakout only has one.
SENSOR_INDEX = 0

# Heater temperature profile in degrees Celsius.
HEATER_TEMP = [320, 100, 100, 100, 200, 200, 200, 320, 320, 320]

# Multipliers of the shared heater duration, one per profile step.
HEATER_MUL = [5, 2, 10, 30, 5, 5, 5, 5, 5, 5]

HEATER_LEN = 10

# MicroPython counts the seconds from the year 2000, the format wants them
# counted from 1970.
UNIX_EPOCH_OFFSET = 946684800

# The column description of the data block. The order of the columns here is
# the order of the values in every recorded row.
COLUMNS = [
    ("Sensor Index", "", "integer", "sensor_index"),
    ("Sensor ID", "", "integer", "sensor_id"),
    (
        "Time Since PowerOn",
        "Milliseconds",
        "integer",
        "timestamp_since_poweron",
    ),
    (
        "Real time clock",
        "Unix Timestamp: seconds since Jan 01 1970. (UTC); 0 = missing",
        "integer",
        "real_time_clock",
    ),
    ("Temperature", "DegreesCelcius", "float", "temperature"),
    ("Pressure", "Hectopascals", "float", "pressure"),
    ("Relative Humidity", "Percent", "float", "relative_humidity"),
    ("Resistance Gassensor", "Ohms", "float", "resistance_gassensor"),
    (
        "Heater Profile Step Index",
        "",
        "integer",
        "heater_profile_step_index",
    ),
    ("Scanning Mode Enabled", "", "boolean", "scanning_enabled"),
    ("Scanning Cycle Index", "", "integer", "scanning_cycle_index"),
    ("Label Tag", "", "integer", "label_tag"),
    ("Error Code", "", "integer", "error_code"),
]


def check_sensor_status(sensor):
    """Stop the example if the sensor reported an error, print warnings."""
    status = sensor.check_status()

    if status == BME69X_ERROR:
        raise Exception("BME690 error: " + sensor.status_string())

    if status == BME69X_WARNING:
        print("BME690 warning: " + sensor.status_string())


def iso8601(unix_time):
    """Format a unix timestamp as an ISO 8601 UTC string."""
    parts = time.gmtime(unix_time - UNIX_EPOCH_OFFSET)

    return "{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}+00:00".format(
        parts[0], parts[1], parts[2], parts[3], parts[4], parts[5]
    )


def file_name_date(unix_time):
    """Format a unix timestamp as the yyyy_mm_dd_hh_mm used in a file name."""
    parts = time.gmtime(unix_time - UNIX_EPOCH_OFFSET)

    return "{:04d}_{:02d}_{:02d}_{:02d}_{:02d}".format(
        parts[0], parts[1], parts[2], parts[3], parts[4]
    )


def make_seed():
    """
    Build the random seed which labels all files of one session.

    The format expects sixteen lowercase alphanumeric characters.
    """
    alphabet = "abcdefghijklmnopqrstuvwxyz0123456789"

    return "".join(random.choice(alphabet) for _ in range(16))


def connect_wifi():
    """Connect to WiFi, the format stores an absolute creation date."""
    station = network.WLAN(network.STA_IF)
    station.active(True)

    if not station.isconnected():
        station.connect(SSID, PASSWORD)
        print("Connecting to WiFi", end="")
        while not station.isconnected():
            print(".", end="")
            time.sleep_ms(500)
        print("\nWiFi connected")

    return station


def sync_time():
    """Set the real time clock over NTP and return the unix time."""
    print("Waiting for NTP", end="")
    while True:
        try:
            ntptime.settime()
            break
        except Exception:
            print(".", end="")
            time.sleep_ms(200)
    print("\nTime synced")

    return time.time() + UNIX_EPOCH_OFFSET


def mount_sd():
    """Mount the SD card on /sd."""
    card = SDCard(
        slot=3, sck=SD_SCK, miso=SD_MISO, mosi=SD_MOSI, cs=SD_CS
    )
    os.mount(card, "/sd")
    print("SD card ready")

    return card


def build_header(now, seed_power_on_off):
    """
    Build everything which comes before the measurements themselves.

    A ten minute recording holds a few thousand rows, which is far more than
    fits in RAM as one document, so the header is written out first with its
    data block left open, and the rows are appended as they are measured.
    """
    iso_now = iso8601(now)

    # The board configuration, the same two objects are stored in a
    # .bmeconfig file by AI-Studio.
    config_header = {
        "dateCreated_ISO": iso_now,
        "appVersion": "2.2.0",
        "boardType": "soldered_bme690",
        "boardMode": "burn_in",
        "boardLayout": "grouped",
    }

    config_body = {
        "heaterProfiles": [
            {
                "id": "heater_354",
                "timeBase": MEAS_DUR,
                "temperatureTimeVectors": [
                    [HEATER_TEMP[i], HEATER_MUL[i]]
                    for i in range(HEATER_LEN)
                ],
            }
        ],
        # The sensor scans continuously, so there are no sleeping cycles.
        "dutyCycleProfiles": [
            {
                "id": "duty_1",
                "numberScanningCycles": 1,
                "numberSleepingCycles": 0,
            }
        ],
        "sensorConfigurations": [
            {
                "sensorIndex": SENSOR_INDEX,
                "heaterProfile": "heater_354",
                "dutyCycleProfile": "duty_1",
            }
        ],
    }

    # The header of the recording itself, the counters and the seed repeat
    # the matching parts of the file name.
    raw_data_header = {
        "counterPowerOnOff": 1,
        "seedPowerOnOff": seed_power_on_off,
        "counterFileLimit": 0,
        "dateCreated": str(now),
        "dateCreated_ISO": iso_now,
        "firmwareVersion": "1.0.0",
        "boardId": BOARD_ID,
    }

    data_columns = [
        {
            "name": column[0],
            "unit": column[1],
            "format": column[2],
            "key": column[3],
            "colId": index + 1,
        }
        for index, column in enumerate(COLUMNS)
    ]

    # The objects are joined by hand instead of dumping one document, so that
    # the data block stays the last member of the last object.
    return (
        '{"configHeader":'
        + json.dumps(config_header)
        + ',"configBody":'
        + json.dumps(config_body)
        + ',"rawDataHeader":'
        + json.dumps(raw_data_header)
        + ',"rawDataBody":{"dataColumns":'
        + json.dumps(data_columns)
        + ',"dataBlock":['
    )


def record(sensor, file, button):
    """Log every valid gas measurement until the session is over."""
    start = time.ticks_ms()

    # The Raw Data Format counts the Scanning Cycles from one.
    scanning_cycle_index = 1

    rows = 0
    dropped_cycles = 0

    # Recording starts already tagged, so that the whole session forms a
    # Specimen even when the button is never pressed. Pressing the button
    # switches the tag, which starts the next Specimen.
    label_tag = 1
    last_button_press = time.ticks_ms()

    # No step has been seen yet, the first one is expected to be step zero.
    last_step_index = -1

    while time.ticks_diff(time.ticks_ms(), start) < LOG_DURATION_MS:
        # The shortest heater step of the profile lasts two time bases, so
        # polling twice per time base is fast enough not to miss one.
        time.sleep_ms(MEAS_DUR // 2)

        # Mark the start of a new Specimen when the button is pressed. The
        # tag alternates between one and two, the same way the two buttons of
        # the BME688 Development Kit are used.
        if (
            button is not None
            and button.value() == 0
            and time.ticks_diff(time.ticks_ms(), last_button_press)
            > LABEL_BUTTON_DEBOUNCE_MS
        ):
            last_button_press = time.ticks_ms()
            label_tag = 2 if label_tag == 1 else 1
            print("New specimen, label tag is now " + str(label_tag))

        if not sensor.fetch_data():
            continue

        fields_left = 1

        while fields_left:
            data, fields_left = sensor.get_data()

            # Skip the fields which hold no valid gas measurement.
            if data.status != BME690_VALID_DATA:
                continue

            # One scanning cycle is one full sweep through the heater
            # profile, so the counter moves on whenever the step index wraps
            # around.
            if data.gas_index <= last_step_index:
                scanning_cycle_index += 1

            # A step which is not the one after the previous step means the
            # measurement in between was lost. The format reports that as
            # error code 2, and AI-Studio drops the whole cycle on import.
            error_code = 0
            if data.gas_index != (last_step_index + 1) % HEATER_LEN:
                error_code = 2
                dropped_cycles += 1

            last_step_index = data.gas_index

            # The format wants the pressure in hectopascals, the sensor
            # reports it in pascals.
            file.write(
                "{}[{},{},{},{},{:f},{:f},{:f},{:f},{},{},{},{},{}]".format(
                    "" if rows == 0 else ",",
                    SENSOR_INDEX,
                    SENSOR_ID,
                    time.ticks_diff(time.ticks_ms(), start),
                    time.time() + UNIX_EPOCH_OFFSET,
                    data.temperature,
                    data.pressure / 100.0,
                    data.humidity,
                    data.gas_resistance,
                    data.gas_index,
                    1,
                    scanning_cycle_index,
                    label_tag,
                    error_code,
                )
            )
            rows += 1

            print(
                "T: {:.2f} C | P: {:.2f} hPa | H: {:.2f} % | R: {:.1f} "
                "Ohm | step: {}".format(
                    data.temperature,
                    data.pressure / 100.0,
                    data.humidity,
                    data.gas_resistance,
                    data.gas_index,
                )
            )

    return rows, dropped_cycles


print("\nBME690 AI-Studio raw data logger")

station = connect_wifi()
now = sync_time()

# Seed the generator with the synced time so every session gets its own seed
# instead of repeating the same one after every reset.
random.seed(now)
seed_power_on_off = make_seed()

button = None
if LABEL_BUTTON_PIN is not None:
    button = Pin(LABEL_BUTTON_PIN, Pin.IN, Pin.PULL_UP)

card = mount_sd()

sensor = BME690()
check_sensor_status(sensor)

# Default temperature, pressure and humidity oversampling.
sensor.set_tph()
check_sensor_status(sensor)

# The shared heating duration is the total measurement duration minus the
# time needed for the temperature, pressure and humidity measurement.
shared_heatr_dur = (
    MEAS_DUR - sensor.get_meas_dur(BME69X_PARALLEL_MODE) // 1000
)

sensor.set_heater_prof(HEATER_TEMP, HEATER_MUL, shared_heatr_dur)
check_sensor_status(sensor)

sensor.set_op_mode(BME69X_PARALLEL_MODE)
check_sensor_status(sensor)

# The file name carries the date, the board, the power on counter, the
# session seed and the file counter, each part separated by an underscore.
file_name = (
    "/sd/"
    + file_name_date(now)
    + "_Board_"
    + BOARD_ID
    + "_PowerOnOff_1_"
    + seed_power_on_off
    + "_File_0.bmerawdata"
)

file = open(file_name, "w")
file.write(build_header(now, seed_power_on_off))

print("Recording, this takes {} minutes.".format(LOG_DURATION_MS // 60000))

rows, dropped_cycles = record(sensor, file, button)

# Close the data block, the rawDataBody object and the root object.
file.write("]}}")
file.close()

os.umount("/sd")
card.deinit()
station.disconnect()

print(
    "Recording finished, {} rows written, {} cycles marked as lost.".format(
        rows, dropped_cycles
    )
)
print("File saved as: " + file_name)
print("Import it into BME AI-Studio to label the data and train an "
      "algorithm.")
