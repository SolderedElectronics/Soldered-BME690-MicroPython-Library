# FILE: bme690.py
# AUTHOR: Josip Simun Kuci @ Soldered
# BRIEF: MicroPython driver for the Bosch BME690 temperature, humidity,
#        pressure and gas sensor, ported from the Soldered BME690 Arduino
#        library and the Bosch BME69x Sensor API (BSD-3-Clause)
# LAST UPDATED: 2026-09-16

from machine import I2C, Pin
from os import uname
import time

# I2C addresses
BME69X_I2C_ADDR_LOW = 0x76
BME69X_I2C_ADDR_HIGH = 0x77

# Chip identifier and the soft reset command
BME69X_CHIP_ID = 0x61
BME69X_SOFT_RESET_CMD = 0xB6

# Return codes of the driver
BME69X_OK = 0
BME69X_E_NULL_PTR = -1
BME69X_E_COM_FAIL = -2
BME69X_E_DEV_NOT_FOUND = -3
BME69X_E_INVALID_LENGTH = -4
BME69X_E_SELF_TEST = -5
BME69X_W_DEFINE_OP_MODE = 1
BME69X_W_NO_NEW_DATA = 2
BME69X_W_DEFINE_SHD_HEATR_DUR = 3

# Information, only available through the info_msg attribute
BME69X_I_PARAM_CORR = 1

# Aggregated results of check_status()
BME69X_ERROR = -1
BME69X_WARNING = 1

# Register map
BME69X_REG_COEFF3 = 0x00
BME69X_REG_FIELD0 = 0x1D
BME69X_REG_IDAC_HEAT0 = 0x50
BME69X_REG_RES_HEAT0 = 0x5A
BME69X_REG_GAS_WAIT0 = 0x64
BME69X_REG_SHD_HEATR_DUR = 0x6E
BME69X_REG_CTRL_GAS_0 = 0x70
BME69X_REG_CTRL_GAS_1 = 0x71
BME69X_REG_CTRL_HUM = 0x72
BME69X_REG_CTRL_MEAS = 0x74
BME69X_REG_CONFIG = 0x75
BME69X_REG_UNIQUE_ID = 0x83
BME69X_REG_COEFF1 = 0x8A
BME69X_REG_CHIP_ID = 0xD0
BME69X_REG_SOFT_RESET = 0xE0
BME69X_REG_COEFF2 = 0xE1
BME69X_REG_VARIANT_ID = 0xF0

# Enable / disable
BME69X_ENABLE = 0x01
BME69X_DISABLE = 0x00

# Variant identifiers
BME69X_VARIANT_GAS_LOW = 0x00
BME69X_VARIANT_GAS_HIGH = 0x01
BME690_VARIANT_GAS_HIGH = 0x02

# Oversampling settings
BME69X_OS_NONE = 0
BME69X_OS_1X = 1
BME69X_OS_2X = 2
BME69X_OS_4X = 3
BME69X_OS_8X = 4
BME69X_OS_16X = 5

# IIR filter settings
BME69X_FILTER_OFF = 0
BME69X_FILTER_SIZE_1 = 1
BME69X_FILTER_SIZE_3 = 2
BME69X_FILTER_SIZE_7 = 3
BME69X_FILTER_SIZE_15 = 4
BME69X_FILTER_SIZE_31 = 5
BME69X_FILTER_SIZE_63 = 6
BME69X_FILTER_SIZE_127 = 7

# Standby time between two sequential mode measurements
BME69X_ODR_0_59_MS = 0
BME69X_ODR_62_5_MS = 1
BME69X_ODR_125_MS = 2
BME69X_ODR_250_MS = 3
BME69X_ODR_500_MS = 4
BME69X_ODR_1000_MS = 5
BME69X_ODR_10_MS = 6
BME69X_ODR_20_MS = 7
BME69X_ODR_NONE = 8

# Operation modes
BME69X_SLEEP_MODE = 0
BME69X_FORCED_MODE = 1
BME69X_PARALLEL_MODE = 2
BME69X_SEQUENTIAL_MODE = 3

# Status bits of a measured field
BME69X_NEW_DATA_MSK = 0x80
BME69X_GASM_VALID_MSK = 0x20
BME69X_HEAT_STAB_MSK = 0x10
BME69X_GAS_INDEX_MSK = 0x0F
BME69X_GAS_RANGE_MSK = 0x0F

# Bit masks and positions of the configuration registers
BME69X_NBCONV_MSK = 0x0F
BME69X_FILTER_MSK = 0x1C
BME69X_FILTER_POS = 2
BME69X_ODR3_MSK = 0x80
BME69X_ODR3_POS = 7
BME69X_ODR20_MSK = 0xE0
BME69X_ODR20_POS = 5
BME69X_OST_MSK = 0xE0
BME69X_OST_POS = 5
BME69X_OSP_MSK = 0x1C
BME69X_OSP_POS = 2
BME69X_OSH_MSK = 0x07
BME69X_HCTRL_MSK = 0x08
BME69X_HCTRL_POS = 3
BME69X_RUN_GAS_MSK = 0x30
BME69X_RUN_GAS_POS = 5
BME69X_MODE_MSK = 0x03
BME69X_RHRANGE_MSK = 0x30
BME69X_RSERROR_MSK = 0xF0

# Heater and gas measurement control
BME69X_ENABLE_HEATER = 0x00
BME69X_DISABLE_HEATER = 0x01
BME69X_ENABLE_GAS_MEAS = 0x01
BME69X_DISABLE_GAS_MEAS = 0x00

# Lengths of the register blocks read in one transaction
BME69X_LEN_COEFF_ALL = 42
BME69X_LEN_COEFF1 = 23
BME69X_LEN_COEFF2 = 14
BME69X_LEN_COEFF3 = 5
BME69X_LEN_FIELD = 17
BME69X_LEN_CONFIG = 5
BME69X_LEN_INTERLEAVE_BUFF = 20

# Timing, all in microseconds
BME69X_PERIOD_POLL = 10000
BME69X_PERIOD_RESET = 10000

# Self test parameters
BME69X_HEATR_DUR1 = 1000
BME69X_HEATR_DUR2 = 2000
BME69X_HEATR_DUR1_DELAY = 1000000
BME69X_HEATR_DUR2_DELAY = 2000000
BME69X_N_MEAS = 6
BME69X_LOW_TEMP = 150
BME69X_HIGH_TEMP = 350

# Plausibility limits of the self test, floating point output
BME69X_MIN_TEMPERATURE = 0
BME69X_MAX_TEMPERATURE = 60
BME69X_MIN_PRESSURE = 90000
BME69X_MAX_PRESSURE = 110000
BME69X_MIN_HUMIDITY = 20
BME69X_MAX_HUMIDITY = 80

# Text descriptions of the status codes, used by status_string()
_STATUS_STRINGS = {
    BME69X_OK: "",
    BME69X_E_NULL_PTR: "Null pointer",
    BME69X_E_COM_FAIL: "Communication failure",
    BME69X_E_DEV_NOT_FOUND: "Sensor not found",
    BME69X_E_INVALID_LENGTH: "Invalid length",
    BME69X_E_SELF_TEST: "Self test failed",
    BME69X_W_DEFINE_OP_MODE: "Set the operation mode",
    BME69X_W_NO_NEW_DATA: "No new data",
    BME69X_W_DEFINE_SHD_HEATR_DUR: "Set the shared heater duration",
}

# Measurement cycles per oversampling setting
_OS_TO_MEAS_CYCLES = (0, 1, 2, 4, 8, 16)


def _s8(value):
    """Interpret an unsigned byte as a signed 8 bit integer."""
    return value - 256 if value > 127 else value


def _s16(value):
    """Interpret an unsigned word as a signed 16 bit integer."""
    return value - 65536 if value > 32767 else value


class BME690Data:
    """One measured field of the BME690."""

    def __init__(self):
        self.status = 0
        self.gas_index = 0
        self.meas_index = 0
        self.res_heat = 0
        self.idac = 0
        self.gas_wait = 0
        self.temperature = 0.0
        self.pressure = 0.0
        self.humidity = 0.0
        self.gas_resistance = 0.0

    def copy_from(self, other):
        """Copy every field of another BME690Data instance."""
        self.status = other.status
        self.gas_index = other.gas_index
        self.meas_index = other.meas_index
        self.res_heat = other.res_heat
        self.idac = other.idac
        self.gas_wait = other.gas_wait
        self.temperature = other.temperature
        self.pressure = other.pressure
        self.humidity = other.humidity
        self.gas_resistance = other.gas_resistance

    def __repr__(self):
        return (
            "BME690Data(temperature={:.2f}, pressure={:.1f}, "
            "humidity={:.2f}, gas_resistance={:.1f}, status=0x{:02x}, "
            "gas_index={})".format(
                self.temperature,
                self.pressure,
                self.humidity,
                self.gas_resistance,
                self.status,
                self.gas_index,
            )
        )


class BME690:
    """
    MicroPython driver for the Soldered BME690 breakout board, I2C only.

    The sensor is initialized by the constructor, which raises an exception
    when it cannot be reached. Every other method stores its result in the
    status attribute instead of raising, the same way the Arduino library
    does, so check_status() and status_string() report what went wrong.

    The compensation uses the floating point path of the Bosch API. Ports of
    MicroPython which use single precision floats, the ESP32 among them, are
    therefore a little less precise than the Arduino library, well below the
    accuracy of the sensor itself.
    """

    def __init__(self, i2c=None, address=BME69X_I2C_ADDR_LOW, amb_temp=25):
        """
        Initialize the BME690.

        :param i2c: Initialized I2C object, auto-detected on known boards
        :param address: I2C address, BME69X_I2C_ADDR_LOW by default
        :param amb_temp: Ambient temperature in degrees Celsius, used by the
                         heater resistance calculation
        """
        if i2c is not None:
            self.i2c = i2c
        else:
            if uname().sysname in (
                "esp32",
                "esp8266",
                "Soldered Dasduino CONNECTPLUS",
            ):
                self.i2c = I2C(0, scl=Pin(22), sda=Pin(21))
            else:
                raise Exception(
                    "Board not recognized, enter I2C pins manually"
                )

        self.address = address
        self.status = BME69X_OK
        self.info_msg = BME69X_OK
        self.intf_rslt = BME69X_OK
        self.chip_id = 0
        self.variant_id = 0
        self.amb_temp = amb_temp
        self._calib = {}
        self._conf = {
            "os_hum": BME69X_OS_NONE,
            "os_temp": BME69X_OS_NONE,
            "os_pres": BME69X_OS_NONE,
            "filter": BME69X_FILTER_OFF,
            "odr": BME69X_ODR_0_59_MS,
        }
        self._heatr_conf = {
            "enable": BME69X_DISABLE,
            "heatr_temp": 0,
            "heatr_dur": 0,
            "heatr_temp_prof": None,
            "heatr_dur_prof": None,
            "profile_len": 0,
            "shared_heatr_dur": 0,
        }
        self._sensor_data = [BME690Data(), BME690Data(), BME690Data()]
        self._n_fields = 0
        self._i_fields = 0
        self._last_op_mode = BME69X_SLEEP_MODE

        self._init_sensor()

    # ------------------------------------------------------------------
    # Bus access
    # ------------------------------------------------------------------

    def _get_regs(self, reg_addr, length):
        """Read a block of registers, raises OSError on a bus failure."""
        data = self.i2c.readfrom_mem(self.address, reg_addr, length)
        self.intf_rslt = BME69X_OK
        return data

    def _set_regs(self, reg_addrs, reg_data):
        """
        Write single bytes to a list of registers in one transaction.

        The addresses and the data are interleaved the same way the Bosch API
        does it, so that registers which are not next to each other can be
        written with a single transfer.
        """
        length = len(reg_addrs)
        if length == 0 or length > BME69X_LEN_INTERLEAVE_BUFF // 2:
            self.status = BME69X_E_INVALID_LENGTH
            return

        payload = bytearray()
        for index in range(1, length):
            payload.append(reg_addrs[index])
            payload.append(reg_data[index])

        buf = bytearray()
        buf.append(reg_data[0])
        buf.extend(payload)

        self.i2c.writeto_mem(self.address, reg_addrs[0], buf)
        self.intf_rslt = BME69X_OK

    def read_reg(self, reg_addr, length=1):
        """
        Read one or more registers.

        :param reg_addr: Address of the first register
        :param length: Number of bytes to read
        :return: An integer for a single byte, bytes otherwise, None on error
        """
        try:
            data = self._get_regs(reg_addr, length)
            self.status = BME69X_OK
            return data[0] if length == 1 else data
        except OSError:
            self.intf_rslt = BME69X_E_COM_FAIL
            self.status = BME69X_E_COM_FAIL
            return None

    def write_reg(self, reg_addr, reg_data):
        """
        Write one or more registers.

        :param reg_addr: Register address, or a list of addresses
        :param reg_data: Byte to write, or a list of bytes
        """
        if not isinstance(reg_addr, (list, tuple)):
            reg_addr = [reg_addr]
            reg_data = [reg_data]

        try:
            self.status = BME69X_OK
            self._set_regs(reg_addr, reg_data)
        except OSError:
            self.intf_rslt = BME69X_E_COM_FAIL
            self.status = BME69X_E_COM_FAIL

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    def _init_sensor(self):
        """Reset the sensor, check the chip ID, read the calibration."""
        try:
            self._soft_reset()
            self.chip_id = self._get_regs(BME69X_REG_CHIP_ID, 1)[0]
            if self.chip_id != BME69X_CHIP_ID:
                self.status = BME69X_E_DEV_NOT_FOUND
                raise Exception(
                    "BME690 not found, chip ID 0x{:02x} was read instead of "
                    "0x{:02x}".format(self.chip_id, BME69X_CHIP_ID)
                )

            self.variant_id = self._get_regs(BME69X_REG_VARIANT_ID, 1)[0]
            self._get_calib_data()
            self.status = BME69X_OK
        except OSError:
            self.intf_rslt = BME69X_E_COM_FAIL
            self.status = BME69X_E_COM_FAIL
            raise Exception(
                "BME690 not responding on address 0x{:02x}".format(
                    self.address
                )
            )

    def _get_calib_data(self):
        """Read and unpack the calibration coefficients of the sensor."""
        coeff = bytearray(BME69X_LEN_COEFF_ALL)
        coeff[0:BME69X_LEN_COEFF1] = self._get_regs(
            BME69X_REG_COEFF1, BME69X_LEN_COEFF1
        )
        coeff[BME69X_LEN_COEFF1:BME69X_LEN_COEFF1 + BME69X_LEN_COEFF2] = (
            self._get_regs(BME69X_REG_COEFF2, BME69X_LEN_COEFF2)
        )
        coeff[BME69X_LEN_COEFF1 + BME69X_LEN_COEFF2:] = self._get_regs(
            BME69X_REG_COEFF3, BME69X_LEN_COEFF3
        )

        calib = self._calib

        # Temperature related coefficients
        calib["par_t1"] = (coeff[32] << 8) | coeff[31]
        calib["par_t2"] = (coeff[1] << 8) | coeff[0]
        calib["par_t3"] = _s8(coeff[2])

        # Pressure related coefficients
        calib["par_p5"] = _s16((coeff[5] << 8) | coeff[4])
        calib["par_p6"] = _s16((coeff[7] << 8) | coeff[6])
        calib["par_p7"] = _s8(coeff[8])
        calib["par_p8"] = _s8(coeff[9])
        calib["par_p1"] = (coeff[11] << 8) | coeff[10]
        calib["par_p2"] = (coeff[13] << 8) | coeff[12]
        calib["par_p3"] = _s8(coeff[14])
        calib["par_p4"] = _s8(coeff[15])
        calib["par_p9"] = _s16((coeff[19] << 8) | coeff[18])
        calib["par_p10"] = _s8(coeff[20])
        calib["par_p11"] = _s8(coeff[21])

        # Humidity related coefficients
        par_h5 = (coeff[23] << 4) | (coeff[24] >> 4)
        if par_h5 > 2047:
            par_h5 -= 4096
        calib["par_h5"] = par_h5

        par_h1 = (coeff[25] << 4) | (coeff[24] & 0x0F)
        if par_h1 > 2047:
            par_h1 -= 4096
        calib["par_h1"] = par_h1

        calib["par_h2"] = _s8(coeff[26])
        calib["par_h4"] = _s8(coeff[27])
        calib["par_h3"] = coeff[28]
        calib["par_h6"] = coeff[29]

        # Gas heater related coefficients
        calib["par_g1"] = _s8(coeff[35])
        calib["par_g2"] = _s16((coeff[34] << 8) | coeff[33])
        calib["par_g3"] = _s8(coeff[36])

        # Other coefficients
        calib["res_heat_range"] = (coeff[39] & BME69X_RHRANGE_MSK) >> 4
        calib["res_heat_val"] = _s8(coeff[37])
        calib["range_sw_err"] = _s8(coeff[41] & BME69X_RSERROR_MSK) // 16

    def _soft_reset(self):
        """Issue the soft reset command and wait for the sensor to restart."""
        self._set_regs([BME69X_REG_SOFT_RESET], [BME69X_SOFT_RESET_CMD])
        time.sleep_us(BME69X_PERIOD_RESET)

    def soft_reset(self):
        """Soft reset the sensor, the configuration is lost."""
        try:
            self._soft_reset()
            self.status = BME69X_OK
        except OSError:
            self.intf_rslt = BME69X_E_COM_FAIL
            self.status = BME69X_E_COM_FAIL

    def set_ambient_temp(self, temp=25):
        """
        Set the ambient temperature used by the heater calculation.

        :param temp: Ambient temperature in degrees Celsius
        """
        self.amb_temp = temp

    # ------------------------------------------------------------------
    # Operation mode
    # ------------------------------------------------------------------

    def _set_op_mode(self, op_mode):
        """Put the sensor to sleep first, then set the requested mode."""
        pow_mode = BME69X_FORCED_MODE
        tmp_pow_mode = 0

        while pow_mode != BME69X_SLEEP_MODE:
            tmp_pow_mode = self._get_regs(BME69X_REG_CTRL_MEAS, 1)[0]
            pow_mode = tmp_pow_mode & BME69X_MODE_MSK
            if pow_mode != BME69X_SLEEP_MODE:
                tmp_pow_mode &= ~BME69X_MODE_MSK & 0xFF
                self._set_regs([BME69X_REG_CTRL_MEAS], [tmp_pow_mode])
                time.sleep_us(BME69X_PERIOD_POLL)

        if op_mode != BME69X_SLEEP_MODE:
            tmp_pow_mode = (tmp_pow_mode & ~BME69X_MODE_MSK & 0xFF) | (
                op_mode & BME69X_MODE_MSK
            )
            self._set_regs([BME69X_REG_CTRL_MEAS], [tmp_pow_mode])

    def set_op_mode(self, op_mode):
        """
        Set the operation mode.

        :param op_mode: BME69X_SLEEP_MODE, BME69X_FORCED_MODE,
                        BME69X_PARALLEL_MODE or BME69X_SEQUENTIAL_MODE
        """
        try:
            self._set_op_mode(op_mode)
            self.status = BME69X_OK
            if op_mode != BME69X_SLEEP_MODE:
                self._last_op_mode = op_mode
        except OSError:
            self.intf_rslt = BME69X_E_COM_FAIL
            self.status = BME69X_E_COM_FAIL

    def get_op_mode(self):
        """
        Get the operation mode the sensor is in.

        :return: The mode, or None when the sensor could not be read
        """
        try:
            mode = self._get_regs(BME69X_REG_CTRL_MEAS, 1)[0]
            self.status = BME69X_OK
            return mode & BME69X_MODE_MSK
        except OSError:
            self.intf_rslt = BME69X_E_COM_FAIL
            self.status = BME69X_E_COM_FAIL
            return None

    def get_meas_dur(self, op_mode=BME69X_SLEEP_MODE):
        """
        Get the duration of one measurement in microseconds.

        The duration is calculated from the cached oversampling settings, so
        set_tph() has to be called before it.

        :param op_mode: Operation mode, the last used one when left out
        :return: Measurement duration in microseconds
        """
        if op_mode == BME69X_SLEEP_MODE:
            op_mode = self._last_op_mode

        conf = self._conf
        self._boundary_check(conf, "os_temp", BME69X_OS_16X)
        self._boundary_check(conf, "os_pres", BME69X_OS_16X)
        self._boundary_check(conf, "os_hum", BME69X_OS_16X)

        meas_cycles = _OS_TO_MEAS_CYCLES[conf["os_temp"]]
        meas_cycles += _OS_TO_MEAS_CYCLES[conf["os_pres"]]
        meas_cycles += _OS_TO_MEAS_CYCLES[conf["os_hum"]]

        meas_dur = meas_cycles * 1963
        meas_dur += 477 * 4  # TPH switching duration
        meas_dur += 477 * 5  # Gas measurement duration

        if op_mode != BME69X_PARALLEL_MODE:
            meas_dur += 1000  # Wake up duration of 1 ms

        return meas_dur

    # ------------------------------------------------------------------
    # Oversampling, filter and standby time
    # ------------------------------------------------------------------

    def _boundary_check(self, conf, key, maximum):
        """Clamp a configuration value and note that it was corrected."""
        if conf[key] > maximum:
            conf[key] = maximum
            self.info_msg |= BME69X_I_PARAM_CORR

    def _get_conf(self):
        """Read the oversampling, filter and standby time into the cache."""
        data = self._get_regs(BME69X_REG_CTRL_GAS_1, BME69X_LEN_CONFIG)
        conf = self._conf
        conf["os_hum"] = data[1] & BME69X_OSH_MSK
        conf["filter"] = (data[4] & BME69X_FILTER_MSK) >> BME69X_FILTER_POS
        conf["os_temp"] = (data[3] & BME69X_OST_MSK) >> BME69X_OST_POS
        conf["os_pres"] = (data[3] & BME69X_OSP_MSK) >> BME69X_OSP_POS
        if (data[0] & BME69X_ODR3_MSK) >> BME69X_ODR3_POS:
            conf["odr"] = BME69X_ODR_NONE
        else:
            conf["odr"] = (data[4] & BME69X_ODR20_MSK) >> BME69X_ODR20_POS

    def _set_conf(self):
        """Write the cached configuration, the sensor has to be in sleep."""
        current_op_mode = self._get_regs(BME69X_REG_CTRL_MEAS, 1)[0]
        current_op_mode &= BME69X_MODE_MSK
        self._set_op_mode(BME69X_SLEEP_MODE)

        reg_array = [0x71, 0x72, 0x73, 0x74, 0x75]
        data_array = bytearray(
            self._get_regs(reg_array[0], BME69X_LEN_CONFIG)
        )
        self.info_msg = BME69X_OK

        conf = self._conf
        self._boundary_check(conf, "filter", BME69X_FILTER_SIZE_127)
        self._boundary_check(conf, "os_temp", BME69X_OS_16X)
        self._boundary_check(conf, "os_pres", BME69X_OS_16X)
        self._boundary_check(conf, "os_hum", BME69X_OS_16X)
        self._boundary_check(conf, "odr", BME69X_ODR_NONE)

        data_array[4] = (data_array[4] & ~BME69X_FILTER_MSK & 0xFF) | (
            (conf["filter"] << BME69X_FILTER_POS) & BME69X_FILTER_MSK
        )
        data_array[3] = (data_array[3] & ~BME69X_OST_MSK & 0xFF) | (
            (conf["os_temp"] << BME69X_OST_POS) & BME69X_OST_MSK
        )
        data_array[3] = (data_array[3] & ~BME69X_OSP_MSK & 0xFF) | (
            (conf["os_pres"] << BME69X_OSP_POS) & BME69X_OSP_MSK
        )
        data_array[1] = (data_array[1] & ~BME69X_OSH_MSK & 0xFF) | (
            conf["os_hum"] & BME69X_OSH_MSK
        )

        self._set_regs(reg_array, list(data_array))

        if current_op_mode != BME69X_SLEEP_MODE:
            self._set_op_mode(current_op_mode)

    def get_tph(self):
        """
        Get the temperature, pressure and humidity oversampling.

        :return: Tuple of (os_hum, os_temp, os_pres), None on error
        """
        try:
            self._get_conf()
            self.status = BME69X_OK
            return (
                self._conf["os_hum"],
                self._conf["os_temp"],
                self._conf["os_pres"],
            )
        except OSError:
            self.intf_rslt = BME69X_E_COM_FAIL
            self.status = BME69X_E_COM_FAIL
            return None

    def set_tph(
        self,
        os_temp=BME69X_OS_2X,
        os_pres=BME69X_OS_16X,
        os_hum=BME69X_OS_1X,
    ):
        """
        Set the temperature, pressure and humidity oversampling.

        Calling it without arguments sets the defaults of the Arduino library.

        :param os_temp: Temperature oversampling, BME69X_OS_NONE to OS_16X
        :param os_pres: Pressure oversampling, BME69X_OS_NONE to OS_16X
        :param os_hum: Humidity oversampling, BME69X_OS_NONE to OS_16X
        """
        try:
            self._get_conf()
            self._conf["os_hum"] = os_hum
            self._conf["os_temp"] = os_temp
            self._conf["os_pres"] = os_pres
            self._set_conf()
            self.status = BME69X_OK
        except OSError:
            self.intf_rslt = BME69X_E_COM_FAIL
            self.status = BME69X_E_COM_FAIL

    def get_filter(self):
        """
        Get the IIR filter setting.

        :return: BME69X_FILTER_OFF to BME69X_FILTER_SIZE_127, None on error
        """
        try:
            self._get_conf()
            self.status = BME69X_OK
            return self._conf["filter"]
        except OSError:
            self.intf_rslt = BME69X_E_COM_FAIL
            self.status = BME69X_E_COM_FAIL
            return None

    def set_filter(self, filter_size=BME69X_FILTER_OFF):
        """
        Set the IIR filter setting.

        :param filter_size: BME69X_FILTER_OFF to BME69X_FILTER_SIZE_127
        """
        try:
            self._get_conf()
            self._conf["filter"] = filter_size
            self._set_conf()
            self.status = BME69X_OK
        except OSError:
            self.intf_rslt = BME69X_E_COM_FAIL
            self.status = BME69X_E_COM_FAIL

    def get_seq_sleep(self):
        """
        Get the standby time used in sequential mode.

        :return: BME69X_ODR_0_59_MS to BME69X_ODR_NONE, None on error
        """
        try:
            self._get_conf()
            self.status = BME69X_OK
            return self._conf["odr"]
        except OSError:
            self.intf_rslt = BME69X_E_COM_FAIL
            self.status = BME69X_E_COM_FAIL
            return None

    def set_seq_sleep(self, odr=BME69X_ODR_0_59_MS):
        """
        Set the standby time used in sequential mode.

        :param odr: BME69X_ODR_0_59_MS to BME69X_ODR_NONE
        """
        try:
            self._get_conf()
            self._conf["odr"] = odr
            self._set_conf()
            self.status = BME69X_OK
        except OSError:
            self.intf_rslt = BME69X_E_COM_FAIL
            self.status = BME69X_E_COM_FAIL

    # ------------------------------------------------------------------
    # Gas heater
    # ------------------------------------------------------------------

    def set_heater_prof(self, temp, dur, shared_heatr_dur=None):
        """
        Set the gas heater profile.

        The mode the profile is written for follows from the arguments, the
        same way the three overloads of the Arduino library work.

        Forced mode, a single temperature and duration:
            set_heater_prof(300, 100)
        Sequential mode, a temperature and a duration per step:
            set_heater_prof(temps, durs)
        Parallel mode, a temperature and a multiplier per step, together with
        the shared heating duration:
            set_heater_prof(temps, muls, shared_heatr_dur)

        :param temp: Heater temperature in degrees Celsius, or a list of them
        :param dur: Heating duration in milliseconds, or a list of durations,
                    or a list of multipliers of the shared duration
        :param shared_heatr_dur: Shared heating duration in milliseconds,
                                 parallel mode only
        """
        conf = self._heatr_conf
        conf["enable"] = BME69X_ENABLE

        if not isinstance(temp, (list, tuple)):
            conf["heatr_temp"] = temp
            conf["heatr_dur"] = dur
            op_mode = BME69X_FORCED_MODE
        else:
            conf["heatr_temp_prof"] = temp
            conf["heatr_dur_prof"] = dur
            conf["profile_len"] = len(temp)
            if shared_heatr_dur is None:
                op_mode = BME69X_SEQUENTIAL_MODE
            else:
                conf["shared_heatr_dur"] = shared_heatr_dur
                op_mode = BME69X_PARALLEL_MODE

        try:
            self.status = BME69X_OK
            self._set_heatr_conf(op_mode)
        except OSError:
            self.intf_rslt = BME69X_E_COM_FAIL
            self.status = BME69X_E_COM_FAIL

    def _set_heatr_conf(self, op_mode):
        """Write the cached heater configuration for the given mode."""
        self._set_op_mode(BME69X_SLEEP_MODE)

        nb_conv = self._write_heatr_profile(op_mode)
        if self.status != BME69X_OK:
            return

        ctrl_gas_data = bytearray(self._get_regs(BME69X_REG_CTRL_GAS_0, 2))

        if self._heatr_conf["enable"] == BME69X_ENABLE:
            hctrl = BME69X_ENABLE_HEATER
            run_gas = BME69X_ENABLE_GAS_MEAS
        else:
            hctrl = BME69X_DISABLE_HEATER
            run_gas = BME69X_DISABLE_GAS_MEAS

        ctrl_gas_data[0] = (ctrl_gas_data[0] & ~BME69X_HCTRL_MSK & 0xFF) | (
            (hctrl << BME69X_HCTRL_POS) & BME69X_HCTRL_MSK
        )
        ctrl_gas_data[1] = (ctrl_gas_data[1] & ~BME69X_NBCONV_MSK & 0xFF) | (
            nb_conv & BME69X_NBCONV_MSK
        )
        ctrl_gas_data[1] = (ctrl_gas_data[1] & ~BME69X_RUN_GAS_MSK & 0xFF) | (
            (run_gas << BME69X_RUN_GAS_POS) & BME69X_RUN_GAS_MSK
        )

        self._set_regs(
            [BME69X_REG_CTRL_GAS_0, BME69X_REG_CTRL_GAS_1],
            list(ctrl_gas_data),
        )

    def _write_heatr_profile(self, op_mode):
        """Write the heater resistance and gas wait registers of a profile."""
        conf = self._heatr_conf
        rh_reg_addr = []
        rh_reg_data = []
        gw_reg_addr = []
        gw_reg_data = []

        if op_mode == BME69X_FORCED_MODE:
            rh_reg_addr.append(BME69X_REG_RES_HEAT0)
            rh_reg_data.append(self._calc_res_heat(conf["heatr_temp"]))
            gw_reg_addr.append(BME69X_REG_GAS_WAIT0)
            gw_reg_data.append(self._calc_gas_wait(conf["heatr_dur"]))
            nb_conv = 0
        elif op_mode == BME69X_SEQUENTIAL_MODE:
            if not conf["heatr_temp_prof"] or not conf["heatr_dur_prof"]:
                self.status = BME69X_E_NULL_PTR
                return 0

            for i in range(conf["profile_len"]):
                rh_reg_addr.append(BME69X_REG_RES_HEAT0 + i)
                rh_reg_data.append(
                    self._calc_res_heat(conf["heatr_temp_prof"][i])
                )
                gw_reg_addr.append(BME69X_REG_GAS_WAIT0 + i)
                gw_reg_data.append(
                    self._calc_gas_wait(conf["heatr_dur_prof"][i])
                )

            nb_conv = conf["profile_len"]
        elif op_mode == BME69X_PARALLEL_MODE:
            if not conf["heatr_temp_prof"] or not conf["heatr_dur_prof"]:
                self.status = BME69X_E_NULL_PTR
                return 0

            if conf["shared_heatr_dur"] == 0:
                self.status = BME69X_W_DEFINE_SHD_HEATR_DUR

            for i in range(conf["profile_len"]):
                rh_reg_addr.append(BME69X_REG_RES_HEAT0 + i)
                rh_reg_data.append(
                    self._calc_res_heat(conf["heatr_temp_prof"][i])
                )
                gw_reg_addr.append(BME69X_REG_GAS_WAIT0 + i)
                gw_reg_data.append(conf["heatr_dur_prof"][i] & 0xFF)

            nb_conv = conf["profile_len"]

            if self.status == BME69X_OK:
                shared_dur = self._calc_heatr_dur_shared(
                    conf["shared_heatr_dur"]
                )
                self._set_regs([BME69X_REG_SHD_HEATR_DUR], [shared_dur])
        else:
            self.status = BME69X_W_DEFINE_OP_MODE
            return 0

        if self.status == BME69X_OK:
            self._set_regs(rh_reg_addr, rh_reg_data)
            self._set_regs(gw_reg_addr, gw_reg_data)

        return nb_conv

    def get_heater_configuration(self):
        """
        Get the heater configuration which was set last.

        :return: Dictionary holding the cached heater configuration
        """
        return self._heatr_conf

    # ------------------------------------------------------------------
    # Measurement data
    # ------------------------------------------------------------------

    def fetch_data(self):
        """
        Read the measured fields of the sensor into the local buffer.

        :return: Number of new data fields, zero when there are none
        """
        self._n_fields = 0
        self._i_fields = 0

        try:
            if self._last_op_mode == BME69X_FORCED_MODE:
                self._read_field_data(0, self._sensor_data[0])
                if self._sensor_data[0].status & BME69X_NEW_DATA_MSK:
                    self._n_fields = 1
                    self.status = BME69X_OK
                else:
                    self.status = BME69X_W_NO_NEW_DATA
            elif self._last_op_mode in (
                BME69X_PARALLEL_MODE,
                BME69X_SEQUENTIAL_MODE,
            ):
                fields = self._read_all_field_data()

                for field in fields:
                    if field.status & BME69X_NEW_DATA_MSK:
                        self._n_fields += 1

                # Sort the fields, the oldest sample ends up first
                for i in range(2):
                    for j in range(i + 1, 3):
                        self._sort_sensor_data(i, j, fields)

                for i in range(3):
                    self._sensor_data[i].copy_from(fields[i])

                if self._n_fields == 0:
                    self.status = BME69X_W_NO_NEW_DATA
                else:
                    self.status = BME69X_OK
            else:
                self.status = BME69X_W_DEFINE_OP_MODE
        except OSError:
            self.intf_rslt = BME69X_E_COM_FAIL
            self.status = BME69X_E_COM_FAIL

        return self._n_fields

    def get_data(self):
        """
        Get the next data field out of the local buffer.

        :return: Tuple of (data, fields_left), where data is a BME690Data
                 instance, or None when there is no new data
        """
        if self._last_op_mode == BME69X_FORCED_MODE:
            return self._sensor_data[0], 0

        if self._n_fields:
            data = self._sensor_data[self._i_fields]
            self._i_fields += 1

            # Reading past the last new field keeps returning that field
            if self._i_fields >= self._n_fields:
                self._i_fields = self._n_fields - 1
                return data, 0

            return data, self._n_fields - self._i_fields

        return None, 0

    def get_all_data(self):
        """
        Get the whole local buffer.

        :return: List of the three BME690Data fields
        """
        return self._sensor_data

    def _read_field_data(self, index, data):
        """Read and compensate a single field of the sensor."""
        tries = 5

        while tries:
            buff = self._get_regs(
                BME69X_REG_FIELD0 + index * BME69X_LEN_FIELD,
                BME69X_LEN_FIELD,
            )

            data.status = buff[0] & BME69X_NEW_DATA_MSK
            data.gas_index = buff[0] & BME69X_GAS_INDEX_MSK
            data.meas_index = buff[1]

            adc_pres = (buff[2] << 16) | (buff[3] << 8) | buff[4]
            adc_temp = (buff[5] << 16) | (buff[6] << 8) | buff[7]
            adc_hum = (buff[8] << 8) | buff[9]
            adc_gas_res = (buff[15] << 2) | (buff[16] >> 6)
            gas_range = buff[16] & BME69X_GAS_RANGE_MSK

            data.status |= buff[16] & BME69X_GASM_VALID_MSK
            data.status |= buff[16] & BME69X_HEAT_STAB_MSK

            if data.status & BME69X_NEW_DATA_MSK:
                data.res_heat = self._get_regs(
                    BME69X_REG_RES_HEAT0 + data.gas_index, 1
                )[0]
                data.idac = self._get_regs(
                    BME69X_REG_IDAC_HEAT0 + data.gas_index, 1
                )[0]
                data.gas_wait = self._get_regs(
                    BME69X_REG_GAS_WAIT0 + data.gas_index, 1
                )[0]

                data.temperature = self._calc_temperature(adc_temp)
                data.pressure = self._calc_pressure(
                    adc_pres, data.temperature
                )
                data.humidity = self._calc_humidity(
                    adc_hum, data.temperature
                )
                data.gas_resistance = self._calc_gas_resistance(
                    adc_gas_res, gas_range
                )
                return

            time.sleep_us(BME69X_PERIOD_POLL)
            tries -= 1

    def _read_all_field_data(self):
        """Read and compensate all three fields of the sensor."""
        buff = self._get_regs(BME69X_REG_FIELD0, BME69X_LEN_FIELD * 3)
        set_val = self._get_regs(BME69X_REG_IDAC_HEAT0, 30)

        fields = [BME690Data(), BME690Data(), BME690Data()]

        for i in range(3):
            off = i * BME69X_LEN_FIELD
            data = fields[i]

            data.status = buff[off] & BME69X_NEW_DATA_MSK
            data.gas_index = buff[off] & BME69X_GAS_INDEX_MSK
            data.meas_index = buff[off + 1]

            adc_pres = (
                (buff[off + 2] << 16)
                | (buff[off + 3] << 8)
                | buff[off + 4]
            )
            adc_temp = (
                (buff[off + 5] << 16)
                | (buff[off + 6] << 8)
                | buff[off + 7]
            )
            adc_hum = (buff[off + 8] << 8) | buff[off + 9]
            adc_gas_res = (buff[off + 15] << 2) | (buff[off + 16] >> 6)
            gas_range = buff[off + 16] & BME69X_GAS_RANGE_MSK

            data.status |= buff[off + 16] & BME69X_GASM_VALID_MSK
            data.status |= buff[off + 16] & BME69X_HEAT_STAB_MSK

            # Keep the index inside the profile, it addresses set_val below
            if data.gas_index > 9:
                data.gas_index = 9

            data.idac = set_val[data.gas_index]
            data.res_heat = set_val[10 + data.gas_index]
            data.gas_wait = set_val[20 + data.gas_index]

            data.temperature = self._calc_temperature(adc_temp)
            data.pressure = self._calc_pressure(adc_pres, data.temperature)
            data.humidity = self._calc_humidity(adc_hum, data.temperature)
            data.gas_resistance = self._calc_gas_resistance(
                adc_gas_res, gas_range
            )

        return fields

    @staticmethod
    def _sort_sensor_data(low_index, high_index, fields):
        """
        Sort two fields so that the older sample is in the lower position.

        The three fields are filled in a fixed order with an incrementing
        8 bit sub-measurement index, which wraps around at 255. Comparing the
        difference of two indices therefore tells which of the two samples is
        the older one, as long as only two fields are looked at a time.
        """
        low = fields[low_index]
        high = fields[high_index]

        if (low.status & BME69X_NEW_DATA_MSK) and (
            high.status & BME69X_NEW_DATA_MSK
        ):
            diff = high.meas_index - low.meas_index
            if (-3 < diff < 0) or (diff > 2):
                fields[low_index], fields[high_index] = high, low
        elif high.status & BME69X_NEW_DATA_MSK:
            fields[low_index], fields[high_index] = high, low

    # ------------------------------------------------------------------
    # Compensation, the floating point path of the Bosch API
    # ------------------------------------------------------------------

    def _calc_temperature(self, temp_adc):
        """Compensate the raw temperature, result in degrees Celsius."""
        calib = self._calib

        do1 = calib["par_t1"] << 8
        dtk1 = calib["par_t2"] / (1 << 30)
        dtk2 = calib["par_t3"] / float(1 << 48)

        cf = temp_adc - do1

        return cf * dtk1 + cf * cf * dtk2

    def _calc_pressure(self, pres_adc, comp_temperature):
        """Compensate the raw pressure, result in Pascal."""
        calib = self._calib
        temp = comp_temperature

        o = calib["par_p1"] * (1 << 3)
        tk10 = calib["par_p2"] / (1 << 6)
        tk20 = calib["par_p3"] / (1 << 8)
        tk30 = calib["par_p4"] / (1 << 15)

        s = (calib["par_p5"] - (1 << 14)) / (1 << 20)
        tk1s = (calib["par_p6"] - (1 << 14)) / float(1 << 29)
        tk2s = calib["par_p7"] / float(1 << 32)
        tk3s = calib["par_p8"] / float(1 << 37)

        nls = calib["par_p9"] / float(1 << 48)
        tknls = calib["par_p10"] / float(1 << 48)

        # 2^65 overflows a double, so the shift is split into two factors
        nls3 = calib["par_p11"] / (float(1 << 35) * float(1 << 30))

        tmp1 = (
            o
            + tk10 * temp
            + tk20 * temp * temp
            + tk30 * temp * temp * temp
        )
        tmp2 = pres_adc * (
            s + tk1s * temp + tk2s * temp * temp + tk3s * temp * temp * temp
        )
        tmp3 = pres_adc * pres_adc * (nls + tknls * temp)
        tmp4 = pres_adc * pres_adc * pres_adc * nls3

        return tmp1 + tmp2 + tmp3 + tmp4

    def _calc_humidity(self, hum_adc, comp_temperature):
        """Compensate the raw humidity, result in percent relative humidity."""
        calib = self._calib

        temp_comp = comp_temperature * 5120 - 76800

        oh = calib["par_h1"] * (1 << 6)
        sh = calib["par_h5"] / float(1 << 16)
        tk10h = calib["par_h2"] / float(1 << 14)
        tk1sh = calib["par_h4"] / float(1 << 26)
        tk2sh = calib["par_h3"] / float(1 << 26)
        hlin2 = calib["par_h6"] / float(1 << 19)

        hoff = hum_adc - (oh + tk10h * temp_comp)
        hsens = (
            hoff
            * sh
            * (
                1
                + tk1sh * temp_comp
                + tk1sh * tk2sh * temp_comp * temp_comp
            )
        )
        hum = hsens * (1 - hlin2 * hsens)

        if hum < 0.0:
            hum = 0.0
        elif hum >= 100.0:
            hum = 100.0

        return hum

    @staticmethod
    def _calc_gas_resistance(gas_res_adc, gas_range):
        """Compensate the raw gas reading, result in Ohm."""
        var1 = 262144 >> gas_range
        var2 = 4096 + (gas_res_adc - 512) * 3

        return 1000000.0 * var1 / var2

    def _calc_res_heat(self, temp):
        """Convert a heater temperature into a heater resistance register."""
        calib = self._calib

        if temp > 400:  # Cap the temperature
            temp = 400

        var1 = calib["par_g1"] / 16.0 + 49.0
        var2 = calib["par_g2"] / 32768.0 * 0.0005 + 0.00235
        var3 = calib["par_g3"] / 1024.0
        var4 = var1 * (1.0 + var2 * temp)
        var5 = var4 + var3 * self.amb_temp

        res_heat = int(
            3.4
            * (
                var5
                * (4 / (4 + calib["res_heat_range"]))
                * (1 / (1 + calib["res_heat_val"] * 0.002))
                - 25
            )
        )

        return res_heat & 0xFF

    @staticmethod
    def _calc_gas_wait(dur):
        """Convert a heating duration in milliseconds into a register value."""
        if dur >= 0xFC0:
            return 0xFF  # Maximum duration

        factor = 0
        while dur > 0x3F:
            dur //= 4
            factor += 1

        return (dur + factor * 64) & 0xFF

    @staticmethod
    def _calc_heatr_dur_shared(dur):
        """Convert the shared heating duration into a register value."""
        if dur >= 0x783:
            return 0xFF  # Maximum duration

        # The step size of the shared duration is 0.477 ms
        dur = dur * 1000 // 477
        factor = 0
        while dur > 0x3F:
            dur >>= 2
            factor += 1

        return (dur + factor * 64) & 0xFF

    # ------------------------------------------------------------------
    # Identification, self test and status
    # ------------------------------------------------------------------

    def get_unique_id(self):
        """
        Get the unique ID of the sensor.

        :return: Unique ID as an integer, None when the read failed
        """
        id_regs = self.read_reg(BME69X_REG_UNIQUE_ID, 4)
        if id_regs is None:
            return None

        id1 = (id_regs[3] + (id_regs[2] << 8)) & 0x7FFF

        return (id1 << 16) + (id_regs[1] << 8) + id_regs[0]

    def self_test(self):
        """
        Run the built-in self test of the sensor.

        The test takes a few seconds and leaves the sensor in sleep mode with
        its own configuration, so the sensor has to be set up again afterwards.

        :return: BME69X_OK when the test passed, an error code otherwise
        """
        amb_temp = self.amb_temp
        self.amb_temp = 25

        try:
            self.status = BME69X_OK
            self._self_test()
        except OSError:
            self.intf_rslt = BME69X_E_COM_FAIL
            self.status = BME69X_E_COM_FAIL
        finally:
            self.amb_temp = amb_temp

        return self.status

    def _self_test(self):
        """Measure with two heater profiles and check the readings."""
        self._init_sensor()

        self._conf["os_hum"] = BME69X_OS_1X
        self._conf["os_pres"] = BME69X_OS_16X
        self._conf["os_temp"] = BME69X_OS_2X

        self._heatr_conf["enable"] = BME69X_ENABLE
        self._heatr_conf["heatr_dur"] = BME69X_HEATR_DUR1
        self._heatr_conf["heatr_temp"] = BME69X_HIGH_TEMP

        data = [BME690Data() for _ in range(BME69X_N_MEAS)]

        self._forced_measurement(BME69X_HEATR_DUR1_DELAY, data[0])

        if not (
            data[0].idac not in (0x00, 0xFF)
            and (data[0].status & BME69X_GASM_VALID_MSK)
        ):
            self.status = BME69X_E_SELF_TEST
            return

        self._heatr_conf["heatr_dur"] = BME69X_HEATR_DUR2

        for i in range(BME69X_N_MEAS):
            if i % 2 == 0:
                self._heatr_conf["heatr_temp"] = BME69X_HIGH_TEMP
            else:
                self._heatr_conf["heatr_temp"] = BME69X_LOW_TEMP

            self._forced_measurement(BME69X_HEATR_DUR2_DELAY, data[i])

        self._analyze_sensor_data(data, BME69X_N_MEAS)

    def _forced_measurement(self, wait_time, data):
        """Run one forced mode measurement with the cached configuration."""
        self._set_heatr_conf(BME69X_FORCED_MODE)
        self._set_conf()
        self._set_op_mode(BME69X_FORCED_MODE)
        time.sleep_us(wait_time)
        self._read_field_data(0, data)

    def _analyze_sensor_data(self, data, n_meas):
        """Check whether the self test measurements are plausible."""
        failed = 0

        if not (
            BME69X_MIN_TEMPERATURE
            <= data[0].temperature
            <= BME69X_MAX_TEMPERATURE
        ):
            failed += 1

        if not (
            BME69X_MIN_PRESSURE <= data[0].pressure <= BME69X_MAX_PRESSURE
        ):
            failed += 1

        if not (
            BME69X_MIN_HUMIDITY <= data[0].humidity <= BME69X_MAX_HUMIDITY
        ):
            failed += 1

        # Every gas measurement has to be valid
        for i in range(n_meas):
            if not (data[i].status & BME69X_GASM_VALID_MSK):
                failed += 1

        cent_res = 0
        if n_meas >= 6 and data[4].gas_resistance:
            cent_res = int(
                5
                * (data[3].gas_resistance + data[5].gas_resistance)
                / (2 * data[4].gas_resistance)
            )

        if cent_res < 6:
            failed += 1

        if failed:
            self.status = BME69X_E_SELF_TEST

    def intf_error(self):
        """
        Get the result of the last I2C transfer.

        :return: BME69X_OK, or BME69X_E_COM_FAIL when a transfer failed
        """
        return self.intf_rslt

    def check_status(self):
        """
        Check whether an error or a warning has occurred.

        :return: BME69X_ERROR, BME69X_WARNING or BME69X_OK
        """
        if self.status < BME69X_OK:
            return BME69X_ERROR

        if self.status > BME69X_OK:
            return BME69X_WARNING

        return BME69X_OK

    def status_string(self):
        """
        Get a short description of the current status code.

        :return: Description of the status, an empty string when it is OK
        """
        return _STATUS_STRINGS.get(self.status, "Undefined error code")
