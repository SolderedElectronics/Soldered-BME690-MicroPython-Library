# Soldered BME690 MicroPython Library

| ![BME690 breakout board](https://upload.wikimedia.org/wikipedia/commons/8/8f/Example_image.svg) |
| :--------------------------------------------------------------------------------------------: |
|                          [BME690 breakout board](https://www.solde.red/333411)                     |

Breakout board for the Bosch BME690 sensor, which measures temperature, relative humidity, barometric pressure and gas resistance (VOC). The board communicates over I2C only and is part of the [Qwiic ecosystem](https://soldered.com/collections/qwiic-ecosystem).

### Quick start

```python
from bme690 import BME690, BME69X_FORCED_MODE
import time

sensor = BME690()               # Or BME690(address=BME69X_I2C_ADDR_HIGH)
sensor.set_tph()                # Default over-sampling
sensor.set_heater_prof(300, 100)  # 300 degrees C for 100 ms

while True:
    sensor.set_op_mode(BME69X_FORCED_MODE)
    time.sleep_us(sensor.get_meas_dur())
    if sensor.fetch_data():
        data, _ = sensor.get_data()
        print(data.temperature)
    time.sleep(1)
```

Have a look at the scripts in `Examples/` for forced, parallel and sequential mode, for the built-in self test and for the BME AI-Studio raw data logger.

The driver uses the floating point compensation of the Bosch API. Ports of MicroPython which use single precision floats, the ESP32 among them, are therefore slightly less precise than the Arduino library, well below the accuracy of the sensor itself.

### How to install

Use [mim](https://checkmim.com/packages).

or

After [**installing the mpremote package**](https://docs.micropython.org/en/latest/reference/mpremote.html), install the library on your board using the following command:

```sh
  mpremote mip install github:SolderedElectronics/Soldered-BME690-MicroPython-Library
```
Or, if you're running a Windows OS:

```sh
  python -m mpremote mip install github:SolderedElectronics/Soldered-BME690-MicroPython-Library
```

### Repository Contents

- **bme690.py** - MicroPython driver class, I2C only
- **package.json** - mip install manifest
- **/Examples** - examples for forced, parallel and sequential mode, the built-in self test and the BME AI-Studio raw data logger

### Examples

| Example | What it does |
| :------ | :----------- |
| `bme690-forcedMode.py` | One measurement at a time, the usual way to read the sensor |
| `bme690-sequentialMode.py` | The sensor steps through a heater profile on its own, sleeping in between |
| `bme690-parallelMode.py` | The gas sensor sweeps a heater profile while TPH is measured continuously |
| `bme690-selfTest.py` | Runs the built-in self test and prints the unique ID |
| `bme690-aiStudioLogger.py` | Records a `.bmerawdata` file on an SD card for BME AI-Studio. Needs an ESP32 with WiFi for the NTP synced real time clock, an SD card module on the SPI pins and the `sdcard` driver of micropython-lib, which `mip` installs along with this library |

### Hardware design

You can find hardware design for this board in _BME690 breakout board_ hardware repository.

### Documentation

Access library documentation [here](https://docs.soldered.com/).

### About Soldered

![Soldered Logo](https://raw.githubusercontent.com/SolderedElectronics/Soldered-Generic-Arduino-Library/dev/extras/Soldered-logo-color.png)

At Soldered, we design and manufacture a wide selection of electronic products to help you turn your ideas into acts and bring you one step closer to your final project. Our products are intented for makers and crafted in-house by our experienced team in Osijek, Croatia. We believe that sharing is a crucial element for improvement and innovation, and we work hard to stay connected with all our makers regardless of their skill or experience level. Therefore, all our products are open-source. Finally, we always have your back. If you face any problem concerning either your shopping experience or your electronics project, our team will help you deal with it, offering efficient customer service and cost-free technical support anytime. Some of those might be useful for you:

- [Web Store](https://www.soldered.com/shop)
- [Tutorials & Projects](https://soldered.com/learn)
- [Documentation](https://docs.soldered.com)

### Original source

This library is a port of the [Soldered BME690 Arduino library](https://github.com/SolderedElectronics/Soldered-BME690-Arduino-Library), which wraps the [BME69x Sensor API](https://github.com/boschsensortec/BME69x_SensorAPI) by Bosch Sensortec. Thank you, Bosch Sensortec.

### Open-source license

Soldered invests vast amounts of time into hardware & software for these products, which are all open-source. Please support future development by buying one of our products.

Check license details in the LICENSE file. Long story short, use these open-source files for any purpose you want to, as long as you apply the same open-source licence to it and disclose the original source. No warranty - all designs in this repository are distributed in the hope that they will be useful, but without any warranty. They are provided "AS IS", therefore without warranty of any kind, either expressed or implied. The entire quality and performance of what you do with the contents of this repository are your responsibility. In no event, Soldered (TAVU) will be liable for your damages, losses, including any general, special, incidental or consequential damage arising out of the use or inability to use the contents of this repository.

## Have fun!

And thank you from your fellow makers at Soldered Electronics.
