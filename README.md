# HB-UNI-Sen-CO2

### CO2 Sensor for Homematic with STM32 or RP2040 pico

Inspired by the excellent work of [MHSteve](https://github.com/HMSteve/HB-UNI-Sen-CO2/blob/main/README.md) I enhanced his solution by the following features:
- Use of STM32 boards (Blue Pill or Maple Mini) or RP2040 pico board
- Compatibility with standard sensor casing (86x86 mm)
- Use of Winsen MH-Z19 CO2 Sensor instead of SCD30 as option
- Use of touch pad as config button
- Use of RGB LED for traffic light instead of red/green LED

Due to the missing battery support of the STM32 and RP2040 boards and the space restrictions I left out the rechargeable batteries which seems to be bearable for me, since a power supply is needed anyway.

Here you can see some photos how the assembled boards look like

![Top view of assembled PCB with Maple Mini](https://github.com/bestfan/HB-UNI-Sen-CO2-STM32/blob/main/Pictures/maplemini_topview.jpg)

![Perspective view of assembled PCB with Maple Mini](https://github.com/bestfan/HB-UNI-Sen-CO2-STM32/blob/main/Pictures/maplemini_perspective.jpg)

![Perspective view of assembled PCB with Blue Pill](https://github.com/bestfan/HB-UNI-Sen-CO2-STM32/blob/main/Pictures/bluepill_perspective.jpg)

And here how the sensor casing looks at the end

![Perspective view of assembled PCB with Blue Pill](https://github.com/bestfan/HB-UNI-Sen-CO2-STM32/blob/main/Pictures/casing_perspective.jpg)


## Software

### CCU Addon

Please, use the [Addon](https://github.com/HMSteve/SG-HB-Devices-Addon/raw/master/CCU_RM/sg-hb-devices-addon.tgz) of the original project. As stated there, the CO2 sensor ist supported from the version 1.23 onward. Due to the missing battery the voltage values shown in the CCU should be ignored.

### Sketch

In comparison to the original work, where the following libraries have been required
- AskSinPP
- EnableInterrupt
- LowPower
- SparkFun SCD30 Arduino Library
- Adafruit BME280 Library
- GxEPD (replaced by GxEPD2)
- Adafruit GFX

some additional libraries have to be applied
- MH-Z19
- bluepill_ws2812 (STM32)
- Adafruit NeoPixel (RP2040)
- GxEPD2 (STM32 and RP2040)
- OneWire (for use DS18B20 on RP2040 only), use of main version necessary.

If you decide to use a STM32 Blue Pill microcontroller, please, make sure that the STM32 bootloader is installed. It is possible to use the standard serial port to flash the software (or the bootloader), however the display won't work, because some of the connections are attached to debug pins.

## Credits

Special thanks to [MHSteve](https://github.com/HMSteve/HB-UNI-Sen-CO2/blob/main/README.md) for his high sophisticated work on the HB-UNI-Sen-CO2 Sensor. I also have to thank [der-pw](https://github.com/der-pw/hm_stm32_2ch_dimmer) for the very helpful STM32 example.

## Disclaimer

The usage of the published software and information is at your own risk and without any warranty.

## License

**Creative Commons BY-NC-SA**<br>
Give Credit, NonCommercial, ShareAlike

<a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://mirrors.creativecommons.org/presskit/buttons/88x31/svg/by-nc-sa.eu.svg" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/">Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License</a>.
