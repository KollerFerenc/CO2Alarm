# Carbon dioxide (CO<sub>2</sub>) alarm

## Table of Contents

- [Hardware Requirements](#hardware-requirements)
- [Assembly and wiring](#assembly-and-wiring)
  - [LiPo SHIM for Pico](#lipo-shim-for-pico)
  - [Buzzer](#buzzer)
  - [SCD41 CO2 Sensor](#scd41-co2-sensor)
- [Software and installation](#software-and-installation)

## Hardware Requirements

- 1x [Raspberry Pi Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/)
- 1x [SCD41 CO2 Sensor by Pimoroni](https://shop.pimoroni.com/products/scd41-co2-sensor-breakout?variant=39652270833747)
- 1x [Buzzer 5V by Adafruit](https://www.adafruit.com/product/1536)
  - NOTE: can be substituted with regular [piezo buzzer](https://www.adafruit.com/product/160)
  - OBSERVATION: this seemed louder than a regular piezo while playing around with them
- (Optional) 1x [LiPo SHIM for Pico by Pimoroni](https://shop.pimoroni.com/products/pico-lipo-shim?variant=32369543086163)
  - For wired operation this is not required
  - NOTICE: installing headers onto the Pico is recommended
  - NOTE: Pimoroni sells these built into their own Pico models with [4MB](https://shop.pimoroni.com/products/pimoroni-pico-lipo?variant=39386149093459) and [16MB](https://shop.pimoroni.com/products/pimoroni-pico-lipo?variant=39335427080275) of flash memory.
- (Optional) 1x 3.7V rechargeable battery
  - [6700mAh lithium ion battery](https://shop.pimoroni.com/products/high-capacity-lithium-ion-battery-pack?variant=32012684591187)
- Solder, soldering iron and wires to conenct everything together
- (Optional) Breadboard for testing
- (Optional) Headers for the Pico and the CO2 sensor make testing easy

## Assembly and wiring

- [Raspberry Pi Pico documentation and pinout](https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html)
- [SCD41 CO2 Sensor by Pimoroni schematic](https://cdn.shopify.com/s/files/1/0174/1800/files/scd41_breakout_schematic.pdf)
- [LiPo SHIM for Pico schematic](https://cdn.shopify.com/s/files/1/0174/1800/files/lipo_shim_for_pico_schematic.pdf)

### LiPo SHIM for Pico

> [You'll need to solder the SHIM to the back of your Pico, with the power button at the same end as the USB port. The text on the SHIM and the pin labels on the back of the Pico should be facing each other.](https://shop.pimoroni.com/products/pico-lipo-shim?variant=32369543086163)

### Buzzer

- Positive pin -> GPIO 15 (pin 20)
- Negative pin -> GND (pin 18)
  - NOTE: any GND pin is okay

### SCD41 CO2 Sensor

- 3-5V -> 3V3 (pin 36)
- SDA -> GPIO 4 (pin 6)
- SCL -> GPIO 5 (pin 7)
- GND -> GND (pin 8)
  - NOTE: any GND pin is okay

## Software and installation

- [A. MicroPython version](./src/MicroPython/README.md)
