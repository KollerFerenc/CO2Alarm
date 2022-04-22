# CO2 levels - https://www.kane.co.uk/knowledge-centre/what-are-safe-levels-of-co-and-co2-in-rooms
# 250-400 ppm - Normal background concentration in outdoor ambient air
# 400-1000 ppm - Concentrations typical of occupied indoor spaces with good air exchange
# 1000-2000 ppm - Complaints of drowsiness and poor air
# 2000-5000 ppm - Headaches, sleepiness and stagnant, stale, stuffy air.
#                 Poor concentration, loss of attention, increased heart rate and slight nausea may also be present.
# 5000 - Workplace exposure limit (as 8-hour TWA) in most jurisdictions.
# >40000 ppm - Exposure may lead to serious oxygen deprivation resulting in permanent brain damage, coma, even death.

# SCD41 range: 400–5000 ppm


############################################################
# Imports
############################################################
import time
import machine
import os
import micropython
import math

from machine import Pin
from machine import ADC
from machine import PWM
from buzzers import Buzzer3v5v
from buzzers import PiezoBuzzer

import pimoroni_i2c
import breakout_scd41

############################################################
# Configurables
############################################################
SCD41_SDA_PIN = 4  # GPIO pin number on the Pico
SCD41_SCL_PIN = 5  # GPIO pin number on the Pico
BUZZER_PIN = 15  # GPIO pin number on the Pico

# # NOTICE: buzzer selection!

# Use this with the 5V Buzzer by Adafruit (https://www.adafruit.com/product/1536)
# BUZZER = Buzzer3v5v(BUZZER_PIN)

# Use this with any regular piezo buzzer
# BUZZER = PiezoBuzzer(BUZZER_PIN)

# co2 measurement interval in minutes
MEASURE_INTERVAL_MINUTES = 5
# when high co2 is detected, sleep for this fraction of measurement interval
MEASURE_HIGH_NOTICE_SLEEP_FACTOR = 5

# co2 level: normal upper threshold in ppm
CO2_LEVEL_NORMAL_UPPER = 1500
# co2 level: high upper threshold in ppm. Anything higher is considered dangerous!
CO2_LEVEL_HIGH_UPPER = 2500

# battery low warning percent
BATTERY_LOW_WARNING_PERCENT = 15

# print debug message to console
DEBUG = False
# treats first measurement as not trusted, ie sleep after first measurement as if high co2 happened
CHECK_FOR_FIRST_MEASURE = False
ALLOW_LED = False

############################################################
# Constants
############################################################
PINS_BREAKOUT_GARDEN = {"sda": SCD41_SDA_PIN, "scl": SCD41_SCL_PIN}
I2C = pimoroni_i2c.PimoroniI2C(**PINS_BREAKOUT_GARDEN)

ONBOARD_LED = Pin(25, Pin.OUT)
VSYS_PIN = ADC(29)
CHARGING_PIN = Pin(24, Pin.IN)

BATTERY_CONVERSION_FACTOR = 3 * 3.3 / 65535
FULL_BATTERY_VOLTAGE = 4.2
EMPTY_BATTERY_VOLTAGE = 2.8

############################################################
# Functions
############################################################
def blink_led(times):
    if not ALLOW_LED:
        return

    turn_off_led()
    for _ in range(times):
        turn_on_led()
        time.sleep(0.1)
        turn_off_led()


def turn_on_led():
    if not ALLOW_LED:
        return

    ONBOARD_LED.value(1)


def turn_off_led():
    ONBOARD_LED.value(0)


def debug(value):
    if DEBUG:
        print("[DBG] {}".format(value))


def battery_status():
    charging = CHARGING_PIN.value() == 1
    voltage = VSYS_PIN.read_u16() * BATTERY_CONVERSION_FACTOR
    percentage = 100 * (
        (voltage - EMPTY_BATTERY_VOLTAGE)
        / (FULL_BATTERY_VOLTAGE - EMPTY_BATTERY_VOLTAGE)
    )
    percentage = max(0.0, min(percentage, 100.0))
    return charging, voltage, percentage


def setup():
    turn_off_led()
    turn_on_led()
    BUZZER.bequiet()
    time.sleep(1.0)
    debug("scd41 init and start...")
    breakout_scd41.init(I2C)
    time.sleep(1.0)
    breakout_scd41.stop()
    time.sleep(1.0)
    breakout_scd41.start()
    debug("scd41 init and start done.")
    turn_off_led()


def loop():
    firstmeasure = CHECK_FOR_FIRST_MEASURE
    measured = False
    stopped = False
    co2_high_count = 0
    while True:
        if stopped:
            breakout_scd41.start()

        chrg, v, p = battery_status()
        debug("Charging: {}, voltage: {:.2f}V, percentage: {:.0f}%".format(chrg, v, p))

        if not chrg and p <= BATTERY_LOW_WARNING_PERCENT:
            debug("Battery level low!")
            BUZZER.beep_boop(2, False)
            time.sleep(2.0)
            BUZZER.beep_boop(2, False)

        debug("Trying to measure...")
        co2_high = False

        if breakout_scd41.ready():
            blink_led(1)
            co2, temperature, humidity = breakout_scd41.measure()
            measured = True
            debug(
                "CO2: {}, Temp.: {:.2f}C, Humi.: {:.0f}%".format(
                    co2, temperature, humidity
                )
            )

            if co2 <= CO2_LEVEL_NORMAL_UPPER:
                debug("co2 level good")
            elif co2 <= CO2_LEVEL_HIGH_UPPER:
                debug("co2 high")
                co2_high = True
                co2_high_count = co2_high_count + 1
                BUZZER.beep_boop(5, False)
            else:
                debug("co2 danger")
                co2_high = True
                co2_high_count = co2_high_count + 1
                BUZZER.beep_boop(30, True)
        else:
            measured = False
            debug("scd41 not ready.")
            blink_led(2)

        if measured:
            measured = False

            debug("co2 high: {}, High count: {}".format(co2_high, co2_high_count))

            if not co2_high and co2_high_count < 1:
                co2_high_count = 0
                sleep_time = MEASURE_INTERVAL_MINUTES * 60

                if firstmeasure:
                    firstmeasure = False
                    sleep_time = (
                        MEASURE_INTERVAL_MINUTES * 60
                    ) // MEASURE_HIGH_NOTICE_SLEEP_FACTOR
                    debug("fm lightsleep start. ({}s)".format(sleep_time))
                    breakout_scd41.stop()
                    stopped = True
                    machine.lightsleep(sleep_time * 1000)
                else:
                    debug("deepsleep start. ({}s)".format(sleep_time))
                    breakout_scd41.stop()
                    stopped = True
                    machine.deepsleep(sleep_time * 1000)
            else:
                co2_high_count = co2_high_count - 1
                sleep_time = (
                    MEASURE_INTERVAL_MINUTES * 60
                ) // MEASURE_HIGH_NOTICE_SLEEP_FACTOR
                debug("lightsleep start. ({}s)".format(sleep_time))
                breakout_scd41.stop()
                stopped = True
                machine.lightsleep(sleep_time * 1000)

        else:
            time.sleep(1.0)


def main():
    setup()
    loop()


############################################################
# Script execution
############################################################

if __name__ == "__main__":
    main()
