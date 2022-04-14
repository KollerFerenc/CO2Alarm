############################################################
# Imports
############################################################
import time

from machine import Pin
from machine import PWM

############################################################
# Classes
############################################################


class PiezoBuzzer:
    def __init__(self, id):
        self._buzzer = PWM(Pin(id))
        self._duty_u16 = 5000
        self._tone_gap = 0.3

    @property
    def duty_u16(self):
        return self._duty_u16

    @duty_u16.setter
    def duty_u16(self, value):
        self._duty_u16 = value

    @property
    def tone_gap(self):
        return self._tone_gap

    @tone_gap.setter
    def tone_gap(self, value):
        self._tone_gap = value

    def playtone(self, frequency):
        self._buzzer.duty_u16(self._duty_u16)
        self._buzzer.freq(frequency)

    def bequiet(self):
        self._buzzer.duty_u16(0)

    def beep_boop(self, times, cont, frequency=2000):
        if cont:
            self.playtone(frequency)
            time.sleep(times * self._tone_gap)
            self.bequiet()
        else:
            for _ in range(times):
                self.playtone(frequency)
                time.sleep(self._tone_gap)
                self.bequiet()
                time.sleep(self._tone_gap)
        self.bequiet()


class Buzzer3v5v:
    def __init__(self, id):
        self._buzzer = Pin(id, Pin.OUT)
        self._tone_gap = 0.3

    @property
    def tone_gap(self):
        return self._tone_gap

    @tone_gap.setter
    def tone_gap(self, value):
        self._tone_gap = value

    def bequiet(self):
        self._buzzer.value(0)

    def beep(self):
        self._buzzer.value(1)

    def beep_boop(self, times, cont):
        if cont:
            self.beep()
            time.sleep(times * self._tone_gap)
            self.bequiet()
        else:
            for _ in range(times):
                self.beep()
                time.sleep(self._tone_gap)
                self.bequiet()
                time.sleep(self._tone_gap)
        self.bequiet()


############################################################
# Script execution
############################################################

if __name__ == "__main__":
    pass
