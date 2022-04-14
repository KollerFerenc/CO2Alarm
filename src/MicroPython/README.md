# MicroPython

MicroPython implementation of the project.

## Prerequisites

- Install the latest version of [Pimoroni's custom MicroPython](https://github.com/pimoroni/pimoroni-pico) which includes the libraries necessary to use their products. [Follow their installation guide.](https://github.com/pimoroni/pimoroni-pico/blob/main/setting-up-micropython.md)

## Instalation

**1. Clone the repository.**

``` shell
git clone https://github.com/KollerFerenc/CO2Alarm
```

**2. Connect the Raspberry Pi Pico to the computer.**

**3. Launch [Thonny IDE](https://thonny.org/) or the Python IDE of your choice.**

**4. Open the python files in the IDE and save them to the Pico.**
   - src -> MicroPython -> main.py
   - src -> MicroPython -> buzzers.py

**5. Buzzer selection**

In main.py uncomment which type of buzzer you are using.

*A. Regular piezo buzzer*

```python
# NOTICE: buzzer selection!

# Use this with the 5V Buzzer by Adafruit (https://www.adafruit.com/product/1536)
# BUZZER = Buzzer3v5v(BUZZER_PIN)

# Use this with any regular piezo buzzer
BUZZER = PiezoBuzzer(BUZZER_PIN)
```

*B. [Buzzer 5V by Adafruit](https://www.adafruit.com/product/1536)*

```python
# NOTICE: buzzer selection!

# Use this with the 5V Buzzer by Adafruit (https://www.adafruit.com/product/1536)
BUZZER = Buzzer3v5v(BUZZER_PIN)

# Use this with any regular piezo buzzer
# BUZZER = PiezoBuzzer(BUZZER_PIN)
```

Save the modified file to the Pico.

**6. (Optional) Adjust the values in the Configurables section to match your setup.**

Save the modified file to the Pico.