# C++

C++ implementation of the project.

## Prerequisites

- Setup the latest verison the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk), [Pico examples](https://github.com/raspberrypi/pico-examples) and [Pico extras](https://github.com/raspberrypi/pico-extras).
    - [Official guide by the Raspberry Pi Foundation.](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf)
- Setup the latest verison the [Pimoroni Pico Libraries](https://github.com/pimoroni/pimoroni-pico).
    - [Installation guide by Pimoroni.](https://github.com/pimoroni/pimoroni-pico/blob/main/setting-up-the-pico-sdk.md)

## Build and installation

**1. Clone the repository.**

``` shell
git clone https://github.com/KollerFerenc/CO2Alarm
```

**2. Buzzer selection**

In CO2Alarm -> src -> cpp -> main.cpp uncomment which type of buzzer you are using.

*A. Regular piezo buzzer*

```cpp
// NOTICE: buzzer selection!

// Use this with the 5V Buzzer by Adafruit (https://www.adafruit.com/product/1536)
// #define BUZZER_TYPE_BUZZER3V5V

// Use this with any regular piezo buzzer
#define BUZZER_TYPE_PIEZO
```

*B. [Buzzer 5V by Adafruit](https://www.adafruit.com/product/1536)*

```cpp
// NOTICE: buzzer selection!

// Use this with the 5V Buzzer by Adafruit (https://www.adafruit.com/product/1536)
#define BUZZER_TYPE_BUZZER3V5V

// Use this with any regular piezo buzzer
// #define BUZZER_TYPE_PIEZO
```

Save the file.

**3. (Optional) Adjust the values in the Configurables section to match your setup.**

Save the modified file.

**4. Create build directory**

``` shell
cd CO2Alarm
mkdir build
```

**5. Build project**

``` shell
cd build
cmake ..
make
```

**6. Copy the uf2 file to the Pico**

- Binary location: CO2Alarm -> build -> src -> cpp -> co2alarm.uf2
- Put the Pico into bootloader mode and copy the uf2 file to it.
