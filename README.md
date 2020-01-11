# TIG-Pedal

## Building
This project is configured with PlatformIO and `esp8266-nonos-sdk`  PlatformIO stagnates with esp sdk updates, so this was build on 2.1.x (not 3.x). Keep that in mind while porting to other platforms.

## Pinout
GPIO16 - speaker from hell
GPIO13 - ADC switch (Input for reading Pedal pot, LOW otherwise)
GPIO14 - ADC switch (Input for reading Limit pot, LOW otherwise)
GPIO12 - PWM Output