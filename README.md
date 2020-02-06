# TIG-Pedal

## Building

This project is configured with PlatformIO and `esp8266-nonos-sdk`  PlatformIO stagnates with esp sdk updates, so this was build on 2.1.x (not 3.x). Keep that in mind while porting to other platforms.

## Pinout

- GPIO16 - speaker from hell
- GPIO13 - ADC switch (Input for reading Pedal pot, LOW otherwise)
- GPIO14 - ADC switch (Input for reading Limit pot, LOW otherwise)
- GPIO12 - PWM Output
- GPIO4 - Arc enable output

## Hardware

Schematics and PCB: https://easyeda.com/d02456cb/Tig_Pedal-43fc62ab19784925907a64371148c296

Mechanical drawings of the pedal itself
https://cad.onshape.com/documents/dafd589e2836768df6ee7de4/w/4cb48ad8f9a988a87bdeca1b/e/d1f0b4f194d8639d6ab9de63
