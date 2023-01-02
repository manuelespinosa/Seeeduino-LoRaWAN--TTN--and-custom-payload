# Seeeduino-LoRaWAN--TTN--and-custom-payload

Simple sketch for sending data with LoRaWAN (TTN was used) with a Seeeduino LoRaWAN board.

This repo send Battery Voltage and Location at almost regular intervals (time drift due to processing and transfer times). Feel free to clone and edit to your needs.

![imagen](https://user-images.githubusercontent.com/48633457/210238046-fb00c1d0-6098-40af-b9d8-d370ffdc34cc.png)


## Steps
1. Clone this repo (prefered with Visual Studio Code)
2. Modify src/credentials.h with your LoRaWAN credentials.
3. Upload


## Basic Features
1. Uncomment `#define USBDEBUG` to avoid sleep mode (and keep device connected to USB)
2. tinyFrame embedded. Check https://github.com/manuelespinosa/tinyFrame for more info.
3. GPS and Battery Voltage send every `GPS_DATA_INTERVAL_S` and `BATTER_DATA_INTERVAL_S` (modify to change the interval)
4. LowPower GPS when not in use. Select your GPS Enable pin with `#define GPSEnable pinNumber`

## Seeeduino LoRaWAN issue:
### Battery Voltage can't not be read properly when battery is not charging.
This is due to missing resistor (R56) on the board, according to schematics:

![imagen](https://user-images.githubusercontent.com/48633457/210239672-7bc01309-a427-4e3f-9cc6-c7084abe6b41.png)

If your Battery voltage readings seems not working properly, check for R56 and short circuit. Check the following image to fix it:

![imagen](https://user-images.githubusercontent.com/48633457/210240174-d1c6c4a5-182f-4a22-853e-683f2c5e91b8.png)
