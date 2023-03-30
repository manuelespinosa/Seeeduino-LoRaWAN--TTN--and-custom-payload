# Seeeduino-LoRaWAN (TTN) with custom payload

Simple sketch for sending data with LoRaWAN (TTN was used) with a Seeeduino LoRaWAN board.

This repo send Battery Voltage and Location at almost regular intervals (time drift due to processing and transfer times). Feel free to clone and edit to your needs.

![imagen](https://user-images.githubusercontent.com/48633457/210238046-fb00c1d0-6098-40af-b9d8-d370ffdc34cc.png)

This repo use an external GPS module, but the code is compatible with Seeeduino LoRaWAN with GPS version without changes, except for GPSEnable pin which is routed to an external pin instead of GPS footprint.


## Steps
1. Clone this repo (prefered with Visual Studio Code)
2. Modify src/credentials.h with your LoRaWAN credentials.
3. Upload


## Basic Features
1. Uncomment `#define USBDEBUG` to avoid sleep mode (and keep device link via USB)
2. tinyFrame embedded. Check https://github.com/manuelespinosa/tinyFrame for more info.
3. GPS and Battery Voltage send every `GPS_DATA_INTERVAL_S` and `BATTERY_DATA_INTERVAL_S` (modify to change the interval)
4. LowPower GPS when not in use. Select your GPS Enable pin with `#define GPSEnable pinNumber`
5. Change the GPS_DATA_INTERVAL_S and BATTERY_DATA_INTERVAL_S via downlink message

## Payload decoder/encoder

### Uplink Decoder
Use the following basic javascript decoder in The Things Network Uplink Payload Formatter to get the repo demo data decoding:

``` javascript
function decodeUplink(input) {
  decoded = {};
  if (input.fPort == 1){
    decoded.bytes = input.bytes;
  }else if (input.fPort == 2){
    decoded.bytes = input.bytes;
  }else if (input.fPort == 3){
    decoded.bytes = input.bytes;
  }else if (input.fPort == 66){ // Device GPS fPort
    decoded.latitude = (input.bytes[0] << 0 | input.bytes[1] << 8 | input.bytes[2] << 16 | input.bytes[3] << 24)/100000;
    decoded.longitude = (input.bytes[4] << 0 | input.bytes[5] << 8 | input.bytes[6] << 16 | input.bytes[7] << 24)/100000;
    decoded.altitude = (input.bytes[8] << 0 | input.bytes[9] << 8) << 16 >> 16;
    decoded.satellites = input.bytes[10] >>> 0;
  }else if (input.fPort == 99){ // Device Status fPort
    decoded.batteryVoltage = ((input.bytes[0] << 0 | input.bytes[1] << 8)>>> 0)/1000;
  }
  return {
    data: decoded,
    warnings: [],
    errors: []
  };
}
```
fPort 1, 2 and 3 are unused, and will show the raw bytes as decoded payload.
fPort 66 is used to send latitute, longitude, altitude and satellites seen by the GPS module. Interpreting this data in this way, will allow to TTN to update automatically the device "location" atribute.
fPort 99 is used for device status info, in this case, only battery voltage is sent.


### Downlink Encoder/Decoder
As stated previously, this repository demo has the ability to change the send data interval (battery and GPS) via downlink message. You can use the following javascript encoder/decoder:

``` javascript
function encodeDownlink(input) {
  var data = input.data;
  if(input.fPort==100){
      var bytearray = [];
      bytearray[0] = data.GPSInterval>>8;
      bytearray[1] = data.GPSInterval & 0xFF;
      bytearray[2] = data.BatteryInterval >> 8;
      bytearray[3] = data.BatteryInterval & 0xFF;
  }

  return {
    bytes: bytearray,
    fPort: input.fPort,
    warnings: [],
    errors: []
  };
}

function decodeDownlink(input) {
    if(input.fPort==100){
      decoded = {};
      bytes = input.bytes;
      decoded.GPSInterval =  bytes[0] << 8 | bytes[1];
      decoded.BatteryInterval = bytes[2] << 8 | bytes[3];
  }
  return {
    data: {
      bytes: input.bytes,
      fPort: input.fPort,
      decoded: decoded
    },
    warnings: [],
    errors: []
  }
}
```

fPort 100 is used for changing the GPS and Battery sending interval:
First 2 bytes (0 and 1) is used for GPS Interval, the following 2 bytes (2 and 3) is used to interpret the Battery Interval.

After saving the downlink encoder you can enqueue downlink the message to change the sending interval. Go to messaging of the device, select downlink, specify fPort 100 and json payload. Schedule a json payload with the following template template:
```json
{
  "GPSInterval": 1800, 
  "BatteryInterval": 1800 
}
```
Then, press the button to schedule the downlink message. It should looks like the following image:

<details>
  <summary>Click to show an image of downlink message ready to schedule</summary>
  
  ![imagen](https://user-images.githubusercontent.com/48633457/211188156-ceafac23-b636-4b82-a866-06d4e99598a1.png)

</details>


### Seeduino Response to the downlink message:
```console
+MSGHEX: PORT: 100; RX: "07080708"
+MSGHEX: RXWIN1, RSSI -70, SNR 5.5
+MSGHEX: Done
Length is: 4
RSSI is: -70
fPort is: 100
Data is: 0x7 0x8 0x7 0x8
fport is Sample Rate Configurator
Current GPS and Battery Interval (s): 300 30
New GPS and Battery Interval (s): 1800 1800
``` 

## Seeeduino LoRaWAN issue:
### Battery Voltage can't not be read properly when battery is not charging.
This is due to missing resistor (R56) on the board, according to schematics:

<img src="https://user-images.githubusercontent.com/48633457/210239672-7bc01309-a427-4e3f-9cc6-c7084abe6b41.png" width="500">

If your Battery voltage readings seems not working properly, check for R56 and short circuit it. Check the following image to fix it:

<img src="https://user-images.githubusercontent.com/48633457/210240174-d1c6c4a5-182f-4a22-853e-683f2c5e91b8.png" width="500" height="500">
