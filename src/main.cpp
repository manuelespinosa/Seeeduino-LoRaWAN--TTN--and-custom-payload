#include <Arduino.h> 
#include <LoRaWan.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include "ArduinoLowPower.h"
#include <RTCZero.h>

#define FRAME_SERIAL SerialUSB
#include <tinyFrame.h>

#include "credentials.h"


#define DEVICE_STATUS_PORT 99
#define GPSDATA_PORT 66 

#define SLEEP_TIME_MILLIS 10*1000
#define DELAY_TIME_MILLIS 100

#define GPS_DATA_INTERVAL_S 15*60 
#define BATTERY_DATA_INTERVAL_S 10*60 

//#define USBDEBUG // While not commented, the device will not enter in sleep mode, and Serial will be available

#define GPSRx 8
#define GPSEnable 9
RTCZero rtc;
SoftwareSerial gpsSerial(GPSRx, -1); //Rx Tx
TinyGPSPlus gps;
struct tinyFrame frame;


const int pin_battery_status  = A5;
const int pin_battery_voltage = A4;

char buffer[256];
uint8_t buff[30];

uint32_t LAST_BATTERY_SEND = 0;
uint32_t LAST_GPS_DATA = 0;


void setup_lora(void);
void setup_rtc(void);

void handleGPS(void);

void read_battery(void);
void displayInfo(void);

bool loraTransfer(uint8_t *buff, uint8_t size, uint8_t fport);


void setup()
{

    SerialUSB.begin(115200);
    gpsSerial.begin(9600);


    for(unsigned char i = 0; i < 26; i ++){ // important, set all pins to HIGH to save power  
        pinMode(i, OUTPUT);
        digitalWrite(i, HIGH);
    }

    pinMode(pin_battery_status, INPUT);
    pinMode(GPSEnable, OUTPUT);

    setup_lora();
    while(!lora.setOTAAJoin(JOIN));
    //frame.printDecoder = true;

    setup_rtc();


    delay(5000); // Delay to allow upload sketch after boot
}


void loop()
{
    handleGPS();
    read_battery();

    digitalWrite(13, LOW);   // turn the LED off    
    #ifndef USBDEBUG
    LowPower.sleep(SLEEP_TIME_MILLIS);
    #endif    
    digitalWrite(13, HIGH);    // turn the LED on
    delay(DELAY_TIME_MILLIS);
}




void setup_lora (){
    lora.init();
    //memset(buffer, 0, 256);
    //lora.getVersion(buffer, 256, 1);
    //SerialUSB.print(buffer); 

    //memset(buffer, 0, 256);
    //lora.getId(buffer, 256, 1);
    //SerialUSB.print(buffer);

    lora.setKey(NULL, NULL, AppKey);
    lora.setId(NULL, devEUI, AppEUI);

    lora.setDeviceMode(LWOTAA);
    lora.setDataRate(DR0, EU868);

    //lora.setChannel(0, 868.1);
    //lora.setChannel(1, 868.3);
    //lora.setChannel(2, 868.5);

    //lora.setReceiceWindowFirst(0, 868.1);
    //lora.setReceiceWindowSecond(869.5, DR3);
    lora.setDutyCycle(false);
    lora.setJoinDutyCycle(false);
    //lora.setDutyCycle(true);
    //lora.setJoinDutyCycle(true);

    lora.setPower(14);
}


void setup_rtc(void){
    rtc.setEpoch(1672531726);
    LAST_BATTERY_SEND = rtc.getEpoch();
    LAST_GPS_DATA = rtc.getEpoch();
    rtc.begin();
}




void handleGPS(void){
    if (rtc.getEpoch() - LAST_GPS_DATA < GPS_DATA_INTERVAL_S){
        digitalWrite(GPSEnable, LOW);
        return; // Si se han mandado datos hace menos de 15 min nos vemos
    } 

    SerialUSB.println("GPS handler running");
    digitalWrite(GPSEnable, HIGH);
    delay(1500);
    while (gpsSerial.available() > 0){
        if(gps.encode(gpsSerial.read())){
            displayInfo();
            if (gps.location.isValid()){
                frame.clear();
                frame.append_int32_t(gps.location.lat()*100000);
                frame.append_int32_t(gps.location.lng()*100000);
                frame.append_int16_t(gps.altitude.meters());
                if(loraTransfer(frame.buffer, frame.size(), GPSDATA_PORT)){
                    LAST_GPS_DATA = rtc.getEpoch();
                }
            }
        }
    }
}


void read_battery(){
    if (rtc.getEpoch() - LAST_BATTERY_SEND < BATTERY_DATA_INTERVAL_S) return; //Si no han pasado 5 minutos, vuelve
    
    
    SerialUSB.println("Battery handler running");
    int a = analogRead(pin_battery_voltage); // Leemos la primera vez para descartarla
    float v = 0;
    for (int i = 0; i<10; i++){
        a = analogRead(pin_battery_voltage);
        v += (a/1023.0*3300.0*11.0);  // there's an 1M and 100k resistor divider
    }
    v = v/10.0; //-255
    SerialUSB.print("BatVoltage: "); SerialUSB.println(v);
    //bool batStatus = digitalRead(pin_battery_status);  // Charge status return 0 while charging, return 1 while charge done or no battery insert.
    frame.clear();
    frame.append_uint16_t((uint16_t) v);
    if(loraTransfer(frame.buffer, frame.size(), DEVICE_STATUS_PORT)){
        LAST_BATTERY_SEND = rtc.getEpoch();
    }
    
}



void displayInfo(){
  SerialUSB.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    SerialUSB.print(gps.location.lat(), 6);
    SerialUSB.print(F(","));
    SerialUSB.print(gps.location.lng(), 6);
  }
  else
  {
    SerialUSB.print(F("INVALID"));
  }

  SerialUSB.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    SerialUSB.print(gps.date.month());
    SerialUSB.print(F("/"));
    SerialUSB.print(gps.date.day());
    SerialUSB.print(F("/"));
    SerialUSB.print(gps.date.year());
  }
  else
  {
    SerialUSB.print(F("INVALID"));
  }

  SerialUSB.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) SerialUSB.print(F("0"));
    SerialUSB.print(gps.time.hour());
    SerialUSB.print(F(":"));
    if (gps.time.minute() < 10) SerialUSB.print(F("0"));
    SerialUSB.print(gps.time.minute());
    SerialUSB.print(F(":"));
    if (gps.time.second() < 10) SerialUSB.print(F("0"));
    SerialUSB.print(gps.time.second());
    SerialUSB.print(F("."));
    if (gps.time.centisecond() < 10) SerialUSB.print(F("0"));
    SerialUSB.print(gps.time.centisecond());
  }
  else
  {
    SerialUSB.print(F("INVALID"));
  }

  SerialUSB.println();
  delay(100);
}


bool loraTransfer(uint8_t *buff, uint8_t size, uint8_t fport){
    bool result = false;
    lora.setPort(fport); // Different Frame decoder can be assigned according to port
    result = lora.transferPacket(buff, size, 10);
    if(result){
        short length;
        short rssi;
        memset(buffer, 0, 256);
        length = lora.receivePacket(buffer, 256, &rssi);

        if(length){
            SerialUSB.print("Length is: "); SerialUSB.println(length);
            SerialUSB.print("RSSI is: "); SerialUSB.println(rssi);
            SerialUSB.print("Data is: ");
            for(unsigned char i = 0; i < length; i ++)
            {
                SerialUSB.print("0x"); SerialUSB.print(buffer[i], HEX); SerialUSB.print(" ");
            }
            SerialUSB.println();
        }
    }

    return result;
}

