#include <Arduino.h> 
#include <LoRaWan.h>
#include <TinyGPSPlus.h>
#include "ArduinoLowPower.h"
#include <RTCZero.h>

#define FRAME_SERIAL SerialUSB
#include <tinyFrame.h>

#include "credentials.h"


#define DEVICE_STATUS_PORT 99
#define GPSDATA_PORT 66 

#define SLEEP_TIME_MILLIS 30*1000
#define DELAY_TIME_MILLIS 0



//#define USBDEBUG // While not commented, the device will not enter in sleep mode, and Serial will be available

#define GPSEnable 9
#define gpsSerial Serial

RTCZero rtc;
TinyGPSPlus gps;
struct tinyFrame frame;


const int pin_battery_status  = A5;
const int pin_battery_voltage = A4;

char buffer[256];
uint8_t buff[30];

uint16_t GPS_DATA_INTERVAL_S = 15*60;
uint16_t BATTERY_DATA_INTERVAL_S = 10*60;

uint32_t LAST_BATTERY_SEND = 0;
uint32_t LAST_GPS_DATA = 0;
uint32_t GPS_START_TIME = 0;

void setup_lora(void);
void setup_rtc(void);

void handleGPS(void);

void read_battery(void);
void displayInfo(void);

bool loraTransfer(uint8_t *buff, uint8_t size, uint8_t fport);
void payloadDecoder(char*, uint16_t, short);


void setup()
{   
    SerialUSB.begin(115200);
    gpsSerial.begin(9600);

    for(unsigned char i = 0; i < 26; i ++){ // important, set all pins to HIGH to save power  
        pinMode(i, OUTPUT);
        digitalWrite(i, HIGH);
    }
    //pinMode(pin_battery_status, INPUT);
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
    #else
    delay(SLEEP_TIME_MILLIS);
    #endif    
    digitalWrite(13, HIGH);    // turn the LED on
    delay(DELAY_TIME_MILLIS); // Use delay time millis > 0 if you need to see the led on every device wake up
}




void setup_lora (){
    lora.init();
    //memset(buffer, 0, 256); lora.getVersion(buffer, 256, 1); SerialUSB.print(buffer); 
    //memset(buffer, 0, 256); lora.getId(buffer, 256, 1); SerialUSB.print(buffer);

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
        return; // Si se han mandado datos hace menos de 15 min, abortamos
    }
    SerialUSB.println("GPS handler running");
    if (!digitalRead(GPSEnable)) GPS_START_TIME = rtc.getEpoch();
    digitalWrite(GPSEnable, HIGH);

    while(gpsSerial.available() > 0){
        if(gps.encode(gpsSerial.read())){
            if (gps.location.isValid()){
                displayInfo();
                SerialUSB.print("Satellites: ");SerialUSB.println(gps.satellites.value());
                frame.clear();
                frame.append_int32_t(gps.location.lat()*100000);
                frame.append_int32_t(gps.location.lng()*100000);
                frame.append_int16_t(gps.altitude.meters());
                frame.append_uint8_t(gps.satellites.value());
                LAST_GPS_DATA = rtc.getEpoch();
                loraTransfer(frame.buffer, frame.size(), GPSDATA_PORT);
                gpsSerial.flush();
                break;
            }
        }
    }
    if (rtc.getEpoch() - GPS_START_TIME > 60*2) digitalWrite(GPSEnable, LOW); // Shutdown GPS after 2 min with no fix
}


void read_battery(){
    if (rtc.getEpoch() - LAST_BATTERY_SEND < BATTERY_DATA_INTERVAL_S) return; //Si no han pasado 5 minutos, vuelve
    
    
    SerialUSB.println("Battery handler running");
    int a = analogRead(pin_battery_voltage); // Leemos la primera vez para descartarla
    float v = 0;
    //for (int i = 0; i<10; i++){
    //   a = analogRead(pin_battery_voltage);
        v += (a/1023.0*3300.0*11.0);  // there's an 1M and 100k resistor divider
    //}
    //v = v/10.0; //-255
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
        short fport;
        memset(buffer, 0, 256);
        length = lora.receivePacket(buffer, 256, &rssi, &fport);

        if(length){
            SerialUSB.print("Length is: "); SerialUSB.println(length);
            SerialUSB.print("RSSI is: "); SerialUSB.println(rssi);
            SerialUSB.print("fPort is: "); SerialUSB.println(fport);
            SerialUSB.print("Data is: ");
            for(unsigned char i = 0; i < length; i ++)
            {
                SerialUSB.print("0x"); SerialUSB.print(buffer[i], HEX); SerialUSB.print(" ");
            }
            SerialUSB.println();
            payloadDecoder(buffer, length, fport);
        }
    }

    return result;
}


void payloadDecoder(char* buffer, uint16_t size, short fport){
    if (fport == 100){
        SerialUSB.println("fport is Sample Rate Configurator");
        if(size >= 4){
            SerialUSB.print("Current GPS and Battery Interval (s): "); SerialUSB.print(GPS_DATA_INTERVAL_S); SerialUSB.print(" "); SerialUSB.println(BATTERY_DATA_INTERVAL_S);
            
            uint16_t aux_uint = 0;
            aux_uint = buffer[0] << 8 | buffer[1];
            if (aux_uint > 10) GPS_DATA_INTERVAL_S = aux_uint;
            aux_uint = buffer[2] << 8 | buffer[3];
            if (aux_uint > 10) BATTERY_DATA_INTERVAL_S = aux_uint;

            SerialUSB.print("New GPS and Battery Interval (s): "); SerialUSB.print(GPS_DATA_INTERVAL_S); SerialUSB.print(" "); SerialUSB.println(BATTERY_DATA_INTERVAL_S);
            
        }else SerialUSB.println("Payload size is too short");
    }

}