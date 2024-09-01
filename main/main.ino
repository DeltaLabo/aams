#include <WiFi.h>
#include <ESP32Time.h> //RTC
#include "Adafruit_INA219.h"
#include <DHT.h>
#include <SPI.h>

//Interval definitions in seconds
#define MeasureInterval     1
#define ThinkSpeakInterval  60
#define SDCardInterval      10

//Sensor related definitions
#define DHT22_PIN  6
#define DHTTYPE DHT22

//WiFi related variables
const char* ssid = "RM_interior";   // your network SSID (name)
const char* pass = "RM20JUNE";        // your network password

//Interruption related variables
hw_timer_t *timer = NULL;
bool timer_flag = true;

//RTC related variables
ESP32Time rtc(0);

//NTP
const char* ntpServer = "pool.ntp.org";
const long  UTCOffset = -21600;
const int   daylightOffset = 0;
struct tm timeinfo;

//Data sending related variables
unsigned long currEpoch = 0; 
unsigned long prevEpochMeas = 0;
unsigned long prevEpochSD = 0; 

//INA219 related variables
Adafruit_INA219 ina219;

//DHT related variables
DHT dht(DHT22_PIN, DHTTYPE, 22);

//Sensor related variables
typedef struct sensorDataType {
    float voltage;
    float current;
    float humidity;
    float temperature;
};
sensorDataType sensorData = {0};
sensorDataType sensorDataAcumTS = {0};
sensorDataType sensorDataAvgTS = {0};
sensorDataType sensorDataAcumSD = {0};
sensorDataType sensorDataAvgSD = {0};

bool tests = false;

void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); 
  WiFi.begin(ssid, pass);
    // Set timer frequency to 1Mhz
  
  //Interruption related setup
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer); //handler onTimer at the end of the program
  timerAlarm(timer, 1000000, true, 0); // repeat = true, unlimited cycles
  //NTP server related setup
  syncTime();
  rtc.setTimeStruct(timeinfo);
  prevEpochMeas = rtc.getEpoch();
  prevEpochSD = rtc.getEpoch();
  //Iterval testing
  if((SDCardInterval % MeasureInterval) != 0){
    Serial.println("SD card saving interval is wrong");
    tests = true;
  }
  if((ThinkSpeakInterval % MeasureInterval) != 0){
    Serial.println("ThinkSpeak saving interval is wrong");
    tests = true;
  }
  //INA219 related setup
  if (!ina219.begin()){
    Serial.println("Could not find INA219");
  }
  if(tests){
    while(true);
  }
}



void loop()
{
  if(timer_flag){
    if(WiFi.status() != WL_CONNECTED){
      Serial.print("Could not connect to: ");
      Serial.println(ssid);
    }else
    {
      // Serial.print("Connected to: ");
      // Serial.println(ssid);
    }
    // Serial.println(&timeinfo, "%Y-%m-d %H:%M:%S");
    // Serial.println(rtc.getTime("%Y-%m-%d %H:%M:%S,"));
    currEpoch = rtc.getEpoch();
    // Make measurements every "MeasureInterval"
    if(intervalEval(MeasureInterval,currEpoch,prevEpochMeas,&prevEpochMeas)){
      sensorData.voltage = ina219.getBusVoltage_V();
      sensorData.current = ina219.getCurrent_mA();
      sensorData.humidity = dht.readHumidity();
      sensorData.temperature = dht.readTemperature();
      sensorDataAcumSD = acumulating(sensorData,sensorDataAcumSD);
      sensorDataAcumTS = acumulating(sensorData,sensorDataAcumSD);
    }
    // Make averaging an pub in the SD card every "SDCardInterval"
    if(intervalEval(SDCardInterval,currEpoch,prevEpochSD,&prevEpochSD)){
      sensorDataAvgSD = averaging(sensorDataAcumSD,SDCardInterval,MeasureInterval);
      sensorDataAcumSD = emptyData();
      Serial.println(sensorData.voltage);
      Serial.println(sensorDataAvgSD.voltage);
    }
    timer_flag = 0;
  }

}

void syncTime() {
  configTime(UTCOffset, daylightOffset, ntpServer);
  if (!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println("RTC set from NTP time");
}

//Time interruption handler
void ARDUINO_ISR_ATTR onTimer() {
  timer_flag = true;
}

unsigned long intervalEval(unsigned long interval,unsigned long currE,unsigned long prevE,unsigned long *prevEpointer)
{
  if ((currE - prevE) >= interval)
  {
    *prevEpointer = currE;
    return true;
  }else
  {
    return false;
  }
}

sensorDataType acumulating(sensorDataType measurements, sensorDataType acumulate)
{
  acumulate.voltage += measurements.voltage;
  acumulate.current += measurements.current;
  acumulate.humidity += measurements.humidity;
  acumulate.temperature += measurements.temperature;
  return acumulate;
}

sensorDataType averaging(sensorDataType acumulator, int avgInterval, int measInterval)
{
  sensorDataType average;
  average.voltage = acumulator.voltage / ( avgInterval / measInterval );
  average.current = acumulator.current / ( avgInterval / measInterval );
  average.humidity = acumulator.humidity / ( avgInterval / measInterval );
  average.temperature = acumulator.temperature / ( avgInterval / measInterval );
  return average;
}



sensorDataType emptyData()
{
  sensorDataType empty = {0};
  return empty;
}