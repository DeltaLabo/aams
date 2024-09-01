#include <WiFi.h>
#include <ESP32Time.h> //RTC
#include "Adafruit_INA219.h"


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
float voltage = 0;
float current = 0;

//Sensor related variables
struct sensorDataType {
    float voltage;
    float current;
};
sensorDataType sensorData = {0,0};
sensorDataType sensorDataAcumTS = {0,0};
sensorDataType sensorDataAvgTS = {0,0};
sensorDataType sensorDataAcumSD = {0,0};
sensorDataType sensorDataAvgSD = {0,0};

//Interval definitions in seconds
#define MeasureInterval     2
#define ThinkSpeakInterval  60
#define SDCardInterval      60


int counter = 0;


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
  //INA219 related setup
  if (!ina219.begin()){
    Serial.println("Could not find INA219");
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
    if(intervalEval(MeasureInterval,currEpoch,prevEpochMeas,&prevEpochMeas))
    {
      sensorData.voltage = ina219.getBusVoltage_V();
      sensorDataAcumSD.voltage += sensorData.voltage;
      sensorData.current = ina219.getCurrent_mA();
      sensorDataAcumSD.current += sensorData.current;
      Serial.println(counter++);
    }
    if(intervalEval(SDCardInterval,currEpoch,prevEpochSD,&prevEpochSD))
    {
      sensorDataAvgSD.voltage = sensorDataAcumSD.voltage / ((SDCardInterval/MeasureInterval) - 1);
      sensorDataAcumSD.voltage = 0;
      sensorDataAvgSD.current = sensorDataAcumSD.current / ((SDCardInterval/MeasureInterval) - 1);
      sensorDataAcumSD.voltage = 0;
      Serial.println(sensorDataAvgSD.voltage);
      Serial.println(sensorData.voltage);
      counter = 0;
    }
    timer_flag = 0;
  }

}

void syncTime() {
  configTime(UTCOffset, daylightOffset, ntpServer);
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println("RTC set from NTP time");
}

//Time interruption handler
void ARDUINO_ISR_ATTR onTimer() {
  timer_flag = true;
}