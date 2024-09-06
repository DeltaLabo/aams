#include <WiFi.h>
#include "ThingSpeak.h"
#include <ESP32Time.h> //RTC
#include "Adafruit_INA219.h"
#include <DHTStable.h>
#include <SPI.h>
#include <SdFat.h>
#include <SdFatConfig.h>

//Interval definitions in seconds
#define InternetInterval    43200
#define MeasureInterval     1
#define ThingSpeakInterval  60
#define SDCardInterval      10

//Sensor related definitions
#define DHT22_PIN  6
#define CO_pin  7
#define NO_pin 5
#define SO_pin 3
#define PIN_MISO_OPC 35
#define PIN_MOSI_OPC 34
#define PIN_SCK_OPC 36
#define PIN_CS_OPC 33

//SD card related definitions
#define PIN_MISO_SD 45
#define PIN_MOSI_SD 46
#define PIN_SCK_SD 47
#define PIN_CS_SD 26

//Terminal printing and files related definitions
#define terminalHeader  "timestamp,voltage (V),current (mA),humidity (%),temperature (°C),CO (ppb),NO2 (ppb),SO2 (ppb),PM1 (ug/m^3),PM2.5 (ug/m^3),PM10 (ug/m^3),WiFi,IoT"
#define fileHeader  "timestamp,voltage,current,humidity,temperature,CO,NO2,SO2,PM1,PM2.5,PM10,WiFi,IoT"

//WiFi related variables
const char* ssid = "LaboratorioDelta";
const char* pass = "labdelta21!";
bool connected = false;
WiFiClient  client;

// ThingSpeak related variables
unsigned long myChannelNumber = 2363549;
const char* myWriteAPIKey = "NR8GHTJF3IC27CIP";
bool updateTS = false;

//Interruption related variables
hw_timer_t *timer = NULL;
bool timer_flag = true;

//RTC related variables
ESP32Time rtc(0);

//NTP
const char* ntpServer = "pool.ntp.org";
const long  UTCOffset = -21600; // UTC-6 (Costa Rica)
const int   daylightOffset = 0;
struct tm timeinfo;

//Data sending related variables
unsigned long currEpoch = 0; 
unsigned long prevEpochInter = 0;
unsigned long prevEpochMeas = 0;
unsigned long prevEpochSD = 0; 
unsigned long prevEpochTS = 0; 

//INA219 related variables
Adafruit_INA219 ina219;

//DHT related variables
DHTStable dht;

//Sensor related variables
typedef struct sensorDataType {
    float voltage;
    float current;
    float humidity;
    float temperature;
    float CO;
    float NO;
    float SO;
    float PM1;
    float PM2;
    float PM10;
};
sensorDataType sensorData = {0};
sensorDataType sensorDataAcumTS = {0};
sensorDataType sensorDataAvgTS = {0};
sensorDataType sensorDataAcumSD = {0};
sensorDataType sensorDataAvgSD = {0};

int sizeofData = sizeof(sensorDataType)/sizeof(float);

//SD card related variables
SPIClass SD_SPI(HSPI);
SdFat SD;
SdFile myFile;
String filename = "";
bool fileUpdate = false;

//Time related variables
String timestamp = "";
int yesterday = 0;

bool testsFailed = false;

void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); 
  WiFi.begin(ssid, pass);
  ThingSpeak.begin(client);
  //Interruption related setup
  timer = timerBegin(1000000); //1e6 us
  timerAttachInterrupt(timer, &onTimer); //handler onTimer at the end of the program
  timerAlarm(timer, 1000000, true, 0); // repeat = true, unlimited cycles
  //Iterval testing
  if((SDCardInterval % MeasureInterval) != 0){
    Serial.println("SD card saving interval is wrong");
    testsFailed = true;
  }
  if((ThingSpeakInterval % MeasureInterval) != 0){
    Serial.println("ThingSpeak saving interval is wrong");
    testsFailed = true;
  }
  //INA219 related setup
  if(!ina219.begin()){
    Serial.println("Could not find INA219");
    testsFailed = true;
  }else Serial.println("INA219 initialized");
  //DHT related setup
  if(dht.read22(DHT22_PIN) != 0){
    Serial.println("Could not read from DHT22");
    testsFailed = true;
  }else Serial.println("DHT22 initialized");
  //SD card related setup
  SD_SPI.begin(PIN_SCK_SD, PIN_MISO_SD, PIN_MOSI_SD, PIN_CS_SD);
  // inicialize SD Card
  if(!SD.begin(SdSpiConfig(PIN_CS_SD, SHARED_SPI, SD_SCK_MHZ(50), &SD_SPI))){
    Serial.println("Could not find SD card");
    testsFailed = true;
  }else Serial.println("SD card initialized");
  //If the tests are not passed it stays here forever
  if(testsFailed){
    while(true);
  }
  delay(1000);
  //NTP server related setup
  while(WiFi.status() != WL_CONNECTED);
  connected = true;
  Serial.println("Internet connected");
  while(!syncTime());
  Serial.println("Time synchronization with NTC server succesful");
  rtc.setTimeStruct(timeinfo);
  Serial.println(terminalHeader);
  //Sync before start measuring
  prevEpochInter = rtc.getEpoch();
  prevEpochMeas = rtc.getEpoch();
  prevEpochSD = rtc.getEpoch();
  prevEpochTS = rtc.getEpoch();
}

void loop()
{
  if(timer_flag){
    // Sync times before measurements
    timestamp = rtc.getTime("%Y-%m-%d %H:%M:%S");
    currEpoch = rtc.getEpoch();
    // Check if there is day change
    if(rtc.getDay()!=yesterday){
      filename = rtc.getTime("%Y-%m-%d.csv");
      yesterday = rtc.getDay();
      fileUpdate = true;

    }
    // Check internet every "InternetInterval"
    if(intervalEval(InternetInterval,currEpoch,prevEpochInter,&prevEpochInter)){
      if(WiFi.status() != WL_CONNECTED){
        connected = false;
      }else{
        connected = true;
        syncTime();
        rtc.setTimeStruct(timeinfo);
      }
    }
    // Make measurements every "MeasureInterval"
    if(intervalEval(MeasureInterval,currEpoch,prevEpochMeas,&prevEpochMeas)){
      sensorData.voltage = ina219.getBusVoltage_V();
      sensorData.current = ina219.getCurrent_mA();
      sensorData.humidity = dht.getHumidity();
      sensorData.temperature = dht.getTemperature();
      sensorData.CO = gasSensor(CO_pin,285,420);
      sensorData.NO = gasSensor(NO_pin,250,800);
      sensorData.SO = gasSensor(SO_pin,350,500);
      acumulating(&sensorData,&sensorDataAcumSD,sizeofData);
      acumulating(&sensorData,&sensorDataAcumTS,sizeofData);
    }
    // Make averaging a pub in the SD card every "SDCardInterval"
    if(intervalEval(SDCardInterval,currEpoch,prevEpochSD,&prevEpochSD)){
      averaging(&sensorDataAcumSD,&sensorDataAvgSD,sizeofData,SDCardInterval,MeasureInterval);
      sensorDataAcumSD = emptyData();
      if (!myFile.open(filename.c_str(), O_RDWR | O_CREAT | O_AT_END)) {
        Serial.print("SD Card: error on opening file ");
        Serial.println(filename);
      }
      if(fileUpdate){
        myFile.println(fileHeader);
        fileUpdate = false;
      }
      printing(&myFile,&sensorDataAvgSD,sizeofData,timestamp,connected, updateTS);
      printing(&Serial,&sensorDataAvgSD,sizeofData,timestamp,connected, updateTS);
    }
    // Averaging for ThingSpeak
    if(intervalEval(ThingSpeakInterval,currEpoch,prevEpochTS,&prevEpochTS)){
      averaging(&sensorDataAcumTS,&sensorDataAvgTS,sizeofData,ThingSpeakInterval,MeasureInterval);
      sensorDataAcumTS = emptyData();
      printing(&Serial,&sensorDataAvgTS,sizeofData,timestamp,connected,updateTS);
      // Upload data to ThingSpeak
      ThingSpeak.setField(1, sensorDataAvgTS.CO);
      ThingSpeak.setField(2, sensorDataAvgTS.NO);
      ThingSpeak.setField(3, sensorDataAvgTS.SO);
      ThingSpeak.setField(4, sensorDataAvgTS.voltage);
      ThingSpeak.setField(5, sensorDataAvgTS.current);
      ThingSpeak.setField(6, sensorDataAvgTS.humidity);
      ThingSpeak.setField(7, sensorDataAvgTS.PM2);
      ThingSpeak.setField(8, sensorDataAvgTS.PM10);
      
      if (ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey) == 200){
        updateTS = true;
      } else{
        updateTS = false;
      }
    }
    timer_flag = 0;
  } 
}

bool syncTime() {
  configTime(UTCOffset, daylightOffset, ntpServer);
  if (!getLocalTime(&timeinfo)){
    return false;
  }
  return true;
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

void acumulating(sensorDataType* measpointer, sensorDataType* acumpointer, int size)
{
  float *floatMeasPtr = (float*)measpointer;
  float *floatAcumPtr = (float*)acumpointer;
  for (int i =0; i <= size; i++){
    floatAcumPtr[i] += floatMeasPtr[i];
  }
}

void averaging(sensorDataType* acumpointer, sensorDataType* avgpointer, int size, int avgInterval, int measInterval)
{
  float *floatAcumPtr = (float*)acumpointer;
  float *floatAvgPtr = (float*)avgpointer;
  for (int i =0; i <= size; i++){
    floatAvgPtr[i] = floatAcumPtr[i] / ( avgInterval / measInterval );
  }
}

void printing(Print* printtype, sensorDataType* datapointer, int size, String tstamp, bool connStatus, bool serverStatus){
  float *floatPtr = (float*)datapointer;
  printtype->print(tstamp);
  printtype->print(',');
  for (int i = 0; i <= (size - 1); i++){
    printtype->print(*(floatPtr + i));
    printtype->print(',');
  }
  printtype->print(connStatus);
  printtype->print(',');
  printtype->print(serverStatus);
  printtype->print('\n');
}

sensorDataType emptyData()
{
  sensorDataType empty = {0};
  return empty;
}

float gasSensor(int pin, int offset, int sens) {
  float value = (analogRead(pin)*5000.0)/4096.0;
  value -= offset; //substract offset
  value /= sens; //apply sensitivity
  value *= 1000; //convert to ppb
  return value;
}
