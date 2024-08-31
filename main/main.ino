// Code AAMS Project - DELTA Lab
#include <WiFi.h>
#include "ThingSpeak.h"
#include <DHT.h>
#include <SPI.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "time.h"

//Libraries for OPC
#define ArduinoUNO
#define opSerial Serial
#define BaudRate 115200
#define SPI_OPC_busy 0x31
#define SPI_OPC_ready 0xF3

//Libraries for SD Card
#include <BufferedPrint.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <RingBuf.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include <sdios.h>

//Libraries for RTC Module
#include <RTClib.h>
#include <Wire.h>

//Libraries for INA219
#include "Adafruit_Sensor.h"
#include "Adafruit_INA219.h"

//const char* ssid = "LaboratorioDelta";   // your network SSID (name)
//const char* pass = "labdelta21!";        // your network password
const char* ssid = "SensorAire";   // your network SSID (name)
const char* pass = "Biblio8385";        // your network password
int keyIndex = 0;                 // your network key Index number (needed only for WEP)
WiFiClient  client;

// NTP server
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -21600;  // GMT -6
const int   daylightOffset_sec = 0;

unsigned long myChannelNumber = 2363549;
const char* myWriteAPIKey = "NR8GHTJF3IC27CIP";

// define pin for tempertaure and humidity sensor
#define DHT22_PIN  6
#define DHTTYPE DHT22
DHT dht(DHT22_PIN, DHTTYPE, 22);
float humi = 0;
float tempC = 0;

// define custom SPI pins
#define PIN_MISO 45 //DO
#define PIN_MOSI 46 //DI
#define PIN_SCK 47 //CLK
#define PIN_CS 26  // chip select pin for SD card

// global variables
Adafruit_INA219 ina219;
RTC_DS3231 rtc;
DateTime lastSyncTime;
SdFat SD;
SdFile myFile;
char filename[13]; // Buffer for filename with room for null terminator

// initialize custom SPI class for SD card
SPIClass sdSPI(HSPI);

// pins for sensors
int CO_pin = 7;
int NO_pin = 5;
int SO_pin = 3;

// variables to store the value coming from the sensor
float CO_value = 0;
float NO_value = 0;
float SO_value = 0;
float PM1 = 0;
float PM2 = 0;
float PM10 = 0;
//Counter to update values on Things Speak
int segundos = 0;

// variables for INA219
float busvoltage = 0;
float current_mA = 0;

unsigned long currentTime;
unsigned long cloopTime;
unsigned char SPI_in[68], SPI_in_index, ssPin_OPC;

struct SEND_DATA_STRUCTURE{
  unsigned int hist[16];
  float temp;
  float humid;
  float PM[3];
};
SEND_DATA_STRUCTURE mydata;

void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);   
  ThingSpeak.begin(client);
  WiFi.begin(ssid, pass);
  delay(500);
  dht.begin(); // initialize the DHT22 sensor
  Wire.begin(41,42); // initialize I2C for RTC (GPIO41 for SDA and GPIO42 for SCL)

  //Set all the pins available for use as SS pins to outputs and set HIGH
  for (unsigned char i=2;i<11;i++)
  {
    digitalWrite(i, HIGH); //Initiate pin HIGH
    pinMode(i, OUTPUT); //Set pin as output
  }

  delay(1000); //delay in case of noise on power connection. Also allows OPC to boot up.

  //Start serial port
  opSerial.begin(BaudRate);

  // start the SPI library:
  SPI.begin(); //Enable SPI for OPC comms

  //Device #1 (ssPin_OPC = 10)
  ssPin_OPC = 33;
  InitDevice();
  //END Device #1
  delay(1000);
  PrintDataLabels(opSerial); //Print labels to serial port - optional BL

  // setup RTC module
  if (!rtc.begin()) {
    Serial.println(F("Couldn't find RTC"));
    while (1);
  }

  // initialize INA219
  if (!ina219.begin()) {
    Serial.println("Could not find a valid INA219 sensor, check wiring");
  }

  // Synchronize time
  syncTime();

  if(WiFi.status() != WL_CONNECTED){
    rtc.adjust(DateTime(__DATE__, __TIME__));
    Serial.println("RTC Compilation Time");
  }

  // initialize SPI for SD card on custom pins
  sdSPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS); // SCK, MISO, MOSI, SS

  // inicialize SD Card
  if (!SD.begin(SdSpiConfig(PIN_CS, SHARED_SPI, SD_SCK_MHZ(50), &sdSPI))) {
    Serial.println(F("SD CARD FAILED, OR NOT PRESENT!"));
    //while (1);  // Do nothing more if SD card fails
  } else{
    Serial.println(F("SD CARD INITIALIZED."));
    Serial.println(F("--------------------"));    
  }
}

void syncTime() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
  rtc.adjust(DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec));
  lastSyncTime = rtc.now();
  Serial.println("RTC set from NTP time");
}

void updateFilename(char* filename, const DateTime& dt) {
  sprintf(filename, "/%04d%02d%02d.txt", dt.year(), dt.month(), dt.day());
}

void InitDevice (void){

  ReadOPCstring(0x10); //Get serialstr from OPC device
  ReadOPCstring(0x3F); //Get infostr from OPC device

  StartOPC(); //Switch on power to fan and laser
  
  //ReadOPCconfig(opSerial); //Get Config data (bin boundaries etc.) from OPC device
}

void CollectData(){ 
  Serial.println("\nNUEVA LECTURA.");

  // read data from DHT22
  humi  = dht.readHumidity();    // read humidity
  tempC = dht.readTemperature(); // read temperature
  // check if the reading is successful or not
  if (isnan(tempC) || isnan(humi)) {
    Serial.println("ERROR");
  } 
  else {
    // display the temperature
    Serial.print("Temp: ");
    Serial.print(tempC); 
    Serial.println("°C");

    // display the humidity
    Serial.print("Humidity: ");
    Serial.print(humi);      
    Serial.println("%");
  }
  
  // diaplay sensor values
  CO_value = CO_value + gasSensor(CO_pin,285,420);
  NO_value = NO_value + gasSensor(NO_pin,250,800);
  SO_value = SO_value + gasSensor(SO_pin,350,500);
  delay(1000);
  Serial.print("CO concentration: ");
  Serial.print(CO_value/(segundos+1));
  Serial.println(" ppb");
  Serial.print("NO2 concentration: ");
  Serial.print(NO_value/(segundos+1));
  Serial.println(" ppb");
  Serial.print("SO2 concentration: ");
  Serial.print(SO_value/(segundos+1));
  Serial.println(" ppb");

  // bus Voltage data
  busvoltage = ina219.getBusVoltage_V();
  Serial.print("Bus Voltage: ");
  Serial.print(busvoltage);
  Serial.println(" V");

  // current data
  current_mA = ina219.getCurrent_mA();
  Serial.print("Current: ");
  Serial.print(current_mA);
  Serial.println(" mA");

  ssPin_OPC = 33;
  unsigned long GetHistTime = millis(); //Set initial GetHistTime
  ReadOPChist(); //Read OPC histogram data
  opSerial.print(millis());
  PrintData(opSerial); //Print data to serial

  //display PMs
  PM1 = mydata.PM[0];
  Serial.print("PM1: ");
  Serial.print(PM1);    
  Serial.println(" ug/m^3");
  
  PM2 = mydata.PM[1];
  Serial.print("PM2.5: ");
  Serial.print(PM2);
  Serial.println(" ug/m^3");
  
  PM10 = mydata.PM[2];
  Serial.print("PM10: ");
  Serial.print(PM10);
  Serial.println(" ug/m^3");
  
  delay(10000);

  if (segundos <= 30){
  segundos = segundos + 1;
  Serial.println(segundos);
  }
  else{
  WriteDataIoT();
  segundos = 0;
  CO_value = 0;
  NO_value = 0;
  SO_value = 0;
  busvoltage = 0;
  current_mA = 0;
  tempC = 0;
  humi = 0;
  PM1 = 0;
  PM2 = 0;
  PM10 = 0;
  }
  return;
}

// Main Loop
// Main Loop
void loop()
{
  DateTime now = rtc.now();

  // Check if a day has passed to resync
  if (now.unixtime() - lastSyncTime.unixtime() > 86400) { // 86400 seconds in a day
    syncTime();
  }

  // Try to connect or reconnect to WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Could not connect to: ");
    Serial.println(ssid);
    while (WiFi.status() != WL_CONNECTED) {
      WiFi.begin(ssid, pass);
      delay(5000);

      // Collect data and save to SD regardless of WiFi status
      CollectData();
      if (segundos <= 30) {
        segundos = segundos + 1;
        Serial.print("Segundos SD: ");
        Serial.println(segundos);
      } else {
        DataLogSD();
        segundos = 0;
        CO_value = 0;
        NO_value = 0;
        SO_value = 0;
        busvoltage = 0;
        current_mA = 0;
        tempC = 0;
        humi = 0;
        PM1 = 0;
        PM2 = 0;
        PM10 = 0;
      }
    }
    Serial.print("\nConnected to: ");
    Serial.println(ssid);
    syncTime();
  }

  // Collect data and save to SD in every loop iteration
  CollectData();
  DataLogSD();
}


float gasSensor(int pin, int offset, int sens) {
  // float value = (analogRead(pin)*2560.0)/1023.0;
  float valor = (analogRead(pin)*5000.0)/4095.0;
  //Serial.println(analogRead(pin));
  //Serial.println("Voltaje del pin: " + String(pin) + " Es de: " + String(voltaje));
  valor -= offset; //substract offset
  valor /= sens; //apply sensitivity
  valor *= 1000; //convert to ppb
  return valor;
}

void WriteDataIoT() {
  // set the fields with the values
  ThingSpeak.setField(1, (CO_value/(segundos+1)));
  ThingSpeak.setField(2, (NO_value/(segundos+1)));
  ThingSpeak.setField(3, (SO_value/(segundos+1)));
  ThingSpeak.setField(4, (busvoltage));
  ThingSpeak.setField(5, (current_mA));
  //ThingSpeak.setField(4, (tempC));
  ThingSpeak.setField(6, (humi));
  //ThingSpeak.setField(6, (PM1));
  ThingSpeak.setField(7, (PM2));
  ThingSpeak.setField(8, (PM10));
  
  // write to the ThingSpeak channel
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if(x == 200){
    Serial.println("Channel update successful.");
  }
  else{
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }
  return;
}


//Get string (serialstr or infostr) from OPC device
void ReadOPCstring (unsigned char SPIcommand)
{
  GetReadyResponse(SPIcommand);
  for (SPI_in_index=0; SPI_in_index<60; SPI_in_index++)
  {
    delayMicroseconds(10);
    SPI_in[SPI_in_index] = SPI.transfer(0x01); //Value of outgoing byte doesn't matter
  }

  SetSSpin(HIGH);
  SPI.endTransaction();

  PrintOPCstring(opSerial);
}

void PrintOPCstring (Stream &port)
{
  port.write(SPI_in, 60); //print 60 characters from SPI_in[] array
  port.println("");
  port.flush();
}

void ReadOPChist (void)
{
  GetReadyResponse(0x30);
  for (SPI_in_index=0; SPI_in_index<64; SPI_in_index++)
  {
    delayMicroseconds(10);
    SPI_in[SPI_in_index] = SPI.transfer(0x01); //Value of outgoing byte doesn't matter
  }
  SetSSpin(HIGH);
  SPI.endTransaction();
  delay(10);
}


void DiscardSPIbytes (byte NumToDiscard)
{
  for (SPI_in_index=0; SPI_in_index<NumToDiscard; SPI_in_index++)
  {
    delayMicroseconds(10);
    SPI.transfer(0x01); //Value of outgoing byte doesn't matter
  }
}


void StartOPC (void)
{
  //Turn ON fan and peripheral power
  GetReadyResponse(0x03);
  SPI.transfer(0x03); //Turn ON fan and peripheral power
  SetSSpin(HIGH);
  SPI.endTransaction();
  delay(10);

  //Wait for fan to reach full speed (and for multiple attempts by OPC firmware to turn on fan)
  for (byte i=0; i<5; i++)
  {
    delay(1000);
  }
}


void GetReadyResponse (unsigned char SPIcommand)
{
  unsigned char Response;

  SPI.beginTransaction(SPISettings(300000, MSBFIRST, SPI_MODE1));

  //Try reading a byte here to clear out anything remnant of SD card SPI activity (WORKS!)
  Response = SPI.transfer(SPIcommand);
  delay(1);  //wait 1ms

  do
  {
    SetSSpin(LOW);
    unsigned char Tries = 0;
    do
    {
      Response = SPI.transfer(SPIcommand);
      if (Response != SPI_OPC_ready) delay(1); //wait 1ms
    }
    while ((Tries++ < 20) && (Response != SPI_OPC_ready));

    if (Response != SPI_OPC_ready)
    {
      if (Response == SPI_OPC_busy)
      {
        SetSSpin(HIGH);
        Serial.println(F("ERROR Waiting 2s (for OPC comms timeout)")); //signal user
        Serial.flush();
        delay(2000); //wait 2s
      }
      else
      {
        //End SPI and wait a few seconds for it to be cleared
        SetSSpin(HIGH);
        Serial.println(F("ERROR Resetting SPI. Check custom SPI pins!")); //signal user
        Serial.flush();
        SPI.endTransaction();
        //Wait 6s here for buffer to be cleared
        delay(6000);
        SPI.beginTransaction(SPISettings(300000, MSBFIRST, SPI_MODE1));
      }
    }
  }
  while ((Response != SPI_OPC_ready) && (Serial.available()==0)); //don't hang on this if data is coming in on serial interface
  delay(10);

}


unsigned int MODBUS_CalcCRC(unsigned char data[], unsigned char nbrOfBytes)
{
  #define POLYNOMIAL_MODBUS 0xA001 //Generator polynomial for MODBUS crc
  #define InitCRCval_MODBUS 0xFFFF //Initial CRC value

  unsigned char _bit; // bit mask
  unsigned int crc = InitCRCval_MODBUS; // initialise calculated checksum
  unsigned char byteCtr; // byte counter

  // calculates 16-Bit checksum with given polynomial
  for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++)
  {
    crc ^= (unsigned int)data[byteCtr];
    for(_bit = 0; _bit < 8; _bit++)
    {
      if (crc & 1) //if bit0 of crc is 1
      {
        crc >>= 1;
        crc ^= POLYNOMIAL_MODBUS;
      }
      else
        crc >>= 1;
    }
  }
  return crc;
}


//Convert SHT31 ST output to Temperature (C)
float ConvSTtoTemperature (unsigned int ST)
{
  return -45 + 175*(float)ST/65535;
}


//Convert SHT31 SRH output to Relative Humidity (%)
float ConvSRHtoRelativeHumidity (unsigned int SRH)
{
  return 100*(float)SRH/65535;
}


//Process OPC data and print
void PrintData (Stream &port)
{
  unsigned char i;
  unsigned int *pUInt16;
  float *pFloat;
  float Afloat;
  int k;

  //Histogram bins (UInt16) x16
  k=0;
  for (i=0; i<32; i+=2)
  {
    AddDelimiter(port);
    pUInt16 = (unsigned int *)&SPI_in[i];
    port.print(*pUInt16, DEC);
    mydata.hist[k] = *pUInt16;
    k++;
  }

  //MToF bytes (UInt8) x4
  for (i=32; i<36; i++)
  {
    AddDelimiter(port);
    Afloat = (float)SPI_in[i];
    Afloat /= 3; //convert to us
    port.print(Afloat, 2);
  }

  //SFR (4-byte float) x1
  AddDelimiter(port);
  pFloat = (float *)&SPI_in[36];
  port.print(*pFloat, 3); //print to 3dp

  //Temperature (2-byte integer) x1
  AddDelimiter(port);
  pUInt16 = (unsigned int *)&SPI_in[40];
  port.print(ConvSTtoTemperature(*pUInt16), 1); //print to 1dp
  mydata.temp = ConvSTtoTemperature(*pUInt16);

  //Relative humidity (2-byte integer) x1
  AddDelimiter(port);
  pUInt16 = (unsigned int *)&SPI_in[42];
  port.print(ConvSRHtoRelativeHumidity(*pUInt16), 1); //print to 1dp
  mydata.humid = ConvSRHtoRelativeHumidity(*pUInt16);

  //Sampling period(s) (4-byte float) x1
  AddDelimiter(port);
  pFloat = (float *)&SPI_in[44];
  port.print(*pFloat, 3); //print to 3dp

  //Reject count Glitch (1-byte integer) x1
  AddDelimiter(port);
  port.print(SPI_in[48], DEC);

  //Reject count LongTOF (1-byte integer) x1
  AddDelimiter(port);
  port.print(SPI_in[49], DEC);

  //PM values(ug/m^3) (4-byte float) x3
  k = 0;
  for (i=50; i<62; i+=4)
  {
    AddDelimiter(port);
    pFloat = (float *)&SPI_in[i];
    port.print(*pFloat, 3); //print to 3dp
    mydata.PM[k] = *pFloat;
    k++;
  }

  //Checksum (UInt16) x1
  AddDelimiter(port);
  pUInt16 = (unsigned int *)&SPI_in[62];
  port.println(*pUInt16, DEC);

  //Compare recalculated Checksum with one sent
  if (*pUInt16 != MODBUS_CalcCRC(SPI_in, 62)) //if checksums aren't equal
    port.println(F("Checksum error in line above!"));

  port.flush();
}


//Print data labels
void PrintDataLabels (Stream &port)
{
  unsigned char i;

  port.print(F("Time(ms)"));

  for (i=0; i<16; i++)
  {
    port.print(F(",Bin"));
    if (i < 10) port.print(F("0")); //leading 0 for single digit bin numbers
    port.print(i, DEC);
  }

  for (i=1; i<9; i+=2)
  {
    port.print(F(",MToFBin"));
    port.print(i, DEC);
    if (i == 1) port.print(F("(us)")); //print units for first value of this type
  }

  port.println(F(",SFR(ml/s),T(C),RH(%),SampPrd(s),#RejectGlitch,#RejectLong,PM_A(ug/m^3),PM_B,PM_C,Checksum"));

  port.flush();
}

void AddDelimiter (Stream &port)
{
  port.print(F(",")); //delimiter
}


void SetSSpin (bool pinState) //pinState is HIGH or LOW
{
  digitalWrite(ssPin_OPC, pinState); //Set output to pinState
}

void DataLogSD(){
  // set filename and state date
  DateTime now = rtc.now();
  updateFilename(filename, now);  // Update filename based on the current date;

  // open file for writing
  if (!myFile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
    Serial.print(F("SD Card: error on opening file "));
    Serial.println(filename);
  }
  else{
    Serial.println(F("Writing log to SD Card"));

    // write timestamp
    myFile.print(now.year(), DEC);
    myFile.print('-');
    myFile.print(now.month(), DEC);+
    myFile.print('-');
    myFile.print(now.day(), DEC);
    myFile.print(' ');
    myFile.print(now.hour(), DEC);
    myFile.print(':');
    myFile.print(now.minute(), DEC);
    myFile.print(':');
    myFile.print(now.second(), DEC);
    myFile.print("  "); // Delimiter between timestamp and data

    // write data
    myFile.println();

    //Titulo
    myFile.println("CO (ppb),  NO2 (ppb),  SO2 (ppb),  Voltage (V), Current (mA), Temperature (°C), Humidity (%), PM1 (ug/m^3), PM2.5 (ug/m^3), PM10 (ug/m^3)");

    // CO sensor data
    myFile.print(CO_value/(segundos+1));
    myFile.print(", ");

    // NO2 sensor data
    //myFile.print("NO2 concentration: ");
    myFile.print(NO_value/(segundos+1));
    myFile.print(", ");

    // SO2 sensor data
    //myFile.print("SO2 concentration: ");
    myFile.print(SO_value/(segundos+1));
    myFile.print(", ");

    // bus voltage data
    myFile.print(busvoltage);
    myFile.print(", ");

    // current data
    myFile.print(current_mA);
    myFile.print(", ");

    // temperature sensor data
    //myFile.print("Temperature: ");
    myFile.print(tempC);
    myFile.print(", ");

    // humidity sensor data
    //myFile.print("Humidity: ");
    myFile.print(humi);
    myFile.print(", ");

    // OPC PM data
    //myFile.print("PM1: ");
    myFile.print(PM1);      
    myFile.print(", ");

    //myFile.print("PM2.5: ");
    myFile.print(PM2);      
    myFile.print(", ");

    //myFile.print("PM10: ");
    myFile.println(PM10);
    //myFile.println(" ug/m^3");

    myFile.println();  // New line

    myFile.close();  // Close the file
    Serial.println("Data appended to file.");
  }
}
