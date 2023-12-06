// Codigo Proyecto AAMS - Laboratorio Delta
#include <WiFi.h>
#include "ThingSpeak.h"
#include <DHT.h>
#define DHT22_PIN  6 // ESP32 pin GPIO23 connected to DHT22 sensor

DHT dht22(DHT22_PIN, DHT22);

//const char* ssid = "LaboratorioDelta";   // your network SSID (name) 
//const char* pass = "labdelta21!";        // your network password
const char* ssid = "JMR";   // your network SSID (name) 
const char* pass = "JDMR0106";        // your network password
int keyIndex = 0;                 // your network key Index number (needed only for WEP)
WiFiClient  client;

unsigned long myChannelNumber = 2363549;
const char * myWriteAPIKey = "NR8GHTJF3IC27CIP";

//Codigo para leer sensores
//Para ESP32

int CO_pin = 7;
int NO_pin = 5;
int SO_pin = 3;

//Variables to store the value coming from the sensor
float CO_value = 0;
float NO_value = 0;
float SO_value = 0;
//Counter to update values on Things Speak
int segundos = 0;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);   
  ThingSpeak.begin(client);
  dht22.begin(); // initialize the DHT22 sensor
}

float gasSensor(int pin, int offset, int sens) {
  // float value = (analogRead(pin)*2560.0)/1023.0;
  float valor = (analogRead(pin)*3300.0)/4095.0;
  Serial.println(analogRead(pin));
  Serial.println("Voltaje del pin: " + String(pin) + " Es de: " + String(valor));
  valor -= offset; //substract offset
  valor /= sens; //apply sensitivity
  valor *= 1000; //convert to ppb
  return valor;
}

void loop() {
  // Connect or reconnect to WiFi
  if(WiFi.status() != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    while(WiFi.status() != WL_CONNECTED){
      WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print(".");
      delay(5000);     
    } 
    Serial.println("\nConnected.");
  } 
  else{
    //Calculate the value of each variable and prints them
    float humi  = dht22.readHumidity();    // read humidity
    float tempC = dht22.readTemperature(); // read temperature
    // check if the reading is successful or not
    if (isnan(tempC) || isnan(humi)) {
      Serial.println("ERROR");
    } 
    else {
      // display the temperature
      Serial.print(tempC);     
      Serial.print("°C");
      Serial.print("Temp: ");

      // display the humidity
      Serial.print("Humidity: ");
      Serial.print(humi);      
      Serial.println("%");
    }
    Serial.println("\nNUEVA LECTURA.");
    CO_value = CO_value + gasSensor(CO_pin,280,445);
    NO_value = NO_value + gasSensor(NO_pin,203,740);
    SO_value = SO_value + gasSensor(SO_pin,350,307);
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
    if (segundos <= 30){
      segundos = segundos + 1;
    } 
    else{
      WriteDataIoT();
      segundos = 0;
      CO_value = 0;
      NO_value = 0;
      SO_value = 0;
    }
  }
}

void WriteDataIoT() {
  // set the fields with the values
  ThingSpeak.setField(1, (CO_value/(segundos+1)));
  ThingSpeak.setField(2, (NO_value/(segundos+1)));
  ThingSpeak.setField(3, (SO_value/(segundos+1)));
  
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
