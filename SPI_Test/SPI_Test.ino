// SPI Bus Test for AAMS
#include <SPI.h>
#include <SD.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include "OPC.h"

// SPI related designs
#define PIN_CS_OPC D7
#define PIN_CS_SD D6

//SD card related variables
SPIClass SD_SPI(HSPI);
SdFat SD;
SdFile myFile;
String filename = "";
bool fileUpdate = false;

//OPC realted variables
#define SPI_OPC_busy 0x31
#define SPI_OPC_ready 0xF3
unsigned char SPI_in[68], SPI_in_index;

void setup() {
  Serial.begin(115200);
  //Initialize SPI
  SPI.begin();

  //Initialize SD card
  pinMode(PIN_CS_SD, OUTPUT);
  digitalWrite(PIN_CS_SD, HIGH); // Deseleccionar mientras se inicializa el otro dispositivo
  if (!SD.begin(PIN_CS_SD)) {
    Serial.println("Error al inicializar la tarjeta SD");
    return;
  }else Serial.println("Tarjeta SD inicializada correctamente");

  //Initialize and turn on OPC
  pinMode(PIN_CS_OPC, OUTPUT);
  digitalWrite(PIN_CS_OPC, HIGH);
  StartOPC(PIN_CS_OPC);
  Serial.println("OPC initialized");
}

void loop() {
  // put your main code here, to run repeatedly:

}
