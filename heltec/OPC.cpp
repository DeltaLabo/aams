#include "Arduino.h"
#include "OPC.h"
#include <SPI.h>

#define SPI_OPC_busy 0x31
#define SPI_OPC_ready 0xF3

void SetSSpin (unsigned char ssPin_OPC, bool pinState) {
   digitalWrite(ssPin_OPC, pinState);
}

void StartOPC (unsigned char ssPin_OPC)
{
  //Turn ON fan and peripheral power
  SetSSpin(ssPin_OPC, HIGH);  
  pinMode(ssPin_OPC, OUTPUT);
  GetReadyResponse(ssPin_OPC, 0x03);
  SPI.transfer(0x03); //Turn ON fan and peripheral power
  SetSSpin(ssPin_OPC, HIGH);
  SPI.endTransaction();
  delay(10);

  //Wait for fan to reach full speed (and for multiple attempts by OPC firmware to turn on fan)
  for (byte i=0; i<5; i++)
  {
   delay(1000);
  }
  delay(1000); //delay in case of noise on power connection. Also allows OPC to boot up.
  
}


void GetReadyResponse (unsigned char ssPin_OPC, unsigned char SPIcommand)
{
  unsigned char Response;

  SPI.beginTransaction(SPISettings(300000, MSBFIRST, SPI_MODE1));

  //Try reading a byte here to clear out anything remnant of SD card SPI activity (WORKS!)
  Response = SPI.transfer(SPIcommand);
  delay(1);  //wait 1ms

  do
  {
    SetSSpin(ssPin_OPC,LOW);
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
        SetSSpin(ssPin_OPC, HIGH);
        Serial.println(F("ERROR Waiting 2s (for OPC comms timeout)")); //signal user
        Serial.flush();
        delay(2000); //wait 2s
      }
      else
      {
        //End SPI and wait a few seconds for it to be cleared
        SetSSpin(ssPin_OPC, HIGH);
        Serial.println(F("ERROR Resetting SPI")); //signal user
        Serial.flush();
        SPI.endTransaction();
        //Wait 6s here for buffer to be cleared
       
        delay(6000);
        SPI.beginTransaction(SPISettings(300000, MSBFIRST, SPI_MODE2));
      }
    }
  }
  while ((Response != SPI_OPC_ready) && (Serial.available()==0)); //don't hang on this if data is coming in on serial interface
  delay(10);

}


void ReadOPChist(unsigned char ssPin_OPC) {
    unsigned char SPI_in_index;
    GetReadyResponse(ssPin_OPC, 0x30);  // Enviar comando inicial
    for (SPI_in_index = 0; SPI_in_index < 64; SPI_in_index++) {
        delayMicroseconds(10);
        SPI_in[SPI_in_index] = SPI.transfer(0x01);  // Transferencia SPI
    }
    SetSSpin(ssPin_OPC, HIGH);          // Finalizar la comunicación SPI
    SPI.endTransaction();    // Terminar la transacción SPI
    delay(10);               // Retardo de 10 ms
}

void StopOPC (unsigned char ssPin_OPC) {
    //Turn OFF fan and laser
    GetReadyResponse(ssPin_OPC, 0x03);
    SPI.transfer(0x00);
    SetSSpin(ssPin_OPC, HIGH);
    SPI.endTransaction();
    delay(10);
}
