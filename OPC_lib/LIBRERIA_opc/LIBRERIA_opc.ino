#include "OPC.h"
#include <SPI.h>

#define opSerial Serial
#define BaudRate 115200   //cambiar
#define SPI_OPC_busy 0x31
#define SPI_OPC_ready 0xF3


unsigned long currentTime;
unsigned long cloopTime;
unsigned char SPI_in[68], SPI_in_index;

unsigned char ssPin_OPC=36;

struct SEND_DATA_STRUCTURE{
  unsigned int hist[16];
  float temp;
  float humid;
  float PM[3];
};
SEND_DATA_STRUCTURE mydata;


void setup() {
  // put your setup code here, to run once:
  delay(1000); //delay in case of noise on power connection. Also allows OPC to boot up.

  //Start serial port
  SetSSpin(ssPin_OPC, HIGH);
  pinMode(ssPin_OPC, OUTPUT);
  delay(1000);
  opSerial.begin(BaudRate);
  
  // start the SPI library:
  SPI.begin(); //Enable SPI for OPC comms
  InitDevice(ssPin_OPC);
  delay(1000); //delay in case of noise on power connection. Also allows OPC to boot up.
  PrintDataLabels(opSerial);
  currentTime = millis();
  cloopTime = currentTime;
}

void loop() {
  
  StartOPC(ssPin_OPC);
  delay(1000);
  ReadOPChist(ssPin_OPC); //Read OPC histogram data
  PrintData(opSerial); //Print data to serial
  Serial.println(mydata.temp);
  Serial.println(mydata.humid);
  Serial.println(mydata.PM[0]);
      
  Serial.println(mydata.PM[1]);
      
  Serial.println(mydata.PM[2]);
}
