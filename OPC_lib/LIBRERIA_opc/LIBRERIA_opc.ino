#include "OPC.h"
#include <SPI.h>

#define opSerial Serial
#define BaudRate 115200   //cambiar

#define PIN_MISO_OPC 35
#define PIN_MOSI_OPC 34
#define PIN_SCK_OPC 36
#define PIN_CS_OPC 33

//OPC realted variables
#define SPI_OPC_busy 0x31
#define SPI_OPC_ready 0xF3
unsigned char SPI_in[68], SPI_in_index;

unsigned long currentTime;
unsigned long cloopTime;

unsigned char ssPin_OPC=33;



struct SEND_DATA_STRUCTURE{
  unsigned int hist[16];
  float temp;
  float humid;
  float PM[3];
};
SEND_DATA_STRUCTURE mydata;

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

void setup() {
  Serial.begin(115200);
  Serial.println("Setup start");
  // put your setup code here, to run once:
  delay(1000); //delay in case of noise on power connection. Also allows OPC to boot up.
  SPI.begin(PIN_SCK_OPC, PIN_MISO_OPC, PIN_MOSI_OPC, PIN_CS_OPC);
  // inicialize OPC
  StartOPC(PIN_CS_OPC);
  delay(1000);
  //PrintDataLabels(Serial); //Print labels to serial port - optional BL

  Serial.println("OPC initialized");
  delay(1000); //delay in case of noise on power connection. Also allows OPC to boot up.
  currentTime = millis();
  cloopTime = currentTime;
}

void loop() {
  StartOPC(PIN_CS_OPC);
  ReadOPChist(PIN_CS_OPC); //Read OPC histogram data 
  //SPI_in[SPI_in_index] 
  transferdata();
  //PrintData(opSerial); //Print data to serial
  // Asignar los valores a las variables en SensorData
  // No estoy seguro si el orden es 1. 2. 10 o 1.2.5.10
  sensorData.PM1 = mydata.PM[0];
  sensorData.PM2 = mydata.PM[1];
  sensorData.PM10 = mydata.PM[2];
  Serial.println(sensorData.PM1);
}

void transferdata()
{
  // Variables para los valores de PM
  unsigned char i;
  unsigned int *pUInt16;
  float *pFloat;
  float Afloat;
  int k;
  //Histogram bins (UInt16) x16
  for (i=0; i<32; i+=2)
  {
    pUInt16 = (unsigned int *)&SPI_in[i];
    mydata.hist[k] = *pUInt16;
    k++;
  }
   //MToF bytes (UInt8) x4
  for (i=32; i<36; i++)
  {
    Afloat = (float)SPI_in[i];
    Afloat /= 3; //convert to us
  }

  //SFR (4-byte float) x1
  pFloat = (float *)&SPI_in[36];

  pUInt16 = (unsigned int *)&SPI_in[40];
  mydata.temp = ConvSTtoTemperature(*pUInt16);

  //Relative humidity (2-byte integer) x1
  pUInt16 = (unsigned int *)&SPI_in[42];  
  mydata.humid = ConvSRHtoRelativeHumidity(*pUInt16);

  //Sampling period(s) (4-byte float) x1
  pFloat = (float *)&SPI_in[44];

  //Reject count Glitch (1-byte integer) x1
  //SPI_in[48]

  //Reject count LongTOF (1-byte integer) x1
  //SPI_in[49]

  //PM values(ug/m^3) (4-byte float) x3
  k = 0;
  for (i=50; i<62; i+=4)
  {
    pFloat = (float *)&SPI_in[i];
    mydata.PM[k] = *pFloat;
    k++;
  }
  
   //Checksum (UInt16) x1
  pUInt16 = (unsigned int *)&SPI_in[62];

  //Compare recalculated Checksum with one sent
  // if (*pUInt16 != MODBUS_CalcCRC(SPI_in, 62)) //if checksums aren't equal
  // port.println(F("Checksum error in line above!"));
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