#include "OPC.h"
#include <SPI.h>

#define opSerial Serial
#define BaudRate 115200   //cambiar

#define PIN_MISO_OPC 35
#define PIN_MOSI_OPC 34
#define PIN_SCK_OPC 36
#define PIN_CS_OPC 33

//OPC realted variables
SPIClass OP_SPI(HSPI);
#define SPI_OPC_busy 0x31
#define SPI_OPC_ready 0xF3
unsigned char SPI_in[68], SPI_in_index;

unsigned long currentTime;
unsigned long cloopTime;

unsigned char ssPin_OPC=33;



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
  OP_SPI.begin(PIN_SCK_OPC, PIN_MISO_OPC, PIN_MOSI_OPC, PIN_CS_OPC);
  // inicialize OPC
  PIN_CS_OPC = 33;
  InitDevice(PIN_CS_OPC);
  delay(1000);
  //PrintDataLabels(Serial); //Print labels to serial port - optional BL

  Serial.println("OPC initialized");
  delay(1000); //delay in case of noise on power connection. Also allows OPC to boot up.
  currentTime = millis();
  cloopTime = currentTime;
}

void loop() {
  StartOPC(PIN_CS_OPC);
  ReadOPChist(ssPin_OPC); //Read OPC histogram data
  transferPM();
  //PrintData(opSerial); //Print data to serial
  Serial.println(sensorData.PM1);
}

void transferPM()
{
  // Variables para los valores de PM
  float PMs[3]; // Ajustado para 3 valores de PM (PM1, PM2, PM10)
  
  // Extraer y calcular los valores de PM desde SPI_in
  for (int i = 50; i < 62; i += 4) {
      // Unir la funcionalidad de _calc_float aquí
      uint8_t bytes[4] = {SPI_in[i], SPI_in[i+1], SPI_in[i+2], SPI_in[i+3]};
      float result;
      memcpy(&result, bytes, 4); // Convertir los 4 bytes a float
      PMs[(i-50)/4] = result;    // Asignar el resultado
  }
  
  // Asignar los valores a las variables en SensorData
  // No estoy seguro si el orden es 1. 2. 10 o 1.2.5.10
  sensorData.PM1 = PMs[0];
  sensorData.PM2 = PMs[1];
  sensorData.PM10 = PMs[2];
}
