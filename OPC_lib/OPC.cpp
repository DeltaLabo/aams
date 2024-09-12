#include "Arduino.h"
#include "OPC.h"
#include <SPI.h>

#define SPI_OPC_busy 0x31
#define SPI_OPC_ready 0xF3

void SetSSpin (unsigned char ssPin_OPC, bool pinState) {
   digitalWrite(ssPin_OPC, pinState);
}

void PrintDataLabels (Stream &port)
{
   unsigned char i;
    port.print(F("Time(ms)"));
    for (i=0; i<16; ++i) {
        port.print(F(",Bin"));
        if (i<10)
            port.print(F("0"));
        port.print(i, DEC);
    }
    port.print(F(",MToF Bin1"));
    port.print(F(",MToF Bin3"));
    port.print(F(",MToF Bin5"));
    port.print(F(",MToF Bin7"));
    port.print(F(",SFR"));
    port.print(F(",Temp(C)"));
    port.print(F(",RH(%)"));
    port.print(F(",Sampling Period(s)"));
    port.print(F(",Reject count Glitch"));
    port.print(F(",Reject count Long"));
    port.print(F(",PM_A(ug/m3)"));
    port.print(F(",PM_B(ug/m3)"));
    port.print(F(",PM_C(ug/m3)"));
    port.print(F(",Checksum"));
    port.println();
    port.flush();
}


void InitDevice (unsigned char ssPin_OPC)
{
   StartOPC(ssPin_OPC); //Switch on power to fan and laser
}



void StartOPC (unsigned char ssPin_OPC)
{
  //Turn ON fan and peripheral power

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

//Process OPC data and print
void PrintData (Stream &port)
{
   unsigned char i;
    uint16_t *pUInt16;
    float *pFloat;
    float Afloat;

    //Histogram bins (unsigned 16bit integers occupying 2 bytes each)
    for (i=0; i<32; i+=2) {
        AddDelimiter(port);
        pUInt16 = (uint16_t *)&SPI_in[i];
        port.print(*pUInt16, DEC);
    }

    //MToF 8bit integer occupying 1 byte each representing the avarage time that particles sized in the stated bin took to cross the laser beam
    for (i=32; i<36; ++i) {
        AddDelimiter(port);
        Afloat = (float)SPI_in[i];
        Afloat /= 3; //convert to microseconds
        port.print(Afloat, 2);
    }

    //SFR (Sample Flow Rate) (float occupying 4 bytes) (ml/s)
    AddDelimiter(port);
    pFloat = (float *)&SPI_in[36];
    port.print(*pFloat, 3);

    //Temperature (unsigned 16bit integer occupying 2 bytes)
    AddDelimiter(port);
    pUInt16 = (uint16_t *)&SPI_in[40];
    port.print(ConvSTtoTemperature(*pUInt16), 1);

    //Relative Humidity (unsigned 16bit integer occupying 2 bytes)
    AddDelimiter(port);
    pUInt16 = (uint16_t *)&SPI_in[42];
    port.print(ConvSRHtoRelativeHumidity(*pUInt16), 1);

    //Sampling Period (float occupying 4 bytes) measure of the histograms actual sampling period in seconds
    AddDelimiter(port);
    pFloat = (float *)&SPI_in[44];
    port.print(*pFloat, 3);

    //Reject count Glitch and Reject count Long (unsigned 8bit integers occupying 1 byte each)
    for (i=48; i<50; ++i) {
        AddDelimiter(port);
        port.print(SPI_in[i], DEC);
    }

    //PM_A (float occupying 4 bytes) (ug/m3)
    AddDelimiter(port);
    pFloat = (float *)&SPI_in[50];
    port.print(*pFloat, 3);

    //PM_B (float occupying 4 bytes) (ug/m3)
    AddDelimiter(port);
    pFloat = (float *)&SPI_in[54];
    port.print(*pFloat, 3);

    //PM_C (float occupying 4 bytes) (ug/m3)
    AddDelimiter(port);
    pFloat = (float *)&SPI_in[58];
    port.print(*pFloat, 3);

 //PMs
    float PMs[3];
    for (i=50; i<62; i+=4) {
        uint8_t bytes[4] = {SPI_in[i], SPI_in[i+1], SPI_in[i+2], SPI_in[i+3]};
        PMs[(i-50)/4] = _calc_float(bytes);
        AddDelimiter(port);
        port.print(PMs[(i-50)/4], 3);
    }

    //Checksum (unsigned 16bit integer occupying 2 bytes)
    AddDelimiter(port);
    pUInt16 = (uint16_t *)&SPI_in[62];
    port.println(*pUInt16, DEC);

    //Compare recalculated Checksum with the one received from the OPC
    if (*pUInt16 == MODBUS_CalcCRC(SPI_in, 62)) {
        port.println(F("Checksum OK"));
    }
    else {
        port.println(F("Checksum ERROR"));
    }
    port.flush();
}


void AddDelimiter (Stream &port)
{
  port.print(F(",")); //delimiter
}

float _calc_float(uint8_t (&bytes)[4]) {
    static_assert(sizeof(float) == 4, "float size expected to be 4 bytes");
    float result;
    memcpy(&result, bytes, 4);
    return result;
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

void StopOPC (unsigned char ssPin_OPC) {
    //Turn OFF fan and laser
    GetReadyResponse(ssPin_OPC, 0x03);
    SPI.transfer(0x00);
    SetSSpin(ssPin_OPC, HIGH);
    SPI.endTransaction();
    delay(10);
}
