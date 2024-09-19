#include <SPI.h>

#define ArduinoUNO
#define opSerial Serial
#define BaudRate 9600
#define SPI_OPC_busy 0x31
#define SPI_OPC_ready 0xF3

#define PIN_MISO_OPC 35
#define PIN_MOSI_OPC 34
#define PIN_SCK_OPC 36
#define PIN_CS_OPC 33

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
  ssPin_OPC = 10;
  InitDevice();
  //END Device #1
  PrintDataLabels(opSerial); //Print labels to serial port - optional BL
}

void InitDevice (void)
{

  ReadOPCstring(0x10); //Get serialstr from OPC device
  ReadOPCstring(0x3F); //Get infostr from OPC device

  StartOPC(); //Switch on power to fan and laser
  //ReadOPCconfig(opSerial); //Get Config data (bin boundaries etc.) from OPC device
}


// Main Loop
void loop()
{

      ssPin_OPC = 10;

      unsigned long GetHistTime = millis(); //Set initial GetHistTime
      ReadOPChist(); //Read OPC histogram data
      opSerial.print(millis());
      PrintData(opSerial); //Print data to serial
      delay(5000);
      Serial.println(mydata.temp);
      Serial.println(mydata.humid);
      Serial.println(mydata.PM[0]);
      
      Serial.println(mydata.PM[1]);
      
      Serial.println(mydata.PM[2]);
      
      //Serial.println(GetHistTime);
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

  SetSSpin(HIGH); //lo que indica el final de la comunicación SPI.
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

  SPI.beginTransaction(SPISettings(300000, MSBFIRST, SPI_MODE2));

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
        Serial.println(F("ERROR Resetting SPI")); //signal user
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

  port.flush(); //vacía el búfer de salida del puerto serie
}

void AddDelimiter (Stream &port)
{
  port.print(F(",")); //delimiter
}


void SetSSpin (bool pinState) //pinState is HIGH or LOW
{
  digitalWrite(ssPin_OPC, pinState); //Set output to pinState
}
