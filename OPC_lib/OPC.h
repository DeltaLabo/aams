#include "Arduino.h"
#include <SPI.h>      // Incluye SPI para usar SPI.transfer

extern unsigned char SPI_in[68]; // SE USA EN ReadOPChist

void SetSSpin (unsigned char ssPin_OPC, bool pinState);
void PrintDataLabels (Stream &port);

void InitDevice (unsigned char ssPin_OPC);
void StartOPC (unsigned char ssPin_OPC);
void GetReadyResponse (unsigned char ssPin_OPC, unsigned char SPIcommand);
void ReadOPChist(unsigned char ssPin_OPC);
void PrintData (Stream &port);
void AddDelimiter (Stream &port);
float _calc_float(uint8_t (&bytes)[4]);
float ConvSTtoTemperature (unsigned int ST);
float ConvSRHtoRelativeHumidity (unsigned int SRH);
unsigned int MODBUS_CalcCRC(unsigned char data[], unsigned char nbrOfBytes);
void StopOPC (void);