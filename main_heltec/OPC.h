#include "Arduino.h"
#include <SPI.h>      

extern unsigned char SPI_in[68]; // For ReadOPChist and GetReadyResponse

void SetSSpin (unsigned char ssPin_OPC, bool pinState);
void StartOPC (unsigned char ssPin_OPC);
void GetReadyResponse (unsigned char ssPin_OPC, unsigned char SPIcommand);
void ReadOPChist(unsigned char ssPin_OPC);
void StopOPC (unsigned char ssPin_OPC);