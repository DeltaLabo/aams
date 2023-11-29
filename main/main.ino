//Codigo para leer sensores
//
//Para Arduino
/*int CO_pin = A5;
int NO_pin = A4;
int SO_pin = A3;
int CO_aux_pin = A0;
int NO_aux_pin = A1;
int SO_aux_pin = A2;*/

//Para ESP32
int CO_pin = 34;
int NO_pin = 35;
int SO_pin = 36;

int ledPin = 13;      // select the pin for the LED
float CO_value = 0;  // variable to store the value coming from the sensor
float NO_value = 0;  // 
float SO_value = 0;

void setup() {
  Serial.begin(9600);
  analogReference(INTERNAL2V56);
}

float gasSensor(int pin, int offset, int sens) {
  float value = (analogRead(pin)*2560.0)/1023.0;
  value -= offset; //substract offset
  value /= sens; //apply sensitivity
  value *= 1000; //convert to ppb
  return value;
}

void loop() {
  // read the value from the sensor:
  // Serial.println(analogRead(CO_pin));
  // CO_value = (analogRead(CO_pin)*2560.0)/1023.0;
  // CO_value -= 280; //substract offset
  // CO_value /= 445; //apply sensitivity
  // CO_value *= 1000; //convert to ppb
  CO_value = gasSensor(CO_pin,280,445);
  NO_value = gasSensor(NO_pin,203,740);
  SO_value = gasSensor(SO_pin,350,307);
  delay(2000);
  Serial.print("CO concentration: ");
  Serial.print(CO_value);
  Serial.println(" ppb");
  Serial.print("NO2 concentration: ");
  Serial.print(NO_value);
  Serial.println(" ppb");
  Serial.print("SO2 concentration: ");
  Serial.print(SO_value);
  Serial.println(" ppb");
}
