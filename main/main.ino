//Codigo para leer sensores

int sensorPin = A0;   // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED
float sensorValue = 0;  // variable to store the value coming from the sensor

void setup() {
  Serial.begin(9600);
}

void loop() {
  // read the value from the sensor:
  sensorValue = (analogRead(sensorPin)/1023.0)*5000 - 280;
  sensorValue = sensorValue / 445;
  delay(500);
  Serial.print("Concentration: ");
  Serial.print(sensorValue);
  Serial.println(" ppm");
}
