Ambient air monitoring system
============
## DeltaLAB - Instituto Tecnológico de Costa Rica

### Developers

* [Juan J. Rojas](mailto:juan.rojas@itcr.ac.cr) 
* [Joseph Muñoz](mailto:munozcascante.j@estudiantec.cr)
* [Nicole Prado](mailto:nicoleprado@estudiantec.cr)

### What is this repository for? ###

* This repository was created to develop an ESP32-based air quality monitoring device.

### Microcontroller info

* Model: Heltec ESP32 WiFi LoRa 32(V3)
* [Documentation page](https://heltec.org/project/wifi-lora-32-v3/)

### Sensors info

* [OPC-R2 Optical Particle Counter](https://drive.google.com/file/d/1VDuOj8a8o7cUKEGa6ehXuCjHjkZSZghj/view?usp=drivesdk)
* [Alphasense CO Sensor (CO-B4)](https://drive.google.com/file/d/1JNV_i5KHD0BKV2ffySytdRTA-Bh3-L8U/view?usp=drive_link)
* [Alphasense NO2 Sensor (NO2-B43F](https://drive.google.com/file/d/1ekLD1FF8v9zRlR9MNiv1X1ECm4lZbT9F/view?usp=drive_link)
* [Alphasense SO2 Sensor (SO2-B4)](https://drive.google.com/file/d/1T6IZ1pmTH6bRZZzv0rK4N2md5-tMnQ_p/view?usp=drive_link)
* [DHT22 Humidity Temperature Sensor](https://drive.google.com/file/d/1V-DVVbrcwCpVjzvk9U7t66gLOcytCYI-/view?usp=drive_link)
* [Adafruit INA219 Current Sensor Breakout](https://cdn-shop.adafruit.com/datasheets/ina219.pdf)

### Data visualization in ThingSpeak

* [Here](https://thingspeak.com/channels/2363549)

### How do I get set up? ###

* Install Git
* Install Arduino IDE
* Follow this [instructions](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html) to set up the Heltec Wifi LoRa 32 (V3) in the Arduino IDE
* Install all the Libraries in the Arduino Library Manager
Open Arduino IDE, then Select `Sketch`->`Include Library`->`Manage Libraries...`
* Search `WiFi`, `ThingSpeak`, `Adafruit_INA219`, `DHTStable', `SdFat', and `ESP32Time` libraries and install them.
  
* Connect the components as shown in the diagram found in schematic_diagram.pdf
* Clone this repo and upload main.ino to the ESP32

### Library source code and examples
* [WiFi.h](https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi)
* [ThingSpeak.h](https://github.com/mathworks/thingspeak-arduino/tree/master)
* [ESP32Time.h](https://github.com/fbiego/ESP32Time)
* [Adafruit_INA219.h](https://github.com/adafruit/Adafruit_INA219/tree/master)
* [DHTStable.h](https://github.com/RobTillaart/DHTstable)
* [SPI.h](https://github.com/PaulStoffregen/SPI)
* [SdFat.h](https://github.com/greiman/SdFat)
* [SdFatConfig.h](https://github.com/greiman/SdFat)

### Contribution guidelines ###

* If you want to propose a review or need to modify the code for any reason first clone this [repository](https://github.com/DeltaLabo/aams) in your PC and create a new branch for your changes. Once your changes are complete and fully tested ask the administrator permission to push this new branch into the source.
* If you just want to do local changes instead you can download a zip version of the repository and do all changes locally in your PC. 
