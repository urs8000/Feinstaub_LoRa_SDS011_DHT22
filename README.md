### Feinstaub-Sensor connected with LoRaWan by RFM95
 based on the ESP8266 version of the community Stuttgart (Open Data Stuttgart)
http://codefor.de/stuttgart/  build: http://luftdaten.info/feinstaubsensor-bauen/

### used hardware
* microcontroller:	ESP32 (DOIT)
* Feinstaubsensor:	SDS011		http://inovafitness.com/en/Laser-PM2-5-Sensor-SDS011-35.html
* Temp/Hum:			DHT22 (no resistor between Vcc & Data when powered by 3.3V!)
* LoRa Module:		RFM95 on breadboard
* Battery			Power Pack (10000mAh)

### power supply
* powered via USB: 5V, Vin on board connected to SDS011 

### power measurement
running:  110 - 130mA
waiting:   70mA  (no sleep mode implemented)

### remarks
* please read carefully the changes made in lmic/config.h
* and the compiling selections made for the board
* you need to install ESP32 in the Arduino IDE (link in code)
* Keys for ABP were separeted, structure explained
* at the end of the code the decoder for the TTN console is shown

