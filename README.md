# Optimize Farming with Environmental Indexes Tracking and Irrigation Control via LoRaWAN
The library used in this project is IBM Lmic which ported from AVR/espressif and only supports LoRaWan class A, B. I used class A for this project, you may  docs this reference [LoRaWan classes](https://www.semtech.com/uploads/technology/LoRa/lorawan-device-classes.pdf) before continuing  
I would recommend trying to use STM32WB or STM32WL series so you can explore as much potential as possible of LoRaWan with [I-CUBE-LRWAN Expansion Package](https://www.st.com/en/embedded-software/i-cube-lrwan.html)  
## Components:
-> STM32F303RET6
-> ES32-AI-Thinker (leaf detection optional, you can build your own model [here](https://edgeimpulse.com/))
-> RFM95(W)
-> DHT22
-> BH1750
-> Capacitive soil moisture sensor
-> Rain pad (use pull-up resistor)
-> SSD1306
-> L298N
-> 5V relay
-> Mini560 5V buck converter
-> 3S 18650 Li-Po
## Pin assignment:
(images/1.png)
## How to import project in CubeMX
 _File > Import > Existing Projects into Workspace > Select archive file > ***Where you clone this repo***_

 
