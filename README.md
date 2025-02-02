# Optimize Farming with Environmental Indexes Tracking and Irrigation Control via LoRaWAN
This project uses the IBM LMIC library which ported from AVR/espressif and only supports LoRaWan class A, B. I used class A for this project, you may  refer to [LoRaWan classes](https://www.semtech.com/uploads/technology/LoRa/lorawan-device-classes.pdf) for more details before proceeding.<br>
I recommend using the STM32WB or STM32WL series to fully explore the potential of LoRaWAN with [I-CUBE-LRWAN Expansion Package](https://www.st.com/en/embedded-software/i-cube-lrwan.html).<br>
Include: OneWire, I2C, SPI, UART, DMA, ADC, PWM Timer, Interrupt, and Sleep mode <br>
## Setup Guide
### Components:
-> STM32F303RET6<br>
-> ES32-AI-Thinker (leaf detection optional, you can build your own model [here](https://edgeimpulse.com/))<br>
-> RFM95(W)<br>
-> DHT22<br>
-> BH1750<br>
-> Capacitive soil moisture sensor<br>
-> Rain pad (use pull-up resistor)<br>
-> SSD1306<br>
-> L298N<br>
-> 5V relay<br>
-> Mini560 5V buck converter<br>
-> 3S 18650 Li-Po<br>
### Pin assignment:
<div align="center">
  <img src="images/1.png"> <br>
</div> 

PB12 Read battery level Here I have used a voltage divider circuit with a ratio of 4.28<br>
PB0 rain<br>
PB1 soil<br>
PB10 push button to change the screen view<br>
PC2 dht22<br>
PC10 En2<br>
PC11 In1<br>
PC12 Relay module<br>
I2C_1 SSD1306<br>
I2C_2 Bh1750<br>
### How to import project in CubeMX
 _File > Import > Existing Projects into Workspace > Select archive file > ***Where you clone this repo***_
 > Support me by leaving a star at the upper right corner xD
