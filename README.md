# RQLabs Commander
ESP32 module to process and log sensor data from our rocket - MiniNeutrina.

## Components
### Module
We use dual core ESP32 module shown on image below.
![ESP32Module](https://img.joomcdn.net/84152a14c11919e9336d83f3117bf1867d3fea79_original.jpeg)

### Main module power
Main module is powered by 18650 cell with a 5V buck-boost converter. Its 5V output is connected to **VIN**. 

### Buzzer
To be sure, that everything is working correctly we use buzzer module. It is also powered by buck-boost converter (5V) and its signal pin is connected to **GPIO13**.

### MPU6050
![MPU6050](https://cdn2.botland.com.pl/61319-pdt_540/mpu-6050-3-osiowy-akcelerometr-i-zyroskop-i2c-modul-dfrobot.jpg)

Current repository supports logging data only from MPU6050 sensor. Its VIN pin is connected to 3V output from ESP32**(VDD 3V3)**. SDA pin is connected to the **GPIO21** and SCL to **GPIO22**.

### MicroSD card reader
![MicroSD card reader](https://cdn1.botland.com.pl/64074-pdt_540/modul-czytnika-kart-microsd.jpg)
Module is powered by buck-boost converter (5V). Others pins connection:
- Module CS to **GPIO5**
- Module SCK to **GPIO18**
- Module MOSI to **GPIO23**
- Module MISO to **GPIO19**

## Code logic
We've decided to use [ESP32's tasks](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html) mainly because of dual core CPU inside ESP32. 

