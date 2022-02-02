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

Using this sensor are collecting current acceleration, rotation and temperature data.

Its VIN pin is connected to 3V output from ESP32**(VDD 3V3)**. SDA pin is connected to the **GPIO21** and SCL to **GPIO22**.

### BMP388
![BMP388](https://tatrashike.pl/imgs-44071_thumbs/5-Diymore-BMP388-CJMCU-388-24-bit-cyfrowy-czujnik-temperatury.jpeg)

Using this sensor we are collecting current altitude, atmospheric pressure and temperature data.

Its VIN pin is connected to 3V output from ESP32**(VDD 3V3)**. SDA pin is connected to the **GPIO21** and SCL to **GPIO22**.

### MicroSD card reader
![MicroSD card reader](https://cdn1.botland.com.pl/64074-pdt_540/modul-czytnika-kart-microsd.jpg)

Module is powered by buck-boost converter (5V). Others pins connection:
- Module CS to **GPIO5**
- Module SCK to **GPIO18**
- Module MOSI to **GPIO23**
- Module MISO to **GPIO19**

## Code logic
We've decided to use [ESP32's tasks](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html) mainly because of dual core CPU inside ESP32. 

At the same moment two tasks can run. In our setup whole time we are saving data to SD card (one core) while on a second one we can collect data from the sensors.

Data collecting task was divided into two parts. One part is responsible for collecting the data from MPU6050 sensor. Another one do the same thing but for the BMP388 sensor. Each of this tasks puts collected data into queue which is a connector between "sensor" tasks and data saving task. BMP388 task goes sleep for a longer time than MPU6050 task, because BMP388 jest slower than MPU. We've decided to do that, beacause we want to collect as many data as we can and waiting for new data coming from BMP388 was a waste of time.

For both sensors we use the same interface - I2C. We can collect data from specific sensor using its addresses:
- 0x68 for MPU6050
- 0x77 for BMP388

## Deployment
To deploy the project connect ESP32 module to Your PC and than using IDE compile the code and send it to the module.

It is broadcasting logs to the serial port (speed set to: 115200). If there is an error opening SD card or when it can't connect to senors You will hear slow beeping followed by onboard blue LED blinking.

## Lessons Learned
In previous version of the software we've used ESP8266 and synchronous data collection and saving to the micro SD card. We've achieved a speed ~40 records/sec.

Using ESP's tasks and ESP32 we are able to collect ~160 records/sec. It is 4 times faster than before!

Also we've decided to change the altitude sensor from BMP280 to BMP388 because its much faster.

## 🚀 About us
We are rocket enthusiasts! Our goal is to build our own rocket, 
which is able to put some payload to the orbit.


## 🔗 Links
[![Website](https://img.shields.io/badge/🔗_Website-000?style=for-the-badge)](https://rqlabs.space)
[![Instagram](https://img.shields.io/badge/Instagram-000?style=for-the-badge&logo=instagram&logoColor=white)](https://instagram.com/RQLabs)
[![Facebook](https://img.shields.io/badge/Facebook-000?style=for-the-badge&logo=facebook&logoColor=white)](https://www.facebook.com/RQLabs)
[![Discord](https://img.shields.io/badge/Discord-000?style=for-the-badge&logo=discord&logoColor=white)](https://discord.gg/9qAMMVjWe8)
[![Twitter](https://img.shields.io/badge/twitter-000?style=for-the-badge&logo=twitter&logoColor=white)](https://twitter.com/RQLabs)

## License
[MIT](https://choosealicense.com/licenses/mit/)

