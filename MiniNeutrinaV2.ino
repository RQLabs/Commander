#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include "Adafruit_BMP3XX.h"
#include "SdFat.h"

int pinBuzzer = 13;
#define SEALEVELPRESSURE_HPA (1013.25)

// I2C device found at address 0x68
// I2C device found at address 0x77  
Adafruit_MPU6050 mpu;
Adafruit_BMP3XX bmp;

QueueHandle_t queue;
SdFat sd; 
File logfile;
char filename[15];
float baseAltitude = 0;

struct SensorData
{
    unsigned long currenttime = 0;
    float MPUaccX = -1;
    float MPUaccY = -1;
    float MPUaccZ = -1;

    float MPUgyroX = -1;
    float MPUgyroY = -1;
    float MPUgyroZ = -1;

    float BMPtemperature = -1;
    float BMPpressure = -1;
    float BMPaltitude = -1;
};


void collectMPUData(void * parameters) {
    for (;;) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        SensorData sensData;

        sensData.currenttime = millis();
        sensData.MPUaccX = a.acceleration.x;
        sensData.MPUaccY = a.acceleration.y;
        sensData.MPUaccZ = a.acceleration.z;
        sensData.MPUgyroX = g.gyro.x;
        sensData.MPUgyroY = g.gyro.y;
        sensData.MPUgyroZ = g.gyro.z;

        xQueueSend(queue, &sensData, portMAX_DELAY);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void collectBMPData(void * parameters) {
    for (;;) {
        bmp.performReading();

        SensorData sensData;

        sensData.currenttime = millis();
        sensData.BMPtemperature = bmp.temperature;
        sensData.BMPpressure = bmp.pressure / 100.0; //hPa
        sensData.BMPaltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA) - baseAltitude;

        xQueueSend(queue, &sensData, portMAX_DELAY);
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

void logData(void * parameters) {
    logfile = sd.open(filename, O_CREAT | O_APPEND | O_WRITE);
    for (;;) {
        // Serial.print("Task 2 counter: ");
        // Serial.println(count2++);
        SensorData element;
        xQueueReceive(queue, &element, portMAX_DELAY);

        
        logfile.print(element.currenttime);
        logfile.print(",");
        logfile.print(element.MPUaccX);
        logfile.print(",");
        logfile.print(element.MPUaccY);
        logfile.print(",");
        logfile.print(element.MPUaccZ);
        logfile.print(",");
        logfile.print(element.MPUgyroX);
        logfile.print(",");
        logfile.print(element.MPUgyroY);
        logfile.print(",");
        logfile.print(element.MPUgyroZ);
        logfile.print(",");
        logfile.print(element.BMPaltitude);
        logfile.print(",");
        logfile.print(element.BMPpressure);
        logfile.print(",");
        logfile.println(element.BMPtemperature);
        // logfile.write((const uint8_t *)&element, sizeof(element));

        digitalWrite(pinBuzzer, LOW);
        digitalWrite(LED_BUILTIN, HIGH);
        logfile.flush();
        digitalWrite(pinBuzzer, HIGH);
        digitalWrite(LED_BUILTIN, LOW);
        // vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    logfile.close();
}

void beep(int beep_time) {
  digitalWrite(pinBuzzer, HIGH);
  delay(beep_time);
  digitalWrite(pinBuzzer, LOW);
}

void error() {
  while(true) {
    Serial.println("Error"); 
    digitalWrite(LED_BUILTIN, LOW);
    beep(300);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
  }
}

void setup()
{
    // ---------------------------------------------------------------

    Serial.begin(115200);

    // ---------------------------------------------------------------

    pinMode(pinBuzzer, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    // ---------------------------------------------------------------

    Serial.print("MPU6050 state: ");
    if(!mpu.begin(0x68)) {
        error();
    }
    Serial.println("Ok!");

    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);

    // ---------------------------------------------------------------

    Serial.print("BMP388 state: ");
    if(!bmp.begin_I2C(0x77)) {
        error();
    }
    Serial.println("Ok!");

    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_200_HZ);

    Serial.print("BMP388 reading: ");
    if (! bmp.performReading()) {
        error();
    }
    Serial.println("Ok!");
    baseAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

    // ---------------------------------------------------------------

    Serial.print("sd Card state: ");
    if (!sd.begin(5, SD_SCK_MHZ(11))) {
        error();
    }
    Serial.println("Ok!");

    
    strcpy(filename, "/DATA00.TXT");
    for (uint8_t i = 0; i < 100; i++) {
        filename[5] = '0' + i/10;
        filename[6] = '0' + i%10;

        if (! sd.exists(filename)) {
            break;
        }
    }

    // ---------------------------------------------------------------

    queue = xQueueCreate(2000, sizeof( SensorData ) );
    if(queue == NULL){
        Serial.println("Error creating the queue");
        error();
    }

    // ---------------------------------------------------------------

    logfile = sd.open(filename, O_WRITE | O_CREAT |O_TRUNC); 
    if( ! logfile ) {
        Serial.print("Couldnt create "); 
        Serial.println(filename);
        error();
    }
    Serial.print("Ready to write to:"); Serial.println(filename);
    logfile.close();

    // ---------------------------------------------------------------

	xTaskCreate(
        collectMPUData,
        "collectMPUData",
        14000,
        NULL,
        1,
        NULL
    );

    xTaskCreate(
        collectBMPData,
        "collectBMPData",
        14000,
        NULL,
        1,
        NULL
    );

    xTaskCreate(
        logData,
        "LogData",
        16000,
        NULL,
        1,
        NULL
    );
}

void loop()
{
	delay(2000);
}
