#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include "SdFat.h"

Adafruit_MPU6050 mpu;

int pinBuzzer = 13;

QueueHandle_t queue;
SdFat sd; 
File logfile;
char filename[15];

struct AccelerometerData
{
    unsigned long currenttime;
    float valX;
    float valY;
    float valZ;
};


void collectMPUData(void * parameters) {
    for (;;) {
        digitalWrite(pinBuzzer, LOW);

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        AccelerometerData accData;

        accData.currenttime = millis();
        accData.valX = a.acceleration.x;
        accData.valY = a.acceleration.y;
        accData.valZ = a.acceleration.z;

        xQueueSend(queue, &accData, portMAX_DELAY);

        digitalWrite(pinBuzzer, HIGH);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void logData(void * parameters) {
    logfile = sd.open(filename, O_CREAT | O_APPEND | O_WRITE);
    for (;;) {
        // Serial.print("Task 2 counter: ");
        // Serial.println(count2++);
        digitalWrite(LED_BUILTIN, LOW);
        
        AccelerometerData element;
        xQueueReceive(queue, &element, portMAX_DELAY);

        
        logfile.print(element.currenttime);
        logfile.print("|");
        logfile.print(element.valX);
        logfile.print("|");
        logfile.print(element.valY);
        logfile.print("|");
        logfile.println(element.valZ);
        // logfile.write((const uint8_t *)&element, sizeof(element));

        logfile.flush();
        digitalWrite(LED_BUILTIN, HIGH);
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
    mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);

    // ---------------------------------------------------------------

    Serial.print("sd Card state: ");
    if (!sd.begin(5, SD_SCK_MHZ(10))) {
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

    logfile = sd.open(filename, O_WRITE | O_CREAT |O_TRUNC); 
    if( ! logfile ) {
        Serial.print("Couldnt create "); 
        Serial.println(filename);
        error();
    }
    Serial.print("Ready to write to:"); Serial.println(filename);
    logfile.close();

    // ---------------------------------------------------------------

    queue = xQueueCreate( 5000, sizeof( AccelerometerData ) );
    if(queue == NULL){
        Serial.println("Error creating the queue");
    }

    // ---------------------------------------------------------------

	xTaskCreate(
        collectMPUData,
        "collectMPUData",
        15000,
        NULL,
        1,
        NULL
    );

    xTaskCreate(
        logData,
        "LogData",
        15000,
        NULL,
        1,
        NULL
    );
}

void loop()
{
	delay(2000);
}
