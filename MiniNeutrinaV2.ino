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

QueueHandle_t queueBMP;
QueueHandle_t queueMPU;
SdFat sd;
SdFat sd2; 
File MPUlogfile;
File BMPlogfile;

char MPUfilename[14];
char BMPfilename[14];

float baseAltitude = 0;

struct MPUData
{
    unsigned long currenttime = 0;
    float MPUaccX = -1;
    float MPUaccY = -1;
    float MPUaccZ = -1;

    float MPUgyroX = -1;
    float MPUgyroY = -1;
    float MPUgyroZ = -1;
};

struct BMPData
{
    unsigned long currenttime = 0;

    float BMPtemperature = -1;
    float BMPpressure = -1;
    float BMPaltitude = -1;
};


void collectMPUData(void * parameters) {
    for (;;) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        MPUData sensData;

        sensData.currenttime = millis();
        sensData.MPUaccX = a.acceleration.x;
        sensData.MPUaccY = a.acceleration.y;
        sensData.MPUaccZ = a.acceleration.z;
        sensData.MPUgyroX = g.gyro.x;
        sensData.MPUgyroY = g.gyro.y;
        sensData.MPUgyroZ = g.gyro.z;

        xQueueSend(queueMPU, &sensData, portMAX_DELAY);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void collectBMPData(void * parameters) {
    for (;;) {
        bmp.performReading();

        BMPData sensData;

        sensData.currenttime = millis();
        sensData.BMPtemperature = bmp.temperature;
        sensData.BMPpressure = bmp.pressure / 100.0; //hPa
        sensData.BMPaltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA) - baseAltitude;

        xQueueSend(queueBMP, &sensData, portMAX_DELAY);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void logMPUData(void * parameters) {
    MPUlogfile = sd.open(MPUfilename, O_CREAT | O_APPEND | O_WRITE);
    for (;;) {
        // Serial.print("Task 2 counter: ");
        // Serial.println(count2++);
        MPUData element;
        xQueueReceive(queueMPU, &element, portMAX_DELAY);

        
        MPUlogfile.print(element.currenttime);
        MPUlogfile.print(",");
        MPUlogfile.print(element.MPUaccX);
        MPUlogfile.print(",");
        MPUlogfile.print(element.MPUaccY);
        MPUlogfile.print(",");
        MPUlogfile.print(element.MPUaccZ);
        MPUlogfile.print(",");
        MPUlogfile.print(element.MPUgyroX);
        MPUlogfile.print(",");
        MPUlogfile.print(element.MPUgyroY);
        MPUlogfile.print(",");
        MPUlogfile.println(element.MPUgyroZ);
        // logfile.print(",");
        // logfile.print(element.BMPaltitude);
        // logfile.print(",");
        // logfile.print(element.BMPpressure);
        // logfile.print(",");
        // logfile.println(element.BMPtemperature);
        // logfile.write((const uint8_t *)&element, sizeof(element));

        digitalWrite(pinBuzzer, LOW);
        MPUlogfile.flush();
        digitalWrite(pinBuzzer, HIGH);
        // vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    MPUlogfile.close();
}

void logBMPData(void * parameters) {
    BMPlogfile = sd2.open(BMPfilename, O_CREAT | O_APPEND | O_WRITE);
    for (;;) {
        // Serial.print("Task 2 counter: ");
        // Serial.println(count2++);
        BMPData element;
        xQueueReceive(queueBMP, &element, portMAX_DELAY);

        
        BMPlogfile.print(element.currenttime);
        BMPlogfile.print(",");
        BMPlogfile.print(element.BMPaltitude);
        BMPlogfile.print(",");
        BMPlogfile.print(element.BMPpressure);
        BMPlogfile.print(",");
        BMPlogfile.println(element.BMPtemperature);
        // logfile.write((const uint8_t *)&element, sizeof(element));

        digitalWrite(LED_BUILTIN, LOW);
        BMPlogfile.flush();
        digitalWrite(LED_BUILTIN, HIGH);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    BMPlogfile.close();
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
    if (!sd.begin(5, SD_SCK_MHZ(15))) {
        error();
    }
    Serial.println("Ok!");

    Serial.print("sd Card state: ");
    if (!sd2.begin(5, SD_SCK_MHZ(15))) {
        error();
    }
    Serial.println("Ok!");

    
    strcpy(BMPfilename, "/BMP00.TXT");
    strcpy(MPUfilename, "/MPU00.TXT");
    for (uint8_t i = 0; i < 100; i++) {
        BMPfilename[4] = '0' + i/10;
        BMPfilename[5] = '0' + i%10;

        MPUfilename[4] = '0' + i/10;
        MPUfilename[5] = '0' + i%10;

        if (! sd.exists(BMPfilename)) {
            break;
        }
    }

    // ---------------------------------------------------------------

    queueMPU = xQueueCreate(2000, sizeof( MPUData ) );
    if(queueMPU == NULL){
        Serial.println("Error creating the queueMPU");
        error();
    }

    // ---------------------------------------------------------------

    queueBMP = xQueueCreate(1000, sizeof( BMPData ) );
    if(queueBMP == NULL){
        Serial.println("Error creating the queueBMP");
        error();
    }

    // ---------------------------------------------------------------

    MPUlogfile = sd.open(MPUfilename, O_WRITE | O_CREAT |O_TRUNC); 
    if( ! MPUlogfile ) {
        Serial.print("Couldnt create "); 
        Serial.println(MPUfilename);
        error();
    }
    Serial.print("Ready to write to:"); Serial.println(MPUfilename);
    MPUlogfile.close();

    // ---------------------------------------------------------------

    BMPlogfile = sd.open(BMPfilename, O_WRITE | O_CREAT |O_TRUNC); 
    if( ! BMPlogfile ) {
        Serial.print("Couldnt create "); 
        Serial.println(BMPfilename);
        error();
    }
    Serial.print("Ready to write to:"); Serial.println(BMPfilename);
    BMPlogfile.close();

    // ---------------------------------------------------------------

	xTaskCreate(
        collectMPUData,
        "collectMPUData",
        16000,
        NULL,
        1,
        NULL
    );

    xTaskCreate(
        collectBMPData,
        "collectBMPData",
        16000,
        NULL,
        1,
        NULL
    );

    xTaskCreate(
        logMPUData,
        "logMPUData",
        18000,
        NULL,
        1,
        NULL
    );

    xTaskCreate(
        logBMPData,
        "logBMPData",
        18000,
        NULL,
        1,
        NULL
    );
}

void loop()
{
	delay(2000);
}
