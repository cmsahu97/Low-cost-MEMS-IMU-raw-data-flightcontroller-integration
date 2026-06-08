#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>

// Initialize the LSM9DS1 sensor object
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LED_PIN PC13 
#define SBG_SYNC1 0xFF
#define SBG_SYNC2 0x5A
#define SBG_ETX   0x33

#define SBG_CLASS_LOG_ECOM_0 0x00
#define SBG_MSG_IMU_DATA     3  
#define SBG_MSG_MAG          4  

// Gyro calibration offset array
float gyroBias[3] = {0.0, 0.0, 0.0};

// SBG Binary Structures packed tightly to prevent compiler padding
struct __attribute__((packed)) SbgLogImuLegacy {
    uint32_t timeStamp;      
    uint16_t status;         
    float accelerometers[3]; 
    float gyroscopes[3];     
    float temperature;       
    float deltaVelocity[3];  
    float deltaAngle[3];     
};

struct __attribute__((packed)) SbgLogMag {
    uint32_t timeStamp;      
    uint16_t status;         
    float magnetometers[3];  
    float accelerometers[3]; 
};

uint32_t frameCount = 0;
uint32_t lastLoopTimeUs = 0;
uint32_t lastPrintTimeMs = 0; 
bool ledState = false;

// FIX: Declaring the structures globally so both loop() and the safe print zone can access them
SbgLogImuLegacy imuData; 
SbgLogImuLegacy lastImuData;
float rawMagX = 0;
uint32_t diagnosticLoopTimeUs = 0;

// 16-bit CRC Calculation matching SBG Systems protocol standard
uint16_t calcCRC(const uint8_t *buffer, size_t size) {
    uint16_t crc = 0;
    for (size_t j = 0; j < size; j++) {
        crc ^= buffer[j];
        for (int i = 0; i < 8; i++) {
            if (crc & 1) crc = (crc >> 1) ^ 0x8408;
            else crc >>= 1;
        }
    }
    return crc;
}

// Function to assemble and broadcast a framed packet over Serial1
void sendSbgFrame(uint8_t msgId, uint8_t msgClass, uint8_t *payload, uint16_t payloadLen) {
    Serial1.write(SBG_SYNC1);
    Serial1.write(SBG_SYNC2);

    uint8_t crcBuffer[payloadLen + 4];
    crcBuffer[0] = msgId;
    crcBuffer[1] = msgClass;
    crcBuffer[2] = (uint8_t)(payloadLen & 0xFF);
    crcBuffer[3] = (uint8_t)((payloadLen >> 8) & 0xFF);
    memcpy(&crcBuffer[4], payload, payloadLen);

    Serial1.write(crcBuffer, payloadLen + 4);

    uint16_t crc = calcCRC(crcBuffer, payloadLen + 4);
    Serial1.write((uint8_t)(crc & 0xFF));
    Serial1.write((uint8_t)((crc >> 8) & 0xFF));
    Serial1.write(SBG_ETX);
}

// Power-on calibration routine for gyroscopes
void calibrateGyros() {
    Serial.println("--- DO NOT MOVE: CALIBRATING GYROS ---");
    delay(1000); 
    int samples = 500; 
    float sumG[3] = {0, 0, 0};
    for (int i = 0; i < samples; i++) {
        lsm.read();
        sensors_event_t a, m, g, temp;
        lsm.getEvent(&a, &m, &g, &temp);
        sumG[0] += g.gyro.x;
        sumG[1] += -g.gyro.y; 
        sumG[2] += -g.gyro.z; 
        delay(2); 
    }
    gyroBias[0] = sumG[0] / samples;
    gyroBias[1] = sumG[1] / samples;
    gyroBias[2] = sumG[2] / samples;
    Serial.println("--- STREAMING ACTIVATED ---");
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); 

    // USB Serial Monitor
    Serial.begin(115200);   

    // HARDWARE ENFORCEMENT: Force Serial1 to use PA2 (TX) and PA3 (RX) instead of default PA9/PA10
    Serial1.setTx(PA2);
    Serial1.setRx(PA3);
    Serial1.begin(230400);  

    // Setup I2C interface pins for Black Pill
    Wire.setSCL(PB6);
    Wire.setSDA(PB7);
    Wire.begin();
    Wire.setClock(400000); 

    if (!lsm.begin()) {
        while (1) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100); }
    }

    lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G); 
    lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);

    calibrateGyros();
    lastLoopTimeUs = micros(); 
    lastPrintTimeMs = millis();
}

void loop() {
    uint32_t currentUs = micros();
    
    // Time-Critical 250Hz Calculation Window
    if (currentUs - lastLoopTimeUs >= 4000) {
        diagnosticLoopTimeUs = currentUs - lastLoopTimeUs; // Capture timing profile
        lastLoopTimeUs = currentUs;
        frameCount++;

        lsm.read();
        sensors_event_t a, m, g, temp;
        lsm.getEvent(&a, &m, &g, &temp);

        // Populate IMU Structure
        imuData.timeStamp = currentUs;
        imuData.status = 0x03FF; 
        imuData.accelerometers[0] = a.acceleration.x;
        imuData.accelerometers[1] = -a.acceleration.y;
        imuData.accelerometers[2] = -a.acceleration.z;
        imuData.gyroscopes[0] = g.gyro.x - gyroBias[0];
        imuData.gyroscopes[1] = (-g.gyro.y) - gyroBias[1];
        imuData.gyroscopes[2] = (-g.gyro.z) - gyroBias[2];
        imuData.temperature = temp.temperature;
        
        float dt = (float)diagnosticLoopTimeUs / 1000000.0f;
        imuData.deltaVelocity[0] = imuData.accelerometers[0] * dt;
        imuData.deltaVelocity[1] = imuData.accelerometers[1] * dt;
        imuData.deltaVelocity[2] = imuData.accelerometers[2] * dt;
        imuData.deltaAngle[0] = imuData.gyroscopes[0] * dt;
        imuData.deltaAngle[1] = imuData.gyroscopes[1] * dt;
        imuData.deltaAngle[2] = imuData.gyroscopes[2] * dt;

        sendSbgFrame(SBG_MSG_IMU_DATA, SBG_CLASS_LOG_ECOM_0, (uint8_t*)&imuData, sizeof(SbgLogImuLegacy));

        if (frameCount % 5 == 0) {
            SbgLogMag magData;
            magData.timeStamp = currentUs;
            magData.status = 0x03FF;
            magData.magnetometers[0] = m.magnetic.x;
            magData.magnetometers[1] = -m.magnetic.y;
            magData.magnetometers[2] = -m.magnetic.z;
            magData.accelerometers[0] = imuData.accelerometers[0];
            magData.accelerometers[1] = imuData.accelerometers[1];
            magData.accelerometers[2] = imuData.accelerometers[2];

            sendSbgFrame(SBG_MSG_MAG, SBG_CLASS_LOG_ECOM_0, (uint8_t*)&magData, sizeof(SbgLogMag));
            rawMagX = m.magnetic.x; 
        }

        // Save snapshots for safe printing outside the tight loop timing
        lastImuData = imuData;

        if (frameCount % 125 == 0) { 
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState ? LOW : HIGH); 
        }
    }

    // SAFE ZONE: Run printing asynchronously in the empty time blocks between frames
    uint32_t currentMs = millis();
    if (currentMs - lastPrintTimeMs >= 1000) { 
        lastPrintTimeMs = currentMs;
        
        Serial.print("[DATA CHECK] Loops Run: "); Serial.print(frameCount);
        Serial.print(" | Target Loop Timing: "); Serial.print(diagnosticLoopTimeUs); Serial.print("us");
        Serial.print(" | AccelX: "); Serial.print(lastImuData.accelerometers[0], 2);
        Serial.print(" m/s2 | GyroX: "); Serial.print(lastImuData.gyroscopes[0], 3);
        Serial.print(" rad/s | MagX: "); Serial.print(rawMagX, 1);
        Serial.println(" uT");
    }
}