#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>

#define INPUT1_PIN  34  
#define INPUT2_PIN  35  
#define INPUT3_PIN  32
#define INPUT4_PIN  33  
#define MOTOR_X_PWM 4
#define MOTOR_X_DIR 16
#define MOTOR_X_BRAKE 17
#define MOTOR_Y_PWM 25
#define MOTOR_Y_DIR 26
#define MOTOR_Y_BRAKE 27
#define VSP_PIN1 18
#define VSP_PIN2 19
#define MPU6050 0x68
#define PWR_MGMT_1 0x6B

#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "87654321-4321-4321-4321-cba987654321"

Servo motor_1, motor_2;
BLECharacteristic *pCharacteristic;

bool deviceConnected = false;
bool send = false;
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 200;

int min_speed = 22;
int power_up = min_speed;
int power_bt = min_speed;

double alpha = 0.96;
unsigned long prevTime = 0;
double roll = 0, pitch = 0;
double setpointX = 0, setpointY = 0;
double inputX, inputY;
double outputX, outputY;
double Kp = 0, Ki = 0, Kd = 0;

long long pulseStartTime[4] = {0};  
long long pulseEndTime[4] = {0};    
long long pulseWidth[4] = {0};      
int PWMMAX[4] = {1980, 2000, 2000, 1960};
int PWMMIN[4] = {1010, 980, 990, 1000};

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
    }
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        BLEDevice::startAdvertising();
    }
};

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String receivedValue = pCharacteristic->getValue();
        if (receivedValue.length() > 0) {
            send = true;

            if (receivedValue == "SPACE_PRESSED") {
                power_bt = min_speed;
                power_up = min_speed;
                motor_1.write(power_up);
                motor_2.write(power_bt);
            } else if (receivedValue == "KP_PLUS") {
                Kp += 1;
            } else if (receivedValue == "KP_MINUS") {
                Kp = max(Kp - 1, 0.0);
            } else if (receivedValue == "KI_PLUS") {
                Ki += 0.01;
            } else if (receivedValue == "KI_MINUS") {
                Ki = max(Ki - 0.01, 0.0);
            } else if (receivedValue == "KD_PLUS") {
                Kd += 0.05;
            } else if (receivedValue == "KD_MINUS") {
                Kd = max(Kd - 0.05, 0.0);
            }
        } else {
            send = false;
        }
    }
};

void IRAM_ATTR isr_pwm1() {
    digitalRead(INPUT1_PIN) == HIGH ?
        pulseStartTime[0] = micros() :
        pulseWidth[0] = micros() - pulseStartTime[0];
}
void IRAM_ATTR isr_pwm2() {
    digitalRead(INPUT2_PIN) == HIGH ?
        pulseStartTime[1] = micros() :
        pulseWidth[1] = micros() - pulseStartTime[1];
}
void IRAM_ATTR isr_pwm3() {
    digitalRead(INPUT3_PIN) == HIGH ?
        pulseStartTime[2] = micros() :
        pulseWidth[2] = micros() - pulseStartTime[2];
}
void IRAM_ATTR isr_pwm4() {
    digitalRead(INPUT4_PIN) == HIGH ?
        pulseStartTime[3] = micros() :
        pulseWidth[3] = micros() - pulseStartTime[3];
}

void setup() {
    Serial.begin(115200);

    BLEDevice::init("ESP32_BLE");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pCharacteristic->setCallbacks(new MyCallbacks());
    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();
    BLEDevice::getAdvertising()->addServiceUUID(SERVICE_UUID);
    BLEDevice::startAdvertising();

    motor_1.attach(VSP_PIN1);
    motor_2.attach(VSP_PIN2);
    motor_1.write(power_up);
    motor_2.write(power_bt);

    Wire.begin();
    Wire.beginTransmission(MPU6050);
    Wire.write(PWR_MGMT_1);
    Wire.write(0);
    Wire.endTransmission(true);

    setpins();

    ledcAttach(MOTOR_X_PWM, 20000, 8);
    ledcAttach(MOTOR_Y_PWM, 20000, 8);

    prevTime = millis();
}

void loop() {
    yield(); // Reset WDT

    Wire.beginTransmission(MPU6050);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050, 14, true);
    int16_t AcX = Wire.read() << 8 | Wire.read();
    int16_t AcY = Wire.read() << 8 | Wire.read();
    int16_t AcZ = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();
    int16_t GyX = Wire.read() << 8 | Wire.read();
    int16_t GyY = Wire.read() << 8 | Wire.read();
    int16_t GyZ = Wire.read() << 8 | Wire.read();

    double ax = AcX / 16384.0, ay = AcY / 16384.0, az = AcZ / 16384.0;
    double gx = GyX / 131.0, gy = GyY / 131.0;
    double accRoll = atan2(ay, az) * 180.0 / PI;
    double accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

    unsigned long currTime = millis();
    double dt = (currTime - prevTime) / 1000.0;
    prevTime = currTime;

    roll = alpha * (roll + gx * dt) + (1 - alpha) * accRoll;
    pitch = alpha * (pitch + gy * dt) + (1 - alpha) * accPitch;
    inputX = roll;
    inputY = pitch;

    static double prevErrorX = 0, integralX = 0;
    static double prevErrorY = 0, integralY = 0;
    computePID(inputX, setpointX, outputX, Kp, Ki, Kd, prevErrorX, integralX);
    computePID(inputY, setpointY, outputY, Kp, Ki, Kd, prevErrorY, integralY);
    controlMotor(MOTOR_X_PWM, MOTOR_X_DIR, outputX);
    controlMotor(MOTOR_Y_PWM, MOTOR_Y_DIR, outputY);

    if (deviceConnected && millis() - lastSendTime > sendInterval) {
        char msg[128];
        snprintf(msg, sizeof(msg), "%.3d %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
                 power_up, Kp, Ki, Kd, accRoll, accPitch, roll, pitch);
        pCharacteristic->setValue(msg);
        pCharacteristic->notify();
        lastSendTime = millis();
    }

    // RC PWM處理
    for (int i = 0; i < 4; i++) {
        if (pulseWidth[i] > 0) {
            double pwmPercent = ((double)(pulseWidth[i] - PWMMIN[i]) / (PWMMAX[i] - PWMMIN[i])) * 100.0;
            pwmPercent = constrain(pwmPercent, 0, 100);
            
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "CH%d : %d", i + 1, pulseWidth[i]);
            Serial.println(buffer);
            
            if (i == 2) {
                power_up = min_speed + (int)pwmPercent;
                power_bt = min_speed + (int)pwmPercent;
                motor_1.write(power_up);
                motor_2.write(power_bt);
            }

            pulseWidth[i] = 0;
        }
    }

    delay(5); // 簡短 delay，避免 WDT
}
