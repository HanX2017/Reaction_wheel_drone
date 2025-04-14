#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include "driver/gpio.h"

#define INPUT1_PIN  34  
#define INPUT2_PIN  35  
#define INPUT3_PIN  32
#define INPUT4_PIN  33  
#define MOTOR_X_PWM 25
#define MOTOR_X_DIR 26
#define MOTOR_X_BRAKE 27
#define MOTOR_Y_PWM 4
#define MOTOR_Y_DIR 16
#define MOTOR_Y_BRAKE 17
#define VSP_PIN1 18
#define VSP_PIN2 19
#define MPU6050 0x68
#define PWR_MGMT_1 0x6B

#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "87654321-4321-4321-4321-cba987654321"

#define DEFINE_ISR_PWM(idx, pin) \
    void IRAM_ATTR isr_pwm##idx() { \
        if (gpio_get_level((gpio_num_t)pin)) { \
            pulseStartTime[idx - 1] = micros(); \
        } else { \
            pulseWidth[idx - 1] = micros() - pulseStartTime[idx - 1]; \
        } \
    }



Servo motor_1, motor_2;
BLECharacteristic *pCharacteristic;

bool deviceConnected = false;
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
double ax, ay, az, gx, gy, accRoll, accPitch;
double roll_er, pitch_er;
double angle = 5, angle_axies = angle/1.414, roll_act = 0, pitch_act = 0;

long long pulseStartTime[4] = {0};  
long long pulseEndTime[4] = {0};    
long long pulseWidth[4] = {0};      
double pwmPercent[4] = {0};
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
                Kd += 1;
            } else if (receivedValue == "KD_MINUS") {
                Kd = max(Kd - 1, 0.0);
            }
        }
    }
};

DEFINE_ISR_PWM(1, INPUT1_PIN)
DEFINE_ISR_PWM(2, INPUT2_PIN)
DEFINE_ISR_PWM(3, INPUT3_PIN)
DEFINE_ISR_PWM(4, INPUT4_PIN)

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
    controlMotor(MOTOR_X_PWM, MOTOR_X_DIR, 0);
    controlMotor(MOTOR_Y_PWM, MOTOR_Y_DIR, 0);

    angle_reset();


    prevTime = millis();
}

void loop() {
    yield(); // Reset WDT
    roll_act = 0;pitch_act = 0;
    if (millis() - lastSendTime > sendInterval){
        //printf("roll: %.2f, pitch: %.2f, %2f, %2f, %2f, %2f\n", inputX, inputY, roll, pitch, roll_er, pitch_er);
        printf("x: %.2f, y: %.2f\n", outputX, outputY);

        if(deviceConnected){
            char msg[128];
            snprintf(msg, sizeof(msg), "PWR=%d,KP=%.2f,KI=%.2f,KD=%.2f,ACCX=%.2f,ACCY=%.2f,ROLL=%.2f,PITCH=%.2f",
                    power_up, Kp, Ki, Kd, accRoll, accPitch, roll, pitch);
            pCharacteristic->setValue(msg);
            pCharacteristic->notify();
            lastSendTime = millis();
        }
    }

    // RC PWM處理
    for (int i = 0; i < 4; i++) {
        if (pulseWidth[i] > 0) {
            double new_pwmPercent = ((double)(pulseWidth[i] - PWMMIN[i]) / (PWMMAX[i] - PWMMIN[i])) * 100.0;
            if ((new_pwmPercent-pwmPercent[i])>10){
              new_pwmPercent = pwmPercent[i]+10;
            }
            new_pwmPercent = constrain(pwmPercent, 0, 100);
            pwm_Percent[i] = new_pwmPercent;
            
            //char buffer[128];
            //snprintf(buffer, sizeof(buffer), "CH%d : %lld", i + 1, pulseWidth[i]);
            //Serial.println(buffer);
            
            if (i == 2) {
                power_up = min_speed + (int)pwmPercent;
                power_bt = min_speed + (int)pwmPercent;
                motor_1.write(power_up);
                motor_2.write(power_bt);
            }
            else if (i == 3){
                if (pwmPercent>60){
                    power_up = max(power_up*0.9, min_speed);
                    motor_1.write(power_up);
                }
                else if (pwmPercent<40){
                    power_bt = max(power_bt*0.9, min_speed);
                    motor_2.write(power_bt);
                }
            }
            else if (i == 0){
                if (pwmPercent>60){
                    roll_act = angle_axies;
                    pitch_act = angle_axies;
                }
                else if (pwmPercent<40){
                    roll_act = -1*angle_axies;
                    pitch_act = -1*angle_axies;
                }
            }
            else if (i == 1){
                if (pwmPercent>60){
                    roll_act = -1*angle_axies;
                    pitch_act = angle_axies;
                }
                else if (pwmPercent<40){
                    roll_act = angle_axies;
                    pitch_act = -1*angle_axies;
                }
            }

            pulseWidth[i] = 0;
        }
    }

    angle_read();
    inputX = roll-roll_er+roll_act;
    inputY = pitch-pitch_er+pitch_act;

    static double prevErrorX = 0, integralX = 0;
    static double prevErrorY = 0, integralY = 0;
    computePID(inputX, setpointX, outputX, Kp, Ki, Kd, prevErrorX, integralX);
    computePID(inputY, setpointY, outputY, Kp, Ki, Kd, prevErrorY, integralY);
    controlMotor(MOTOR_X_PWM, MOTOR_X_DIR, outputX);
    controlMotor(MOTOR_Y_PWM, MOTOR_Y_DIR, outputY);    

    delay(5); // 簡短 delay，避免 WDT
}
