#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <Arduino.h>
#include <ESP32Servo.h>  // 引入 ESP32Servo 庫
#include <Wire.h>

int PWMMAX[6] = {1860, 1870, 2000, 1960, 0, 0};
int PWMMIN[6] = {1130, 1110, 990, 1000, 0, 0};

//https://www.cnblogs.com/lnsane/p/13756430.html 腳位圖
#define INPUT1_PIN  34  
#define INPUT2_PIN  35  
#define INPUT3_PIN  32
#define INPUT4_PIN  33  
/*
#define INPUT5_PIN  
#define INPUT6_PIN    
*/
#define MOTOR_X_PWM 5;//改配線
#define MOTOR_X_DIR 13;
#define MOTOR_X_BRAKE 27;
#define MOTOR_Y_PWM 16;
#define MOTOR_Y_DIR 7;
#define MOTOR_Y_BRAKE 27;

#define VSP_PIN1 18;  // 第一個螺旋槳的 PWM 引腳
#define VSP_PIN2 19;  // 第二個螺旋槳的 PWM 引腳

//SCL TO 22 SDA TO 21
#define MPU6050       0x68         // Device address
#define ACCEL_CONFIG  0x1C         // Accelerometer configuration address
#define GYRO_CONFIG   0x1B         // Gyro configuration address
#define PWR_MGMT_1    0x6B
#define PWR_MGMT_2    0x6C
// 定義 BLE 服務與特徵值
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "87654321-4321-4321-4321-cba987654321"

Servo motor_1;
Servo motor_2;

String UP = "UP";
String DOWN = "DOWN";
String LEFT_PITCH = "LEFT_PITCH";
String RIGHT_PITCH = "RIGHT_PITCH";     
String LEFT = "LEFT";
String RIGHT = "RIGHT";
String FORWARD = "FORWARD";
String BACKWARD = "BACKWARD";

BLECharacteristic *pCharacteristic;
bool send = false; // notify python
bool deviceConnected = false;
int send_time = 0;
int send_time_max = 100;

int min_speed = 22;
int power_up = min_speed;
int power_bt = min_speed;

// 補償濾波器係數
double alpha = 0.96;
// 儲存上一次的時間
unsigned long prevTime = 0;
// 初始角度
double roll = 0, pitch = 0;
// 目標角度 (保持水平)
double setpointX = 0, setpointY = 0;
// 當前角度
double inputX, inputY;
// 輸出到馬達的 PWM
double outputX, outputY;
// PID 參數 (P, I, D)
double Kp = 0, Ki = 0, Kd = 0;

long long pulseStartTime[6] = {0};  
long long pulseEndTime[6] = {0};    
long long pulseWidth[6] = {0};      

void setpins(){
    // 設定馬達腳位
    pinMode(MOTOR_X_PWM, OUTPUT);
    pinMode(MOTOR_X_DIR, OUTPUT);
    pinMode(MOTOR_X_BRAKE, OUTPUT);
    pinMode(MOTOR_Y_PWM, OUTPUT);
    pinMode(MOTOR_Y_DIR, OUTPUT);
    pinMode(MOTOR_Y_BRAKE, OUTPUT);
    digitalWrite(MOTOR_X_BRAKE, HIGH);
    digitalWrite(MOTOR_Y_BRAKE, HIGH);
    
    pinMode(INPUT1_PIN, INPUT);
    pinMode(INPUT2_PIN, INPUT);
    pinMode(INPUT3_PIN, INPUT);
    pinMode(INPUT4_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(INPUT1_PIN), isr_pwm1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(INPUT2_PIN), isr_pwm2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(INPUT3_PIN), isr_pwm3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(INPUT4_PIN), isr_pwm4, CHANGE);
    /*
    pinMode(INPUT5_PIN, INPUT);
    pinMode(INPUT6_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(INPUT5_PIN), isr_pwm5, CHANGE);
    attachInterrupt(digitalPinToInterrupt(INPUT6_PIN), isr_pwm6, CHANGE);
    */

}

void setup() {
    BLEDevice::init("ESP32_BLE");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_INDICATE
    );
    pCharacteristic->setCallbacks(new MyCallbacks());
    pCharacteristic->setValue("");
    // **確保 BLE 特徵值允許 notify**
    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pServer->getAdvertising()->start();
    Serial.println("ESP32 BLE 伺服器已啟動");
    Serial.begin(115200);
    // 初始化 MPU6050
    Wire.begin();
    Wire.beginTransmission(MPU5060);
    Wire.write(0x68);
    Wire.write(0);
    Wire.endTransmission(true);

    setpins()

    // 設定 PWM 頻率與頻道
    ledcAttach(MOTOR_X_PWM, 20000, 8);
    ledcAttach(MOTOR_Y_PWM, 20000, 8);
    prevTime = millis();
    motor_1.attach(VSP_PIN1);
    motor_2.attach(VSP_PIN2);
    power_up = min_speed;
    power_bt = min_speed;
    motor_1.write(power_up);
    motor_2.write(power_bt);
}
void loop() {
    // 讀取 MPU6050 數據
    Wire.beginTransmission(MPU6050);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050, 14, true);
    int16_t AcX = Wire.read() << 8 | Wire.read();
    int16_t AcY = Wire.read() << 8 | Wire.read();
    int16_t AcZ = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); // 跳過溫度
    int16_t GyX = Wire.read() << 8 | Wire.read();
    int16_t GyY = Wire.read() << 8 | Wire.read();
    int16_t GyZ = Wire.read() << 8 | Wire.read();
    double ax = AcX / 16384.0, ay = AcY / 16384.0, az = AcZ / 16384.0;
    double gx = GyX / 131.0, gy = GyY / 131.0, gz = GyZ / 131.0;
    double accRoll  = atan2(ay, az) * 180.0 / PI;
    double accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
    unsigned long currTime = millis();
    double dt = (currTime - prevTime) / 1000.0;
    prevTime = currTime;
    roll  = alpha * (roll  + gx * dt) + (1 - alpha) * accRoll;
    pitch = alpha * (pitch + gy * dt) + (1 - alpha) * accPitch;
    inputX = roll;
    inputY = pitch;
    static double prevErrorX = 0, integralX = 0;
    static double prevErrorY = 0, integralY = 0;
    computePID(inputX, setpointX, outputX, Kp, Ki, Kd, prevErrorX, integralX);
    computePID(inputY, setpointY, outputY, Kp, Ki, Kd, prevErrorY, integralY);
    controlMotor(MOTOR_X_PWM, MOTOR_X_DIR, outputX);
    controlMotor(MOTOR_Y_PWM, MOTOR_Y_DIR, outputY);
    if (deviceConnected) {
        if (send) {
            // 預先創建一個足夠大的 buffer 來存儲 PID 數值
            char PIDS[1024]; // 假設最大需要 256 字節
            snprintf(PIDS, sizeof(PIDS), "%.3d %.2f %.2f %.2f %.2f %.2f %.2f %.2f", power_up, Kp, Ki, Kd, accRoll, accPitch, roll, pitch);
            
            pCharacteristic->setValue(PIDS);
            pCharacteristic->notify();
        }
        /*
        if (send_time > send_time_max) {
            send_time = 0;
            
            // 預先創建一個足夠大的 buffer 來存儲 PID 數值
            char PIDS[1024]; // 假設最大需要 256 字節
            snprintf(PIDS, sizeof(PIDS), "%.3d %.2f %.2f %.2f %.2f %.2f %.2f %.2f", power_up, Kp, Ki, Kd, accRoll, accPitch, roll, pitch);
            
            pCharacteristic->setValue(PIDS);
            pCharacteristic->notify();
        }
        send_time++;
        */
    }
    for (int i = 0; i < 6; i++) {
        if (pulseWidth[i] > 0) {
            double pwmPercent = ((double)(pulseWidth[i] - PWMMIN[i]) / (PWMMAX[i] - PWMMIN[i])) * 100.0;
            pwmPercent = constrain(pwmPercent, 0, 100);  // 限制範圍在 0% ~ 100%

            if (i+1 == 3){
                motor_1.write(power_up+(int)pwmPercent);
                motor_2.write(power_bt+(int)pwmPercent);
                Serial.print(power_up+(int)pwmPercent);
                Serial.print("\n");
            }      
            Serial.print(i+1);
            Serial.print("  :  ");
            Serial.print(pwmPercent, 2);  // 顯示兩位小數
            Serial.print("\n");

            pulseWidth[i] = 0;  // 重置 PWM 值
        }
    }
    //delay(10);
}
