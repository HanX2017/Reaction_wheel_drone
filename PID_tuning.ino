#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <Arduino.h>
#include <ESP32Servo.h>  // 引入 ESP32Servo 庫
#include <Wire.h>
//https://www.cnblogs.com/lnsane/p/13756430.html 腳位圖
//SCL TO 22 SDA TO 21
#define MPU6050_ADDR 0x68
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
String FORWARD = "FORWARD";|
String BACKWARD = "BACKWARD";
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
int send_time = 0;
int send_time_max = 100;
int min_speed = 22;
int tune = 3;
int power_up = min_speed;
int power_bt = min_speed;
// 補償濾波器係數
float alpha = 0.96;
// 儲存上一次的時間
unsigned long prevTime = 0;
// 初始角度
float roll = 0, pitch = 0;
// 目標角度 (保持水平)
double setpointX = 0, setpointY = 0;
// 當前角度
double inputX, inputY;
// 輸出到馬達的 PWM
double outputX, outputY;
// PID 參數 (P, I, D)
double Kpx = 8.5, Kix = 0.1, Kdx = Kpx/3;
double Kpy = 10.5, Kiy = 0.1, Kdy = Kpy/3;
// 馬達腳位義
const int MOTOR_X_PWM = 32;
const int MOTOR_X_DIR = 13;
const int MOTOR_X_BRAKE = 27;
const int MOTOR_Y_PWM = 16;
const int MOTOR_Y_DIR = 33;
const int MOTOR_Y_BRAKE = 4;
// X 軸馬達
const int VSP_PIN1 = 18;  // 第一個螺旋槳的 PWM 引腳
const int VSP_PIN2 = 19;  // 第二個螺旋槳的 PWM 引腳
// 連線狀態回調函式
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
    }
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        BLEDevice::startAdvertising(); // 重新開始廣播
    }
};
// 設定 BLE 讀寫特徵值
class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String receivedValue = pCharacteristic->getValue();
        if (receivedValue.length() > 0) {
            if (receivedValue == "SPACE_PRESSED") {
                power_bt = min_speed;
                power_up = min_speed;
                motor_1.write(power_up);
                motor_2.write(power_bt);
            }
            else if (receivedValue == UP) {
                if (power_up < 120) {
                    power_up += tune;
                }
                if (power_bt < 120) {
                    power_bt += tune;
                }
                if (power_up > 120) {
                    power_up = 120;
                }
                if (power_bt > 120) {
                    power_bt = 120;
                } 
            }
            else if (receivedValue == DOWN) {
                if (power_up > min_speed) {
                    power_up -= 2*tune;
                }
                if (power_bt > min_speed) {
                    power_bt -= 2*tune;
                }
                if (power_up < min_speed) {
                    power_up = min_speed;
                }
                if (power_bt < min_speed) {
                    power_bt = min_speed;
                }
            }
            else if (receivedValue == "KPX_PLUS") {
              Kpx += 1;
            }
            else if (receivedValue == "KPX_MINUS") {
              Kpx -= 1;
              if(Kpx<0){Kpx = 0;}
            }
            else if (receivedValue == "KIX_PLUS"){
              Kix += 0.01;
            }
            else if (receivedValue == "KIX_MINUS"){
              Kix -= 0.01;
              if(Kix<0){Kix = 0;}
            }
            else if (receivedValue == "KDX_PLUS"){
              Kdx += 0.05;
            }
            else if (receivedValue == "KDX_MINUS"){
              Kdx -= 0.05;
              if(Kdx<0){Kdx = 0;}
            }
            else if (receivedValue == "KPY_PLUS") {
              Kpy += 1;
            }
            else if (receivedValue == "KPY_MINUS") {
              Kpy -= 1;
              if(Kpy<0){Kpy = 0;}
            }
            else if (receivedValue == "KIY_PLUS"){
              Kiy += 0.01;
            }
            else if (receivedValue == "KIY_MINUS"){
              Kiy -= 0.01;
              if(Kiy<0){Kiy = 0;}
            }
            else if (receivedValue == "KDY_PLUS"){
              Kdy += 0.05;
            }
            else if (receivedValue == "KDY_MINUS"){
              Kdy -= 0.05;
              if(Kdy<0){Kdy = 0;}
            }
        }
    }
};

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
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    // 設定馬達腳位
    pinMode(MOTOR_X_PWM, OUTPUT);
    pinMode(MOTOR_X_DIR, OUTPUT);
    pinMode(MOTOR_X_BRAKE, OUTPUT);
    pinMode(MOTOR_Y_PWM, OUTPUT);
    pinMode(MOTOR_Y_DIR, OUTPUT);
    pinMode(MOTOR_Y_BRAKE, OUTPUT);
    digitalWrite(MOTOR_X_BRAKE, HIGH);
    digitalWrite(MOTOR_Y_BRAKE, HIGH);
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
void computePIDX(double input, double setpoint, double &output, double Kp, double Ki, double Kd, double &prevError, double &integral) {
  double error = setpoint - input;
  integral = constrain(integral + error, -1000, 1000); // 限制積分避免發散
  double derivative = error - prevError;
  output = constrain(Kp * error + Ki * integral + Kd * derivative, -255, 255);
  prevError = error;
}
void computePIDY(double input, double setpoint, double &output, double Kp, double Ki, double Kd, double &prevError, double &integral) {
  double error = setpoint - input;
  integral = constrain(integral + error, -1000, 1000); // 限制積分避免發散
  double derivative = error - prevError;
  output = constrain(Kp * error + Ki * integral + Kd * derivative, -255, 255);
  prevError = error;
}
void controlMotor(int pwmPin, int dirPin, double output) {
  int pwmValue = constrain(abs(output), 0, 255);
  // 如果 output 是正數，讓馬達正轉；負數則反轉
  if (dirPin == MOTOR_Y_DIR){
      digitalWrite(dirPin, output >= 0 ? LOW : HIGH);
  }
  else {
      digitalWrite(dirPin, output >= 0 ? HIGH : LOW);
  }
  // 輸出 PWM，將值反轉以符合要求
  ledcWrite(pwmPin, 255 - pwmValue); // 0 為停止，255 為全速
}
void loop() {
    // 讀取 MPU6050 數據
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true);
    int16_t AcX = Wire.read() << 8 | Wire.read();
    int16_t AcY = Wire.read() << 8 | Wire.read();
    int16_t AcZ = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); // 跳過溫度
    int16_t GyX = Wire.read() << 8 | Wire.read();
    int16_t GyY = Wire.read() << 8 | Wire.read();
    int16_t GyZ = Wire.read() << 8 | Wire.read();
    float ax = AcX / 16384.0, ay = AcY / 16384.0, az = AcZ / 16384.0;
    float gx = GyX / 131.0, gy = GyY / 131.0, gz = GyZ / 131.0;
    float accRoll  = atan2(ay, az) * 180.0 / PI;
    float accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
    unsigned long currTime = millis();
    float dt = (currTime - prevTime) / 1000.0;
    prevTime = currTime;
    roll  = alpha * (roll  + gx * dt) + (1 - alpha) * accRoll;
    pitch = alpha * (pitch + gy * dt) + (1 - alpha) * accPitch;
    inputX = roll;
    inputY = pitch;
    static double prevErrorX = 0, integralX = 0;
    static double prevErrorY = 0, integralY = 0;
    computePIDX(inputX, setpointX, outputX, Kpx, Kix, Kdx, prevErrorX, integralX);
    computePIDY(inputY, setpointY, outputY, Kpy, Kiy, Kdy, prevErrorY, integralY);
    controlMotor(MOTOR_X_PWM, MOTOR_X_DIR, outputX);
    controlMotor(MOTOR_Y_PWM, MOTOR_Y_DIR, outputY);
    if (deviceConnected) {
        if (send_time > send_time_max) {
            send_time = 0;
            
            // 預先創建一個足夠大的 buffer 來存儲 PID 數值
            char PIDS[1024]; // 假設最大需要 256 字節
            snprintf(PIDS, sizeof(PIDS), "%.3d %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f", power_up, Kpx, Kix, Kdx, Kpy, Kiy, Kdy, accRoll, accPitch, roll, pitch);
            
            pCharacteristic->setValue(PIDS);
            pCharacteristic->notify();
        }

        send_time++;
    }
    else {
        power_up = min_speed;
        power_bt = min_speed;
        motor_1.write(power_up);
        motor_2.write(power_bt);
    }
}
