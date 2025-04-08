#include <Wire.h>
#include <EEPROM.h>
#include <PWM.h> 

#define MPU6050       0x68         // Device address
#define ACCEL_CONFIG  0x1C         // Accelerometer configuration address
#define GYRO_CONFIG   0x1B         // Gyro configuration address
#define PWR_MGMT_1    0x6B
#define PWR_MGMT_2    0x6C

#define BRAKE         8 
#define PWM_X         9
#define PWM_Y         10
#define DIRECTION_X   4
#define DIRECTION_Y   3

#define BUZZER        12
#define VBAT          A7

float K1 = 70;
float K2 = 5.15;
float K3 = 0.035;
float loop_time = 10;

float alpha = 0.4; 

struct OffsetsObj {
  int ID;
  float X;
  float Y;
};

OffsetsObj offsets;

int pwm_X, pwm_Y = 0;
byte brake_t = 1;   // stabdis - '0 stop'

int32_t motor_speed_pwmX; 
int32_t motor_speed_pwmY;

uint32_t timer;
long currentT, previousT_1, previousT_2 = 0;  // laiko periodai

/* IMU Data */
int16_t  AcX, AcY, AcZ;
int32_t GyZ, GyX, GyY, gyroZ, gyroY;

//Sensor output scaling
#define accSens 0             // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1            // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s
#define Gyro_amount 0.996     //percent of gyro in complementary filter

//IMU offset values
int16_t  AcX_offset = -750;
int16_t  AcY_offset = 280;
int16_t  AcZ_offset = 100;
int16_t  GyZ_offset = 0;
int16_t  GyY_offset = 0;
int32_t  GyZ_offset_sum = 0;
int32_t  GyY_offset_sum = 0;

float robot_angleX, robot_angleY;
float angleX, angleY;
float Acc_angleX, Acc_angleY;
float gyroZfilt, gyroYfilt;

bool vertical = false;  
bool calibrating = false;
bool calibrated = false;

uint8_t i2cData[14]; // Buffer for I2C data

void battVoltage(double voltage) {
  //Serial.print("batt: "); Serial.println(voltage); 
  if (voltage > 8 && voltage <= 9.5) {
    digitalWrite(BUZZER, HIGH);
  } else {
    digitalWrite(BUZZER, LOW);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);
  pinMode(BRAKE, OUTPUT);
  pinMode(DIRECTION_X, OUTPUT);
  pinMode(DIRECTION_Y, OUTPUT);
  InitTimersSafe();
  SetPinFrequencySafe(PWM_X, 20000);
  SetPinFrequencySafe(PWM_Y, 20000);
  pwmWrite(PWM_X, 255);
  pwmWrite(PWM_Y, 255);
  digitalWrite(BRAKE, HIGH);
  delay(1000);
  EEPROM.get(0, offsets);
  if (offsets.ID == 11) calibrated = true;
    else calibrated = false;
  Serial.println("Calibrating gyroscope...");
  angle_setup();
}

void loop() {

  currentT = millis();

  if (currentT - previousT_1 >= loop_time) {

    Tuning(); 
    angle_calc();

    if (vertical && calibrated) {
      digitalWrite(BRAKE, HIGH);
      gyroZ = GyZ / 131.0; // Convert to deg/s
      gyroY = GyY / 131.0; // Convert to deg/s

      gyroZfilt = alpha * gyroZ + (1 - alpha) * gyroZfilt;
      gyroYfilt = alpha * gyroY + (1 - alpha) * gyroYfilt;
	  
      pwm_X = constrain(K1 * angleX + K2 * gyroZfilt + K3 * motor_speed_pwmX, -255, 255); 
      pwm_Y = constrain(K1 * angleY + K2 * gyroYfilt + K3 * motor_speed_pwmY, -255, 255); 
      
      if (!calibrating) {
        Motor_controlX(pwm_X);
        motor_speed_pwmX += pwm_X;
        Motor_controlY(pwm_Y);
        motor_speed_pwmY += pwm_Y;
      } else {
          Motor_controlX(0);
          Motor_controlY(0);
      }
      previousT_1 = currentT;
    } else {
      Motor_controlX(0);
      Motor_controlY(0);
      digitalWrite(BRAKE, LOW);
      motor_speed_pwmX = 0;
      motor_speed_pwmY = 0;
    }
  }
  if (currentT - previousT_2 >= 2000) {
    battVoltage((double)analogRead(VBAT) / 38.4); // This is then connected to a 47k-12k voltage divider
	if (!calibrated && !calibrating) {
      Serial.println("first you need to calibrate the balancing point...");
    }
    previousT_2 = currentT;
  }
}

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

//setup MPU6050
void angle_setup() {
  Wire.begin();
  delay (100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay (100);

  // calc Y gyro offset by averaging 1024 values
  for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyZ_offset_sum += GyZ;
    delay(5);
  }
  GyZ_offset = GyZ_offset_sum >> 10;
  Serial.print("GyZ offset value = "); Serial.println(GyZ_offset);

  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
  
  for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyY_offset_sum += GyY;
    delay(5);
  }
  GyY_offset = GyY_offset_sum >> 10;
  Serial.print("GyY offset value = "); Serial.println(GyY_offset);
  
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
  delay(80);
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
}

void angle_calc() {
  
  Wire.beginTransmission(MPU6050);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);  
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);  
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  AcX += AcX_offset;
  AcY += AcY_offset;  
  AcZ += AcZ_offset;
  GyZ -= GyZ_offset;
  GyY -= GyY_offset;

  robot_angleX += GyZ * loop_time / 1000 / 65.536; 
  Acc_angleX = atan2(AcY, -AcX) * 57.2958;               // angle from acc. values  * 57.2958 (deg/rad)
  robot_angleX = robot_angleX * Gyro_amount + Acc_angleX * (1.0 - Gyro_amount);

  robot_angleY += GyY * loop_time / 1000 / 65.536;   
  Acc_angleY = -atan2(AcZ, -AcX) * 57.2958;              //angle from acc. values  * 57.2958 (deg/rad)
  robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0 - Gyro_amount);
  
  angleX = robot_angleX - offsets.X;
  angleY = robot_angleY - offsets.Y;
  
  if (abs(angleX) > 6 || abs(angleY) > 6) vertical = false;
  if (abs(angleX) < 0.3 && abs(angleY) < 0.3) vertical = true;

  //Serial.print("AngleX: "); Serial.print(angleX); Serial.print(" AngleY: "); Serial.println(angleY);
}

void Motor_controlX(int pwm) {
  if (pwm < 0) {
    digitalWrite(DIRECTION_X, LOW);
    pwm = -pwm;
  } else {
    digitalWrite(DIRECTION_X, HIGH);
  }
  pwmWrite(PWM_X, pwm > 255 ? 255 : 255 - pwm);
}

void Motor_controlY(int pwm) {
  if (pwm < 0) {
    digitalWrite(DIRECTION_Y, LOW);
    pwm = -pwm;
  } else {
    digitalWrite(DIRECTION_Y, HIGH);
  }
  pwmWrite(PWM_Y, pwm > 255 ? 255 : 255 - pwm);
}

int Tuning() {
  if (!Serial.available())  return 0;
  delay(2);
  char param = Serial.read();               // get parameter byte
  if (!Serial.available()) return 0;
  char cmd = Serial.read();                 // get command byte
  Serial.flush();
  switch (param) {
    case 'p':
      if (cmd == '+')    K1 += 1;
      if (cmd == '-')    K1 -= 1;
      printValues();
      break;
	case 'i':
      if (cmd == '+')    K2 += 0.01;
      if (cmd == '-')    K2 -= 0.01;
      printValues();
      break;  
    case 's':
      if (cmd == '+')    K3 += 0.005;
      if (cmd == '-')    K3 -= 0.005;
      printValues();
      break;  
    case 'c':
      if (cmd == '+' && !calibrating) {
		 calibrating = true;
         Serial.println("calibrating on");
      }
      if (cmd == '-' && calibrating)  {
        calibrated = true;
        calibrating = false;
        Serial.println("calibrating off");
        Serial.print("X: "); Serial.print(robot_angleX); Serial.print(" Y: "); Serial.println(robot_angleY);
        if (abs(robot_angleX) < 15 && abs(robot_angleY) < 15) {
		      offsets.ID = 11;
          offsets.X = robot_angleX;
          offsets.Y = robot_angleY;
          EEPROM.put(0, offsets);
          digitalWrite(BUZZER, HIGH);
          delay(70);
          digitalWrite(BUZZER, LOW);
        } else {
          Serial.println("The angles are wrong!!!");
          digitalWrite(BUZZER, HIGH);
          delay(50);
          digitalWrite(BUZZER, LOW);
          delay(70);
          digitalWrite(BUZZER, HIGH);
          delay(50);
          digitalWrite(BUZZER, LOW);
        }
      }
      break;         
   }
}

void printValues() {
  Serial.print("K1: "); Serial.print(K1);
  Serial.print(" K2: "); Serial.print(K2);
  Serial.print(" K3: "); Serial.println(K3,3);
}