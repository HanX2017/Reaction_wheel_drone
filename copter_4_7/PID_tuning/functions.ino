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

            send = true;

            if (receivedValue == "SPACE_PRESSED") {
                power_bt = min_speed;
                power_up = min_speed;
                motor_1.write(power_up);
                motor_2.write(power_bt);
            }

            else if (receivedValue == "KP_PLUS") {
              Kp += 1;
            }
            else if (receivedValue == "KP_MINUS") {
              Kp -= 1;
              if(Kp<0){Kp = 0;}
            }
            else if (receivedValue == "KI_PLUS"){
              Ki += 0.01;
            }
            else if (receivedValue == "KI_MINUS"){
              Ki -= 0.01;
              if(Ki<0){Ki = 0;}
            }
            else if (receivedValue == "KD_PLUS"){
              Kd += 0.05;
            }
            else if (receivedValue == "KD_MINUS"){
              Kd -= 0.05;
              if(Kd<0){Kd = 0;}
            }
        }
        else{
            send = false;
        }
    }
};

void IRAM_ATTR isr_pwm1() {
  if (digitalRead(INPUT1_PIN) == HIGH) {
    pulseStartTime[0] = micros();
  } else {
    pulseEndTime[0] = micros();
    pulseWidth[0] = pulseEndTime[0] - pulseStartTime[0];
  }
}

void IRAM_ATTR isr_pwm2() {
  if (digitalRead(INPUT2_PIN) == HIGH) {
    pulseStartTime[1] = micros();
  } else {
    pulseEndTime[1] = micros();
    pulseWidth[1] = pulseEndTime[1] - pulseStartTime[1];
  }
}

void IRAM_ATTR isr_pwm3() {
  if (digitalRead(INPUT3_PIN) == HIGH) {
    pulseStartTime[2] = micros();
  } else {
    pulseEndTime[2] = micros();
    pulseWidth[2] = pulseEndTime[2] - pulseStartTime[2];
  }
}

void IRAM_ATTR isr_pwm4() {
  if (digitalRead(INPUT4_PIN) == HIGH) {
    pulseStartTime[3] = micros();
  } else {
    pulseEndTime[3] = micros();
    pulseWidth[3] = pulseEndTime[3] - pulseStartTime[3];
  }
}
/*
void IRAM_ATTR isr_pwm5() {
  if (digitalRead(INPUT5_PIN) == HIGH) {
    pulseStartTime[4] = micros();
  } else {
    pulseEndTime[4] = micros();
    pulseWidth[4] = pulseEndTime[4] - pulseStartTime[4];
  }
}

void IRAM_ATTR isr_pwm6() {
  if (digitalRead(INPUT6_PIN) == HIGH) {
    pulseStartTime[5] = micros();
  } else {
    pulseEndTime[5] = micros();
    pulseWidth[5] = pulseEndTime[5] - pulseStartTime[5];
  }
}
*/


void computePID(double input, double setpoint, double &output, double Kp, double Ki, double Kd, double &prevError, double &integral) {
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
