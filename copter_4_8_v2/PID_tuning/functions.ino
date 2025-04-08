void computePID(double input, double setpoint, double &output, double Kp, double Ki, double Kd, double &prevError, double &integral) {
    double error = setpoint - input;
    integral += error;
    double derivative = error - prevError;
    output = Kp * error + Ki * integral + Kd * derivative;
    prevError = error;
}

void controlMotor(int pwmPin, int dirPin, double value) {
    int pwmVal = constrain(abs(value), 0, 255);
    digitalWrite(dirPin, value > 0 ? HIGH : LOW);
    ledcWrite(pwmPin, pwmVal);
}

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

void angle_read(){
    Wire.beginTransmission(MPU6050);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050, 12, true);
    int16_t AcX = Wire.read() << 8 | Wire.read();
    int16_t AcY = Wire.read() << 8 | Wire.read();
    int16_t AcZ = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();
    int16_t GyX = Wire.read() << 8 | Wire.read();
    int16_t GyY = Wire.read() << 8 | Wire.read();

    ax = AcX / 16384.0, ay = AcY / 16384.0, az = AcZ / 16384.0;
    gx = GyX / 131.0, gy = GyY / 131.0;
    accRoll = atan2(ay, az) * 180.0 / PI;
    accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

    unsigned long currTime = millis();
    double dt = (currTime - prevTime) / 1000.0;
    prevTime = currTime;

    roll = alpha * (roll + gx * dt) + (1 - alpha) * accRoll;
    pitch = alpha * (pitch + gy * dt) + (1 - alpha) * accPitch;
}

void angle_reset(){
    double roll_sum = 0, pitch_sum = 0;
    for (int i = 0; i<1024; i++){
        angle_read();
        roll_sum+=roll;
        pitch_sum+=pitch;
        delay(5);
    }
    roll_er = roll_sum/1024;
    pitch_er = pitch_sum/1024;
}

