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



