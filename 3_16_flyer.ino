// 假設已知離線計算好的 K 矩陣，這裡直接以二維陣列表示
float K[2][4] = {
    {K11, K12, K13, K14},
    {K21, K22, K23, K24}
};

// 讀取狀態的函式（需根據你的感測器實作）
void readState(float* x) {
    x[0] = readAngleX();      // phi
    x[1] = readAngleY();      // theta
    x[2] = readAngularVelX(); // phi_dot
    x[3] = readAngularVelY(); // theta_dot
}

// 將控制信號 u 映射到 PWM 值的函式
int mapControlToPWM(float u, float u_min, float u_max) {
    // 假設 PWM 輸出範圍 0 - 255
    // 若 u 為負數，可能需要設置方向腳位，這裡僅示意 PWM 值部分
    u = constrain(u, u_min, u_max);
    return (int)map(u, u_min, u_max, 0, 255);
}

void loop() {
    float x[4];
    readState(x);

    // 計算控制輸入 u = -Kx
    float u[2] = {0.0, 0.0};
    for (int i = 0; i < 4; i++) {
        u[0] -= K[0][i] * x[i];
        u[1] -= K[1][i] * x[i];
    }

    // 根據控制信號 u 計算 PWM 佔空比
    // 假設 u 的範圍為 [-10, 10]
    int pwmDuty1 = mapControlToPWM(u[0], -10, 10);
    int pwmDuty2 = mapControlToPWM(u[1], -10, 10);

    // 將 PWM 輸出到對應通道
    ledcWrite(pwmChannel1, pwmDuty1);
    ledcWrite(pwmChannel2, pwmDuty2);

    // 進行控制迴圈延時，確保更新速率符合系統需求
    delay(10); // 例如 10 毫秒
}
