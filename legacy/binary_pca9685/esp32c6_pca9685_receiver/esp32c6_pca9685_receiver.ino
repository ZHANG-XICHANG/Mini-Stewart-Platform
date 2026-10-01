/*
 * ESP32-C6 Stewart Platform Controller
 * 接收視覺化程式的伺服角度並通過 PCA9685 控制 6 個舵機
 * 
 * 硬體連接：
 * ESP32-C6 <-> PCA9685
 * GPIO22 (SDA) <-> SDA
 * GPIO23 (SCL) <-> SCL
 * 3.3V <-> VCC
 * GND <-> GND
 * 
 * 舵機連接到 PCA9685 的通道 0-5
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9685 設置
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);  // I2C 地址 0x40

// 舵機參數設置
#define SERVO_FREQ 50  // 舵機 PWM 頻率 50Hz

// PCA9685 脈衝寬度設置（12位分辨率，4096步）
// 對於 50Hz：20ms 週期，4096 步
// 1ms = 205 步，2ms = 410 步

// 選項 1：使用 ±60° 系統（與視覺化程式匹配）
#define SERVOMIN  150  // 最小脈衝寬度（對應 -60°）大約 0.73ms
#define SERVOMID  307  // 中間脈衝寬度（對應 0°）大約 1.5ms
#define SERVOMAX  464  // 最大脈衝寬度（對應 +60°）大約 2.27ms

// 選項 2：使用標準 0-180° 系統（取消下面的註釋並註釋掉上面的）
// #define SERVOMIN  102  // 最小脈衝寬度（對應 0°）大約 0.5ms
// #define SERVOMID  307  // 中間脈衝寬度（對應 90°）大約 1.5ms
// #define SERVOMAX  512  // 最大脈衝寬度（對應 180°）大約 2.5ms
// 注意：如果使用此選項，需要修改 angleToPulse() 函數

// I2C 引腳定義（ESP32-C6）
#define I2C_SDA 22
#define I2C_SCL 23

// 串口配置
#define SERIAL_BAUD 115200

// 接收緩衝區
uint16_t receivedValues[6] = {2047, 2047, 2047, 2047, 2047, 2047};  // 預設中心位置
float servoAngles[6] = {0, 0, 0, 0, 0, 0};  // 當前角度（度）

// 平滑控制參數
float currentAngles[6] = {0, 0, 0, 0, 0, 0};
const float SMOOTH_FACTOR = 0.2;  // 平滑係數（0-1，越小越平滑）

// 通道角度反轉設置（true = 反轉角度）
bool invertChannel[6] = {true, false, true, false, true, false};  // Ch0,2,4 反轉

void setup() {
  // 初始化串口
  Serial.begin(SERIAL_BAUD);
  Serial.println("ESP32-C6 Stewart Platform Controller Starting...");
  
  // 初始化 I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // 初始化 PCA9685
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  
  delay(100);
  
  // 將所有舵機設置到中心位置
  Serial.println("Setting servos to center position...");
  for (int i = 0; i < 6; i++) {
    setServoPulse(i, SERVOMID);
  }
  
  Serial.println("Ready to receive commands!");
  Serial.println("Waiting for data from visualizer...");
}

void loop() {
  // 檢查是否有數據可讀（6個 uint16_t = 12 bytes）
  if (Serial.available() >= 12) {
    // 讀取 12 bytes（6 個 uint16_t，大端序）
    uint8_t buffer[12];
    Serial.readBytes(buffer, 12);
    
    // 解析數據（大端序）
    for (int i = 0; i < 6; i++) {
      receivedValues[i] = (buffer[i*2] << 8) | buffer[i*2 + 1];
    }
    
    // 轉換為角度並更新舵機
    updateServos();
    
    // 發送 DEBUG 回饋（可選）
    sendDebugData();
  }
  
  delay(20);  // 50Hz 更新頻率
}

/**
 * 將接收的數值轉換為角度並更新舵機
 */
void updateServos() {
  for (int i = 0; i < 6; i++) {
    // 轉換 0-4095 數值為角度（±60°）
    // 公式：angle = (value - 2047) * 120 / 4095
    float targetAngle = (receivedValues[i] - 2047) * 120.0 / 4095.0;
    
    // 限制角度範圍
    targetAngle = constrain(targetAngle, -60.0, 60.0);
    
    // 反轉特定通道的角度（如果需要）
    if (invertChannel[i]) {
      targetAngle = -targetAngle;
    }
    
    // 平滑過渡（可選，移除此行可獲得即時響應）
    currentAngles[i] = currentAngles[i] * (1.0 - SMOOTH_FACTOR) + targetAngle * SMOOTH_FACTOR;
    
    // 更新角度
    servoAngles[i] = currentAngles[i];
    
    // 轉換為 PCA9685 脈衝寬度
    uint16_t pulseWidth = angleToPulse(currentAngles[i]);
    
    // 設置舵機
    setServoPulse(i, pulseWidth);
  }
}

/**
 * 將角度（-60到+60度）轉換為 PCA9685 脈衝寬度
 */
uint16_t angleToPulse(float angle) {
  // 線性映射：-60° -> SERVOMIN, 0° -> SERVOMID, +60° -> SERVOMAX
  if (angle < 0) {
    // -60° 到 0°
    return SERVOMID + (angle / 60.0) * (SERVOMID - SERVOMIN);
  } else {
    // 0° 到 +60°
    return SERVOMID + (angle / 60.0) * (SERVOMAX - SERVOMID);
  }
}

/**
 * 設置指定通道的舵機脈衝寬度
 */
void setServoPulse(uint8_t channel, uint16_t pulse) {
  pwm.setPWM(channel, 0, pulse);
}

/**
 * 發送 DEBUG 數據回視覺化程式（可選）
 */
void sendDebugData() {
  static unsigned long lastDebugTime = 0;
  unsigned long currentTime = millis();
  
  // 每 100ms 發送一次
  if (currentTime - lastDebugTime > 100) {
    lastDebugTime = currentTime;
    
    // 格式：DEBUG,timestamp,x,y,z,roll,pitch,yaw,s1,s2,s3,s4,s5,s6
    Serial.print("DEBUG,");
    Serial.print(currentTime);
    Serial.print(",0.00,0.00,0.00,0.00,0.00,0.00");  // 位置和姿態（暫時為 0）
    
    // 發送當前 6 個舵機角度
    for (int i = 0; i < 6; i++) {
      Serial.print(",");
      Serial.print(servoAngles[i], 2);
    }
    Serial.println();
  }
}

/**
 * 舵機校準函數（用於測試）
 * 可以在 setup() 中調用來校準舵機的最小、中間、最大位置
 */
void calibrateServos() {
  Serial.println("\n=== Servo Calibration Mode ===");
  Serial.println("Testing SERVOMIN position (-60 degrees)...");
  
  for (int i = 0; i < 6; i++) {
    setServoPulse(i, SERVOMIN);
  }
  delay(2000);
  
  Serial.println("Testing SERVOMID position (0 degrees)...");
  for (int i = 0; i < 6; i++) {
    setServoPulse(i, SERVOMID);
  }
  delay(2000);
  
  Serial.println("Testing SERVOMAX position (+60 degrees)...");
  for (int i = 0; i < 6; i++) {
    setServoPulse(i, SERVOMAX);
  }
  delay(2000);
  
  Serial.println("Returning to center...");
  for (int i = 0; i < 6; i++) {
    setServoPulse(i, SERVOMID);
  }
  
  Serial.println("Calibration complete!");
}
