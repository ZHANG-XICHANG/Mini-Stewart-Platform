/**
 * ESP32-C6 + PCA9685 舵機控制器（含逆運動學）
 * 
 * 新協議：接收平台位置和姿態，自動計算舵機角度
 * 格式：$P,x,y,z,roll,pitch,yaw,checksum
 * 
 * 優點：
 * - Python 端只需發送 6 個浮點數
 * - ESP32 自主計算 IK
 * - 降低通訊負擔
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "stewart_kinematics.h"

// =========================
// PCA9685 設定
// =========================
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// ESP32-C6 I2C 腳位
constexpr int I2C_SDA = 22;
constexpr int I2C_SCL = 23;

// PCA9685 更新頻率
constexpr float SERVO_FREQ = 330.0f;

// PCA9685 校正參數
constexpr float PCA_CALIBRATION_OFFSET = 1.7f;

// 靜默模式（高頻運行時關閉回應以節省頻寬）
bool verbose_mode = false;  // 預設靜默
constexpr float PCA_CALIBRATION_SCALE = 0.935f;
constexpr bool ENABLE_CALIBRATION = true;

// 舵機數量
constexpr uint8_t SERVO_COUNT = 6;

// 通道偏移校正（微秒）
constexpr int16_t CHANNEL_OFFSET[SERVO_COUNT] = {
  // 0, 50, 50, -80, 100, 0
  80, 100, 100, 0, 100, 50
};
constexpr bool ENABLE_CHANNEL_OFFSET = true;

// 舵機脈寬範圍（微秒）
constexpr uint16_t SERVO_US_MIN = 500;
constexpr uint16_t SERVO_US_MAX = 2500;
constexpr uint16_t SERVO_US_DEFAULT = 1500;

// 安全逾時
constexpr uint32_t SIGNAL_TIMEOUT_MS = 1000;

// 舵機角度反轉設定（對應 Python 的反轉）
constexpr bool INVERT_SERVO[SERVO_COUNT] = {
  true,   // S0: 反轉
  false,  // S1
  true,   // S2: 反轉
  false,  // S3
  true,   // S4: 反轉
  false   // S5
};

// =========================
// 逆運動學引擎
// =========================
StewartKinematics kinematics;

// =========================
// 序列接收緩衝
// =========================
constexpr size_t RX_BUF_SIZE = 128;
char rxBuf[RX_BUF_SIZE];
size_t rxIndex = 0;
bool frameStarted = false;
uint32_t frameStartTime = 0;
constexpr uint32_t PACKET_TIMEOUT_MS = 100;

// =========================
// 控制資料
// =========================
float current_position[3] = {0, 0, 0};      // x, y, z (mm)
float current_rotation[3] = {0, 0, 0};      // roll, pitch, yaw (度)
float servo_angles[SERVO_COUNT] = {0};      // 計算出的舵機角度（度）
uint16_t servoUs[SERVO_COUNT] = {
  SERVO_US_DEFAULT, SERVO_US_DEFAULT, SERVO_US_DEFAULT,
  SERVO_US_DEFAULT, SERVO_US_DEFAULT, SERVO_US_DEFAULT
};

uint32_t lastValidPacketTime = 0;

// =========================
// 性能監控
// =========================
uint32_t packetCount = 0;
uint32_t lastPacketTime = 0;
uint32_t packetIntervalSum = 0;
uint32_t lastStatsTime = 0;
uint32_t ikFailCount = 0;

// =========================
// 工具函式
// =========================

uint16_t clampServoUs(int value) {
  if (value < SERVO_US_MIN) return SERVO_US_MIN;
  if (value > SERVO_US_MAX) return SERVO_US_MAX;
  return (uint16_t)value;
}

uint16_t usToPcaCount(uint16_t us, float freq) {
  float period_us = 1000000.0f / freq;
  float counts = (us * 4096.0f) / period_us;
  if (counts < 0) counts = 0;
  if (counts > 4095) counts = 4095;
  return (uint16_t)(counts + 0.5f);
}

uint16_t calibrateServoUs(uint16_t targetUs) {
  if (!ENABLE_CALIBRATION) {
    return targetUs;
  }
  float calibrated = (targetUs + PCA_CALIBRATION_OFFSET) / PCA_CALIBRATION_SCALE;
  return (uint16_t)(calibrated + 0.5f);
}

void writeServoUs(uint8_t ch, uint16_t us) {
  int32_t offsetUs = us;
  if (ENABLE_CHANNEL_OFFSET && ch < SERVO_COUNT) {
    offsetUs += CHANNEL_OFFSET[ch];
    if (offsetUs < 0) offsetUs = 0;
    if (offsetUs > 4000) offsetUs = 4000;
  }
  
  uint16_t calibratedUs = calibrateServoUs((uint16_t)offsetUs);
  uint16_t offCount = usToPcaCount(calibratedUs, SERVO_FREQ);
  pca.setPWM(ch, 0, offCount);
}

// 角度轉 PWM（微秒）
uint16_t angleToPwm(float angle_deg) {
  // 限制範圍
  if (angle_deg < -60.0f) angle_deg = -60.0f;
  if (angle_deg > 60.0f) angle_deg = 60.0f;
  
  // 轉換：1500μs 為中心，±500μs 對應 ±60°
  int pwm = 1500 + (int)(angle_deg / 60.0f * 500.0f);
  return clampServoUs(pwm);
}

// 應用舵機角度（從 IK 計算結果）
void applyServoAngles() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    // 應用反轉設定
    float angle = INVERT_SERVO[i] ? -servo_angles[i] : servo_angles[i];
    servoUs[i] = angleToPwm(angle);
    writeServoUs(i, servoUs[i]);
  }
}

// 解析平台姿態封包
// 格式：$P,x,y,z,roll,pitch,yaw,CS
bool parsePositionPacket(char *packet) {
  if (packet[0] != '$') return false;

  char *token = strtok(packet, ",");
  if (token == nullptr || strcmp(token, "$P") != 0) return false;

  // 解析 6 個浮點數
  float values[6];
  for (uint8_t i = 0; i < 6; i++) {
    token = strtok(nullptr, ",");
    if (token == nullptr) return false;
    values[i] = atof(token);
  }

  // Checksum（簡單累加）
  token = strtok(nullptr, ",");
  if (token == nullptr) return false;
  int receivedCS = atoi(token);
  
  // 計算 checksum（浮點數的簡單 hash）
  int calculatedCS = 0;
  for (uint8_t i = 0; i < 6; i++) {
    calculatedCS += (int)(values[i] * 10);  // 放大 10 倍後取整
  }
  calculatedCS = calculatedCS & 0xFF;

  if (receivedCS != calculatedCS) {
    Serial.print("[ERR] Checksum mismatch, recv=");
    Serial.print(receivedCS);
    Serial.print(" calc=");
    Serial.println(calculatedCS);
    return false;
  }

  // 更新當前位置和旋轉
  current_position[0] = values[0];  // x
  current_position[1] = values[1];  // y
  current_position[2] = values[2];  // z
  current_rotation[0] = values[3];  // roll
  current_rotation[1] = values[4];  // pitch
  current_rotation[2] = values[5];  // yaw

  // **計算逆運動學**
  bool ik_success = kinematics.calculateServoAngles(
    current_position,
    current_rotation,
    servo_angles
  );

  if (!ik_success) {
    ikFailCount++;
    Serial.println("[WARN] IK calculation failed (out of limits)");
    return false;
  }

  uint32_t now = millis();
  lastValidPacketTime = now;
  
  // 性能監控
  if (lastPacketTime > 0) {
    uint32_t interval = now - lastPacketTime;
    packetIntervalSum += interval;
    packetCount++;
  }
  lastPacketTime = now;
  
  return true;
}

void serviceSerial() {
  if (frameStarted && (millis() - frameStartTime) > PACKET_TIMEOUT_MS) {
    Serial.println("[WARN] Packet timeout, resetting buffer");
    frameStarted = false;
    rxIndex = 0;
  }
  
  uint8_t processedChars = 0;
  constexpr uint8_t MAX_CHARS_PER_LOOP = 128;
  
  while (Serial.available() > 0 && processedChars < MAX_CHARS_PER_LOOP) {
    char c = (char)Serial.read();
    processedChars++;

    if (!frameStarted) {
      if (c == '$') {
        frameStarted = true;
        rxIndex = 0;
        rxBuf[rxIndex++] = c;
        frameStartTime = millis();
      }
      continue;
    }

    if (c == '\n' || c == '\r') {
      if (rxIndex > 0) {
        rxBuf[rxIndex] = '\0';

        char parseBuf[RX_BUF_SIZE];
        strncpy(parseBuf, rxBuf, RX_BUF_SIZE);
        parseBuf[RX_BUF_SIZE - 1] = '\0';

        // 檢查 Verbose 模式切換命令：$V,0 或 $V,1
        if (strncmp(parseBuf, "$V,", 3) == 0) {
          int mode = atoi(parseBuf + 3);
          verbose_mode = (mode != 0);
          Serial.print("[CONFIG] Verbose mode: ");
          Serial.println(verbose_mode ? "ON" : "OFF");
        }
        else if (parsePositionPacket(parseBuf)) {
          applyServoAngles();

          // 只在 verbose 模式下回應（節省頻寬）
          if (verbose_mode) {
            Serial.print("[OK] Pos:(");
            Serial.print(current_position[0], 1);
            Serial.print(",");
            Serial.print(current_position[1], 1);
            Serial.print(",");
            Serial.print(current_position[2], 1);
            Serial.print(") Rot:(");
            Serial.print(current_rotation[0], 1);
            Serial.print(",");
            Serial.print(current_rotation[1], 1);
            Serial.print(",");
            Serial.print(current_rotation[2], 1);
            Serial.print(") -> Angles:");
            for (uint8_t i = 0; i < SERVO_COUNT; i++) {
              Serial.print(servo_angles[i], 1);
              if (i < SERVO_COUNT - 1) Serial.print(",");
            }
            Serial.println();
          }
        }
      }

      frameStarted = false;
      rxIndex = 0;
      continue;
    }

    if (rxIndex < RX_BUF_SIZE - 1) {
      rxBuf[rxIndex++] = c;
    } else {
      frameStarted = false;
      rxIndex = 0;
      Serial.println("[ERR] RX buffer overflow");
    }
  }
}

void serviceFailsafe() {
  static bool failsafeActive = false;
  uint32_t now = millis();

  if ((now - lastValidPacketTime) > SIGNAL_TIMEOUT_MS) {
    if (!failsafeActive) {
      // 回到中位
      for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        servo_angles[i] = 0;
        servoUs[i] = SERVO_US_DEFAULT;
      }
      applyServoAngles();
      failsafeActive = true;
      Serial.println("[WARN] Signal timeout -> servos back to neutral");
    }
  } else {
    failsafeActive = false;
  }
}

void setup() {
  Serial.begin(460800);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);  // I2C Fast Mode: 400kHz
  pca.begin();
  pca.setPWMFreq(SERVO_FREQ);

  // 初始化到中位
  applyServoAngles();
  lastValidPacketTime = millis();

  Serial.println("ESP32-C6 Stewart Platform Controller (with IK)");
  Serial.println("Format: $P,x,y,z,roll,pitch,yaw,CS");
  Serial.println("  Position: x,y,z in mm");
  Serial.println("  Rotation: roll,pitch,yaw in degrees");
  Serial.println("  Checksum: sum(values*10) & 0xFF");
  
  if (ENABLE_CALIBRATION) {
    Serial.print("PCA Calibration: ENABLED (offset=");
    Serial.print(PCA_CALIBRATION_OFFSET);
    Serial.print(", scale=");
    Serial.print(PCA_CALIBRATION_SCALE);
    Serial.println(")");
  }
  
  Serial.println("IK Engine: READY");
}

void loop() {
  serviceSerial();
  serviceFailsafe();
  
  // 性能統計
  uint32_t now = millis();
  if (now - lastStatsTime > 5000 && packetCount > 0) {
    float avgInterval = (float)packetIntervalSum / packetCount;
    float actualHz = 1000.0f / avgInterval;
    
    Serial.print("[ESP32-PERF] Packets: ");
    Serial.print(packetCount);
    Serial.print(", Avg: ");
    Serial.print(avgInterval, 1);
    Serial.print("ms (");
    Serial.print(actualHz, 1);
    Serial.print("Hz), IK fails: ");
    Serial.println(ikFailCount);
    
    packetCount = 0;
    packetIntervalSum = 0;
    ikFailCount = 0;
    lastStatsTime = now;
  }
  
  // 定期清理緩衝區
  static uint32_t lastFlushTime = 0;
  if ((millis() - lastFlushTime) > 1000) {
    if (!frameStarted && Serial.available() > 50) {
      while (Serial.available() > 0) Serial.read();
      Serial.println("[INFO] Flushed stale serial data");
    }
    lastFlushTime = millis();
  }
}
