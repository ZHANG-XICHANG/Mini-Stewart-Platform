/**
 * Stewart Platform Inverse Kinematics
 * 
 * 從 Python 版本移植的逆運動學計算
 * 輸入：平台位置 (x,y,z) 和姿態 (roll,pitch,yaw)
 * 輸出：6 個舵機角度（度）
 */

#ifndef STEWART_KINEMATICS_H
#define STEWART_KINEMATICS_H

#include <math.h>

// =========================
// 平台幾何參數（與 stewart_config.py 一致）
// =========================
namespace StewartConfig {
  // 半徑（mm）
  constexpr float BASE_RADIUS = 68.0f;
  constexpr float PLATFORM_RADIUS = 68.0f;
  
  // 臂長（mm）
  constexpr float SERVO_ARM_LENGTH = 24.0f;     // L1
  constexpr float CONNECTING_ARM_LENGTH = 99.0f; // L2
  
  // 平台高度（mm）
  constexpr float PLATFORM_HEIGHT = 96.04f;

  
  // 角度參數（度）
  constexpr float THETA_R = 30.0f;
  constexpr float THETA_P = 30.0f;
  
  // 舵機角度（度）
  const float THETA_S[6] = {150, -90, 30, 150, -90, 30};
  
  // 角度限制（度）
  constexpr float SERVO_MIN_ANGLE = -60.0f;
  constexpr float SERVO_MAX_ANGLE = 60.0f;
  
  // 幾何係數（從 Python 移植）
  const int DX_MULTIPLIER[6] = {1, 1, 1, -1, -1, -1};
  const int ANGLE_MULTIPLIER[6] = {1, -1, 1, 1, -1, 1};
  
  // 偏移角度（弧度）
  const float OFFSET_ANGLE[6] = {
    M_PI/6,    // 30 degrees
    M_PI/6,    // 30 degrees
    -M_PI/2,   // -90 degrees
    -M_PI/2,   // -90 degrees
    M_PI/6,    // 30 degrees
    M_PI/6     // 30 degrees
  };
}

// =========================
// 逆運動學計算類別
// =========================
class StewartKinematics {
private:
  // 平台和基座坐標（預先計算）
  float platform_coords_x[6];
  float platform_coords_y[6];
  float base_coords_x[6];
  float base_coords_y[6];
  
  // 角度轉換
  inline float deg2rad(float deg) { return deg * M_PI / 180.0f; }
  inline float rad2deg(float rad) { return rad * 180.0f / M_PI; }

public:
  StewartKinematics() {
    calculateGeometry();
  }
  
  // 計算平台幾何（建構時執行一次）
  void calculateGeometry() {
    using namespace StewartConfig;
    
    for (int i = 0; i < 6; i++) {
      // 平台坐標
      float platform_pd_x = DX_MULTIPLIER[i] * BASE_RADIUS;
      float platform_pd_y = BASE_RADIUS;
      float platform_angle = OFFSET_ANGLE[i] + ANGLE_MULTIPLIER[i] * deg2rad(THETA_R);
      platform_coords_x[i] = platform_pd_x * cos(platform_angle);
      platform_coords_y[i] = platform_pd_y * sin(platform_angle);
      
      // 基座坐標
      float base_pd_x = DX_MULTIPLIER[i] * PLATFORM_RADIUS;
      float base_pd_y = PLATFORM_RADIUS;
      float base_angle = OFFSET_ANGLE[i] + ANGLE_MULTIPLIER[i] * deg2rad(THETA_P);
      base_coords_x[i] = base_pd_x * cos(base_angle);
      base_coords_y[i] = base_pd_y * sin(base_angle);
    }
  }
  
  /**
   * 計算舵機角度
   * 
   * @param position 位置 [x, y, z] (mm)
   * @param rotation 旋轉 [roll, pitch, yaw] (度)
   * @param servo_angles 輸出：舵機角度陣列 (度)
   * @return true 如果成功，false 如果超出限制
   */
  bool calculateServoAngles(
    const float position[3],
    const float rotation[3],
    float servo_angles[6]
  ) {
    using namespace StewartConfig;
    
    // 轉換旋轉角度為弧度
    float roll = deg2rad(rotation[0]);
    float pitch = deg2rad(rotation[1]);
    float yaw = deg2rad(rotation[2]);
    
    // 預先計算三角函數
    float cos_roll = cos(roll);
    float sin_roll = sin(roll);
    float cos_pitch = cos(pitch);
    float sin_pitch = sin(pitch);
    float cos_yaw = cos(yaw);
    float sin_yaw = sin(yaw);
    
    // 計算每個舵機的角度
    for (int i = 0; i < 6; i++) {
      // 變換平台坐標（基於位置和旋轉）
      float platform_pivot_x = 
        platform_coords_x[i] * cos_roll * cos_yaw + 
        platform_coords_y[i] * (sin_pitch * sin_roll * cos_yaw - cos_pitch * sin_yaw) + 
        position[0];
      
      float platform_pivot_y = 
        platform_coords_x[i] * cos_pitch * sin_yaw + 
        platform_coords_y[i] * (cos_roll * cos_yaw + sin_roll * sin_pitch * sin_yaw) + 
        position[1];
      
      float platform_pivot_z = 
        -platform_coords_x[i] * sin_roll + 
        platform_coords_y[i] * sin_pitch * cos_roll + 
        PLATFORM_HEIGHT + position[2];
      
      // 計算腿向量
      float delta_Lx = base_coords_x[i] - platform_pivot_x;
      float delta_Ly = base_coords_y[i] - platform_pivot_y;
      float delta_Lz = -platform_pivot_z;
      
      // 計算虛擬腿長
      float delta_L2_virtual = sqrt(
        delta_Lx * delta_Lx + 
        delta_Ly * delta_Ly + 
        delta_Lz * delta_Lz
      );
      
      // 檢查連接桿長度限制
      if (fabs(delta_L2_virtual - SERVO_ARM_LENGTH) > CONNECTING_ARM_LENGTH) {
        return false;  // 超出連接桿長度
      }
      
      // 計算中間值
      float l = delta_L2_virtual * delta_L2_virtual - 
                (CONNECTING_ARM_LENGTH * CONNECTING_ARM_LENGTH - 
                 SERVO_ARM_LENGTH * SERVO_ARM_LENGTH);
      float m = 2 * SERVO_ARM_LENGTH * platform_pivot_z;
      
      float theta_s_rad = deg2rad(THETA_S[i]);
      float n = 2 * SERVO_ARM_LENGTH * 
                (cos(theta_s_rad) * (platform_pivot_x - base_coords_x[i]) + 
                 sin(theta_s_rad) * (platform_pivot_y - base_coords_y[i]));
      
      // 檢查數學有效性
      float discriminant = m * m + n * n;
      if (discriminant <= 0) {
        return false;  // 無有效解
      }
      
      float check_val = l / sqrt(discriminant);
      if (fabs(check_val) > 1.0f) {
        return false;  // asin 定義域錯誤
      }
      
      // 計算舵機角度（弧度）
      float alpha = asin(check_val) - atan2(n, m);
      
      // 轉換為度並檢查範圍
      float alpha_deg = rad2deg(alpha);
      if (alpha_deg < SERVO_MIN_ANGLE || alpha_deg > SERVO_MAX_ANGLE) {
        return false;  // 超出舵機角度限制
      }
      
      servo_angles[i] = alpha_deg;
    }
    
    return true;  // 計算成功
  }
};

#endif // STEWART_KINEMATICS_H
