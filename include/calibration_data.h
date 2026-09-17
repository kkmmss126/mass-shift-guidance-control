#pragma once

/*
 * ============================================================
 * File Name : calibration_data.h
 *
 * Project
 * ------------------------------------------------------------
 * Mass Shift Guidance Control System
 *
 * Description
 * ------------------------------------------------------------
 * 시스템의 실제 캘리브레이션 결과를 한 곳에서 관리한다.
 *
 * 원칙:
 * 1. 아래 [MEASURED VALUES] 영역만 수정한다.
 * 2. main.cpp / phase 코드에는 실측값을 직접 작성하지 않는다.
 * 3. 프로그램 시작 시
 *
 *      Body Yaw      : 정면 = 0 step
 *      Mass Shift    : 기계적 중심 = 0 step
 *
 *    으로 정의한다.
 *
 * Control Structure
 * ------------------------------------------------------------
 * Seeker Yaw Error
 *      -> Body Yaw Stepper
 *      -> 동체 좌/우 회전
 *
 * Seeker Pitch Error
 *      -> Mass Shift
 *      -> 동체 상/하 Pitch 제어
 * ============================================================
 */

namespace Calibration
{

// ============================================================
//              [ MEASURED VALUES ]
//
//     실제 캘리브레이션 후 이 부분만 수정
// ============================================================


// ------------------------------------------------------------
// 1. Seeker Servo Center
// ------------------------------------------------------------
//
// 실제 최종 캘리브레이션 값
//
// Channel 0 : Pitch
// Channel 1 : Yaw
//

constexpr int SEEKER_PITCH_CENTER_PWM = 321;
constexpr int SEEKER_YAW_CENTER_PWM   = 300;


// ------------------------------------------------------------
// 2. Seeker Servo Safe Range
// ------------------------------------------------------------
//
// 서보 기구물 충돌 방지를 위한 안전 범위
//

constexpr int SEEKER_PITCH_MIN_PWM = 200;
constexpr int SEEKER_PITCH_MAX_PWM = 450;

constexpr int SEEKER_YAW_MIN_PWM = 200;
constexpr int SEEKER_YAW_MAX_PWM = 450;


// ------------------------------------------------------------
// 3. Seeker Sensor Center Offset
// ------------------------------------------------------------
//
// Yaw Raw Error
//
//      Left - Right
//
// 정중앙 표적에서 측정한 평균 오차 입력
//

constexpr double YAW_CENTER_OFFSET_CM = 0.0;


// Pitch Raw Error
//
//      ((Left + Right) / 2) - Bottom
//
// 정중앙 표적에서 측정한 평균 오차 입력
//

constexpr double PITCH_CENTER_OFFSET_CM = 0.0;


// ------------------------------------------------------------
// 4. MPU6050 Gyro Bias
// ------------------------------------------------------------
//
// imu_bias_measure.cpp 실행 후 출력되는 값을 그대로 입력
//
// Corrected Gyro:
//
//      corrected = raw - bias
//

constexpr double GYRO_X_BIAS = 0.0;
constexpr double GYRO_Y_BIAS = 0.0;
constexpr double GYRO_Z_BIAS = 0.0;


// ------------------------------------------------------------
// 5. Body Yaw Stepper Calibration
// ------------------------------------------------------------
//
// 좌/우 제어용 동체 회전 스텝모터
//
// 프로그램 시작 시:
//
//      동체 정면 = 0 step
//
// 실제 좌/우 안전 한계 측정 후 입력
//

constexpr int BODY_YAW_MIN_STEP = 0;
constexpr int BODY_YAW_MAX_STEP = 0;


// 실제 모터 방향 확인 후 설정
//
//  1  : 현재 방향 그대로
// -1  : 방향 반전
//

constexpr int BODY_YAW_DIRECTION = 1;


// ------------------------------------------------------------
// 6. Mass Shift Pitch Calibration
// ------------------------------------------------------------
//
// 상/하 Pitch 제어용 질량이동 장치
//
// 프로그램 시작 시:
//
//      질량 기계적 중심 = 0 step
//
// 실제 이동 안전 한계 측정 후 입력
//

constexpr int MASS_SHIFT_MIN_STEP = 0;
constexpr int MASS_SHIFT_MAX_STEP = 0;


// 실제 모터 방향 확인 후 설정
//
//  1  : 현재 방향 그대로
// -1  : 방향 반전
//

constexpr int MASS_SHIFT_DIRECTION = 1;


// ============================================================
//              [ FIXED REFERENCE VALUES ]
//
//       일반적으로 수정하지 않는 영역
// ============================================================


// ------------------------------------------------------------
// Seeker PCA9685 Channel
// ------------------------------------------------------------

constexpr int SEEKER_PITCH_CHANNEL = 0;
constexpr int SEEKER_YAW_CHANNEL   = 1;


// ------------------------------------------------------------
// Program Coordinate Reference
// ------------------------------------------------------------
//
// 프로그램 시작 시 기계적 중심을 0으로 정의
//

constexpr int BODY_YAW_CENTER_STEP  = 0;
constexpr int MASS_SHIFT_CENTER_STEP = 0;


// ------------------------------------------------------------
// Seeker Error Definition
// ------------------------------------------------------------
//
// Yaw:
//
//      Raw Error = Left - Right
//
//      Corrected Error
//          = Raw Error - YAW_CENTER_OFFSET_CM
//
//
// Pitch:
//
//      Raw Error
//          = ((Left + Right) / 2) - Bottom
//
//      Corrected Error
//          = Raw Error - PITCH_CENTER_OFFSET_CM
//
// ------------------------------------------------------------

} // namespace Calibration