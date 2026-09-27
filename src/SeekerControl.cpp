#include "SeekerControl.h"

#include "calibration_data.h"

#include <gpiod.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>


/*
 * ============================================================
 * V1 Hardware Mapping
 * ============================================================
 *
 * seeker_tracking_v1 기준으로 통일.
 */

// LEFT
constexpr unsigned int LEFT_TRIG_PIN = 12;
constexpr unsigned int LEFT_ECHO_PIN = 13;

// RIGHT
constexpr unsigned int RIGHT_TRIG_PIN = 5;
constexpr unsigned int RIGHT_ECHO_PIN = 20;

// BOTTOM
constexpr unsigned int BOTTOM_TRIG_PIN = 16;
constexpr unsigned int BOTTOM_ECHO_PIN = 6;


/*
 * ============================================================
 * V1 Control Parameters
 * ============================================================
 *
 * 현재는 V1 동작 보존을 위해 그대로 둔다.
 * 추후 control_config.h로 분리 가능.
 */

constexpr int PITCH_PWM_MIN = 200;
constexpr int PITCH_PWM_MAX = 450;

constexpr int YAW_PWM_MIN = 200;
constexpr int YAW_PWM_MAX = 400;

constexpr double YAW_DEADZONE_CM = 2.5;
constexpr int YAW_PWM_STEP = 4;
constexpr int YAW_DIRECTION = 1;

constexpr double PITCH_DEADZONE_CM = 1.0;
constexpr int PITCH_PWM_STEP = 3;
constexpr int PITCH_DIRECTION = 1;

constexpr double PITCH_MAX_ERROR_CM = 15.0;

constexpr double MIN_DISTANCE_CM = 20.0;