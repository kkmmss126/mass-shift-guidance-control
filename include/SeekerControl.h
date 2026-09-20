#pragma once

#include <memory>

#include "PCA9685.h"

/*
 * ============================================================
 * File Name : SeekerControl.h
 *
 * Description
 * ------------------------------------------------------------
 * seeker_tracking_v1의 추적 로직을 라이브러리 형태로 분리한다.
 *
 * 중요한 원칙
 * ------------------------------------------------------------
 * - V1 제어 로직 자체는 변경하지 않는다.
 * - main.cpp에서는 내부 센서 처리 과정을 몰라도 된다.
 * - initialize() / update() / getter만 사용한다.
 *
 * V1 Control
 * ------------------------------------------------------------
 * Yaw
 *   LEFT / RIGHT 거리차 기반
 *   Dead Zone = 2.5 cm
 *   PWM Step  = 4
 *   Target Lost -> HOLD
 *
 * Pitch
 *   LEFT - BOTTOM 기반
 *   Dead Zone = 1.0 cm
 *   PWM Step  = 3
 *   |Error| >= 15 cm -> HOLD
 *   Target Lost -> HOLD
 *
 * P 제어 사용 X
 * EMA 사용 X
 * 자동 중심 복귀 X
 * ============================================================
 */

class SRF05;

class SeekerControl
{
public:
    SeekerControl();
    ~SeekerControl();

    /*
     * PCA9685 초기화
     * 센서 객체 생성
     * 시커를 calibration_data.h의 중심 PWM으로 이동
     */
    bool initialize();

    /*
     * V1 추적 루프 1회 실행
     *
     * LEFT -> RIGHT -> BOTTOM 순서 측정
     * Yaw / Pitch 판단
     * PWM 갱신
     * Servo 출력
     */
    void update();

    /*
     * 현재 Servo PWM 조회
     *
     * 이후 Phase 4에서
     * 중심 PWM과의 편차를 계산할 때 사용한다.
     */
    int getYawPWM() const;
    int getPitchPWM() const;

    /*
     * 최근 측정 거리 조회
     * 디버깅 및 추후 상태 판단용
     */
    double getLeftDistance() const;
    double getRightDistance() const;
    double getBottomDistance() const;

    /*
     * 최근 계산 오차 조회
     */
    double getYawError() const;
    double getPitchError() const;

    bool isInitialized() const;

private:
    PCA9685 m_pwm;

    std::unique_ptr<SRF05> m_leftSensor;
    std::unique_ptr<SRF05> m_rightSensor;
    std::unique_ptr<SRF05> m_bottomSensor;

    int m_yawPWM = 0;
    int m_pitchPWM = 0;

    double m_leftDistance = -1.0;
    double m_rightDistance = -1.0;
    double m_bottomDistance = -1.0;

    double m_yawError = 0.0;
    double m_pitchError = 0.0;

    bool m_initialized = false;
};
