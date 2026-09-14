#include "PCA9685.h"

#include <gpiod.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>


/* seeker_tracking v1
 * ============================================================
 * File Name : seeker_tracking_test.cpp
 *
 * Yaw + Pitch Integrated Seeker Tracking Test
 * ============================================================
 *
 * YAW
 * ------------------------------------------------------------
 * LEFT / RIGHT 센서 사용
 *
 * LEFT  = GPIO 12 / 13
 * RIGHT = GPIO 5  / 20
 *
 * 두 센서 거리차로 좌우 추적
 *
 * Dead Zone = 2.5 cm
 * PWM Step  = 4
 *
 *
 * PITCH
 * ------------------------------------------------------------
 * LEFT / BOTTOM 센서 사용
 *
 * Pitch Error = LEFT - BOTTOM
 *
 * Dead Zone = 1.0 cm
 * PWM Step  = 3
 *
 *
 * 공통
 * ------------------------------------------------------------
 * P 제어 사용 X
 * 고정 PWM Step 방식
 * Target Lost 시 중심 복귀 X
 * 현재 위치 HOLD
 * ============================================================
 */


/*
 * ============================================================
 * GPIO
 * ============================================================
 *
 * Yaw 단독 시험에서 실제 사용한 센서 정의를 기준으로 한다.
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
 * PCA9685 Servo Channel
 * ============================================================
 */

constexpr int PITCH_SERVO_CHANNEL = 0;
constexpr int YAW_SERVO_CHANNEL   = 1;


/*
 * ============================================================
 * Servo Center PWM
 * ============================================================
 */

constexpr int PITCH_CENTER_PWM = 321;
constexpr int YAW_CENTER_PWM   = 300;


/*
 * ============================================================
 * Servo PWM Limit
 * ============================================================
 */

constexpr int PITCH_PWM_MIN = 200;
constexpr int PITCH_PWM_MAX = 450;

constexpr int YAW_PWM_MIN = 200;
constexpr int YAW_PWM_MAX = 400;


/*
 * ============================================================
 * Yaw Control
 * ============================================================
 */

constexpr double YAW_DEADZONE_CM = 2.5;

constexpr int YAW_PWM_STEP = 4;

constexpr int YAW_DIRECTION = 1;


/*
 * ============================================================
 * Pitch Control
 * ============================================================
 *
 * 실험적으로 비교적 잘 동작한
 *
 * LEFT - BOTTOM
 *
 * 방식 사용.
 */

constexpr double PITCH_DEADZONE_CM = 1.0;

constexpr int PITCH_PWM_STEP = 3;

constexpr int PITCH_DIRECTION = 1;


/*
 * Pitch Error가 지나치게 크면
 * 센서 이상치로 판단하고 HOLD
 */

constexpr double PITCH_MAX_ERROR_CM = 15.0;


/*
 * ============================================================
 * Ultrasonic Sensor
 * ============================================================
 */

constexpr double MIN_DISTANCE_CM = 20.0;
constexpr double MAX_DISTANCE_CM = 40.0;


/*
 * 각 초음파 센서 사이 측정 간격
 */

constexpr int SENSOR_INTERVAL_MS = 15;


/*
 * 전체 제어 후 추가 대기
 */

constexpr int CONTROL_PERIOD_MS = 50;


/*
 * ============================================================
 * SRF05
 * ============================================================
 */

class SRF05
{
public:

    SRF05(
        unsigned int trigPin,
        unsigned int echoPin)
        : m_trigPin(trigPin),
          m_echoPin(echoPin)
    {
        /*
         * GPIO Chip Open
         */

        m_chip =
            gpiod_chip_open(
                "/dev/gpiochip0");


        if (m_chip == nullptr)
        {
            std::cerr
                << "[ERROR] gpiochip0 open 실패\n";

            return;
        }


        /*
         * ====================================================
         * TRIG 설정
         * ====================================================
         */

        gpiod_line_settings* trigSettings =
            gpiod_line_settings_new();


        gpiod_line_settings_set_direction(
            trigSettings,
            GPIOD_LINE_DIRECTION_OUTPUT);


        gpiod_line_settings_set_output_value(
            trigSettings,
            GPIOD_LINE_VALUE_INACTIVE);


        gpiod_line_config* trigConfig =
            gpiod_line_config_new();


        gpiod_line_config_add_line_settings(
            trigConfig,
            &m_trigPin,
            1,
            trigSettings);


        m_trigRequest =
            gpiod_chip_request_lines(
                m_chip,
                nullptr,
                trigConfig);


        gpiod_line_settings_free(
            trigSettings);

        gpiod_line_config_free(
            trigConfig);


        /*
         * ====================================================
         * ECHO 설정
         * ====================================================
         */

        gpiod_line_settings* echoSettings =
            gpiod_line_settings_new();


        gpiod_line_settings_set_direction(
            echoSettings,
            GPIOD_LINE_DIRECTION_INPUT);


        gpiod_line_config* echoConfig =
            gpiod_line_config_new();


        gpiod_line_config_add_line_settings(
            echoConfig,
            &m_echoPin,
            1,
            echoSettings);


        m_echoRequest =
            gpiod_chip_request_lines(
                m_chip,
                nullptr,
                echoConfig);


        gpiod_line_settings_free(
            echoSettings);

        gpiod_line_config_free(
            echoConfig);


        if (m_trigRequest == nullptr ||
            m_echoRequest == nullptr)
        {
            std::cerr
                << "[ERROR] GPIO request 실패"
                << " TRIG=" << m_trigPin
                << " ECHO=" << m_echoPin
                << '\n';
        }
    }


    ~SRF05()
    {
        /*
         * GPIO 자원 해제
         */

        if (m_trigRequest != nullptr)
        {
            gpiod_line_request_release(
                m_trigRequest);
        }


        if (m_echoRequest != nullptr)
        {
            gpiod_line_request_release(
                m_echoRequest);
        }


        if (m_chip != nullptr)
        {
            gpiod_chip_close(
                m_chip);
        }
    }


    /*
     * ========================================================
     * 거리 측정
     * ========================================================
     *
     * 정상 -> 거리(cm)
     * 실패 -> -1
     */

    double measureDistance()
    {
        if (m_trigRequest == nullptr ||
            m_echoRequest == nullptr)
        {
            return -1.0;
        }


        /*
         * TRIG LOW
         */

        gpiod_line_request_set_value(
            m_trigRequest,
            m_trigPin,
            GPIOD_LINE_VALUE_INACTIVE);


        std::this_thread::sleep_for(
            std::chrono::microseconds(2));


        /*
         * TRIG HIGH 10us
         */

        gpiod_line_request_set_value(
            m_trigRequest,
            m_trigPin,
            GPIOD_LINE_VALUE_ACTIVE);


        std::this_thread::sleep_for(
            std::chrono::microseconds(10));


        /*
         * TRIG LOW
         */

        gpiod_line_request_set_value(
            m_trigRequest,
            m_trigPin,
            GPIOD_LINE_VALUE_INACTIVE);


        /*
         * ====================================================
         * ECHO HIGH 대기
         * ====================================================
         */

        auto timeoutStart =
            std::chrono::steady_clock::now();


        while (
            gpiod_line_request_get_value(
                m_echoRequest,
                m_echoPin)
            != GPIOD_LINE_VALUE_ACTIVE)
        {
            auto now =
                std::chrono::steady_clock::now();


            double elapsedMs =
                std::chrono::duration<double, std::milli>(
                    now - timeoutStart)
                    .count();


            if (elapsedMs > 30.0)
            {
                return -1.0;
            }
        }


        /*
         * ECHO HIGH 시작
         */

        auto echoStart =
            std::chrono::steady_clock::now();


        /*
         * ====================================================
         * ECHO LOW 대기
         * ====================================================
         */

        while (
            gpiod_line_request_get_value(
                m_echoRequest,
                m_echoPin)
            == GPIOD_LINE_VALUE_ACTIVE)
        {
            auto now =
                std::chrono::steady_clock::now();


            double elapsedMs =
                std::chrono::duration<double, std::milli>(
                    now - echoStart)
                    .count();


            if (elapsedMs > 30.0)
            {
                return -1.0;
            }
        }


        auto echoEnd =
            std::chrono::steady_clock::now();


        /*
         * Pulse Width
         */

        double pulseTimeUs =
            std::chrono::duration<double, std::micro>(
                echoEnd - echoStart)
                .count();


        /*
         * 거리 계산
         */

        double distanceCm =
            pulseTimeUs / 58.0;


        /*
         * 유효 범위
         */

        if (distanceCm < MIN_DISTANCE_CM ||
            distanceCm > MAX_DISTANCE_CM)
        {
            return -1.0;
        }


        return distanceCm;
    }


private:

    unsigned int m_trigPin;
    unsigned int m_echoPin;

    gpiod_chip* m_chip = nullptr;

    gpiod_line_request* m_trigRequest = nullptr;
    gpiod_line_request* m_echoRequest = nullptr;
};


/*
 * ============================================================
 * main
 * ============================================================
 */

int main()
{
    std::cout
        << "========================================\n"
        << " Yaw + Pitch Integrated Seeker Test\n"
        << " Yaw   : LEFT / RIGHT\n"
        << " Pitch : LEFT / BOTTOM\n"
        << " Lost  : HOLD\n"
        << "========================================\n";


    /*
     * ========================================================
     * PCA9685 초기화
     * ========================================================
     */

    PCA9685 pwm;


    if (!pwm.begin())
    {
        std::cerr
            << "[ERROR] PCA9685 초기화 실패\n";

        return 1;
    }


    pwm.setPWMFreq(50);


    /*
     * ========================================================
     * Sensor 생성
     * ========================================================
     */

    SRF05 leftSensor(
        LEFT_TRIG_PIN,
        LEFT_ECHO_PIN);


    SRF05 rightSensor(
        RIGHT_TRIG_PIN,
        RIGHT_ECHO_PIN);


    SRF05 bottomSensor(
        BOTTOM_TRIG_PIN,
        BOTTOM_ECHO_PIN);


    /*
     * ========================================================
     * Servo 초기 위치
     * ========================================================
     */

    int yawPWM =
        YAW_CENTER_PWM;


    int pitchPWM =
        PITCH_CENTER_PWM;


    pwm.setPWM(
        YAW_SERVO_CHANNEL,
        0,
        yawPWM);


    pwm.setPWM(
        PITCH_SERVO_CHANNEL,
        0,
        pitchPWM);


    std::cout
        << "[INIT] Yaw PWM = "
        << yawPWM
        << '\n';


    std::cout
        << "[INIT] Pitch PWM = "
        << pitchPWM
        << '\n';


    std::this_thread::sleep_for(
        std::chrono::seconds(1));


    /*
     * ========================================================
     * Tracking Loop
     * ========================================================
     */

    while (true)
    {
        /*
         * ====================================================
         * LEFT 측정
         * ====================================================
         */

        double leftDistance =
            leftSensor.measureDistance();


        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                SENSOR_INTERVAL_MS));


        /*
         * ====================================================
         * RIGHT 측정
         * ====================================================
         */

        double rightDistance =
            rightSensor.measureDistance();


        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                SENSOR_INTERVAL_MS));


        /*
         * ====================================================
         * BOTTOM 측정
         * ====================================================
         */

        double bottomDistance =
            bottomSensor.measureDistance();


        /*
         * Sensor Validity
         */

        bool leftValid =
            leftDistance >= 0.0;


        bool rightValid =
            rightDistance >= 0.0;


        bool bottomValid =
            bottomDistance >= 0.0;


        /*
         * Debug용 Error
         */

        double yawError = 0.0;

        double pitchError = 0.0;


        /*
         * ====================================================
         * YAW CONTROL
         * ====================================================
         *
         * 기존 Yaw 단독 테스트의 동작을 그대로 사용한다.
         */


        /*
         * LEFT + RIGHT 둘 다 검출
         */

        if (leftValid &&
            rightValid)
        {
            yawError =
                leftDistance -
                rightDistance;


            /*
             * Dead Zone
             */

            if (std::abs(yawError) <=
                YAW_DEADZONE_CM)
            {
                std::cout
                    << "[YAW CENTER] ";
            }


            /*
             * LEFT가 더 가까움
             */

            else if (
                leftDistance <
                rightDistance)
            {
                yawPWM +=
                    YAW_PWM_STEP *
                    YAW_DIRECTION;


                std::cout
                    << "[YAW LEFT] ";
            }


            /*
             * RIGHT가 더 가까움
             */

            else
            {
                yawPWM -=
                    YAW_PWM_STEP *
                    YAW_DIRECTION;


                std::cout
                    << "[YAW RIGHT] ";
            }
        }


        /*
         * LEFT만 검출
         */

        else if (
            leftValid &&
            !rightValid)
        {
            yawPWM +=
                YAW_PWM_STEP *
                YAW_DIRECTION;


            std::cout
                << "[YAW LEFT ONLY] ";
        }


        /*
         * RIGHT만 검출
         */

        else if (
            rightValid &&
            !leftValid)
        {
            yawPWM -=
                YAW_PWM_STEP *
                YAW_DIRECTION;


            std::cout
                << "[YAW RIGHT ONLY] ";
        }


        /*
         * 둘 다 LOST
         */

        else
        {
            /*
             * 중심으로 복귀하지 않는다.
             */

            std::cout
                << "[YAW HOLD] ";
        }


        /*
         * Yaw 안전 범위
         */

        yawPWM =
            std::clamp(
                yawPWM,
                YAW_PWM_MIN,
                YAW_PWM_MAX);


        /*
         * ====================================================
         * PITCH CONTROL
         * ====================================================
         *
         * LEFT - BOTTOM 기반으로 Pitch를 제어한다.
         *
         * LEFT와 BOTTOM 둘 다 검출되어야만
         * Pitch를 움직인다.
         */


        if (leftValid &&
            bottomValid)
        {
            pitchError =
                leftDistance -
                bottomDistance;


            /*
             * 이상치
             *
             * 센서 간 거리 차이가 너무 크면
             * 정상적인 추적 오차가 아니라
             * 측정 이상으로 판단하고 현재 위치를 유지한다.
             */

            if (std::abs(pitchError) >=
                PITCH_MAX_ERROR_CM)
            {
                std::cout
                    << "[PITCH OUTLIER -> HOLD] ";
            }


            /*
             * Dead Zone
             *
             * Pitch Error가 ±1.0 cm 이내라면
             * 중심 부근으로 판단하고 움직이지 않는다.
             */

            else if (std::abs(pitchError) <=
                     PITCH_DEADZONE_CM)
            {
                std::cout
                    << "[PITCH CENTER] ";
            }


            /*
             * Error > Dead Zone
             */

            else if (pitchError > 0.0)
            {
                pitchPWM +=
                    PITCH_PWM_STEP *
                    PITCH_DIRECTION;


                std::cout
                    << "[PITCH A] ";
            }


            /*
             * Error < -Dead Zone
             */

            else
            {
                pitchPWM -=
                    PITCH_PWM_STEP *
                    PITCH_DIRECTION;


                std::cout
                    << "[PITCH B] ";
            }
        }


        /*
         * LEFT 또는 BOTTOM 측정 실패
         */

        else
        {
            /*
             * 자동 복귀하지 않고
             * 현재 Pitch 위치 유지
             */

            std::cout
                << "[PITCH HOLD] ";
        }


        /*
         * Pitch 안전 범위
         */

        pitchPWM =
            std::clamp(
                pitchPWM,
                PITCH_PWM_MIN,
                PITCH_PWM_MAX);


        /*
         * ====================================================
         * Servo 적용
         * ====================================================
         */

        pwm.setPWM(
            YAW_SERVO_CHANNEL,
            0,
            yawPWM);


        pwm.setPWM(
            PITCH_SERVO_CHANNEL,
            0,
            pitchPWM);


        /*
         * ====================================================
         * Debug
         * ====================================================
         */

        std::cout
            << "L="
            << leftDistance

            << " R="
            << rightDistance

            << " B="
            << bottomDistance

            << " YawErr="
            << yawError

            << " PitchErr="
            << pitchError

            << " YawPWM="
            << yawPWM

            << " PitchPWM="
            << pitchPWM

            << '\n';


        /*
         * 다음 제어 루프
         */

        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                CONTROL_PERIOD_MS));
    }


    return 0;
}