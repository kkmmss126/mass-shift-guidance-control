#include "PCA9685.h"

#include <gpiod.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>


/*
 * ============================================================
 * File Name : seeker_pitch_test.cpp
 *
 * Description
 * ------------------------------------------------------------
 * 삼중 초음파 센서를 이용하여
 * 시커의 상하(Pitch) 축만 독립적으로 추적하는 테스트 코드.
 *
 * Yaw 서보:
 *     중심 PWM에 고정
 *
 * Pitch 서보:
 *     상단 센서 평균과 하단 센서 거리차를 이용해 제어
 *
 * 제어 방식:
 *     P 제어 사용 X
 *     Dead Zone + 고정 PWM Step 방식
 * ============================================================
 */


/*
 * ============================================================
 * GPIO 설정
 * ============================================================
 */

// Left Ultrasonic Sensor
constexpr unsigned int LEFT_TRIG_PIN = 5;
constexpr unsigned int LEFT_ECHO_PIN = 20;

// Right Ultrasonic Sensor
constexpr unsigned int RIGHT_TRIG_PIN = 12;
constexpr unsigned int RIGHT_ECHO_PIN = 13;

// Bottom Ultrasonic Sensor
constexpr unsigned int BOTTOM_TRIG_PIN = 16;
constexpr unsigned int BOTTOM_ECHO_PIN = 6;


/*
 * ============================================================
 * PCA9685 Servo Channel
 * ============================================================
 *
 * 기존 시커 코드 기준
 *
 * Channel 0 = Pitch
 * Channel 1 = Yaw
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
 * Servo 안전 범위
 * ============================================================
 *
 * 우선 기존 테스트 범위를 사용한다.
 */

constexpr int PITCH_PWM_MIN = 200;
constexpr int PITCH_PWM_MAX = 450;


/*
 * ============================================================
 * Pitch Control Parameter
 * ============================================================
 *
 * 좌우 테스트에서 STEP=2가 조금 덜덜거렸으므로
 * Pitch는 처음부터 1 PWM씩 움직인다.
 */

constexpr int PITCH_PWM_STEP = 3;


/*
 * ============================================================
 * Pitch Dead Zone
 * ============================================================
 *
 * 상단 평균 거리와 하단 거리의 차이가
 * ±2.5cm 이내면 중앙으로 판단한다.
 *
 * 너무 둔하면 2.0으로 감소.
 * 너무 덜덜거리면 3.0으로 증가.
 */

constexpr double PITCH_DEADZONE_CM = 1.5;


/*
 * ============================================================
 * Servo Direction
 * ============================================================
 *
 * 실제 움직임이 반대라면
 *
 * 1 → -1
 *
 * 로 변경한다.
 */

constexpr int PITCH_DIRECTION = 1;


/*
 * ============================================================
 * 초음파 센서 측정 범위
 * ============================================================
 */

constexpr double MIN_DISTANCE_CM = 20.0;
constexpr double MAX_DISTANCE_CM = 40.0;


/*
 * ============================================================
 * 센서 측정 간격
 * ============================================================
 *
 * 초음파 간섭을 줄이기 위해
 * 각각 순차적으로 측정한다.
 */

constexpr int SENSOR_INTERVAL_MS = 15;


/*
 * 한 번의 전체 측정 이후 대기시간
 */

constexpr int CONTROL_PERIOD_MS = 50;


/*
 * ============================================================
 * SRF05 Class
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
         * TRIG GPIO 설정
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
         * ECHO GPIO 설정
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


        /*
         * GPIO Request 확인
         */

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
     * measureDistance()
     * ========================================================
     *
     * 정상:
     *     거리(cm)
     *
     * 실패:
     *     -1
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
         * ECHO HIGH 시작 시각
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
         * Pulse Width 계산
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
         * 유효 범위 검사
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
 * main()
 * ============================================================
 */

int main()
{
    std::cout
        << "========================================\n"
        << "     Seeker Pitch Tracking Test\n"
        << "     Fixed-Step Control\n"
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
     * 센서 생성
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

    int pitchPWM =
        PITCH_CENTER_PWM;


    /*
     * Yaw는 테스트 동안 계속 중앙에 고정한다.
     */

    pwm.setPWM(
        YAW_SERVO_CHANNEL,
        0,
        YAW_CENTER_PWM);


    pwm.setPWM(
        PITCH_SERVO_CHANNEL,
        0,
        pitchPWM);


    std::cout
        << "[INIT] Yaw fixed   : "
        << YAW_CENTER_PWM
        << '\n';


    std::cout
        << "[INIT] Pitch PWM   : "
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
         * LEFT 측정
         */

        double leftDistance =
            leftSensor.measureDistance();


        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                SENSOR_INTERVAL_MS));


        /*
         * RIGHT 측정
         */

        double rightDistance =
            rightSensor.measureDistance();


        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                SENSOR_INTERVAL_MS));


        /*
         * BOTTOM 측정
         */

        double bottomDistance =
            bottomSensor.measureDistance();


        /*
         * ====================================================
         * 세 센서가 모두 정상일 때만 Pitch 제어
         * ====================================================
         *
         * 이번 테스트에서는 원인 분리를 위해
         * 센서 하나라도 실패하면 서보를 움직이지 않는다.
         */

        if (leftDistance >= 0.0 &&
            rightDistance >= 0.0 &&
            bottomDistance >= 0.0)
        {
            /*
             * 상단 두 센서 평균
             */

            double upperAverage =
                (leftDistance +
                 rightDistance)
                / 2.0;


            /*
             * Pitch 오차
             *
             * 상단 평균 - 하단 거리
             */

            double pitchError =
                upperAverage -
                bottomDistance;


            /*
             * =================================================
             * Dead Zone
             * =================================================
             */

            if (std::abs(pitchError) <=
                PITCH_DEADZONE_CM)
            {
                /*
                 * 중앙 영역에서는 움직이지 않는다.
                 */

                std::cout
                    << "[CENTER] ";
            }


            /*
             * =================================================
             * 한 방향으로 이동
             * =================================================
             */

            else if (pitchError > 0.0)
            {
                pitchPWM +=
                    PITCH_PWM_STEP *
                    PITCH_DIRECTION;


                std::cout
                    << "[TRACK A] ";
            }


            /*
             * =================================================
             * 반대 방향으로 이동
             * =================================================
             */

            else
            {
                pitchPWM -=
                    PITCH_PWM_STEP *
                    PITCH_DIRECTION;


                std::cout
                    << "[TRACK B] ";
            }


            /*
             * 안전 범위 제한
             */

            pitchPWM =
                std::clamp(
                    pitchPWM,
                    PITCH_PWM_MIN,
                    PITCH_PWM_MAX);


            /*
             * Pitch 서보 적용
             */

            pwm.setPWM(
                PITCH_SERVO_CHANNEL,
                0,
                pitchPWM);


            /*
             * Debug 출력
             */

            std::cout
                << "L="
                << leftDistance

                << " R="
                << rightDistance

                << " B="
                << bottomDistance

                << " UpperAvg="
                << upperAverage

                << " Err="
                << pitchError

                << " PWM="
                << pitchPWM

                << '\n';
        }


        /*
         * 센서 하나라도 실패
         */

        else
        {
            /*
             * 현재 위치 HOLD.
             *
             * 센서 하나가 빠졌다고 임의로 움직이지 않는다.
             */

            std::cout
                << "[SENSOR INVALID - HOLD]"

                << " L="
                << leftDistance

                << " R="
                << rightDistance

                << " B="
                << bottomDistance

                << " PWM="
                << pitchPWM

                << '\n';
        }


        /*
         * 다음 제어 루프까지 대기
         */

        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                CONTROL_PERIOD_MS));
    }


    return 0;
}