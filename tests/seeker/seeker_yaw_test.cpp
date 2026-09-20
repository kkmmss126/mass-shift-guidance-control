#include "PCA9685.h"

#include <gpiod.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>


/*
 * ============================================================
 * Seeker Yaw Tracking Test
 * Previous Competition Control Method
 * ============================================================
 *
 * 이전 대회 방식의 핵심 제어 구조를
 * 현재 Raspberry Pi + PCA9685 환경에 맞게 적용한다.
 *
 * 제어 방식
 * ------------------------------------------------------------
 * 1. 좌/우 초음파 센서를 순차 측정한다.
 * 2. 두 센서 거리 차이가 Dead Zone 이내이면 정지한다.
 * 3. Dead Zone을 벗어나면 일정 PWM_STEP만큼 움직인다.
 * 4. P Gain은 사용하지 않는다.
 */


/*
 * ============================================================
 * GPIO
 * ============================================================
 */

// Left Sensor
constexpr unsigned int LEFT_TRIG_PIN = 5;
constexpr unsigned int LEFT_ECHO_PIN = 20;

// Right Sensor
constexpr unsigned int RIGHT_TRIG_PIN = 12;
constexpr unsigned int RIGHT_ECHO_PIN = 13;


/*
 * ============================================================
 * PCA9685 Yaw Servo
 * ============================================================
 */

constexpr int YAW_SERVO_CHANNEL = 1;

/*
 * 현재 시커 Yaw 중심값
 */
constexpr int YAW_CENTER_PWM = 300;


/*
 * 서보 이동 가능 범위
 */
constexpr int YAW_PWM_MIN = 200;
constexpr int YAW_PWM_MAX = 400;


/*
 * ============================================================
 * Previous Competition Control Parameters
 * ============================================================
 *
 * 이전 대회:
 * Dead Zone = 2 cm
 * sensitivity = 8 OCR3A count
 *
 * 현재 PCA9685에서는 OCR3A와 PWM count 스케일이 다르므로
 * 우선 작은 값부터 시작한다.
 */

constexpr double DEADZONE_CM = 2.5;


/*
 * 한 번 판단할 때 움직이는 고정 PWM량.
 *
 * 이전 대회에서는 sensitivity=8이었지만
 * PCA9685에서는 같은 숫자를 그대로 쓰면
 * 실제 서보 이동량이 다르므로 우선 2부터 시작한다.
 */

constexpr int PWM_STEP = 4;


/*
 * ============================================================
 * Ultrasonic Sensor
 * ============================================================
 */

constexpr double MIN_DISTANCE_CM = 20.0;
constexpr double MAX_DISTANCE_CM = 40.0;


/*
 * 이전 대회와 동일한 센서 간 인터리빙
 */

constexpr int SENSOR_INTERVAL_MS = 15;


/*
 * 이전 대회와 동일한 약 50ms 제어 주기
 */

constexpr int CONTROL_PERIOD_MS = 50;


/*
 * ============================================================
 * Servo Direction
 * ============================================================
 *
 * 방향이 반대로 움직이면 1 ↔ -1 변경.
 */

constexpr int YAW_DIRECTION = 1;


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
         * TRIG 설정
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
         * ECHO 설정
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
                << "[ERROR] GPIO request 실패\n";
        }
    }


    ~SRF05()
    {
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
         * ECHO HIGH 대기
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


        auto echoStart =
            std::chrono::steady_clock::now();


        /*
         * ECHO LOW 대기
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
         * 거리 계산
         */

        double pulseTimeUs =
            std::chrono::duration<double, std::micro>(
                echoEnd - echoStart)
                .count();


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
 * main
 * ============================================================
 */

int main()
{
    std::cout
        << "========================================\n"
        << " Previous Competition Yaw Tracking Test\n"
        << "========================================\n";


    PCA9685 pwm;


    if (!pwm.begin())
    {
        std::cerr
            << "[ERROR] PCA9685 초기화 실패\n";

        return 1;
    }


    pwm.setPWMFreq(50);


    /*
     * 센서 생성
     */

    SRF05 leftSensor(
        LEFT_TRIG_PIN,
        LEFT_ECHO_PIN);


    SRF05 rightSensor(
        RIGHT_TRIG_PIN,
        RIGHT_ECHO_PIN);


    /*
     * 중심 위치
     */

    int yawPWM =
        YAW_CENTER_PWM;


    pwm.setPWM(
        YAW_SERVO_CHANNEL,
        0,
        yawPWM);


    std::cout
        << "[INIT] Yaw PWM : "
        << yawPWM
        << '\n';


    std::this_thread::sleep_for(
        std::chrono::seconds(1));


    while (true)
    {
        /*
         * ====================================================
         * LEFT
         * ====================================================
         */

        double leftDistance =
            leftSensor.measureDistance();


        /*
         * 이전 대회와 동일하게
         * 센서 간 15ms 대기
         */

        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                SENSOR_INTERVAL_MS));


        /*
         * ====================================================
         * RIGHT
         * ====================================================
         */

        double rightDistance =
            rightSensor.measureDistance();


        /*
         * ====================================================
         * Sensor Validity
         * ====================================================
         *
         * 이전 대회처럼 한쪽 센서만 검출될 경우에도
         * 해당 방향으로 추적한다.
         */


        /*
         * 양쪽 모두 유효
         */

        if (leftDistance >= 0.0 &&
            rightDistance >= 0.0)
        {
            double difference =
                leftDistance -
                rightDistance;


            /*
             * Dead Zone
             *
             * 거리차가 ±2cm 이내라면
             * 중앙에 있다고 판단하여 서보를 움직이지 않는다.
             */

            if (std::abs(difference) <=
                DEADZONE_CM)
            {
                std::cout
                    << "[CENTER] ";
            }


            /*
             * 이전 대회 알고리즘:
             *
             * dL < dR
             * → 한 방향으로 고정 step 이동
             */

            else if (
                leftDistance <
                rightDistance)
            {
                yawPWM +=
                    PWM_STEP *
                    YAW_DIRECTION;


                std::cout
                    << "[TRACK LEFT] ";
            }


            /*
             * dR < dL
             * → 반대 방향으로 고정 step 이동
             */

            else
            {
                yawPWM -=
                    PWM_STEP *
                    YAW_DIRECTION;


                std::cout
                    << "[TRACK RIGHT] ";
            }
        }


        /*
         * LEFT만 유효
         */

        else if (
            leftDistance >= 0.0 &&
            rightDistance < 0.0)
        {
            yawPWM +=
                PWM_STEP *
                YAW_DIRECTION;


            std::cout
                << "[LEFT ONLY] ";
        }


        /*
         * RIGHT만 유효
         */

        else if (
            rightDistance >= 0.0 &&
            leftDistance < 0.0)
        {
            yawPWM -=
                PWM_STEP *
                YAW_DIRECTION;


            std::cout
                << "[RIGHT ONLY] ";
        }


        /*
         * 둘 다 LOST
         */

        else
        {
            /*
             * 이전 대회에서는 타겟 소실 시
             * 정면 중심 위치로 즉시 복귀했다.
             */

            yawPWM =
                YAW_CENTER_PWM;


            std::cout
                << "[TARGET LOST -> CENTER] ";
        }


        /*
         * PWM 안전범위
         */

        yawPWM =
            std::clamp(
                yawPWM,
                YAW_PWM_MIN,
                YAW_PWM_MAX);


        /*
         * Servo 적용
         */

        pwm.setPWM(
            YAW_SERVO_CHANNEL,
            0,
            yawPWM);


        /*
         * Debug
         */

        std::cout
            << "L="
            << leftDistance

            << " R="
            << rightDistance

            << " Diff="
            << (
                leftDistance >= 0.0 &&
                rightDistance >= 0.0
                ? leftDistance -
                    rightDistance
                : 0.0
            )

            << " PWM="
            << yawPWM
            << '\n';


        /*
         * 이전 대회 메인 루프 약 50ms
         */

        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                CONTROL_PERIOD_MS));
    }


    return 0;
}