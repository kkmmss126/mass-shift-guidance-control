#include "PCA9685.h"

#include <gpiod.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>


/*
 * ============================================================
 * File Name : seeker_pitch_calibration.cpp
 *
 * Description
 * ------------------------------------------------------------
 * 좌/우/하단 초음파 센서를 이용하여
 * 시커의 상하(Pitch) 방향만 독립적으로 추적한다.
 *
 * Yaw 서보:
 *     중심 PWM에 고정
 *
 * Pitch 서보:
 *     상단 두 센서 평균값과 하단 센서값의 차이를 이용해
 *     고정 PWM Step 방식으로 추적한다.
 *
 * P 제어는 사용하지 않는다.
 *
 * Pitch Calibration Result
 * ------------------------------------------------------------
 *
 * 중앙 정지 상태에서:
 *
 * ((Left + Right) / 2) - Bottom
 *
 * 평균값 ≈ +0.807 cm
 *
 * 따라서 제어 오차 계산 시
 * +0.807 cm의 중심 오프셋을 제거한다.
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
 * Pitch Servo 안전 범위
 * ============================================================
 */

constexpr int PITCH_PWM_MIN = 200;
constexpr int PITCH_PWM_MAX = 450;


/*
 * ============================================================
 * Pitch Center Offset
 * ============================================================
 *
 * 중앙 정지 상태에서 30회 측정한 결과:
 *
 * Average Pitch Error ≈ +0.807 cm
 *
 * 따라서:
 *
 * 실제 제어 오차 =
 *
 * ((L + R) / 2 - B)
 * - PITCH_CENTER_OFFSET_CM
 */

constexpr double PITCH_CENTER_OFFSET_CM = 0.807;


/*
 * ============================================================
 * Pitch Dead Zone
 * ============================================================
 *
 * 오프셋 보정 후 ±1cm 이내라면
 * 표적이 중앙에 있다고 판단한다.
 */

constexpr double PITCH_DEADZONE_CM = 1.0;


/*
 * ============================================================
 * Pitch PWM Step
 * ============================================================
 *
 * Dead Zone을 벗어나면
 * 한 제어 주기마다 3 PWM씩 이동한다.
 */

constexpr int PITCH_PWM_STEP = 3;


/*
 * ============================================================
 * Pitch Servo Direction
 * ============================================================
 *
 * 현재 실제 움직임 방향이 맞았으므로 1 유지.
 *
 * 만약 반대로 움직이면:
 *
 * 1 -> -1
 */

constexpr int PITCH_DIRECTION = 1;


/*
 * ============================================================
 * 초음파 측정 범위
 * ============================================================
 */

constexpr double MIN_DISTANCE_CM = 20.0;
constexpr double MAX_DISTANCE_CM = 40.0;


/*
 * ============================================================
 * 센서 간 측정 간격
 * ============================================================
 */

constexpr int SENSOR_INTERVAL_MS = 15;


/*
 * 한 번의 제어 이후 추가 대기시간
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
     * 실패 또는 측정 범위 밖:
     *     -1.0
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
         * ECHO HIGH 시작 시간
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
         * Echo Pulse Width 계산
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
         * 유효 측정 범위 검사
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
        << "    Seeker Pitch Tracking Test\n"
        << "    Center Offset Applied\n"
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
     * Yaw는 중심에 고정한다.
     */

    pwm.setPWM(
        YAW_SERVO_CHANNEL,
        0,
        YAW_CENTER_PWM);


    /*
     * Pitch도 중심에서 시작한다.
     */

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


    std::cout
        << "[INIT] Pitch Offset: "
        << PITCH_CENTER_OFFSET_CM
        << " cm\n";


    std::cout
        << "[INIT] Dead Zone   : +/-"
        << PITCH_DEADZONE_CM
        << " cm\n";


    std::cout
        << "[INIT] PWM Step    : "
        << PITCH_PWM_STEP
        << '\n';


    /*
     * 서보가 초기 위치까지 이동할 시간
     */

    std::this_thread::sleep_for(
        std::chrono::seconds(1));


    std::cout
        << "\nPitch Tracking Start\n"
        << "Ctrl+C : 종료\n\n";


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
         * ====================================================
         * 세 센서 모두 유효한 경우에만 제어
         * ====================================================
         */

        if (leftDistance >= 0.0 &&
            rightDistance >= 0.0 &&
            bottomDistance >= 0.0)
        {
            /*
             * 상단 두 센서의 평균 거리
             */

            double upperAverage =
                (leftDistance +
                 rightDistance)
                / 2.0;


            /*
             * =================================================
             * Offset 보정 전 Pitch Error
             * =================================================
             */

            double rawPitchError =
                upperAverage -
                bottomDistance;


            /*
             * =================================================
             * Offset 보정 후 실제 제어 Error
             * =================================================
             *
             * 중앙 상태에서 약 +0.807cm가 측정되므로
             * 해당 값을 제거한다.
             */

            double pitchError =
                rawPitchError -
                PITCH_CENTER_OFFSET_CM;


            /*
             * =================================================
             * Dead Zone
             * =================================================
             */

            if (std::abs(pitchError) <=
                PITCH_DEADZONE_CM)
            {
                /*
                 * 중앙으로 판단.
                 *
                 * PWM을 변경하지 않고 현재 위치 유지.
                 */

                std::cout
                    << "[CENTER] ";
            }


            /*
             * =================================================
             * Pitch 한 방향 추적
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
             * Pitch 반대 방향 추적
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
             * =================================================
             * Servo 안전 범위 제한
             * =================================================
             */

            pitchPWM =
                std::clamp(
                    pitchPWM,
                    PITCH_PWM_MIN,
                    PITCH_PWM_MAX);


            /*
             * Pitch Servo 적용
             */

            pwm.setPWM(
                PITCH_SERVO_CHANNEL,
                0,
                pitchPWM);


            /*
             * =================================================
             * Debug 출력
             * =================================================
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

                << " RawErr="
                << rawPitchError

                << " Err="
                << pitchError

                << " PWM="
                << pitchPWM

                << '\n';
        }


        /*
         * ====================================================
         * 센서 하나라도 측정 범위 밖 / 실패
         * ====================================================
         *
         * 현재 위치 유지.
         */

        else
        {
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