/*
 * ============================================================
 * File Name : seeker_tracking_test.cpp
 *
 * Description
 * ------------------------------------------------------------
 * 삼중 초음파 센서와 2축 서보모터를 이용하여
 * 시커부만 독립적으로 표적 추적을 테스트하는 코드이다.
 *
 * Raspberry Pi + PCA9685 + SRF-05 x3 구조를 사용한다.
 *
 * 동작 순서
 * 1. PCA9685 초기화
 * 2. 좌/우/하단 초음파 센서 거리 측정
 * 3. 좌우 오차 및 상하 오차 계산
 * 4. P 제어를 이용해 시커 서보 위치 보정
 * 5. 표적 중심으로 수렴하는지 확인
 *
 * Project
 * ------------------------------------------------------------
 * Mass Shift Guidance Control System
 * ============================================================
 */

#include "PCA9685.h"

#include <gpiod.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>


/*
 * ============================================================
 * SRF-05 GPIO 설정
 * ============================================================
 *
 * 회로도 기준 BCM GPIO 번호
 *
 * U9  : 좌측 센서
 * U10 : 우측 센서
 * U11 : 하단 센서
 *
 * ECHO 신호는 반드시 레벨 시프터를 거쳐
 * Raspberry Pi GPIO로 입력되어야 한다.
 */

// 좌측 초음파 센서 U9
constexpr unsigned int LEFT_TRIG_PIN = 5;
constexpr unsigned int LEFT_ECHO_PIN = 20;

// 우측 초음파 센서 U10
constexpr unsigned int RIGHT_TRIG_PIN = 12;
constexpr unsigned int RIGHT_ECHO_PIN = 13;

// 하단 초음파 센서 U11
constexpr unsigned int BOTTOM_TRIG_PIN = 16;
constexpr unsigned int BOTTOM_ECHO_PIN = 6;


/*
 * ============================================================
 * PCA9685 Servo Channel
 * ============================================================
 *
 * 현재 프로젝트에서 사용하는 서보 채널.
 *
 * 실제 배선이 다르면 이 값만 수정하면 된다.
 */
constexpr int SERVO_TOP_CHANNEL = 0;
constexpr int SERVO_BOTTOM_CHANNEL = 1;


/*
 * ============================================================
 * Servo Center PWM
 * ============================================================
 *
 * 기존 캘리브레이션 값.
 */
constexpr int SERVO_TOP_CENTER_PWM = 321;
constexpr int SERVO_BOTTOM_CENTER_PWM = 300;


/*
 * ============================================================
 * Servo PWM Limit
 * ============================================================
 *
 * 초기 테스트에서는 시커가 기구물 끝까지 움직이지 않도록
 * 비교적 좁은 범위로 제한한다.
 *
 * 실제 동작 확인 후 범위를 확장한다.
 */
constexpr int TOP_PWM_MIN = 270;
constexpr int TOP_PWM_MAX = 370;

constexpr int BOTTOM_PWM_MIN = 250;
constexpr int BOTTOM_PWM_MAX = 350;


/*
 * ============================================================
 * P 제어 게인
 * ============================================================
 *
 * 센서 거리 오차에 따라 PWM을 얼마나 변경할지 결정한다.
 *
 * 처음 테스트할 때는 작은 값으로 시작하는 것이 안전하다.
 */
constexpr double KP_YAW = 0.8;
constexpr double KP_PITCH = 0.8;


/*
 * ============================================================
 * Dead Zone
 * ============================================================
 *
 * 센서값이 미세하게 흔들리더라도
 * 서보가 계속 떨리는 것을 방지한다.
 */
constexpr double YAW_DEADZONE_CM = 0.5;
constexpr double PITCH_DEADZONE_CM = 0.5;


/*
 * ============================================================
 * 유효 거리 범위
 * ============================================================
 */
constexpr double MIN_DISTANCE_CM = 2.0;
constexpr double MAX_DISTANCE_CM = 30.0;


/*
 * 제어 루프 주기
 */
constexpr int CONTROL_PERIOD_MS = 50;


/*
 * ============================================================
 * 서보 회전 방향
 * ============================================================
 *
 * 실제 브라켓 장착 방향에 따라
 * 서보 움직임이 반대로 나올 수 있다.
 *
 * 반대로 움직일 경우
 * 해당 값을 1 ↔ -1로 변경한다.
 */
constexpr int YAW_DIRECTION = 1;
constexpr int PITCH_DIRECTION = 1;


/*
 * ============================================================
 * SRF05 Class
 * ============================================================
 */
class SRF05
{
public:
    SRF05(unsigned int trigPin, unsigned int echoPin)
        : m_trigPin(trigPin),
          m_echoPin(echoPin)
    {
        /*
         * Raspberry Pi GPIO chip open
         */
        m_chip = gpiod_chip_open("/dev/gpiochip0");

        if (m_chip == nullptr)
        {
            std::cerr << "[ERROR] gpiochip0 open 실패\n";
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


        gpiod_line_settings_free(trigSettings);
        gpiod_line_config_free(trigConfig);


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


        gpiod_line_settings_free(echoSettings);
        gpiod_line_config_free(echoConfig);


        /*
         * GPIO 설정 실패 확인
         */
        if (m_trigRequest == nullptr ||
            m_echoRequest == nullptr)
        {
            std::cerr
                << "[ERROR] GPIO request 실패 "
                << "TRIG=" << m_trigPin
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
            gpiod_line_request_release(m_trigRequest);
        }

        if (m_echoRequest != nullptr)
        {
            gpiod_line_request_release(m_echoRequest);
        }

        if (m_chip != nullptr)
        {
            gpiod_chip_close(m_chip);
        }
    }


    /*
     * ========================================================
     * measureDistance()
     * ========================================================
     *
     * 초음파 센서의 거리값을 cm 단위로 반환한다.
     *
     * 정상 : 거리(cm)
     * 실패 : -1.0
     */
    double measureDistance()
    {
        if (m_trigRequest == nullptr ||
            m_echoRequest == nullptr)
        {
            return -1.0;
        }


        /*
         * TRIG를 LOW 상태로 유지
         */
        gpiod_line_request_set_value(
            m_trigRequest,
            m_trigPin,
            GPIOD_LINE_VALUE_INACTIVE);


        std::this_thread::sleep_for(
            std::chrono::microseconds(2));


        /*
         * 10us HIGH Pulse 발생
         */
        gpiod_line_request_set_value(
            m_trigRequest,
            m_trigPin,
            GPIOD_LINE_VALUE_ACTIVE);


        std::this_thread::sleep_for(
            std::chrono::microseconds(10));


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


            /*
             * timeout 발생
             */
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
         * ECHO LOW 전환 대기
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


        /*
         * ECHO 종료 시각
         */
        auto echoEnd =
            std::chrono::steady_clock::now();


        /*
         * HIGH Pulse 시간 계산
         */
        double pulseTimeUs =
            std::chrono::duration<double, std::micro>(
                echoEnd - echoStart)
                .count();


        /*
         * ====================================================
         * 거리 계산
         * ====================================================
         *
         * 초음파 왕복시간 기반 계산
         *
         * distance(cm) ≈ pulseTime(us) / 58
         */
        double distanceCm =
            pulseTimeUs / 58.0;


        /*
         * 유효 측정 범위 확인
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
 * clampPWM()
 * ============================================================
 *
 * PWM 값이 설정한 안전 범위를 벗어나지 않도록 제한한다.
 */
int clampPWM(
    int pwm,
    int minPWM,
    int maxPWM)
{
    if (pwm < minPWM)
    {
        return minPWM;
    }

    if (pwm > maxPWM)
    {
        return maxPWM;
    }

    return pwm;
}


/*
 * ============================================================
 * main()
 * ============================================================
 */
int main()
{
    std::cout
        << "========================================\n"
        << "        Seeker Tracking Test\n"
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


    /*
     * 서보 제어 주파수 50Hz 설정
     */
    pwm.setPWMFreq(50);


    /*
     * ========================================================
     * 초음파 센서 객체 생성
     * ========================================================
     */

    // U9 - 좌측 센서
    SRF05 leftSensor(
        LEFT_TRIG_PIN,
        LEFT_ECHO_PIN);


    // U10 - 우측 센서
    SRF05 rightSensor(
        RIGHT_TRIG_PIN,
        RIGHT_ECHO_PIN);


    // U11 - 하단 센서
    SRF05 bottomSensor(
        BOTTOM_TRIG_PIN,
        BOTTOM_ECHO_PIN);


    /*
     * ========================================================
     * 서보 초기 위치
     * ========================================================
     */
    int yawPWM =
        SERVO_BOTTOM_CENTER_PWM;

    int pitchPWM =
        SERVO_TOP_CENTER_PWM;


    /*
     * 시커를 중심 위치로 이동
     */
    pwm.setPWM(
        SERVO_BOTTOM_CHANNEL,
        0,
        yawPWM);


    pwm.setPWM(
        SERVO_TOP_CHANNEL,
        0,
        pitchPWM);


    std::cout
        << "[INIT] Yaw PWM   : "
        << yawPWM
        << '\n';


    std::cout
        << "[INIT] Pitch PWM : "
        << pitchPWM
        << '\n';


    /*
     * 서보 안정화 시간
     */
    std::this_thread::sleep_for(
        std::chrono::seconds(1));


    std::cout
        << "\nTracking Start\n"
        << "Ctrl+C : 종료\n\n";


    /*
     * ========================================================
     * Main Tracking Loop
     * ========================================================
     */
    while (true)
    {
        /*
         * 센서 상호 간섭을 줄이기 위해
         * 초음파 센서를 순차적으로 측정한다.
         */

        double leftDistance =
            leftSensor.measureDistance();


        std::this_thread::sleep_for(
            std::chrono::milliseconds(15));


        double rightDistance =
            rightSensor.measureDistance();


        std::this_thread::sleep_for(
            std::chrono::milliseconds(15));


        double bottomDistance =
            bottomSensor.measureDistance();


        /*
         * 하나라도 측정 실패하면
         * 해당 루프에서는 서보를 움직이지 않는다.
         */
        if (leftDistance < 0.0 ||
            rightDistance < 0.0 ||
            bottomDistance < 0.0)
        {
            std::cout
                << "[TARGET LOST] "
                << "L=" << leftDistance
                << " R=" << rightDistance
                << " B=" << bottomDistance
                << '\n';


            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    CONTROL_PERIOD_MS));


            continue;
        }


        /*
         * ====================================================
         * 좌우 오차 계산
         * ====================================================
         *
         * 좌측 거리 - 우측 거리
         */
        double yawError =
            leftDistance - rightDistance;


        /*
         * ====================================================
         * 상하 오차 계산
         * ====================================================
         *
         * 상단에 위치한 좌/우 센서 거리 평균과
         * 하단 센서 거리의 차이를 이용한다.
         */
        double upperAverage =
            (leftDistance + rightDistance) / 2.0;


        double pitchError =
            upperAverage - bottomDistance;


        /*
         * ====================================================
         * Yaw 제어
         * ====================================================
         */
        if (std::abs(yawError) >
            YAW_DEADZONE_CM)
        {
            int yawCorrection =
                static_cast<int>(
                    KP_YAW *
                    yawError *
                    YAW_DIRECTION);


            yawPWM +=
                yawCorrection;


            /*
             * PWM 안전 범위 제한
             */
            yawPWM =
                clampPWM(
                    yawPWM,
                    BOTTOM_PWM_MIN,
                    BOTTOM_PWM_MAX);


            /*
             * 좌우 서보 구동
             */
            pwm.setPWM(
                SERVO_BOTTOM_CHANNEL,
                0,
                yawPWM);
        }


        /*
         * ====================================================
         * Pitch 제어
         * ====================================================
         */
        if (std::abs(pitchError) >
            PITCH_DEADZONE_CM)
        {
            int pitchCorrection =
                static_cast<int>(
                    KP_PITCH *
                    pitchError *
                    PITCH_DIRECTION);


            pitchPWM +=
                pitchCorrection;


            /*
             * PWM 안전 범위 제한
             */
            pitchPWM =
                clampPWM(
                    pitchPWM,
                    TOP_PWM_MIN,
                    TOP_PWM_MAX);


            /*
             * 상하 서보 구동
             */
            pwm.setPWM(
                SERVO_TOP_CHANNEL,
                0,
                pitchPWM);
        }


        /*
         * ====================================================
         * 실시간 상태 출력
         * ====================================================
         */
        std::cout
            << "L:"
            << leftDistance
            << "cm  "

            << "R:"
            << rightDistance
            << "cm  "

            << "B:"
            << bottomDistance
            << "cm  |  "

            << "YawErr:"
            << yawError
            << "  "

            << "PitchErr:"
            << pitchError
            << "  |  "

            << "YawPWM:"
            << yawPWM
            << "  "

            << "PitchPWM:"
            << pitchPWM
            << '\n';


        /*
         * 제어 주기
         */
        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                CONTROL_PERIOD_MS));
    }


    return 0;
}