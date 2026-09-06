/*
 * ============================================================
 * File Name : seeker_tracking_test.cpp
 *
 * Description
 * ------------------------------------------------------------
 * 삼중 초음파 센서와 2축 서보모터를 이용한
 * 시커부 독립 표적 추적 테스트 프로그램.
 *
 * 개선 사항
 * ------------------------------------------------------------
 * 1. 초음파 센서 순간 측정 실패 허용
 * 2. 이전 정상 측정값 유지
 * 3. 급격한 이상값(Jump) 제거
 * 4. EMA 필터 적용
 * 5. 최소 PWM 이동량 보장
 * 6. 한 루프 최대 PWM 이동량 제한
 * 7. 제어 주기 단축
 *
 * Project
 * ------------------------------------------------------------
 * Mass Shift Guidance Control System
 * ============================================================
 */

#include "PCA9685.h"

#include <gpiod.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>


/*
 * ============================================================
 * GPIO 설정
 * ============================================================
 *
 * 회로도 기준 BCM GPIO 번호
 *
 * U9  : 좌측 초음파 센서
 * U10 : 우측 초음파 센서
 * U11 : 하단 초음파 센서
 */

// U9 - Left
constexpr unsigned int LEFT_TRIG_PIN = 5;
constexpr unsigned int LEFT_ECHO_PIN = 20;

// U10 - Right
constexpr unsigned int RIGHT_TRIG_PIN = 12;
constexpr unsigned int RIGHT_ECHO_PIN = 13;

// U11 - Bottom
constexpr unsigned int BOTTOM_TRIG_PIN = 16;
constexpr unsigned int BOTTOM_ECHO_PIN = 6;


/*
 * ============================================================
 * PCA9685 Servo Channel
 * ============================================================
 */

constexpr int SERVO_TOP_CHANNEL = 0;
constexpr int SERVO_BOTTOM_CHANNEL = 1;


/*
 * ============================================================
 * Servo Center PWM
 * ============================================================
 *
 * 기존 캘리브레이션에서 측정한 중심값.
 */

constexpr int SERVO_TOP_CENTER_PWM = 321;
constexpr int SERVO_BOTTOM_CENTER_PWM = 300;


/*
 * ============================================================
 * Servo PWM 안전 범위
 * ============================================================
 *
 * 초기 테스트용 제한이다.
 * 실제 기구물의 최대 회전 범위를 확인한 후 수정한다.
 */

constexpr int TOP_PWM_MIN = 0;
constexpr int TOP_PWM_MAX = 500;

constexpr int BOTTOM_PWM_MIN = 0;
constexpr int BOTTOM_PWM_MAX = 500;


/*
 * ============================================================
 * P 제어 Gain
 * ============================================================
 *
 * 일단 1.0으로 시작한다.
 *
 * 추적이 너무 느리면 조금씩 증가시키고,
 * 진동하거나 표적을 지나치면 감소시킨다.
 */

constexpr double KP_YAW = 3.0;
constexpr double KP_PITCH = 3.0;


/*
 * ============================================================
 * Dead Zone
 * ============================================================
 *
 * 이 범위 안에서는 중심에 도달했다고 판단한다.
 */

constexpr double YAW_DEADZONE_CM = 0.5;
constexpr double PITCH_DEADZONE_CM = 0.5;


/*
 * ============================================================
 * 초음파 센서 측정 범위
 * ============================================================
 *
 * 현재 실제 테스트 과정에서는 30cm 이상의 값도
 * 확인할 필요가 있으므로 100cm까지 허용한다.
 *
 * 최종 실험에서는 프로젝트 운용거리로 다시 제한할 수 있다.
 */

constexpr double MIN_DISTANCE_CM = 2.0;
constexpr double MAX_DISTANCE_CM = 30.0;


/*
 * ============================================================
 * 센서 측정 간격
 * ============================================================
 *
 * 세 센서를 동시에 발사하면 초음파 간섭이 생길 수 있으므로
 * 순차 측정한다.
 *
 * 기존 15ms보다 조금 줄여 응답속도를 높인다.
 *
 * 만약 센서값이 다시 심하게 튄다면
 * 10 → 15 → 20ms 순으로 증가시키면서 확인한다.
 */

constexpr int SENSOR_INTERVAL_MS = 15;


/*
 * 한 번의 전체 측정 이후 추가 대기시간.
 *
 * 별도의 50ms 대기를 제거하여 추적 응답속도를 높인다.
 */

constexpr int CONTROL_PERIOD_MS = 5;


/*
 * ============================================================
 * 센서 오류 허용 횟수
 * ============================================================
 *
 * 센서가 한 번 측정 실패했다고 바로 TARGET LOST 처리하지 않는다.
 *
 * 최대 2회까지 이전 정상값을 사용하고,
 * 3회 연속 실패하면 해당 센서를 LOST 상태로 판단한다.
 */

constexpr int MAX_SENSOR_FAILURE_COUNT = 2;


/*
 * ============================================================
 * 센서 Jump 제한
 * ============================================================
 *
 * 직전 정상값과 비교하여 한 번에 지나치게 큰 거리 변화가
 * 발생하면 초음파 반사 노이즈로 판단한다.
 *
 * 예:
 *
 *  6.3 cm
 *  6.5 cm
 *  47.2 cm  <-- 이상값
 *  6.4 cm
 *
 * 47.2cm를 제어에 사용하지 않는다.
 */

constexpr double MAX_DISTANCE_JUMP_CM = 1000.0;


/*
 * ============================================================
 * EMA 필터 계수
 * ============================================================
 *
 * filtered =
 *     alpha * newValue
 *   + (1-alpha) * previousFiltered
 *
 * alpha가 클수록:
 *  - 빠르게 반응
 *  - 노이즈 증가
 *
 * alpha가 작을수록:
 *  - 부드러움
 *  - 반응 느림
 *
 * 시커 추적이 목적이므로 비교적 빠른 0.45 사용.
 */

constexpr double EMA_ALPHA = 0.45;


/*
 * ============================================================
 * PWM 제어 제한
 * ============================================================
 */

/*
 * Dead Zone 밖인데 계산 결과가 0이면
 * 최소 이 값만큼 움직인다.
 */
constexpr int MIN_PWM_STEP = 1;


/*
 * 센서 노이즈 때문에 서보가 한 번에 크게 튀는 것을 방지한다.
 *
 * 한 제어 루프에서 PWM은 최대 ±4만 변화한다.
 */
constexpr int MAX_PWM_STEP = 4;


/*
 * ============================================================
 * Servo 방향
 * ============================================================
 *
 * 시커가 표적 반대 방향으로 움직이면
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
         * TRIG 핀 설정
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
         * ECHO 핀 설정
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
     * 정상 측정:
     *     거리(cm)
     *
     * 측정 실패:
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
         * ECHO Pulse 시간 계산
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
         * 유효 거리 검사
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
 * SensorFilter
 * ============================================================
 *
 * 각 초음파 센서의 측정값을 관리한다.
 *
 * 기능:
 * 1. 마지막 정상값 저장
 * 2. 순간 Jump 제거
 * 3. EMA 필터
 * 4. 연속 실패 횟수 관리
 */

class SensorFilter
{
public:

    /*
     * 새 측정값을 입력한다.
     *
     * 반환값:
     * true  = 사용할 수 있는 거리값 존재
     * false = 센서 LOST
     */

    bool update(double newValue)
    {
        /*
         * ====================================================
         * 측정 실패
         * ====================================================
         */

        if (newValue < 0.0)
        {
            m_failureCount++;


            /*
             * 이전에 정상값을 확보했고
             * 실패 횟수가 허용 범위 안이라면
             * 이전 필터값을 계속 사용한다.
             */
            if (m_initialized &&
                m_failureCount <=
                    MAX_SENSOR_FAILURE_COUNT)
            {
                return true;
            }


            return false;
        }


        /*
         * ====================================================
         * 첫 정상 측정값
         * ====================================================
         */

        if (!m_initialized)
        {
            m_filteredValue =
                newValue;

            m_lastRawValue =
                newValue;

            m_initialized =
                true;

            m_failureCount =
                0;

            return true;
        }


        /*
         * ====================================================
         * 급격한 Jump 검사
         * ====================================================
         *
         * 직전 정상 Raw 값에서 갑자기 너무 많이 변하면
         * 반사 노이즈로 판단하고 이번 측정을 무시한다.
         */

        if (std::abs(
                newValue -
                m_lastRawValue)
            > MAX_DISTANCE_JUMP_CM)
        {
            m_failureCount++;


            if (m_failureCount <=
                MAX_SENSOR_FAILURE_COUNT)
            {
                return true;
            }


            return false;
        }


        /*
         * 정상 측정이므로 실패 횟수 초기화
         */

        m_failureCount = 0;


        /*
         * 이번 Raw 값을 정상값으로 저장
         */

        m_lastRawValue =
            newValue;


        /*
         * ====================================================
         * EMA Low Pass Filter
         * ====================================================
         */

        m_filteredValue =
            EMA_ALPHA *
                newValue
            +
            (1.0 - EMA_ALPHA) *
                m_filteredValue;


        return true;
    }


    /*
     * 현재 필터링된 거리값 반환
     */

    double value() const
    {
        return m_filteredValue;
    }


private:

    bool m_initialized = false;

    int m_failureCount = 0;

    double m_lastRawValue = 0.0;

    double m_filteredValue = 0.0;
};


/*
 * ============================================================
 * clampPWM()
 * ============================================================
 */

int clampPWM(
    int pwm,
    int minPWM,
    int maxPWM)
{
    return std::clamp(
        pwm,
        minPWM,
        maxPWM);
}


/*
 * ============================================================
 * calculatePWMCorrection()
 * ============================================================
 *
 * P 제어 출력 계산.
 *
 * 특징:
 * 1. round() 사용
 * 2. Dead Zone 밖이면 최소 PWM 이동 보장
 * 3. 한 번에 지나치게 많이 움직이지 않도록 제한
 */

int calculatePWMCorrection(
    double error,
    double kp,
    int direction)
{
    /*
     * Dead Zone 내부에서는 움직이지 않는다.
     */

    if (std::abs(error) <=
        YAW_DEADZONE_CM)
    {
        return 0;
    }


    /*
     * P 제어
     */

    double controlOutput =
        kp *
        error *
        direction;


    /*
     * 기존 static_cast<int>()는
     * 0.8 같은 값을 0으로 잘라버렸다.
     *
     * round()를 사용하면:
     *
     * 0.8 → 1
     * 1.6 → 2
     */

    int correction =
        static_cast<int>(
            std::round(
                controlOutput));


    /*
     * Dead Zone 밖인데도 0이 나온 경우
     * 최소 이동량을 강제로 부여한다.
     */

    if (correction == 0)
    {
        correction =
            (controlOutput > 0.0)
            ? MIN_PWM_STEP
            : -MIN_PWM_STEP;
    }


    /*
     * 한 루프 최대 이동량 제한
     */

    correction =
        std::clamp(
            correction,
            -MAX_PWM_STEP,
            MAX_PWM_STEP);


    return correction;
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
        << "   Seeker Tracking Test Ver.2\n"
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
     * 초음파 센서 생성
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
     * 각 센서별 필터
     */

    SensorFilter leftFilter;
    SensorFilter rightFilter;
    SensorFilter bottomFilter;


    /*
     * ========================================================
     * Servo 초기값
     * ========================================================
     */

    int yawPWM =
        SERVO_BOTTOM_CENTER_PWM;

    int pitchPWM =
        SERVO_TOP_CENTER_PWM;


    /*
     * 시작 시 시커 중심 정렬
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
     * 서보 중심 이동 대기
     */

    std::this_thread::sleep_for(
        std::chrono::seconds(1));


    std::cout
        << "\nTracking Start\n"
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

        double rawLeft =
            leftSensor.measureDistance();


        bool leftValid =
            leftFilter.update(
                rawLeft);


        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                SENSOR_INTERVAL_MS));


        /*
         * ====================================================
         * RIGHT 측정
         * ====================================================
         */

        double rawRight =
            rightSensor.measureDistance();


        bool rightValid =
            rightFilter.update(
                rawRight);


        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                SENSOR_INTERVAL_MS));


        /*
         * ====================================================
         * BOTTOM 측정
         * ====================================================
         */

        double rawBottom =
            bottomSensor.measureDistance();


        bool bottomValid =
            bottomFilter.update(
                rawBottom);


        /*
         * ====================================================
         * 센서 상태 확인
         * ====================================================
         */

        if (!leftValid ||
            !rightValid ||
            !bottomValid)
        {
            std::cout
                << "[TARGET LOST]"
                << " RawL=" << rawLeft
                << " RawR=" << rawRight
                << " RawB=" << rawBottom
                << '\n';


            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    CONTROL_PERIOD_MS));


            continue;
        }


        /*
         * 필터를 통과한 거리값 사용
         */

        double leftDistance =
            leftFilter.value();


        double rightDistance =
            rightFilter.value();


        double bottomDistance =
            bottomFilter.value();


        /*
         * ====================================================
         * Yaw Error
         * ====================================================
         */

        double yawError =
            leftDistance -
            rightDistance;


        /*
         * ====================================================
         * Pitch Error
         * ====================================================
         */

        double upperAverage =
            (leftDistance +
             rightDistance)
            / 2.0;


        double pitchError =
            upperAverage -
            bottomDistance;


        /*
         * ====================================================
         * Yaw P Control
         * ====================================================
         */

        int yawCorrection =
            calculatePWMCorrection(
                yawError,
                KP_YAW,
                YAW_DIRECTION);


        if (yawCorrection != 0)
        {
            yawPWM +=
                yawCorrection;


            yawPWM =
                clampPWM(
                    yawPWM,
                    BOTTOM_PWM_MIN,
                    BOTTOM_PWM_MAX);


            pwm.setPWM(
                SERVO_BOTTOM_CHANNEL,
                0,
                yawPWM);
        }


        /*
         * ====================================================
         * Pitch P Control
         * ====================================================
         *
         * Pitch에서도 동일한 방식 사용.
         */

        int pitchCorrection = 0;


        if (std::abs(pitchError) >
            PITCH_DEADZONE_CM)
        {
            double pitchControl =
                KP_PITCH *
                pitchError *
                PITCH_DIRECTION;


            pitchCorrection =
                static_cast<int>(
                    std::round(
                        pitchControl));


            if (pitchCorrection == 0)
            {
                pitchCorrection =
                    (pitchControl > 0.0)
                    ? MIN_PWM_STEP
                    : -MIN_PWM_STEP;
            }


            pitchCorrection =
                std::clamp(
                    pitchCorrection,
                    -MAX_PWM_STEP,
                    MAX_PWM_STEP);


            pitchPWM +=
                pitchCorrection;


            pitchPWM =
                clampPWM(
                    pitchPWM,
                    TOP_PWM_MIN,
                    TOP_PWM_MAX);


            pwm.setPWM(
                SERVO_TOP_CHANNEL,
                0,
                pitchPWM);
        }


        /*
         * ====================================================
         * Debug 출력
         * ====================================================
         *
         * RAW 값과 Filter 값을 동시에 출력해서
         * 필터가 제대로 동작하는지 확인한다.
         */

        std::cout
            << "RAW["
            << rawLeft << ", "
            << rawRight << ", "
            << rawBottom << "]  "

            << "FILT["
            << leftDistance << ", "
            << rightDistance << ", "
            << bottomDistance << "]  |  "

            << "Err Y:"
            << yawError
            << " P:"
            << pitchError
            << "  |  "

            << "PWM Y:"
            << yawPWM
            << " P:"
            << pitchPWM
            << "  |  "

            << "dPWM Y:"
            << yawCorrection
            << " P:"
            << pitchCorrection
            << '\n';


        /*
         * 전체 루프 추가 대기
         */

        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                CONTROL_PERIOD_MS));
    }


    return 0;
}