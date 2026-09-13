#include "PCA9685.h"

#include <gpiod.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>


/*
 * ============================================================
 * File Name : seeker_pitch_calibration.cpp
 *
 * Description
 * ------------------------------------------------------------
 * Pitch 중심 오프셋 측정 전용 프로그램.
 *
 * 1. Yaw 서보를 PWM 300에 고정한다.
 * 2. Pitch 서보를 PWM 321에 고정한다.
 * 3. 좌/우/하단 초음파 센서를 순차적으로 측정한다.
 * 4. 표적을 시커 정중앙에 고정한다.
 * 5. 정상 측정값 30개를 수집한다.
 *
 * Pitch Raw Error:
 *
 *      ((Left + Right) / 2) - Bottom
 *
 * 이 값의 평균을 Pitch 중심 오프셋으로 사용한다.
 *
 * 중요:
 * ------------------------------------------------------------
 * 이 프로그램에서는 추적 제어를 하지 않는다.
 *
 * 따라서 측정 중:
 *
 * Yaw  = PWM 300 고정
 * Pitch = PWM 321 고정
 *
 * 센서부는 움직이지 않는다.
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
 * PCA9685 / Servo 설정
 * ============================================================
 *
 * Channel 0 = Pitch
 * Channel 1 = Yaw
 */

constexpr int PITCH_SERVO_CHANNEL = 0;
constexpr int YAW_SERVO_CHANNEL   = 1;


/*
 * 기존에 찾은 서보 중심 PWM
 */

constexpr int PITCH_CENTER_PWM = 321;
constexpr int YAW_CENTER_PWM   = 300;


/*
 * ============================================================
 * 초음파 센서 설정
 * ============================================================
 */

constexpr double MIN_DISTANCE_CM = 20.0;
constexpr double MAX_DISTANCE_CM = 40.0;


/*
 * 각 센서 측정 사이의 대기시간
 *
 * LEFT
 *  ↓ 15ms
 * RIGHT
 *  ↓ 15ms
 * BOTTOM
 */

constexpr int SENSOR_INTERVAL_MS = 15;


/*
 * 한 세트 측정 완료 후
 * 다음 측정까지 대기시간
 */

constexpr int SAMPLE_PERIOD_MS = 50;


/*
 * 정상 측정값 수
 */

constexpr int TARGET_SAMPLE_COUNT = 30;


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
     * 실패 / 범위 밖:
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
         * ECHO Pulse Width 계산
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
 * main()
 * ============================================================
 */

int main()
{
    std::cout
        << "========================================\n"
        << "   Seeker Pitch Center Calibration\n"
        << "========================================\n\n";


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
     * 서보 완전 고정
     * ========================================================
     *
     * Calibration 동안에는
     * 이 PWM 값을 변경하지 않는다.
     */

    pwm.setPWM(
        PITCH_SERVO_CHANNEL,
        0,
        PITCH_CENTER_PWM);


    pwm.setPWM(
        YAW_SERVO_CHANNEL,
        0,
        YAW_CENTER_PWM);


    std::cout
        << "[INIT] Pitch servo fixed at PWM "
        << PITCH_CENTER_PWM
        << '\n';


    std::cout
        << "[INIT] Yaw servo fixed at PWM "
        << YAW_CENTER_PWM
        << '\n';


    /*
     * 서보가 중심까지 이동할 시간을 준다.
     */

    std::this_thread::sleep_for(
        std::chrono::seconds(1));


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


    std::cout
        << "\n표적을 시커 정중앙에 고정하세요.\n"

        << "Pitch/Yaw 서보는 측정 중 움직이지 않습니다.\n"

        << "정상 측정값 "
        << TARGET_SAMPLE_COUNT
        << "개를 자동으로 수집합니다.\n\n";


    /*
     * ========================================================
     * 평균 계산 변수
     * ========================================================
     */

    int validSampleCount = 0;

    int failedSampleCount = 0;


    double leftSum = 0.0;

    double rightSum = 0.0;

    double bottomSum = 0.0;

    double upperAverageSum = 0.0;

    double pitchErrorSum = 0.0;


    /*
     * ========================================================
     * Calibration Loop
     * ========================================================
     */

    while (
        validSampleCount <
        TARGET_SAMPLE_COUNT)
    {
        /*
         * LEFT 측정
         */

        double leftDistance =
            leftSensor.measureDistance();


        /*
         * 센서 간 간섭 방지
         */

        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                SENSOR_INTERVAL_MS));


        /*
         * RIGHT 측정
         */

        double rightDistance =
            rightSensor.measureDistance();


        /*
         * 센서 간 간섭 방지
         */

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
         * 유효성 검사
         * ====================================================
         *
         * 세 센서 중 하나라도 실패하면
         * 해당 샘플은 평균 계산에서 제외한다.
         */

        if (leftDistance < 0.0 ||
            rightDistance < 0.0 ||
            bottomDistance < 0.0)
        {
            failedSampleCount++;


            std::cout
                << "[INVALID]"
                << " L=" << leftDistance
                << " R=" << rightDistance
                << " B=" << bottomDistance
                << " Failed="
                << failedSampleCount
                << '\n';


            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    SAMPLE_PERIOD_MS));


            continue;
        }


        /*
         * ====================================================
         * 상단 센서 평균
         * ====================================================
         */

        double upperAverage =
            (leftDistance +
             rightDistance)
            / 2.0;


        /*
         * ====================================================
         * Pitch Raw Error
         * ====================================================
         *
         * 추적 코드에서 사용하는 것과 동일한 식:
         *
         * ((L + R) / 2) - B
         */

        double pitchError =
            upperAverage -
            bottomDistance;


        /*
         * ====================================================
         * 평균 계산용 누적
         * ====================================================
         */

        leftSum +=
            leftDistance;


        rightSum +=
            rightDistance;


        bottomSum +=
            bottomDistance;


        upperAverageSum +=
            upperAverage;


        pitchErrorSum +=
            pitchError;


        validSampleCount++;


        /*
         * ====================================================
         * 현재 측정값 출력
         * ====================================================
         */

        std::cout
            << std::fixed
            << std::setprecision(3)

            << "["
            << validSampleCount
            << "/"
            << TARGET_SAMPLE_COUNT
            << "] "

            << "L="
            << leftDistance

            << "  R="
            << rightDistance

            << "  B="
            << bottomDistance

            << "  UpperAvg="
            << upperAverage

            << "  PitchErr="
            << pitchError

            << '\n';


        /*
         * 다음 측정까지 대기
         */

        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                SAMPLE_PERIOD_MS));
    }


    /*
     * ========================================================
     * 평균 계산
     * ========================================================
     */

    double averageLeft =
        leftSum /
        validSampleCount;


    double averageRight =
        rightSum /
        validSampleCount;


    double averageBottom =
        bottomSum /
        validSampleCount;


    double averageUpper =
        upperAverageSum /
        validSampleCount;


    double averagePitchOffset =
        pitchErrorSum /
        validSampleCount;


    /*
     * ========================================================
     * 최종 Calibration 결과
     * ========================================================
     */

    std::cout
        << "\n========================================\n"
        << "       Pitch Calibration Result\n"
        << "========================================\n"

        << std::fixed
        << std::setprecision(3)

        << "Valid samples     : "
        << validSampleCount
        << '\n'

        << "Failed samples    : "
        << failedSampleCount
        << '\n'

        << "Average Left      : "
        << averageLeft
        << " cm\n"

        << "Average Right     : "
        << averageRight
        << " cm\n"

        << "Average Bottom    : "
        << averageBottom
        << " cm\n"

        << "Average Upper     : "
        << averageUpper
        << " cm\n"

        << "Average Pitch Err : "
        << averagePitchOffset
        << " cm\n\n"

        << "Recommended code:\n\n"

        << "constexpr double PITCH_CENTER_OFFSET_CM = "
        << averagePitchOffset
        << ";\n"

        << "========================================\n";


    /*
     * 프로그램 종료 시점까지
     * Pitch/Yaw는 중심 PWM을 유지한다.
     */

    return 0;
}