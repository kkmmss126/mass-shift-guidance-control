#include "PCA9685.h"

#include <gpiod.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>


/*
 * ============================================================
 * Seeker Yaw Center Calibration
 * ============================================================
 *
 * 목적
 * ------------------------------------------------------------
 * 1. Yaw 서보를 중심 PWM에 고정한다.
 * 2. 좌/우 초음파 센서만 측정한다.
 * 3. 정지 표적을 중앙에 둔 상태에서 L-R 오차를 측정한다.
 * 4. 여러 번 측정한 평균값을 Yaw 중심 오프셋으로 사용한다.
 *
 * 중요
 * ------------------------------------------------------------
 * 이 프로그램에서는 서보 추적 제어를 하지 않는다.
 * 즉, 측정 중 Yaw 서보는 PWM 300에 계속 고정된다.
 */


/*
 * ============================================================
 * GPIO 설정
 * ============================================================
 */

// Left Ultrasonic Sensor
constexpr unsigned int LEFT_TRIG_PIN = 12;
constexpr unsigned int LEFT_ECHO_PIN = 13;

// Right Ultrasonic Sensor
constexpr unsigned int RIGHT_TRIG_PIN = 5;
constexpr unsigned int RIGHT_ECHO_PIN = 20;


/*
 * ============================================================
 * PCA9685 / Servo 설정
 * ============================================================
 */

constexpr int YAW_SERVO_CHANNEL = 1;

/*
 * 기존 캘리브레이션 중심값
 */
constexpr int YAW_CENTER_PWM = 300;


/*
 * ============================================================
 * 초음파 센서 설정
 * ============================================================
 */

constexpr double MIN_DISTANCE_CM = 20.0;
constexpr double MAX_DISTANCE_CM = 40.0;

/*
 * 좌측 센서 측정 후
 * 우측 센서를 측정하기 전 대기시간
 */
constexpr int SENSOR_INTERVAL_MS = 15;

/*
 * 한 세트 측정 이후 다음 측정까지 대기시간
 */
constexpr int SAMPLE_PERIOD_MS = 50;

/*
 * 정상 측정값을 몇 개 확보할지 설정
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
         *
         * 기존 코드의 1ms가 아니라
         * 10us만 유지한다.
         */

        gpiod_line_request_set_value(
            m_trigRequest,
            m_trigPin,
            GPIOD_LINE_VALUE_ACTIVE);


        std::this_thread::sleep_for(
            std::chrono::microseconds(10));


        /*
         * 다시 LOW로 내려 측정을 시작한다.
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
         * ECHO 펄스 폭 계산
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
         * 사용 범위 확인
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
        << "   Seeker Yaw Center Calibration\n"
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
     * Yaw 서보를 중심 위치에 고정한다.
     */

    pwm.setPWM(
        YAW_SERVO_CHANNEL,
        0,
        YAW_CENTER_PWM);


    std::cout
        << "[INIT] Yaw servo fixed at PWM "
        << YAW_CENTER_PWM
        << "\n";


    /*
     * 서보가 실제 중심 위치까지 이동할 시간을 준다.
     */

    std::this_thread::sleep_for(
        std::chrono::seconds(1));


    /*
     * 초음파 센서 생성
     */

    SRF05 leftSensor(
        LEFT_TRIG_PIN,
        LEFT_ECHO_PIN);


    SRF05 rightSensor(
        RIGHT_TRIG_PIN,
        RIGHT_ECHO_PIN);


    std::cout
        << "\n표적을 시커 정중앙에 고정하세요.\n"
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
    double errorSum = 0.0;


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
         * 센서 간 초음파 간섭 방지를 위해 대기
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
         * 한 센서라도 실패했다면
         * 해당 측정은 평균 계산에 포함하지 않는다.
         */

        if (leftDistance < 0.0 ||
            rightDistance < 0.0)
        {
            failedSampleCount++;


            std::cout
                << "[INVALID]"
                << " L=" << leftDistance
                << " R=" << rightDistance
                << "  Failed="
                << failedSampleCount
                << '\n';


            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    SAMPLE_PERIOD_MS));


            continue;
        }


        /*
         * 좌우 오차 계산
         *
         * 추적 코드와 동일하게
         * Left - Right를 사용한다.
         */

        double yawError =
            leftDistance -
            rightDistance;


        /*
         * 평균 계산용 누적
         */

        leftSum +=
            leftDistance;

        rightSum +=
            rightDistance;

        errorSum +=
            yawError;


        validSampleCount++;


        /*
         * 현재 측정값 출력
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

            << "  L-R="
            << yawError

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


    double averageYawOffset =
        errorSum /
        validSampleCount;


    /*
     * ========================================================
     * 최종 결과 출력
     * ========================================================
     */

    std::cout
        << "\n========================================\n"
        << "         Calibration Result\n"
        << "========================================\n"

        << std::fixed
        << std::setprecision(3)

        << "Valid samples : "
        << validSampleCount
        << '\n'

        << "Failed samples: "
        << failedSampleCount
        << '\n'

        << "Average Left  : "
        << averageLeft
        << " cm\n"

        << "Average Right : "
        << averageRight
        << " cm\n"

        << "Average L-R   : "
        << averageYawOffset
        << " cm\n\n"

        << "Recommended code:\n\n"

        << "constexpr double YAW_CENTER_OFFSET_CM = "
        << averageYawOffset
        << ";\n"

        << "========================================\n";


    /*
     * 서보는 중심 PWM을 유지한 상태로 종료한다.
     */

    return 0;
}
