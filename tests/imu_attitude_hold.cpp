/*
 * ============================================================
 * File Name : imu_attitude_hold.cpp
 *
 * IMU Based Mass-Shift Attitude Hold
 *
 * 목적
 * ------------------------------------------------------------
 * MPU6050에서 측정한 자세가 초기 기준 자세에서 벗어나면
 * 질량이동 액추에이터를 자동으로 움직여 초기 자세를 유지한다.
 *
 * 기준 IMU 값:
 *
 * ACCEL X = -243.46
 * ACCEL Y = +212.14
 * ACCEL Z = -15456.30
 *
 * GYRO X BIAS = +43.13
 * GYRO Y BIAS = -118.01
 * GYRO Z BIAS = -48.03
 *
 * Actuator:
 * 현재 물리적 중심 = 0 step
 * 이동 범위 = -1300 ~ +1300 step
 *
 * 종료:
 * Q 또는 Ctrl+C
 * -> 액추에이터 0 step 복귀
 * -> A4988 Disable
 * -> 프로그램 종료
 * ============================================================
 */

#include "MPU6050.h"
#include "StepperMotor.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <termios.h>
#include <thread>
#include <unistd.h>


/*
 * ============================================================
 * A4988 GPIO
 * ============================================================
 */

constexpr int STEP_PIN = 17;
constexpr int DIR_PIN = 27;
constexpr int ENABLE_PIN = 22;


/*
 * ============================================================
 * Actuator Limit
 * ============================================================
 */

constexpr long ACTUATOR_MIN_STEP = -1300;
constexpr long ACTUATOR_MAX_STEP = 1300;


/*
 * 한 번의 제어 명령에서 이동할 Step
 *
 * 첫 시험은 너무 크게 움직이지 않도록
 * 10 step부터 시작한다.
 */

constexpr int CONTROL_STEP = 10;


/*
 * STEP Pulse Delay
 */

constexpr int PULSE_DELAY_US = 800;


/*
 * ============================================================
 * IMU Calibration
 * ============================================================
 */

/*
 * 정지 상태 가속도 평균값
 */

constexpr double ACCEL_X_REF = -243.46;
constexpr double ACCEL_Y_REF = 212.14;
constexpr double ACCEL_Z_REF = -15456.30;


/*
 * Gyroscope Bias
 */

constexpr double GYRO_X_BIAS = 43.13;
constexpr double GYRO_Y_BIAS = -118.01;
constexpr double GYRO_Z_BIAS = -48.03;


/*
 * MPU6050 ±250 deg/s 기준
 */

constexpr double GYRO_SCALE = 131.0;


/*
 * ============================================================
 * Attitude Control Settings
 * ============================================================
 */

/*
 * 현재 질량이동 축이 Pitch를 제어한다고 가정한다.
 *
 * 초기 시험에서는 가속도계 기반 Pitch를 사용한다.
 *
 * ±1도 이내에서는 액추에이터를 움직이지 않는다.
 * IMU 노이즈로 인한 지속적인 왕복운동을 막기 위한 값이다.
 */

constexpr double ANGLE_DEADZONE_DEG = 1.0;


/*
 * 액추에이터 이동 방향
 *
 * 실제 시험에서 자세 오차가 더 커진다면
 * 1을 -1로 변경하면 된다.
 */

constexpr int ACTUATOR_DIRECTION = 1;


/*
 * 제어 주기
 *
 * 50ms = 20Hz
 */

constexpr int CONTROL_PERIOD_MS = 50;


/*
 * ============================================================
 * Program State
 * ============================================================
 */

volatile std::sig_atomic_t stopRequested = 0;


/*
 * ============================================================
 * Ctrl+C Handler
 * ============================================================
 */

void signalHandler(int signal)
{
    if (signal == SIGINT)
    {
        /*
         * Signal Handler에서는
         * 모터를 직접 움직이지 않는다.
         *
         * 메인 루프에 종료 요청만 전달한다.
         */

        stopRequested = 1;
    }
}


/*
 * ============================================================
 * Keyboard Check
 * ============================================================
 *
 * Q 입력을 확인하기 위한 비동기 키 입력 함수.
 */

bool keyPressed()
{
    fd_set set;

    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);

    timeval timeout {};

    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    return select(
               STDIN_FILENO + 1,
               &set,
               nullptr,
               nullptr,
               &timeout) > 0;
}


/*
 * ============================================================
 * Safe Move
 * ============================================================
 *
 * 액추에이터 위치가
 * -1300 ~ +1300 범위를 벗어나지 않도록 한다.
 */

bool safeMove(
    StepperMotor& motor,
    int requestedSteps)
{
    long currentPosition =
        motor.getCurrentPosition();


    long targetPosition =
        currentPosition +
        requestedSteps;


    /*
     * +1300 제한
     */

    if (targetPosition >
        ACTUATOR_MAX_STEP)
    {
        requestedSteps =
            static_cast<int>(
                ACTUATOR_MAX_STEP -
                currentPosition);
    }


    /*
     * -1300 제한
     */

    else if (targetPosition <
             ACTUATOR_MIN_STEP)
    {
        requestedSteps =
            static_cast<int>(
                ACTUATOR_MIN_STEP -
                currentPosition);
    }


    /*
     * 이미 한계 위치
     */

    if (requestedSteps == 0)
    {
        return true;
    }


    return motor.moveSteps(
        requestedSteps);
}


/*
 * ============================================================
 * Return To Center
 * ============================================================
 *
 * 프로그램 종료 시
 * 액추에이터를 시작 위치인 0 step으로 복귀시킨다.
 */

bool returnToCenter(
    StepperMotor& motor)
{
    long currentPosition =
        motor.getCurrentPosition();


    std::cout
        << "\n========================================\n"
        << " Returning Actuator To Start Position\n"
        << "========================================\n"

        << "[RETURN] Current Position = "
        << currentPosition
        << " step\n";


    /*
     * 이미 시작 위치
     */

    if (currentPosition == 0)
    {
        std::cout
            << "[RETURN] Already At 0 Step\n";

        return true;
    }


    /*
     * 현재 위치의 반대만큼 이동하면
     * 시작 위치인 0 step으로 돌아간다.
     */

    long returnSteps =
        -currentPosition;


    std::cout
        << "[RETURN] Moving "
        << returnSteps
        << " step\n";


    if (!motor.moveSteps(
            static_cast<int>(
                returnSteps)))
    {
        std::cerr
            << "[ERROR] Return Failed\n";

        return false;
    }


    std::cout
        << "[RETURN] Final Position = "
        << motor.getCurrentPosition()
        << " step\n";


    return true;
}


/*
 * ============================================================
 * Pitch Calculation
 * ============================================================
 *
 * 가속도 센서의 중력벡터를 이용해
 * Pitch 각도를 계산한다.
 *
 * 단위: degree
 */

double calculatePitch(
    double ax,
    double ay,
    double az)
{
    constexpr double RAD_TO_DEG =
        180.0 / 3.14159265358979323846;


    return std::atan2(
               -ax,
               std::sqrt(
                   ay * ay +
                   az * az))
           * RAD_TO_DEG;
}


/*
 * ============================================================
 * Main
 * ============================================================
 */

int main()
{
    std::cout
        << "========================================\n"
        << " IMU Mass-Shift Attitude Hold\n"
        << "========================================\n";


    /*
     * Ctrl+C 등록
     */

    std::signal(
        SIGINT,
        signalHandler);


    /*
     * ========================================================
     * MPU6050
     * ========================================================
     */

    MPU6050 imu;


    if (!imu.begin())
    {
        std::cerr
            << "[ERROR] MPU6050 initialization failed\n";

        return 1;
    }


    std::cout
        << "[INIT] MPU6050 OK\n";


    /*
     * ========================================================
     * Actuator
     * ========================================================
     */

    StepperMotor actuator(
        STEP_PIN,
        DIR_PIN,
        ENABLE_PIN);


    if (!actuator.initialize())
    {
        std::cerr
            << "[ERROR] Actuator initialization failed\n";

        return 1;
    }


    actuator.enable();


    actuator.setPulseDelay(
        PULSE_DELAY_US);


    /*
     * 현재 물리적 중심을
     * 프로그램상 0 step으로 정의
     */

    actuator.setCurrentPosition(0);


    /*
     * ========================================================
     * Reference Attitude
     * ========================================================
     *
     * 사용자가 측정한 초기 가속도 평균값으로
     * 목표 Pitch를 계산한다.
     */

    const double TARGET_PITCH_DEG =
        calculatePitch(
            ACCEL_X_REF,
            ACCEL_Y_REF,
            ACCEL_Z_REF);


    std::cout
        << "[INIT] Start Position = 0 step\n"

        << "[INIT] Range = "
        << ACTUATOR_MIN_STEP
        << " ~ +"
        << ACTUATOR_MAX_STEP
        << " step\n"

        << "[INIT] Target Pitch = "
        << TARGET_PITCH_DEG
        << " deg\n"

        << "[INIT] Dead Zone = +/-"
        << ANGLE_DEADZONE_DEG
        << " deg\n"

        << "[INIT] Control Step = "
        << CONTROL_STEP
        << " step\n";


    std::cout
        << "\nAttitude Hold Start\n"
        << "Q / Ctrl+C : Return to 0 + Exit\n\n";


    /*
     * ========================================================
     * Main Control Loop
     * ========================================================
     */

    while (!stopRequested)
    {
        /*
         * ====================================================
         * Q 입력 확인
         * ====================================================
         */

        if (keyPressed())
        {
            char key = 0;

            if (read(
                    STDIN_FILENO,
                    &key,
                    1) > 0)
            {
                if (key == 'q' ||
                    key == 'Q')
                {
                    stopRequested = 1;

                    break;
                }
            }
        }


        /*
         * ====================================================
         * IMU Raw Data
         * ====================================================
         */

        int16_t ax = 0;
        int16_t ay = 0;
        int16_t az = 0;

        int16_t gx = 0;
        int16_t gy = 0;
        int16_t gz = 0;


        if (!imu.readRawData(
                ax,
                ay,
                az,
                gx,
                gy,
                gz))
        {
            std::cerr
                << "[WARNING] IMU Read Failed\n";


            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    CONTROL_PERIOD_MS));


            continue;
        }


        /*
         * ====================================================
         * 현재 Pitch 계산
         * ====================================================
         */

        double currentPitch =
            calculatePitch(
                static_cast<double>(ax),
                static_cast<double>(ay),
                static_cast<double>(az));


        /*
         * 목표 자세와 현재 자세의 차이
         */

        double pitchError =
            TARGET_PITCH_DEG -
            currentPitch;


        /*
         * ====================================================
         * Gyro Bias Correction
         * ====================================================
         *
         * 현재 버전에서는 제어에 직접 사용하지 않지만
         * 다음 complementary filter 적용을 위해 계산한다.
         */

        double correctedGY =
            static_cast<double>(gy) -
            GYRO_Y_BIAS;


        double gyroYDegPerSec =
            correctedGY /
            GYRO_SCALE;


        /*
         * ====================================================
         * Attitude Control
         * ====================================================
         */

        int moveCommand = 0;


        /*
         * 목표보다 Pitch가 한쪽으로 벗어난 경우
         */

        if (pitchError >
            ANGLE_DEADZONE_DEG)
        {
            moveCommand =
                CONTROL_STEP *
                ACTUATOR_DIRECTION;
        }


        /*
         * 반대쪽으로 벗어난 경우
         */

        else if (pitchError <
                 -ANGLE_DEADZONE_DEG)
        {
            moveCommand =
                -CONTROL_STEP *
                ACTUATOR_DIRECTION;
        }


        /*
         * Dead Zone 내부라면
         * 현재 질량 위치 유지
         */

        else
        {
            moveCommand = 0;
        }


        /*
         * ====================================================
         * Mass Shift
         * ====================================================
         */

        if (moveCommand != 0)
        {
            if (!safeMove(
                    actuator,
                    moveCommand))
            {
                std::cerr
                    << "[ERROR] Actuator Move Failed\n";

                stopRequested = 1;

                break;
            }
        }


        /*
         * ====================================================
         * Debug
         * ====================================================
         */

        std::cout
            << "Target:"
            << TARGET_PITCH_DEG

            << " Current:"
            << currentPitch

            << " Error:"
            << pitchError

            << " GyroY:"
            << gyroYDegPerSec

            << " | Step:"
            << moveCommand

            << " Pos:"
            << actuator.getCurrentPosition()

            << '\n';


        /*
         * ====================================================
         * Control Period
         * ====================================================
         */

        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                CONTROL_PERIOD_MS));
    }


    /*
     * ========================================================
     * Safe Shutdown
     * ========================================================
     *
     * Q 또는 Ctrl+C가 들어오면
     * 현재 질량 위치와 관계없이
     * 프로그램 시작 위치인 0으로 복귀한다.
     */

    returnToCenter(
        actuator);


    actuator.disable();


    std::cout
        << "[EXIT] A4988 Disabled\n"
        << "[EXIT] Program Finished\n";


    return 0;
}