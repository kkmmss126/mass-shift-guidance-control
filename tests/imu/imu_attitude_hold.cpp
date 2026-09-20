/*
 * ============================================================
 * File Name : imu_attitude_hold.cpp
 *
 * IMU Based Mass-Shift Attitude Hold
 *
 * Description
 * ------------------------------------------------------------
 * MPU6050에서 측정한 Pitch 자세가 목표 자세에서 벗어나면
 * 질량이동 액추에이터를 이용하여 자세를 보정한다.
 *
 * 제어 방식:
 * - Pitch Error 기반 P 제어
 * - 오차가 클수록 한 번에 더 많은 Step 이동
 * - Dead Zone 내부에서는 질량 위치 유지
 *
 * Actuator:
 * - 프로그램 시작 위치 = 0 step
 * - 소프트웨어 이동 범위 = -1300 ~ +1300 step
 *
 * 종료:
 * - Q 또는 Ctrl+C
 * - 현재 누적 위치의 반대 방향으로 이동
 * - 프로그램 시작 위치인 0 step으로 복귀
 *
 * Project:
 * Mass Shift Guidance Control System
 * ============================================================
 */


#include "MPU6050.h"
#include "StepperMotor.h"
#include "calibration_data.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>
#include <unistd.h>
#include <sys/select.h>


/*
 * ============================================================
 * A4988 GPIO
 * ============================================================
 *
 * BCM GPIO 번호
 */

constexpr int STEP_PIN = 17;
constexpr int DIR_PIN = 27;
constexpr int ENABLE_PIN = 22;


/*
 * ============================================================
 * Actuator Software Limit
 * ============================================================
 *
 * 프로그램 시작 시 현재 물리 위치를 0 step으로 정의한다.
 *
 * 현재 위치를 기준으로 ±1300 step 범위 안에서만
 * 액추에이터가 움직일 수 있도록 제한한다.
 */

constexpr long ACTUATOR_MIN_STEP = -1300;
constexpr long ACTUATOR_MAX_STEP = 1300;


/*
 * ============================================================
 * Stepper Motor Speed
 * ============================================================
 *
 * 기존 테스트:
 * 1500 us
 *
 * Attitude Hold:
 * 빠른 자세 보정을 위해 500 us부터 시험한다.
 *
 * 모터가 Step을 놓치거나 진동하면
 * 600 ~ 800 us로 증가시킨다.
 */

constexpr int PULSE_DELAY_US = 500;


/*
 * ============================================================
 * Target Attitude
 * ============================================================
 *
 * 2026-09-20 imu_reference_test에서 측정한
 * 현재 기준 Pitch 자세.
 */

constexpr double TARGET_PITCH_DEG = 0.1832;


/*
 * ============================================================
 * P Controller
 * ============================================================
 *
 * moveStep = |Pitch Error| * KP_STEP
 *
 * 예:
 *
 * Error 0.5 deg -> 약 7 step
 * Error 1.0 deg -> 15 step
 * Error 2.0 deg -> 30 step
 * Error 3.0 deg -> 45 step
 * Error 4.0 deg -> 60 step
 *
 * 단, MIN / MAX 범위로 제한한다.
 */

constexpr double KP_STEP = 22.0;


/*
 * 최소 제어 Step
 */

constexpr int MIN_CONTROL_STEP = 5;


/*
 * 최대 제어 Step
 *
 * 한 번에 지나치게 많은 Step을 이동하면
 * IMU 측정이 중단되는 시간이 길어질 수 있으므로
 * 우선 60 step으로 제한한다.
 */

constexpr int MAX_CONTROL_STEP = 90;


/*
 * ============================================================
 * Dead Zone
 * ============================================================
 *
 * ±0.5도 이내에서는 질량을 이동시키지 않는다.
 *
 * IMU 노이즈에 의한 지속적인 왕복 이동 방지.
 */

constexpr double ANGLE_DEADZONE_DEG = 0.5;


/*
 * ============================================================
 * Actuator Direction
 * ============================================================
 *
 * 자세 오차가 +일 때 이동해야 하는 방향.
 *
 * 실제 테스트에서 오차가 더 커지는 방향으로
 * 질량이 이동한다면 1 -> -1로 변경한다.
 */

constexpr int ACTUATOR_DIRECTION = 1;


/*
 * ============================================================
 * Static Trim
 * ============================================================
 *
 * 시커부 무게 때문에 동체 앞쪽이 무거운 경우
 * 내부 질량을 시작부터 일정 Step 이동시켜
 * 정적 무게 불균형을 일부 보상할 수 있다.
 *
 * 현재는 기능 확인을 위해 0으로 둔다.
 *
 * 방향 확인 후:
 *
 * +100, +200 ...
 *
 * 또는
 *
 * -100, -200 ...
 *
 * 순서로 시험한다.
 */

constexpr int STATIC_TRIM_STEP = 0;


/*
 * ============================================================
 * Control Period
 * ============================================================
 *
 * 기존 50 ms -> 20 ms
 *
 * 약 50 Hz 주기로 자세 상태를 확인한다.
 *
 * 단 실제 주기는 moveSteps() 실행 시간만큼 추가된다.
 */

constexpr int CONTROL_PERIOD_MS = 20;


/*
 * ============================================================
 * MPU6050 Gyro Scale
 * ============================================================
 *
 * ±250 deg/s
 */

constexpr double GYRO_SCALE = 131.0;


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
         * Signal Handler 내부에서 모터를 직접 움직이지 않는다.
         *
         * 메인 루프에 종료 요청만 전달한다.
         */

        stopRequested = 1;
    }
}


/*
 * ============================================================
 * Keyboard Input Check
 * ============================================================
 *
 * Q 입력을 Non-blocking 방식으로 확인한다.
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
 * 액추에이터가 소프트웨어 제한
 *
 * -1300 ~ +1300 step
 *
 * 범위를 벗어나지 않도록 제한한다.
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
     * + 방향 제한
     */

    if (targetPosition > ACTUATOR_MAX_STEP)
    {
        requestedSteps =
            static_cast<int>(
                ACTUATOR_MAX_STEP -
                currentPosition);
    }


    /*
     * - 방향 제한
     */

    else if (targetPosition < ACTUATOR_MIN_STEP)
    {
        requestedSteps =
            static_cast<int>(
                ACTUATOR_MIN_STEP -
                currentPosition);
    }


    /*
     * 이미 한계 위치인 경우
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
 * Return To Start Position
 * ============================================================
 *
 * 현재 논리 위치의 반대 Step만큼 이동하여
 * 프로그램 시작 위치인 0 step으로 복귀한다.
 */

bool returnToStart(
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
     * 현재 위치의 반대 방향으로 복귀
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
 * 가속도계의 중력 벡터를 이용하여
 * 현재 Pitch 각도를 계산한다.
 *
 * 단위 : degree
 */

double calculatePitch(
    double ax,
    double ay,
    double az)
{
    constexpr double RAD_TO_DEG =
        180.0 /
        3.14159265358979323846;


    return std::atan2(
               -ax,
               std::sqrt(
                   ay * ay +
                   az * az))
           * RAD_TO_DEG;
}


/*
 * ============================================================
 * P Controller
 * ============================================================
 *
 * Pitch Error를 입력받아
 * 실제 액추에이터 이동 Step을 계산한다.
 */

int calculateControlStep(
    double pitchError)
{
    /*
     * Dead Zone
     */

    if (std::abs(pitchError)
        <= ANGLE_DEADZONE_DEG)
    {
        return 0;
    }


    /*
     * P Control
     */

    int controlStep =
        static_cast<int>(
            std::abs(pitchError) *
            KP_STEP);


    /*
     * 최소 / 최대 이동량 제한
     */

    controlStep =
        std::clamp(
            controlStep,
            MIN_CONTROL_STEP,
            MAX_CONTROL_STEP);


    /*
     * Pitch Error 방향 결정
     */

    if (pitchError < 0.0)
    {
        controlStep =
            -controlStep;
    }


    /*
     * 실제 액추에이터 설치 방향 적용
     */

    controlStep *=
        ACTUATOR_DIRECTION;


    return controlStep;
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
        << " IMU Mass-Shift P Attitude Hold\n"
        << "========================================\n";


    /*
     * Ctrl+C 등록
     */

    std::signal(
        SIGINT,
        signalHandler);


    /*
     * ========================================================
     * MPU6050 Initialization
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
     * Actuator Initialization
     * ========================================================
     */

    StepperMotor actuator(
        STEP_PIN,
        DIR_PIN,
        ENABLE_PIN);


    /*
     * motor_center_calibration.cpp와 동일한 방식으로
     * Pulse Delay를 설정한다.
     */

    actuator.setPulseDelay(
        PULSE_DELAY_US);


    /*
     * GPIO 초기화
     */

    if (!actuator.initialize())
    {
        std::cerr
            << "[ERROR] Actuator initialization failed\n";

        return 1;
    }


    /*
     * A4988 활성화
     */

    actuator.enable();


    /*
     * 프로그램 시작 시 현재 물리 위치를
     * 논리적인 0 step으로 정의한다.
     */

    actuator.setCurrentPosition(0);


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

        << "[INIT] KP = "
        << KP_STEP
        << " step/deg\n"

        << "[INIT] Max Control = "
        << MAX_CONTROL_STEP
        << " step\n"

        << "[INIT] Pulse Delay = "
        << PULSE_DELAY_US
        << " us\n"

        << "[INIT] Control Period = "
        << CONTROL_PERIOD_MS
        << " ms\n";


    /*
     * ========================================================
     * Static Trim
     * ========================================================
     */

    if (STATIC_TRIM_STEP != 0)
    {
        std::cout
            << "[TRIM] Static Trim = "
            << STATIC_TRIM_STEP
            << " step\n";


        if (!safeMove(
                actuator,
                STATIC_TRIM_STEP))
        {
            std::cerr
                << "[ERROR] Static Trim Failed\n";

            actuator.disable();

            return 1;
        }
    }


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
         * Keyboard Check
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
         * Current Pitch
         * ====================================================
         */

        double currentPitch =
            calculatePitch(
                static_cast<double>(ax),
                static_cast<double>(ay),
                static_cast<double>(az));


        /*
         * 목표 자세 - 현재 자세
         */

        double pitchError =
            TARGET_PITCH_DEG -
            currentPitch;


        /*
         * ====================================================
         * Gyroscope Bias Correction
         * ====================================================
         *
         * 아직 P 제어에는 직접 사용하지 않는다.
         *
         * 추후 PD / Complementary Filter 적용 시 사용.
         */

        double correctedGY =
            static_cast<double>(gy) -
            GYRO_Y_BIAS;


        double gyroYDegPerSec =
            correctedGY /
            GYRO_SCALE;


        /*
         * ====================================================
         * P Control
         * ====================================================
         */

        int moveCommand =
            calculateControlStep(
                pitchError);


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
         * Debug Output
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

            << " | Cmd:"
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
     */

    returnToStart(
        actuator);


    actuator.disable();


    std::cout
        << "[EXIT] A4988 Disabled\n"
        << "[EXIT] Program Finished\n";


    return 0;
}