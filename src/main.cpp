#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

#include "calibration_data.h"
#include "MPU6050.h"
#include "PCA9685.h"
#include "StepperMotor.h"

/*
 * ============================================================
 * File Name : main.cpp
 *
 * Step 1 Integration Version
 * ------------------------------------------------------------
 * 목적:
 * 1. Phase 2 구조는 보존하되 현재 Step 1에서는 비활성화한다.
 * 2. Body Yaw / Mass Shift Pitch 스텝모터를 초기화한다.
 * 3. 프로그램 시작 시 현재 물리적 위치를 0 step으로 정의한다.
 * 4. 기존 SM1504_CENTER_STEP 자동 중심 이동을 제거한다.
 * 5. 시커 서보를 calibration_data.h의 중심 PWM으로 이동시킨다.
 * 6. Phase 3 진입 후 q 입력을 기다린다.
 * 7. q 입력 시 누적 step 기준으로 두 축을 시작 위치(0)로 복귀한다.
 *
 * 이번 단계에서는:
 * - Seeker V1 미연결
 * - 새 Body Tracking 제어 미연결
 * - P 제어 미연결
 * - IMU는 초기화 및 Bias 로드 확인만 수행
 *
 * Axis Definition
 * ------------------------------------------------------------
 * Body Yaw
 *   STEP : GPIO19
 *   DIR  : GPIO26
 *
 * Mass Shift Pitch (SM1504)
 *   STEP : GPIO17
 *   DIR  : GPIO27
 *
 * A4988 ENABLE은 GND에 고정하므로 GPIO 제어를 사용하지 않는다.
 * StepperMotor 생성자에는 enablePin = -1을 전달한다.
 * ============================================================
 */


// ============================================================
// System State
// ============================================================

enum class SystemState
{
    INITIALIZING,
    MIDCOURSE_GUIDE,    // Phase 2 : 현재 Step 1에서는 비활성
    TERMINAL_LOCKON,
    BODY_TRACKING,
    MISSION_COMPLETE
};


// ============================================================
// Hardware Configuration
// ============================================================

struct HardwareConstants
{
    // Body Yaw Stepper
    static constexpr int BODY_YAW_STEP_PIN = 19;
    static constexpr int BODY_YAW_DIR_PIN  = 26;

    // Mass Shift Pitch / SM1504
    static constexpr int MASS_SHIFT_STEP_PIN = 17;
    static constexpr int MASS_SHIFT_DIR_PIN  = 27;

    // ENABLE is hard-wired to GND.
    static constexpr int UNUSED_ENABLE_PIN = -1;
};


// ============================================================
// Control Constants
// ============================================================

struct ControlConstants
{
    static constexpr int LOOP_TIME_MS = 20;
};


// ============================================================
// Integrated Controller
// ============================================================

class IntegratedMissileController
{
private:

    SystemState m_currentState = SystemState::INITIALIZING;

    std::atomic<bool> m_exitRequested{false};


    // --------------------------------------------------------
    // Hardware Drivers
    // --------------------------------------------------------

    PCA9685 m_pca9685;
    MPU6050 m_imu;

    StepperMotor m_bodyYawStepper;
    StepperMotor m_massShiftStepper;


public:

    IntegratedMissileController()
        : m_bodyYawStepper(
              HardwareConstants::BODY_YAW_STEP_PIN,
              HardwareConstants::BODY_YAW_DIR_PIN,
              HardwareConstants::UNUSED_ENABLE_PIN
          ),
          m_massShiftStepper(
              HardwareConstants::MASS_SHIFT_STEP_PIN,
              HardwareConstants::MASS_SHIFT_DIR_PIN,
              HardwareConstants::UNUSED_ENABLE_PIN
          )
    {
    }


    // ========================================================
    // Exit Input Thread
    // ========================================================

    void monitorExitInput()
    {
        char input;

        while (!m_exitRequested)
        {
            std::cin >> input;

            if (input == 'q' || input == 'Q')
            {
                std::cout
                    << "\n[SAFE EXIT] 종료 요청이 입력되었습니다.\n";

                m_exitRequested = true;
                break;
            }
        }
    }


    bool checkExitRequest()
    {
        if (!m_exitRequested)
        {
            return false;
        }

        m_currentState = SystemState::MISSION_COMPLETE;
        return true;
    }


    // ========================================================
    // Main State Machine
    // ========================================================

    void runSystem()
    {
        std::cout
            << "==================================================\n"
            << "   Mass Shift Guidance Control System - Step 1\n"
            << "==================================================\n"
            << " q 입력 시 안전 종료\n"
            << "==================================================\n";


        std::thread inputThread(
            &IntegratedMissileController::monitorExitInput,
            this
        );

        inputThread.detach();


        while (m_currentState != SystemState::MISSION_COMPLETE)
        {
            if (checkExitRequest())
            {
                break;
            }


            switch (m_currentState)
            {
                case SystemState::INITIALIZING:

                    processPhase1_Initializing();
                    break;


                case SystemState::MIDCOURSE_GUIDE:

                    processPhase2_MidcourseGuide();
                    break;


                case SystemState::TERMINAL_LOCKON:

                    processPhase3_TerminalLockOnPlaceholder();
                    break;


                case SystemState::BODY_TRACKING:

                    processPhase4_BodyTrackingPlaceholder();
                    break;


                default:

                    m_currentState = SystemState::MISSION_COMPLETE;
                    break;
            }


            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    ControlConstants::LOOP_TIME_MS
                )
            );
        }


        // ----------------------------------------------------
        // Safe Return
        // ----------------------------------------------------

        returnSteppersToStart();


        std::cout
            << "\n==================================================\n"
            << "             시스템 종료 완료\n"
            << "==================================================\n";
    }


private:

    // ========================================================
    // Servo Center
    // ========================================================

    void moveSeekerToCenter()
    {
        m_pca9685.setPWM(
            Calibration::SEEKER_PITCH_CHANNEL,
            0,
            static_cast<uint16_t>(
                Calibration::SEEKER_PITCH_CENTER_PWM
            )
        );

        m_pca9685.setPWM(
            Calibration::SEEKER_YAW_CHANNEL,
            0,
            static_cast<uint16_t>(
                Calibration::SEEKER_YAW_CENTER_PWM
            )
        );


        std::cout
            << " -> Seeker center position applied\n"
            << "    Pitch PWM : "
            << Calibration::SEEKER_PITCH_CENTER_PWM
            << '\n'
            << "    Yaw PWM   : "
            << Calibration::SEEKER_YAW_CENTER_PWM
            << '\n';
    }


    // ========================================================
    // Phase 1
    // ========================================================

    void processPhase1_Initializing()
    {
        std::cout
            << "\n[Phase 1] 하드웨어 초기화 시작...\n";


        // ----------------------------------------------------
        // MPU6050
        // ----------------------------------------------------

        if (!m_imu.begin())
        {
            std::cerr
                << " -> [ERROR] MPU6050 초기화 실패\n";

            m_currentState = SystemState::MISSION_COMPLETE;
            return;
        }

        std::cout
            << " -> MPU6050 연결 성공\n";


        // ----------------------------------------------------
        // PCA9685
        // ----------------------------------------------------

        if (!m_pca9685.begin())
        {
            std::cerr
                << " -> [ERROR] PCA9685 초기화 실패\n";

            m_currentState = SystemState::MISSION_COMPLETE;
            return;
        }

        m_pca9685.setPWMFreq(50.0f);

        std::cout
            << " -> PCA9685 연결 성공\n"
            << " -> PWM Frequency : 50 Hz\n";


        // ----------------------------------------------------
        // Body Yaw Stepper
        // ----------------------------------------------------

        if (!m_bodyYawStepper.initialize())
        {
            std::cerr
                << " -> [ERROR] Body Yaw Stepper 초기화 실패\n";

            m_currentState = SystemState::MISSION_COMPLETE;
            return;
        }


        // ----------------------------------------------------
        // Mass Shift Pitch Stepper
        // ----------------------------------------------------

        if (!m_massShiftStepper.initialize())
        {
            std::cerr
                << " -> [ERROR] Mass Shift Stepper 초기화 실패\n";

            m_currentState = SystemState::MISSION_COMPLETE;
            return;
        }


        /*
         * 현재 실제 위치를 프로그램상의 원점으로 선언한다.
         *
         * 여기서 모터를 움직이지 않는다.
         */
        m_bodyYawStepper.setCurrentPosition(0);
        m_massShiftStepper.setCurrentPosition(0);


        std::cout
            << " -> Stepper 초기화 성공\n"
            << "    Body Yaw       : STEP GPIO19 / DIR GPIO26\n"
            << "    Mass Shift     : STEP GPIO17 / DIR GPIO27\n"
            << "    Current Position = 0 step\n";


        // ----------------------------------------------------
        // Calibration Data
        // ----------------------------------------------------

        std::cout
            << " -> MPU6050 Gyro Bias 로드\n"
            << "    X : " << Calibration::GYRO_X_BIAS << '\n'
            << "    Y : " << Calibration::GYRO_Y_BIAS << '\n'
            << "    Z : " << Calibration::GYRO_Z_BIAS << '\n';


        // ----------------------------------------------------
        // Seeker Initial Center
        // ----------------------------------------------------

        moveSeekerToCenter();


        /*
         * 기존 SM1504_CENTER_STEP 이동은 수행하지 않는다.
         *
         * 프로그램 실행 직전 사람이 맞춰둔 실제 위치가
         * Body Yaw / Mass Shift 모두 0점이다.
         */


        std::cout
            << " -> 전체 하드웨어 초기화 완료\n"
            << " -> 현재 Step 1 통합 시험에서는 Phase 2를 비활성화합니다.\n"
            << " -> [Phase 3]으로 직접 천이합니다.\n";


        /*
         * 정상 전체 알고리즘에서는 아래 전이를 사용한다.
         *
         * m_currentState = SystemState::MIDCOURSE_GUIDE;
         *
         * 하지만 현재 Step 1 통합 시험에서는
         * Phase 2 기능을 아직 연결하지 않으므로
         * Phase 3으로 직접 이동한다.
         */
        // m_currentState = SystemState::MIDCOURSE_GUIDE;

        m_currentState = SystemState::TERMINAL_LOCKON;
    }


    // ========================================================
    // Phase 2
    // Midcourse Guidance - Preserved / Currently Inactive
    // ========================================================

    void processPhase2_MidcourseGuide()
    {
        /*
         * 이 함수는 삭제하지 않는다.
         *
         * 향후 전체 시스템에서는:
         *
         * 외부 명령 또는 예상 표적 사분면 입력
         *          ↓
         * 중간 유도 명령 생성
         *          ↓
         * 예상 표적 구역으로 시커/동체 유도
         *          ↓
         * Phase 3 Terminal Lock-On
         *
         * 순서로 사용한다.
         *
         * 현재 Step 1에서는 Phase 1에서 이 함수로
         * 진입하지 않기 때문에 실제 실행되지 않는다.
         */

        std::cout
            << "\n[Phase 2] Midcourse Guidance\n"
            << " -> 현재 Step 1에서는 비활성 상태입니다.\n"
            << " -> Phase 3으로 천이합니다.\n";


        m_currentState = SystemState::TERMINAL_LOCKON;
    }


    // ========================================================
    // Phase 3 Placeholder
    // ========================================================

    void processPhase3_TerminalLockOnPlaceholder()
    {
        /*
         * Step 1에서는 Seeker V1을 아직 연결하지 않는다.
         *
         * 초기화가 정상 완료된 상태에서 대기하면서
         * q 안전 종료 경로만 검증한다.
         */

        static bool printed = false;

        if (!printed)
        {
            std::cout
                << "\n[Phase 3] Placeholder\n"
                << " -> Seeker V1은 아직 연결하지 않았습니다.\n"
                << " -> 현재는 q 종료 시험을 위한 대기 상태입니다.\n"
                << " -> q 입력 시 두 스텝모터가 누적 step 기준 "
                << "0점으로 복귀합니다.\n";

            printed = true;
        }


        checkExitRequest();
    }


    // ========================================================
    // Phase 4 Placeholder
    // ========================================================

    void processPhase4_BodyTrackingPlaceholder()
    {
        /*
         * 이번 단계에서는 진입하지 않는다.
         * 향후 Seeker V1 검증 이후 실제 로직으로 교체한다.
         */

        checkExitRequest();
    }


    // ========================================================
    // Return to Program Start Position
    // ========================================================

    void returnSteppersToStart()
    {
        /*
         * 두 모터 모두 정상 초기화된 경우에만 복귀한다.
         *
         * ENABLE은 GND 고정이므로 disable()은 호출하지 않는다.
         */

        if (
            !m_bodyYawStepper.isInitialized() ||
            !m_massShiftStepper.isInitialized()
        )
        {
            return;
        }


        const long currentBodyYaw =
            m_bodyYawStepper.getCurrentPosition();

        const long currentMassShift =
            m_massShiftStepper.getCurrentPosition();


        std::cout
            << "\n[RETURN] 시작 위치 복귀\n"
            << " -> Body Yaw Current   : "
            << currentBodyYaw
            << " step\n"
            << " -> Mass Shift Current : "
            << currentMassShift
            << " step\n";


        if (currentBodyYaw != 0)
        {
            if (!m_bodyYawStepper.moveSteps(
                    -static_cast<int>(currentBodyYaw)
                ))
            {
                std::cerr
                    << " -> [ERROR] Body Yaw 원점 복귀 실패\n";
            }
        }


        if (currentMassShift != 0)
        {
            if (!m_massShiftStepper.moveSteps(
                    -static_cast<int>(currentMassShift)
                ))
            {
                std::cerr
                    << " -> [ERROR] Mass Shift 원점 복귀 실패\n";
            }
        }


        std::cout
            << " -> 복귀 완료\n"
            << "    Body Yaw Position   : "
            << m_bodyYawStepper.getCurrentPosition()
            << " step\n"
            << "    Mass Shift Position : "
            << m_massShiftStepper.getCurrentPosition()
            << " step\n";
    }
};


// ============================================================
// Main
// ============================================================

int main()
{
    IntegratedMissileController controller;

    controller.runSystem();

    return 0;
}