#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

#include "calibration_data.h"
#include "MPU6050.h"
#include "SeekerControl.h"
#include "StepperMotor.h"

/*
 * ============================================================
 * File Name : main.cpp
 *
 * Step 2 Integration Version
 * ------------------------------------------------------------
 * 목적:
 *
 * 1. Step 1 하드웨어 초기화 / 안전 종료 구조를 유지한다.
 * 2. SeekerControl V1을 main.cpp에 실제 연결한다.
 * 3. Phase 3에서 Seeker 추적 루프를 실행한다.
 * 4. 현재 Seeker Yaw / Pitch PWM을 읽는다.
 * 5. Seeker 중심 PWM과 현재 PWM의 편차를 계산한다.
 * 6. 편차를 출력하여 Phase 4 Body Tracking 입력값을 검증한다.
 *
 * 이번 단계에서는:
 *
 * - Seeker V1 연결 O
 * - Seeker Center Error 계산 O
 *
 * - Body Tracking 실제 구동 X
 * - P 제어 X
 * - IMU 자세 제어 X
 *
 * ------------------------------------------------------------
 * Body Tracking 기본 개념
 *
 * Yaw Error
 *
 *   Current Seeker Yaw PWM
 *          -
 *   Seeker Yaw Center PWM
 *
 *
 * Pitch Error
 *
 *   Current Seeker Pitch PWM
 *          -
 *   Seeker Pitch Center PWM
 *
 *
 * 즉,
 *
 * 시커가 표적을 추적하여 중심에서 벗어나면
 * 그 PWM 편차를 향후 Phase 4에서
 * 동체 추적 제어 입력으로 사용한다.
 * ============================================================
 */


// ============================================================
// System State
// ============================================================

enum class SystemState
{
    INITIALIZING,

    MIDCOURSE_GUIDE,    // Phase 2 : 현재 비활성

    TERMINAL_LOCKON,    // Phase 3 : Seeker Tracking

    BODY_TRACKING,      // Phase 4 : Step 3 이후 연결

    MISSION_COMPLETE
};


// ============================================================
// Hardware Configuration
// ============================================================

struct HardwareConstants
{
    // --------------------------------------------------------
    // Body Yaw Stepper
    // --------------------------------------------------------

    static constexpr int BODY_YAW_STEP_PIN = 19;
    static constexpr int BODY_YAW_DIR_PIN  = 26;


    // --------------------------------------------------------
    // Mass Shift Pitch Stepper
    // --------------------------------------------------------

    static constexpr int MASS_SHIFT_STEP_PIN = 17;
    static constexpr int MASS_SHIFT_DIR_PIN  = 27;


    /*
     * A4988 ENABLE은 GND에 고정되어 있으므로
     * GPIO로 제어하지 않는다.
     */
    static constexpr int UNUSED_ENABLE_PIN = -1;
};


// ============================================================
// Control Constants
// ============================================================

struct ControlConstants
{
    /*
     * main 상태머신 기본 대기시간.
     *
     * SeekerControl::update() 내부에도 센서 간격 및
     * 제어주기 sleep이 존재한다.
     *
     * 현재 Step 2에서는 기존 구조를 그대로 유지하고,
     * 전체 루프 주기 문제는 향후 Step 6에서 점검한다.
     */
    static constexpr int LOOP_TIME_MS = 20;
};


// ============================================================
// Integrated Controller
// ============================================================

class IntegratedMissileController
{
private:

    // --------------------------------------------------------
    // System State
    // --------------------------------------------------------

    SystemState m_currentState =
        SystemState::INITIALIZING;


    std::atomic<bool> m_exitRequested{false};


    // --------------------------------------------------------
    // Hardware / Control Drivers
    // --------------------------------------------------------

    MPU6050 m_imu;

    SeekerControl m_seeker;

    StepperMotor m_bodyYawStepper;

    StepperMotor m_massShiftStepper;


public:

    // ========================================================
    // Constructor
    // ========================================================

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


            if (
                input == 'q' ||
                input == 'Q'
            )
            {
                std::cout
                    << "\n"
                    << "[SAFE EXIT] "
                    << "종료 요청이 입력되었습니다.\n";


                m_exitRequested = true;

                break;
            }
        }
    }


    // ========================================================
    // Exit Check
    // ========================================================

    bool checkExitRequest()
    {
        if (!m_exitRequested)
        {
            return false;
        }


        m_currentState =
            SystemState::MISSION_COMPLETE;


        return true;
    }


    // ========================================================
    // Main State Machine
    // ========================================================

    void runSystem()
    {
        std::cout
            << "==================================================\n"
            << " Mass Shift Guidance Control System - Step 2\n"
            << "==================================================\n"
            << " Seeker V1 Integration Test\n"
            << " q 입력 시 안전 종료\n"
            << "==================================================\n";


        // ----------------------------------------------------
        // Exit Input Thread
        // ----------------------------------------------------

        std::thread inputThread(
            &IntegratedMissileController::monitorExitInput,
            this
        );


        inputThread.detach();


        // ----------------------------------------------------
        // Main Loop
        // ----------------------------------------------------

        while (
            m_currentState !=
            SystemState::MISSION_COMPLETE
        )
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

                    processPhase3_TerminalLockOn();

                    break;


                case SystemState::BODY_TRACKING:

                    processPhase4_BodyTrackingPlaceholder();

                    break;


                default:

                    m_currentState =
                        SystemState::MISSION_COMPLETE;

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
            << "\n"
            << "==================================================\n"
            << "              시스템 종료 완료\n"
            << "==================================================\n";
    }


private:

    // ========================================================
    // Phase 1
    // Hardware Initialization
    // ========================================================

    void processPhase1_Initializing()
    {
        std::cout
            << "\n"
            << "[Phase 1] "
            << "하드웨어 초기화 시작...\n";


        // ====================================================
        // MPU6050
        // ====================================================

        if (!m_imu.begin())
        {
            std::cerr
                << " -> [ERROR] "
                << "MPU6050 초기화 실패\n";


            m_currentState =
                SystemState::MISSION_COMPLETE;


            return;
        }


        std::cout
            << " -> MPU6050 연결 성공\n";


        // ====================================================
        // Body Yaw Stepper
        // ====================================================

        if (!m_bodyYawStepper.initialize())
        {
            std::cerr
                << " -> [ERROR] "
                << "Body Yaw Stepper 초기화 실패\n";


            m_currentState =
                SystemState::MISSION_COMPLETE;


            return;
        }


        // ====================================================
        // Mass Shift Stepper
        // ====================================================

        if (!m_massShiftStepper.initialize())
        {
            std::cerr
                << " -> [ERROR] "
                << "Mass Shift Stepper 초기화 실패\n";


            m_currentState =
                SystemState::MISSION_COMPLETE;


            return;
        }


        /*
         * 현재 실제 위치를 프로그램상의 0 step으로 정의.
         *
         * 여기서는 모터를 움직이지 않는다.
         */

        m_bodyYawStepper.setCurrentPosition(0);

        m_massShiftStepper.setCurrentPosition(0);


        std::cout
            << " -> Stepper 초기화 성공\n"

            << "    Body Yaw   : "
            << "STEP GPIO19 / DIR GPIO26\n"

            << "    Mass Shift : "
            << "STEP GPIO17 / DIR GPIO27\n"

            << "    Current Position = 0 step\n";


        // ====================================================
        // MPU6050 Calibration Information
        // ====================================================

        std::cout
            << " -> MPU6050 Gyro Bias 로드\n"

            << "    X : "
            << Calibration::GYRO_X_BIAS
            << '\n'

            << "    Y : "
            << Calibration::GYRO_Y_BIAS
            << '\n'

            << "    Z : "
            << Calibration::GYRO_Z_BIAS
            << '\n';


        // ====================================================
        // SeekerControl
        // ====================================================

        std::cout
            << " -> SeekerControl 초기화 시작\n";


        if (!m_seeker.initialize())
        {
            std::cerr
                << " -> [ERROR] "
                << "SeekerControl 초기화 실패\n";


            m_currentState =
                SystemState::MISSION_COMPLETE;


            return;
        }


        std::cout
            << " -> SeekerControl 초기화 성공\n"

            << "    Yaw Center PWM   : "
            << Calibration::SEEKER_YAW_CENTER_PWM
            << '\n'

            << "    Pitch Center PWM : "
            << Calibration::SEEKER_PITCH_CENTER_PWM
            << '\n';


        // ====================================================
        // Phase 1 Complete
        // ====================================================

        std::cout
            << " -> 전체 하드웨어 초기화 완료\n"

            << " -> 현재 Step 2에서는 "
            << "Phase 2를 비활성화합니다.\n"

            << " -> Phase 3 Seeker Tracking으로 "
            << "직접 천이합니다.\n";


        /*
         * 정상 전체 알고리즘:
         *
         * INITIALIZING
         *      ↓
         * MIDCOURSE_GUIDE
         *      ↓
         * TERMINAL_LOCKON
         *
         *
         * 현재 Step 2:
         *
         * INITIALIZING
         *      ↓
         * TERMINAL_LOCKON
         */


        // m_currentState =
        //     SystemState::MIDCOURSE_GUIDE;


        m_currentState =
            SystemState::TERMINAL_LOCKON;
    }


    // ========================================================
    // Phase 2
    // Midcourse Guidance
    // ========================================================

    void processPhase2_MidcourseGuide()
    {
        /*
         * 현재 Step 2에서는 비활성.
         *
         * 향후:
         *
         * 예상 표적 영역 입력
         *        ↓
         * 중간 유도
         *        ↓
         * Terminal Lock-On
         */


        std::cout
            << "\n"
            << "[Phase 2] Midcourse Guidance\n"

            << " -> 현재 Step 2에서는 "
            << "비활성 상태입니다.\n"

            << " -> Phase 3으로 천이합니다.\n";


        m_currentState =
            SystemState::TERMINAL_LOCKON;
    }


    // ========================================================
    // Phase 3
    // Terminal Lock-On / Seeker Tracking
    // ========================================================

    void processPhase3_TerminalLockOn()
    {
        if (!m_seeker.isInitialized())
        {
            std::cerr
                << "[ERROR] "
                << "SeekerControl이 초기화되지 않았습니다.\n";


            m_currentState =
                SystemState::MISSION_COMPLETE;


            return;
        }


        // ----------------------------------------------------
        // Seeker Tracking
        // ----------------------------------------------------

        /*
         * LEFT / RIGHT / BOTTOM 센서를 측정하고
         *
         * SeekerControl 내부 V1 알고리즘에 따라
         * Pan / Tilt Servo를 움직인다.
         */

        m_seeker.update();


        // ----------------------------------------------------
        // Current Seeker PWM
        // ----------------------------------------------------

        const int currentYawPWM =
            m_seeker.getYawPWM();


        const int currentPitchPWM =
            m_seeker.getPitchPWM();


        // ----------------------------------------------------
        // Seeker Center Error
        // ----------------------------------------------------

        /*
         * 중요한 부분.
         *
         * 초음파 거리 오차가 아니라
         *
         * "동체에 대한 시커의 현재 방향"
         *
         * 을 Body Tracking 입력으로 사용한다.
         *
         *
         * Error = Current PWM - Center PWM
         */


        const int yawCenterError =
            currentYawPWM -
            Calibration::SEEKER_YAW_CENTER_PWM;


        const int pitchCenterError =
            currentPitchPWM -
            Calibration::SEEKER_PITCH_CENTER_PWM;


        // ----------------------------------------------------
        // Step 2 Debug Output
        // ----------------------------------------------------

        std::cout
            << "[BODY TRACK INPUT] "

            << "YawPWM="
            << currentYawPWM

            << " YawCenterErr="
            << yawCenterError

            << " | PitchPWM="
            << currentPitchPWM

            << " PitchCenterErr="
            << pitchCenterError

            << '\n';


        /*
         * ====================================================
         * Step 3에서 연결 예정
         * ====================================================
         *
         * yawCenterError
         *        ↓
         * Body Yaw P Controller
         *        ↓
         * Body Yaw Stepper
         *
         *
         * pitchCenterError
         *        ↓
         * Mass Shift P Controller
         *        ↓
         * Mass Shift Stepper
         *
         *
         * 현재 Step 2에서는 절대 모터를 움직이지 않는다.
         */


        checkExitRequest();
    }


    // ========================================================
    // Phase 4
    // Body Tracking Placeholder
    // ========================================================

    void processPhase4_BodyTrackingPlaceholder()
    {
        /*
         * Step 3에서 실제 Body Tracking 제어로 교체한다.
         *
         * 현재 Step 2에서는 Phase 4로 진입하지 않는다.
         */


        checkExitRequest();
    }


    // ========================================================
    // Return Steppers to Start Position
    // ========================================================

    void returnSteppersToStart()
    {
        /*
         * 두 Stepper가 정상적으로 초기화된 경우에만
         * 시작 위치 0 step으로 복귀한다.
         *
         * Step 2에서는 모터를 움직이지 않으므로
         * 정상적인 경우 이미 0 step이어야 한다.
         *
         * 이 함수는 Step 3 이후 실제 모터 구동 시
         * 안전 종료 기능으로 그대로 사용한다.
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
            << "\n"
            << "[RETURN] 시작 위치 복귀\n"

            << " -> Body Yaw Current   : "
            << currentBodyYaw
            << " step\n"

            << " -> Mass Shift Current : "
            << currentMassShift
            << " step\n";


        // ----------------------------------------------------
        // Body Yaw Return
        // ----------------------------------------------------

        if (currentBodyYaw != 0)
        {
            if (
                !m_bodyYawStepper.moveSteps(
                    -static_cast<int>(
                        currentBodyYaw
                    )
                )
            )
            {
                std::cerr
                    << " -> [ERROR] "
                    << "Body Yaw 원점 복귀 실패\n";
            }
        }


        // ----------------------------------------------------
        // Mass Shift Return
        // ----------------------------------------------------

        if (currentMassShift != 0)
        {
            if (
                !m_massShiftStepper.moveSteps(
                    -static_cast<int>(
                        currentMassShift
                    )
                )
            )
            {
                std::cerr
                    << " -> [ERROR] "
                    << "Mass Shift 원점 복귀 실패\n";
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