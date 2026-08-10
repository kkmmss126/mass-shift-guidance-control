#include <iostream>   // 콘솔 출력(std::cout)을 위한 라이브러리
#include <cmath>      // abs() 등 수학 함수 사용
#include <thread>     // sleep_for 사용
#include <chrono>     // 시간 제어(ms) 사용
#include <string>     // string 자료형 사용

/*
 * 실제 하드웨어 측정을 통해 확보한
 * 캘리브레이션 값을 불러온다.
 *
 * 현재 저장된 값
 * ------------------------------------------------------------
 * Servo Bottom Init PWM : 300
 * Servo Top Init PWM    : 321
 * SM1504 Center         : 1690 step
 *
 * MPU6050 Gyro Bias
 * X :  76.09
 * Y : -94.12
 * Z : -19.14
 */
#include "calibration_data.h"


// ============================================================
// 시스템 상태 정의
// ============================================================
//
// 상태 머신(State Machine)을 이용하여
// 전체 유도 과정을 Phase별로 관리한다.
//
enum class SystemState
{
    INITIALIZING,       // Phase 1 : 센서 및 구동기 초기화

    MIDCOURSE_GUIDE,    // Phase 2 : 중간 유도

    TERMINAL_LOCKON,    // Phase 3 : 시커 기반 종말 유도

    BODY_TRACKING,      // Phase 4 : 동체-시커 정렬

    MISSION_COMPLETE    // 전체 유도 과정 완료
};


// ============================================================
// 전체 제어 시스템에서 사용하는 공통 상수
// ============================================================
struct ControlConstants
{
    /*
     * 제어 루프 주기
     *
     * 20 ms = 50 Hz
     */
    static constexpr double LOOP_TIME_MS = 20.0;


    /*
     * Phase 2 동체 제한 이동량
     *
     * 현재 시뮬레이션에서는
     * 한 제어 루프당 최대 2도를 이동하도록 제한한다.
     */
    static constexpr double MAX_ANGULAR_VELOCITY = 2.0;


    /*
     * 시커 좌표계 기준 중심각
     *
     * 현재 제어 알고리즘 내부에서는
     * 상대 각도를 계산하기 위한 논리적인 중심값으로 사용한다.
     *
     * 실제 서보모터의 물리적 초기 위치는
     * calibration_data.h에 저장된 PWM 값을 사용한다.
     */
    static constexpr double SEEKER_YAW_CENTER = 90.0;

    static constexpr double SEEKER_PITCH_CENTER = 90.0;


    /*
     * Phase 3 Lock-On 허용 오차
     */
    static constexpr double LOCKON_THRESHOLD = 2.0;


    /*
     * Phase 4 동체-시커 최종 정렬 허용 오차
     */
    static constexpr double TRACKING_TOLERANCE = 0.5;
};


// ============================================================
// 종합 유도 제어 클래스
// ============================================================
class IntegratedMissileController
{
private:

    /*
     * 현재 시스템 상태
     *
     * 시스템 시작 시 반드시 INITIALIZING 상태에서 시작한다.
     */
    SystemState m_currentState = SystemState::INITIALIZING;


    // ========================================================
    // 동체 상태 변수
    // ========================================================

    /*
     * 현재 동체 절대 각도
     *
     * 실제 하드웨어 연결 이후에는
     * IMU의 자세 추정값을 이용하도록 변경한다.
     */
    double m_currentBodyX = 0.0;
    double m_currentBodyY = 0.0;


    // ========================================================
    // 시커 상태 변수
    // ========================================================

    /*
     * 동체 기준 시커 상대 Yaw / Pitch 각도
     */
    double m_seekerYawAngle = 0.0;
    double m_seekerPitchAngle = 0.0;


    /*
     * Phase 4에서 사용하는 시커 절대 방향
     */
    double m_seekerAbsYaw = 0.0;
    double m_seekerAbsPitch = 0.0;


    /*
     * 순간적인 오차 감소를 Lock-On으로 잘못 판단하지 않도록
     * 일정 횟수 연속으로 조건을 만족했는지 확인한다.
     */
    int m_stableLockCount = 0;


    // ========================================================
    // Mock Data
    // ========================================================
    //
    // 아래 값들은 아직 실제 센서/통신 라이브러리가
    // 메인 시스템에 연결되지 않았기 때문에 임시로 사용한다.
    //
    // 추후 외부 라이브러리 연결 시 실제 측정값으로 변경한다.
    //
    // 중요:
    // calibration_data.h에 저장된 값은 Mock 값이 아니다.
    // 실제 측정을 통해 확보한 하드웨어 기준값이다.
    // ========================================================


    /*
     * 현재 시커 추적 알고리즘 검증용 목표 위치
     *
     * 추후 초음파 센서 거리값을 이용한
     * 타겟 방향 계산으로 변경한다.
     */
    double m_mockTargetX = 15.0;
    double m_mockTargetY = 10.0;


    /*
     * 실시간 MPU6050 입력 연결 전
     * 자세 외란을 모사하기 위한 각속도 값
     *
     * 추후:
     *
     * 실제 Gyro Raw
     *      ↓
     * calibration_data.h Bias 제거
     *      ↓
     * 보정된 Gyro Rate
     *
     * 구조로 변경한다.
     */
    double m_gyroYawRate = -5.0;
    double m_gyroPitchRate = 3.0;


public:

    // ========================================================
    // Servo PWM 출력 인터페이스
    // ========================================================
    //
    // 아직 PCA9685 외부 라이브러리를 메인 코드에
    // 연결하지 않았기 때문에 인터페이스 형태만 유지한다.
    //
    // 추후 이 함수 내부만 실제 라이브러리 호출로 변경하면
    // 메인 제어 알고리즘을 수정할 필요가 없다.
    // ========================================================

    void writeServoPWM(
        const std::string& motorName,
        int pwmValue)
    {
        /*
         * 추후 실제 구현 예시
         *
         * PCA9685 외부 라이브러리
         *      ↓
         * 해당 채널에 pwmValue 출력
         */

        std::cout
            << " -> [Servo Init] "
            << motorName
            << " PWM = "
            << pwmValue
            << '\n';
    }


    // ========================================================
    // SM1504 Linear Actuator 출력 인터페이스
    // ========================================================
    //
    // 현재는 실제 StepperMotor 라이브러리를 연결하지 않고
    // 목표 step 위치를 전달할 수 있는 구조만 만들어 둔다.
    // ========================================================

    void writeLinearActuator(int targetStep)
    {
        /*
         * 추후 실제 구현:
         *
         * 기준점 Homing
         *      ↓
         * targetStep만큼 이동
         */

        std::cout
            << " -> [SM1504 Init] Center Position = "
            << targetStep
            << " step\n";
    }


    // ========================================================
    // 전체 상태 머신 실행
    // ========================================================

    void runSystem()
    {
        std::cout
            << "==================================================\n"
            << "      Mass Shift Guidance Control System\n"
            << "==================================================\n";


        while (m_currentState != SystemState::MISSION_COMPLETE)
        {
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

                    processPhase4_BodyTracking();

                    break;


                default:

                    m_currentState =
                        SystemState::MISSION_COMPLETE;

                    break;
            }


            /*
             * 전체 상태 머신 기본 실행 주기
             */
            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    static_cast<long long>(
                        ControlConstants::LOOP_TIME_MS
                    )
                )
            );
        }


        std::cout
            << "\n==================================================\n"
            << "      전체 유도 시뮬레이션 종료\n"
            << "==================================================\n";
    }


private:

    // ========================================================
    // Phase 1
    // Hardware Initialization & Calibration Data Loading
    // ========================================================
    //
    // 기존 코드에서는 가상의 자이로 데이터를 반복 측정하여
    // Bias를 계산했지만,
    //
    // 현재는 실제 하드웨어 측정을 통해 이미 확보한 값을
    // calibration_data.h에서 직접 불러온다.
    //
    // 수행 순서
    //
    // 1. MPU6050 Gyro Bias 불러오기
    // 2. Servo 초기 PWM 위치 설정
    // 3. SM1504 중심 위치 설정
    // 4. 초기화 완료
    // 5. Phase 2로 상태 천이
    // ========================================================

    void processPhase1_Initializing()
    {
        std::cout
            << "\n[Phase 1] 하드웨어 초기화 시작...\n";


        // ----------------------------------------------------
        // 1. MPU6050 Gyro Bias
        // ----------------------------------------------------
        //
        // 별도의 imu_bias_measure 프로그램을 통해
        // 실제 측정한 Bias 값을 사용한다.
        //
        // 최종 하드웨어 제작 후 IMU를 다시 장착하면
        // 재측정 후 calibration_data.h의 값만 변경한다.
        // ----------------------------------------------------

        const double gyroBiasX = GYRO_X_BIAS;
        const double gyroBiasY = GYRO_Y_BIAS;
        const double gyroBiasZ = GYRO_Z_BIAS;


        std::cout
            << " -> MPU6050 Gyro Bias 적용\n"
            << "    X : " << gyroBiasX << '\n'
            << "    Y : " << gyroBiasY << '\n'
            << "    Z : " << gyroBiasZ << '\n';


        // ----------------------------------------------------
        // 2. Servo 초기 위치 설정
        // ----------------------------------------------------
        //
        // 실제 캘리브레이션을 통해 측정한
        // PCA9685 PWM 값을 사용한다.
        // ----------------------------------------------------

        writeServoPWM(
            "Bottom Servo",
            SERVO_BOTTOM_INIT_PWM
        );


        writeServoPWM(
            "Top Servo",
            SERVO_TOP_INIT_PWM
        );


        // ----------------------------------------------------
        // 3. SM1504 중심 위치 설정
        // ----------------------------------------------------
        //
        // 스텝모터 시작 기준 위치에서
        // 실제 측정한 중심 위치인 1690 step으로 이동한다.
        //
        // 추후 Homing 구조가 확정되면
        // 기준점 확보 후 해당 위치로 이동하도록 구현한다.
        // ----------------------------------------------------

        writeLinearActuator(
            SM1504_CENTER_STEP
        );


        std::cout
            << " -> 초기화 기준값 적용 완료.\n"
            << " -> [중간 유도 단계]로 천이합니다.\n";


        m_currentState =
            SystemState::MIDCOURSE_GUIDE;
    }


    // ========================================================
    // Phase 2
    // Midcourse Guidance
    // ========================================================

    void processPhase2_MidcourseGuide()
    {
        std::cout
            << "\n[Phase 2] 중간 유도 단계 진입: "
            << "외부 명령 대기 중...\n";


        /*
         * 아직 실제 UART/RS485 입력을 연결하지 않았으므로
         * Mock 명령을 사용한다.
         *
         * 추후 통신 라이브러리 연결 시 실제 수신값으로 교체한다.
         */
        char mockPuttyInput = '1';


        double targetX = 0.0;
        double targetY = 0.0;


        if (mockPuttyInput == '1')
        {
            targetX = 22.5;
            targetY = 15.0;
        }


        std::cout
            << " -> [Command] '1' 수신"
            << " | 목표 X: "
            << targetX
            << "도"
            << " | 목표 Y: "
            << targetY
            << "도\n";


        while (
            m_currentBodyX < targetX ||
            m_currentBodyY < targetY
        )
        {
            if (m_currentBodyX < targetX)
            {
                m_currentBodyX +=
                    ControlConstants::MAX_ANGULAR_VELOCITY;
            }


            if (m_currentBodyY < targetY)
            {
                m_currentBodyY +=
                    ControlConstants::MAX_ANGULAR_VELOCITY;
            }


            /*
             * 목표값을 초과하지 않도록 제한한다.
             */
            if (m_currentBodyX > targetX)
            {
                m_currentBodyX = targetX;
            }


            if (m_currentBodyY > targetY)
            {
                m_currentBodyY = targetY;
            }


            std::cout
                << " [기동중] 동체 절대 각도"
                << " | X: "
                << m_currentBodyX
                << "도"
                << " | Y: "
                << m_currentBodyY
                << "도\n";


            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    static_cast<long long>(
                        ControlConstants::LOOP_TIME_MS
                    )
                )
            );
        }


        std::cout
            << " -> 목표 사분면 도달\n"
            << " -> [종말 유도 단계]로 천이합니다.\n";


        m_currentState =
            SystemState::TERMINAL_LOCKON;
    }


    // ========================================================
    // Phase 3
    // Terminal Guidance / Seeker Lock-On
    // ========================================================
    //
    // 현재 추적 알고리즘은 유지한다.
    //
    // 초음파 센서 및 MPU6050 실시간 입력은
    // 추후 외부 라이브러리를 연결하면서 교체한다.
    // ========================================================

    void processPhase3_TerminalLockOn()
    {
        std::cout
            << "\n[Phase 3] 종말 유도 단계 진입: "
            << "시커 추적 시작\n";


        /*
         * 제어 주기를 초 단위로 변환한다.
         *
         * Gyro Rate(deg/s)를 이용하여
         * 한 제어 주기 동안 발생한 각도 변화를 계산하기 위해 사용한다.
         */
        const double dt =
            ControlConstants::LOOP_TIME_MS / 1000.0;


        while (true)
        {
            // ------------------------------------------------
            // 현재는 알고리즘 검증용 Mock 목표값 사용
            //
            // 추후 이 부분은 3개의 초음파 센서 거리값을
            // 입력으로 받는 타겟 방향 계산 함수로 교체한다.
            // ------------------------------------------------

            double seekerErrorX =
                m_mockTargetX -
                m_seekerYawAngle;


            double seekerErrorY =
                m_mockTargetY -
                m_seekerPitchAngle;


            /*
             * 시커 P 제어 Gain
             */
            const double trackingGain = 0.3;


            /*
             * 목표 추적 보정량
             *
             * Target Error 기반 P 제어
             * +
             * Gyro Rate 역보상
             */
            double deltaYaw =
                (seekerErrorX * trackingGain)
                -
                (m_gyroYawRate * dt);


            double deltaPitch =
                (seekerErrorY * trackingGain)
                -
                (m_gyroPitchRate * dt);


            m_seekerYawAngle += deltaYaw;
            m_seekerPitchAngle += deltaPitch;


            /*
             * 알고리즘 내부 논리 중심각 90도를 기준으로
             * 현재 시커 절대 방향을 계산한다.
             *
             * 실제 Servo PWM 중심값은
             * calibration_data.h의 값을 사용한다.
             */
            double finalScaleYaw =
                ControlConstants::SEEKER_YAW_CENTER
                +
                m_seekerYawAngle;


            double finalScalePitch =
                ControlConstants::SEEKER_PITCH_CENTER
                +
                m_seekerPitchAngle;


            std::cout
                << " [추적중]"
                << " Error X: "
                << seekerErrorX
                << "도"
                << " | Error Y: "
                << seekerErrorY
                << "도"
                << " | Yaw: "
                << finalScaleYaw
                << "도"
                << " | Pitch: "
                << finalScalePitch
                << "도\n";


            // ------------------------------------------------
            // Lock-On 판단
            // ------------------------------------------------

            if (
                std::abs(seekerErrorX)
                    <= ControlConstants::LOCKON_THRESHOLD
                &&
                std::abs(seekerErrorY)
                    <= ControlConstants::LOCKON_THRESHOLD
            )
            {
                ++m_stableLockCount;


                /*
                 * 5회 연속 오차 조건 만족 시
                 * 안정적인 Lock-On으로 판단한다.
                 */
                if (m_stableLockCount >= 5)
                {
                    std::cout
                        << "\n -> [LOCK-ON SUCCESS]\n";


                    /*
                     * Phase 4에서 동체 정렬에 사용할
                     * 시커의 절대 방향을 저장한다.
                     */
                    m_seekerAbsYaw =
                        finalScaleYaw;


                    m_seekerAbsPitch =
                        finalScalePitch;


                    break;
                }
            }

            else
            {
                /*
                 * 오차 조건을 벗어나면
                 * 안정화 카운터를 다시 초기화한다.
                 */
                m_stableLockCount = 0;
            }


            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    static_cast<long long>(
                        ControlConstants::LOOP_TIME_MS
                    )
                )
            );
        }


        m_currentState =
            SystemState::BODY_TRACKING;
    }


    // ========================================================
    // Phase 4
    // Body Tracking
    // ========================================================
    //
    // 시커가 바라보는 방향을 기준으로
    // 동체를 정렬한다.
    //
    // 현재는 P 제어 기반 Mock 동체 모델을 유지한다.
    // ========================================================

    void processPhase4_BodyTracking()
    {
        std::cout
            << "\n[Phase 4] 동체-시커 정렬 시작\n";


        /*
         * 동체 P 제어 Gain
         *
         * 추후 실험을 통해 튜닝한다.
         */
        const double bodyKpX = 0.25;
        const double bodyKpY = 0.20;


        while (true)
        {
            /*
             * 시커 중심축과 동체 사이의 방향 오차
             */
            double bodyErrorX =
                m_seekerAbsYaw
                -
                ControlConstants::SEEKER_YAW_CENTER;


            double bodyErrorY =
                m_seekerAbsPitch
                -
                ControlConstants::SEEKER_PITCH_CENTER;


            /*
             * 최종 정렬 완료 판정
             */
            if (
                std::abs(bodyErrorX)
                    <= ControlConstants::TRACKING_TOLERANCE
                &&
                std::abs(bodyErrorY)
                    <= ControlConstants::TRACKING_TOLERANCE
            )
            {
                std::cout
                    << "\n -> [ALIGNMENT COMPLETE]\n";


                break;
            }


            /*
             * P 제어
             *
             * 이동량 = Error × Kp
             */
            double bodyMoveX =
                bodyErrorX * bodyKpX;


            double bodyMoveY =
                bodyErrorY * bodyKpY;


            /*
             * 현재 Mock 동체 모델에 제어량 적용
             */
            m_currentBodyX += bodyMoveX;
            m_currentBodyY += bodyMoveY;


            /*
             * 동체가 시커 방향으로 이동했다고 가정하여
             * 시커-동체 상대 오차를 감소시킨다.
             *
             * 실제 하드웨어에서는 IMU 등의
             * 실제 피드백값으로 대체한다.
             */
            m_seekerAbsYaw -= bodyMoveX;
            m_seekerAbsPitch -= bodyMoveY;


            std::cout
                << " [정렬중]"
                << " Error X: "
                << bodyErrorX
                << "도"
                << " | Error Y: "
                << bodyErrorY
                << "도"
                << " | Body X: "
                << m_currentBodyX
                << "도"
                << " | Body Y: "
                << m_currentBodyY
                << "도\n";


            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    static_cast<long long>(
                        ControlConstants::LOOP_TIME_MS
                    )
                )
            );
        }


        m_currentState =
            SystemState::MISSION_COMPLETE;
    }
};


// ============================================================
// Main
// ============================================================

int main()
{
    /*
     * 종합 유도 제어기 생성
     */
    IntegratedMissileController controller;


    /*
     * Phase 1 → Phase 4 상태 머신 실행
     */
    controller.runSystem();


    return 0;
}
```
