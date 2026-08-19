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

/*
 * 실제 하드웨어 제어용 자체 라이브러리
 * 현재 단계에서는 이미 개별 테스트가 완료된
 * 자체 작성 라이브러리를 이용하여 전체 시스템을 통합한다.
 */
#include "PCA9685.h"
#include "MPU6050.h"
#include "StepperMotor.h"

// ============================================================
// 시스템 상태 정의
// ============================================================
/*
 * 상태 머신(State Machine)을 이용하여
 * 전체 유도 과정을 Phase별로 관리한다.
 */
enum class SystemState
{
    INITIALIZING,       // Phase 1 : 센서 및 구동기 초기화

    MIDCOURSE_GUIDE,    // Phase 2 : 중간 유도

    TERMINAL_LOCKON,    // Phase 3 : 시커 기반 종말 유도

    BODY_TRACKING,      // Phase 4 : 동체-시커 정렬

    MISSION_COMPLETE    // 전체 유도 과정 완료
};

// ============================================================
// Hardware Configuration
// ============================================================
/*
 * 실제 회로도에서 결정한 GPIO 및 PCA9685 채널을 한 곳에서 관리한다.
 * 이후 핀맵이 변경되어도 아래 값만 수정하면 된다.
 */
struct HardwareConstants
{
    /*
     * PCA9685 Servo Channel
     * 실제 OrCAD 회로도에서 연결한 채널 번호와
     * 반드시 일치시켜야 한다.
     */
    static constexpr int SERVO_BOTTOM_CHANNEL = 1;
    static constexpr int SERVO_TOP_CHANNEL    = 0;


    /*
     * X축 SM1504
     * 기존 StepperMotor 테스트에서 사용한 BCM GPIO 기준.
     */
    static constexpr int X_STEP_PIN   = 17;
    static constexpr int X_DIR_PIN    = 27;
    static constexpr int X_ENABLE_PIN = 22;


    /*
     * Y축 SM1504
     * 이 부분은 최종 회로도에서 지정한 GPIO 번호로
     * 변경해야 한다.
     */
    static constexpr int Y_STEP_PIN   = 23;   // 최종 핀번호 입력
    static constexpr int Y_DIR_PIN    = 24;   // 최종 핀번호 입력
    static constexpr int Y_ENABLE_PIN = 25;   // 최종 핀번호 입력
};

// ============================================================
// 전체 제어 시스템에서 사용하는 공통 상수
// ============================================================
struct ControlConstants
{
    /*
     * 제어 루프 주기
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
// 실제 Hardware Driver
// ========================================================
/*
 * Phase 1 ~ Phase 4에서 공통으로 사용하기 때문에
 * 각 Phase 내부에서 객체를 새로 생성하지 않고
 * Controller가 하드웨어 객체를 소유하도록 한다.
 *
 * 이렇게 하면
 *
 * 초기화
 *   ↓
 * 중간유도
 *   ↓
 * 종말유도
 *   ↓
 * 동체제어
 *
 * 전체 과정에서 동일한 하드웨어 상태를 유지할 수 있다.
 */

PCA9685 m_pca9685;
MPU6050 m_imu;

StepperMotor m_stepperX;
StepperMotor m_stepperY;

/*
 * StepperMotor의 경우 실제 작성한 라이브러리의
 * 생성자 형태에 맞춰 X/Y 두 객체를 생성한다.
 *
 * 정확한 생성자 형태는 StepperMotor.h 확인 후
 * 바로 확정한다.
 */

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
    /*
     * 아래 값들은 아직 실제 센서/통신 라이브러리가
     * 메인 시스템에 연결되지 않았기 때문에 임시로 사용한다.
     * 추후 외부 라이브러리 연결 시 실제 측정값으로 변경한다.
     *
     * 중요:
     * calibration_data.h에 저장된 값은 Mock 값이 아니다.
     * 실제 측정을 통해 확보한 하드웨어 기준값이다.
     */


    /*
     * 현재 시커 추적 알고리즘 검증용 목표 위치
     *
     * 추후 초음파 센서 거리값을 이용한
     * 타겟 방향 계산으로 변경한다.
     */
    double m_mockTargetX = 25.0;
    double m_mockTargetY = 17.5;


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

    IntegratedMissileController()
    : m_stepperX(
          HardwareConstants::X_STEP_PIN,
          HardwareConstants::X_DIR_PIN,
          HardwareConstants::X_ENABLE_PIN
      ),
      m_stepperY(
          HardwareConstants::Y_STEP_PIN,
          HardwareConstants::Y_DIR_PIN,
          HardwareConstants::Y_ENABLE_PIN
      )
    {
    }
    // ========================================================
    // Servo PWM 출력 인터페이스
    // ========================================================
    /*
     * motorName을 이용하여 출력할 PCA9685 채널을 결정하고,
     * calibration_data.h에서 전달된 PWM 값을 실제 PCA9685에 출력한다.
     *
     * PCA9685의 setPWM() 함수는
     *
     * setPWM(channel, ON count, OFF count)
     *
     * 형식으로 사용한다.
     *
     * 서보 제어에서는 일반적으로 ON count를 0으로 두고
     * OFF count에 목표 PWM 값을 전달한다.
     */

    void writeServoPWM(
        const std::string& motorName,
        int pwmValue)
    {
        uint8_t channel = 0;


        // ----------------------------------------------------
        // Servo 이름에 따라 PCA9685 채널 결정
        // ----------------------------------------------------

        if (motorName == "Bottom Servo")
        {
            channel =
                HardwareConstants::SERVO_BOTTOM_CHANNEL;
        }
        else if (motorName == "Top Servo")
        {
            channel =
                HardwareConstants::SERVO_TOP_CHANNEL;
        }
        else
        {
            std::cerr
                << " -> [ERROR] 알 수 없는 Servo 이름 : "
                << motorName
                << '\n';

            return;
        }


        // ----------------------------------------------------
        // 실제 PCA9685 PWM 출력
        // ----------------------------------------------------
        /*
         * ON  = 0
         * OFF = pwmValue
         *
         * calibration_data.h에 저장된 실제 중심 PWM 값을
         * 그대로 PCA9685에 전달한다.
         */

        m_pca9685.setPWM(
            channel,
            0,
            static_cast<uint16_t>(pwmValue)
        );


        // ----------------------------------------------------
        // 디버깅 출력
        // ----------------------------------------------------

        std::cout
            << " -> [Servo Init] "
            << motorName
            << " | Channel = "
            << static_cast<int>(channel)
            << " | PWM = "
            << pwmValue
            << '\n';
    }


    // ========================================================
    // SM1504 Linear Actuator 중심 위치 설정
    // ========================================================
    /*
     * X축과 Y축 SM1504를 모두 캘리브레이션된
     * 중심 위치로 이동시킨다.
     *
     * 현재 단계에서는 시스템 시작 전에
     * 두 액추에이터가 기준 시작 위치에 놓여 있다고 가정한다.
     *
     * 이후 리미트 스위치를 이용한 Homing 기능이 추가되면
     *
     * Homing
     *   ↓
     * 현재 위치 = 0 설정
     *   ↓
     * 중심 위치까지 이동
     *
     * 구조로 확장한다.
     */
    void writeLinearActuator(int targetStep)
    {
        /*
         * 현재 최소 통합 테스트에서는
         * X/Y 두 축이 동일한 중심 이동량을 가진다고 가정한다.
         * 이후 실제 조립 후 축별 중심값 차이가 확인되면
         * X/Y 값을 별도로 분리한다.
         */

        // ----------------------------------------------------
        // X축 중심 위치 이동
        // ----------------------------------------------------

        if (!m_stepperX.moveSteps(targetStep))
        {
            std::cerr
                << " -> [ERROR] X축 SM1504 이동 실패\n";

            return;
        }

        std::cout
            << " -> X축 SM1504 Center Position = "
            << targetStep
            << " step\n";


        // ----------------------------------------------------
        // Y축 중심 위치 이동
        // ----------------------------------------------------

        if (!m_stepperY.moveSteps(targetStep))
        {
            std::cerr
                << " -> [ERROR] Y축 SM1504 이동 실패\n";

            return;
        }

        std::cout
            << " -> Y축 SM1504 Center Position = "
            << targetStep
            << " step\n";
    }

    // ========================================================
    // Linear Actuator 시작 위치 복귀
    // ========================================================
    /*
     * 마이크로 스위치 적용 전 테스트 목적
     *
     * 프로그램 실행 중 누적된 Step 위치를 기준으로
     * X/Y 액추에이터를 최초 시작 위치(0 step)로 복귀시킨다.
     *
     * 현재 방식은 엔코더나 리미트 스위치가 없는
     * Open-Loop 위치 추정 방식이다.
     *
     * 따라서 모터 탈조가 발생하지 않았다는 전제가 필요하다.
     */
    void returnLinearActuatorToStart()
    {
        // 현재 논리적 위치 확인
        const long currentX = m_stepperX.getCurrentPosition();

        const long currentY =  m_stepperY.getCurrentPosition();


        std::cout
            << "\n[Return] Linear Actuator 원위치 복귀 시작\n"
            << " -> Current X : "
            << currentX
            << " step\n"
            << " -> Current Y : "
            << currentY
            << " step\n";


        // ----------------------------------------------------
        // X축 원점 복귀
        // ----------------------------------------------------
        if (!m_stepperX.moveSteps(-static_cast<int>(currentX)))
        {
            std::cerr
                << " -> [ERROR] X축 원위치 복귀 실패\n";
        }


        // ----------------------------------------------------
        // Y축 원점 복귀
        // ----------------------------------------------------

        if (!m_stepperY.moveSteps(-static_cast<int>(currentY)))
        {
            std::cerr
                << " -> [ERROR] Y축 원위치 복귀 실패\n";
        }


        std::cout
            << " -> Linear Actuator 원위치 복귀 완료\n"
            << " -> X Position : "
            << m_stepperX.getCurrentPosition()
            << " step\n"
            << " -> Y Position : "
            << m_stepperY.getCurrentPosition()
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

            // 전체 상태 머신 기본 실행 주기
            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    static_cast<long long>(
                        ControlConstants::LOOP_TIME_MS
                    )
                )
            );
        }
        
        // 전체 Phase 종료 후 액추에이터 원위치 복귀
        returnLinearActuatorToStart();
        m_stepperX.disable();
        m_stepperY.disable();

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
    /*
     * 기존 코드에서는 가상의 자이로 데이터를 반복 측정하여
     * Bias를 계산했지만,
     *
     * 현재는 실제 하드웨어 측정을 통해 이미 확보한 값을
     * calibration_data.h에서 직접 불러온다.
     *
     * 수행 순서
     *
     * 1. MPU6050 실제 연결 및 초기화
     * 2. PCA9685 실제 연결 및 초기화
     * 3. SM1504 X/Y축 GPIO 초기화
     * 4. 측정된 Gyro Bias 불러오기
     * 5. Servo 초기 위치
     * 6. Linear Actuator 중심 위치
     */

    void processPhase1_Initializing()
    {
      std::cout
            << "\n[Phase 1] 하드웨어 초기화 시작...\n";

        // ----------------------------------------------------
        // 1. MPU6050 실제 연결 및 초기화
        // ----------------------------------------------------
        /*
         * I2C 통신이 정상적으로 이루어지는지 확인한다.
         *
         * 초기화 실패 시 이후 Phase로 넘어가면
         * 잘못된 자세 정보를 이용하여 구동기가 움직일 수 있으므로
         * 시스템을 종료한다.
         */
        if (!m_imu.begin())
        {
            std::cerr
                << " -> [ERROR] MPU6050 초기화 실패\n"
                << " -> 시스템을 종료합니다.\n";

            m_currentState = SystemState::MISSION_COMPLETE;
            return;
        }

        std::cout
            << " -> MPU6050 연결 성공\n";

        // ----------------------------------------------------
        // 2. PCA9685 실제 연결 및 초기화
        // ----------------------------------------------------
        /*
         * PCA9685와 I2C 통신이 정상적으로 이루어지는지 확인한다.
         * 초기화 후 서보모터 구동을 위해 PWM 주파수를 50Hz로 설정한다.
         */
        if (!m_pca9685.begin())
        {
            std::cerr
                << " -> [ERROR] PCA9685 초기화 실패\n"
                << " -> 시스템을 종료합니다.\n";

            m_currentState = SystemState::MISSION_COMPLETE;
            return;
        }

        m_pca9685.setPWMFreq(50.0f);

        std::cout
            << " -> PCA9685 연결 성공\n"
            << " -> PWM Frequency : 50 Hz\n";
        
        // ----------------------------------------------------
        // 3. SM1504 X/Y축 GPIO 초기화
        // ----------------------------------------------------
        /*
         * A4988에 연결된 X축과 Y축 스텝모터의
         * GPIO 자원을 초기화한다.
         *
         * 어느 한 축이라도 초기화에 실패하면
         * 이후 질량 이동 제어를 수행할 수 없으므로
         * 시스템을 종료한다.
         */
        if (!m_stepperX.initialize())
        {
            std::cerr
                << " -> [ERROR] X축 StepperMotor 초기화 실패\n"
                << " -> 시스템을 종료합니다.\n";

            m_currentState = SystemState::MISSION_COMPLETE;
            return;
        }

        if (!m_stepperY.initialize())
        {
            std::cerr
                << " -> [ERROR] Y축 StepperMotor 초기화 실패\n"
                << " -> 시스템을 종료합니다.\n";

            m_currentState = SystemState::MISSION_COMPLETE;
            return;
        }

        /*
         * 프로그램 시작 시 현재 물리적 위치를
         * 논리적 원점(0 step)으로 정의한다.
         */
        m_stepperX.setCurrentPosition(0);
        m_stepperY.setCurrentPosition(0);

        std::cout
            << " -> X/Y축 StepperMotor 초기화 성공\n";

        // ----------------------------------------------------
        // 4. 측정된 Gyro Bias 불러오기
        // ----------------------------------------------------
        /*
         * imu_bias_measure.cpp에서 실제 측정한 값을
         * calibration_data.h를 통해 불러온다.
         */
        const double gyroBiasX = GYRO_X_BIAS;
        const double gyroBiasY = GYRO_Y_BIAS;
        const double gyroBiasZ = GYRO_Z_BIAS;


        std::cout
            << " -> MPU6050 Gyro Bias 로드 완료\n"
            << "    X : " << gyroBiasX << '\n'
            << "    Y : " << gyroBiasY << '\n'
            << "    Z : " << gyroBiasZ << '\n';


        // ----------------------------------------------------
        // 5. Servo 초기 위치
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
        // 6. Linear Actuator 중심 위치
        // ----------------------------------------------------
        writeLinearActuator(
            SM1504_CENTER_STEP
        );


        std::cout
            << " -> 전체 하드웨어 초기화 과정 완료\n"
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
            targetX = 15.0;
            targetY = 12.5;
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
    /*
     * 현재 추적 알고리즘은 유지한다.
     *
     * 초음파 센서 및 MPU6050 실시간 입력은
     * 추후 외부 라이브러리를 연결하면서 교체한다.
     */
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
            // ------------------------------------------------
            /*
             * 추후 이 부분은 3개의 초음파 센서 거리값을
             * 입력으로 받는 타겟 방향 계산 함수로 교체한다.
             */

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
    /*
     * 시커가 바라보는 방향을 기준으로
     * 동체를 정렬한다.
     *
     * 현재는 P 제어 기반 Mock 동체 모델을 유지한다.
     */

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