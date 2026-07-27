#include <iostream>   // 콘솔 출력(std::cout)을 위한 라이브러리
#include <cmath>      // abs() 등 수학 함수 사용
#include <thread>     // sleep_for 사용
#include <chrono>     // 시간 제어(ms) 사용
#include <string>     // string 자료형 사용


// 시스템의 현재 단계를 정의하는 상태 열거형
// 상태 머신(State Machine) 구조를 이용
// Phase별 동작 흐름 관리
enum class SystemState {

    INITIALIZING,       // Phase 1: 센서 영점 및 초기화
                        // IMU/Gyro 등 센서 기준값 설정 단계

    MIDCOURSE_GUIDE,    // Phase 2: 중간 유도
                        // 외부 명령을 받아 목표 방향으로 동체 기동

    TERMINAL_LOCKON,    // Phase 3: 종말 유도
                        // 시커 기반 목표 추적 및 Lock-On 수행

    BODY_TRACKING,      // Phase 4: 최종 동체 고정 추종
                        // 시커 방향과 동체 방향 정렬

    MISSION_COMPLETE    // 모든 제어 과정 완료
};


// 제어 시스템 전역 상수 정의
// 제어 주기, 기준각, 오차 허용 범위 등
// 전체 제어 알고리즘에서 공통으로 사용하는 값
struct ControlConstants {

    // 제어 루프 주기
    // 20ms = 50Hz 제어 시스템
    static constexpr double LOOP_TIME_MS = 20.0;

    // 자이로 오프셋 허용치
    // 초기 센서 Calibration 과정에서 사용
    static constexpr double GYRO_BIAS_TOLERANCE = 0.01;


    // Phase 2 동체 제한 속도
    // 한 제어 루프에서 이동 가능한 최대 각도(deg)
    // 급격한 자세 변화를 방지하기 위한 제한값
    static constexpr double MAX_ANGULAR_VELOCITY = 2.0;


    // 시커 짐벌 Yaw 기준 중심 위치
    // Servo 중앙 위치(90도)
    static constexpr double SEEKER_YAW_CENTER   = 90.0;


    // 시커 짐벌 Pitch 기준 중심 위치
    static constexpr double SEEKER_PITCH_CENTER = 90.0;


    // 동체 방향 기준 중심 위치
    static constexpr double BODY_YAW_CENTER     = 90.0;


    // Phase 3 Lock-On 판단 기준 오차
    // 해당 값 이하로 목표 추적 시 성공 판단
    static constexpr double LOCKON_THRESHOLD = 2.0;


    // Phase 4 최종 정렬 허용 오차
    // 시커와 동체 방향 차이가 이 값 이하이면 완료
    static constexpr double TRACKING_TOLERANCE = 0.5;

};
class IntegratedMissileController {

private:

    // 현재 시스템 상태 저장 변수
    // 초기 상태는 센서 초기화(INITIALIZING)
    // 각 Phase 완료 후 다음 상태로 변경
    SystemState m_currentState = SystemState::INITIALIZING;


    // --- 공유 시스템 제어 변수 ---


    // 현재 동체 절대 각도 X축
    // 실제 구현에서는 IMU 자세값 또는 엔코더 값을 사용
    double m_currentBodyX = 0.0;


    // 현재 동체 절대 각도 Y축
    double m_currentBodyY = 0.0;


    // 시커 상대 Yaw 각도
    // 동체 기준으로 시커가 얼마나 회전했는지 나타냄
    double m_seekerYawAngle = 0.0;


    // 시커 상대 Pitch 각도
    double m_seekerPitchAngle = 0.0;


    // 시커 절대 Yaw 각도
    // Phase 4에서 동체 좌표계와 비교하기 위해 사용
    double m_seekerAbsYaw = 0.0;


    // 시커 절대 Pitch 각도
    double m_seekerAbsPitch = 0.0;


    // 안정적인 Lock-On 판정을 위한 카운터
    // 순간적으로 오차가 작아지는 경우를 방지하기 위해
    // 일정 횟수 이상 조건 만족 시 Lock-On 인정
    int m_stableLockCount = 0;

    // ===============================
    // 가상 센서 모킹 데이터
    // 현재는 하드웨어 센서 대신 테스트용 값 사용
    //
    // 실제 적용 시:
    // m_mockTargetX/Y → 시커 센서 입력값
    // m_gyroYawRate/PitchRate → IMU Gyro 입력값으로 변경
    // ===============================

    // 가상 타겟 X 위치
    // 시커 기준 상대값
    double m_mockTargetX = 15.0;


    // 가상 타겟 Y 위치
    double m_mockTargetY = 10.0;


    // 가상 Gyro Yaw 각속도
    // 외풍 및 회전 외란을 모사
    double m_gyroYawRate = -5.0;


    // 가상 Gyro Pitch 각속도
    double m_gyroPitchRate = 3.0;


public:

    // ==================================================
    // Servo PWM 출력 인터페이스
    //
    // 현재는 하드웨어 연결 전 함수 형태만 정의
    //
    // 실제 구현 시:
    // - PCA9685 I2C PWM 모듈
    // - MCU PWM 출력
    // 등을 통해 Servo 제어
    // ==================================================

    void writeServoPWM(std::string motorName, double angle) {

        // 하드웨어 드라이버 인터페이스 (추후 구현부)

    }

    // ==================================================
    // Linear Actuator 제어 인터페이스
    // 현재는 함수 형태만 정의
    //
    // 실제 구현 시:
    // - A4988 STEP/DIR 신호 출력
    // - 스텝모터 위치 제어
    // 연결 예정
    // ==================================================

    void writeLinearActuator(double position) {

        // 하드웨어 드라이버 인터페이스 (추후 구현부)

    }
        // ==================================================
    // 메인 상태 머신 실행 루프
    // 현재 시스템 상태를 확인하고
    // 해당 Phase 함수를 호출하는 메인 제어 루프
    //
    // 전체 흐름:
    //
    // INITIALIZING
    //       ↓
    // MIDCOURSE_GUIDE
    //       ↓
    // TERMINAL_LOCKON
    //       ↓
    // BODY_TRACKING
    //       ↓
    // MISSION_COMPLETE
    //
    // 실제 임베디드에서는 Timer Interrupt 또는 RTOS Task로 구현
    // ==================================================

    void runSystem() {

        std::cout << "==================================================" << std::endl;
        std::cout << "   [종합 제어 시스템 구동] 메인 가상 테스트베드 가동" << std::endl;
        std::cout << "==================================================" << std::endl;


        // Mission Complete 상태가 될 때까지 반복 실행
        while (m_currentState != SystemState::MISSION_COMPLETE) {


            // 현재 상태에 맞는 Phase 실행
            switch (m_currentState) {


                // Phase 1: 센서 초기화
                case SystemState::INITIALIZING:

                    processPhase1_Initializing();

                    break;


                // Phase 2: 중간 유도
                case SystemState::MIDCOURSE_GUIDE:

                    processPhase2_MidcourseGuide();

                    break;


                // Phase 3: 종말 추적
                case SystemState::TERMINAL_LOCKON:

                    processPhase3_TerminalLockOn();

                    break;


                // Phase 4: 동체 추종
                case SystemState::BODY_TRACKING:

                    processPhase4_BodyTracking();

                    break;


                // 예외 상태 처리
                default:

                    m_currentState = SystemState::MISSION_COMPLETE;

                    break;
            }

            // ==================================================
            // 제어 루프 주기 동기화
            // 20ms 대기
            // → 50Hz 제어 주기 유지
            // 추후 delay보다 Timer 기반으로 최적화 예정
            // ==================================================

            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    static_cast<long long>(ControlConstants::LOOP_TIME_MS)
                )
            );

        }

        std::cout << "\n==================================================" << std::endl;
        std::cout << " 🎉 전체 시뮬레이션 성공적 종료: 모든 페이즈 완벽 통과" << std::endl;
        std::cout << "==================================================" << std::endl;

    }



private:

    // ==================================================
    // Phase 1:
    // 센서 영점 초기화 및 Calibration
    //
    // 수행 내용:
    // 1. Gyro Offset 측정
    // 2. 평균 Bias 계산
    // 3. 초기 센서 기준값 설정
    //
    // 현재는 테스트를 위해 고정값 사용
    //
    // 실제 구현 시:
    // I2C/SPI IMU 데이터 샘플링 후 평균 계산
    // ==================================================

    void processPhase1_Initializing() {

        std::cout << "\n[Phase 1] 센서 영점 초기화 시작..." << std::endl;


        // Gyro Offset 누적 변수
        double gyroOffsetSumX = 0.0;
        double gyroOffsetSumY = 0.0;


        // 센서 샘플 개수
        int sampleCount = 20;

        // ==================================================
        // 가상 센서 데이터 측정
        //
        // 실제:
        // gyro 값을 반복 측정하여 평균 Bias 계산
        //
        // 현재:
        // 테스트용 고정값 입력
        // ==================================================

        for (int i = 0; i < sampleCount; ++i) {
            gyroOffsetSumX += 0.02;
            gyroOffsetSumY += -0.01;
        }


        // 평균 Bias 계산
        double finalBiasX = gyroOffsetSumX / sampleCount;
        double finalBiasY = gyroOffsetSumY / sampleCount;


        std::cout 
            << " -> 자이로 오프셋 측정 완료 X: "
            << finalBiasX
            << " rad/s, Y: "
            << finalBiasY
            << " rad/s\n";

        std::cout 
            << " -> 구동기 영점 셋팅 완료. [중간 유도 단계]로 자동 천이합니다.\n";


        // 초기화 완료 후 Phase 2 이동
        m_currentState = SystemState::MIDCOURSE_GUIDE;

    }    

    // ==================================================
    // Phase 2:
    // 중간 유도 단계
    //
    // 목적:
    // - 외부 명령을 통해 목표 방향 결정
    // - 동체를 목표 방향으로 이동
    //
    // 현재 구현:
    // Putty 입력을 모의 데이터로 처리
    //
    // 실제 구현 시:
    // UART / RS485 / CAN 통신 등을 통해
    // 외부 명령 수신
    // ==================================================

    void processPhase2_MidcourseGuide() {

        std::cout << "\n[Phase 2] 중간 유도 단계 진입: Putty 명령 대기 중..." << std::endl;

        // ==================================================
        // 가상 UART 수신 데이터
        //
        // 실제 시스템에서는:
        // Serial RX Buffer에서 데이터 읽기
        //
        // 예:
        // '1' → 1사분면 목표
        // ==================================================

        char mockPuttyInput = '1';


        // 목표 자세 초기화
        double targetX = 0.0;
        double targetY = 0.0;

        // ==================================================
        // 입력 명령에 따른 목표 방향 설정
        //
        // 현재는 '1' 입력 시
        // X축 22.5도
        // Y축 15도
        // 로 이동
        // ==================================================

        if (mockPuttyInput == '1') {
            targetX = 22.5;
            targetY = 15.0;
        }

        std::cout 
            << " -> [Putty] '1' 수신. 목표 사분면 확정 -> X: "
            << targetX
            << "도, Y: "
            << targetY
            << "도\n";

        // ==================================================
        // 목표 방향까지 동체 이동
        //
        // 현재 방식:
        // 최대 각속도 제한을 적용한 단순 위치 제어
        //
        // MAX_ANGULAR_VELOCITY:
        // 한 루프당 이동 가능한 최대 각도
        //
        // 목적:
        // 급격한 조향 방지
        // ==================================================

        while (m_currentBodyX < targetX || 
               m_currentBodyY < targetY) {


            // X축 이동
            if (m_currentBodyX < targetX)

                m_currentBodyX += ControlConstants::MAX_ANGULAR_VELOCITY;


            // Y축 이동
            if (m_currentBodyY < targetY)

                m_currentBodyY += ControlConstants::MAX_ANGULAR_VELOCITY;

            // ==================================================
            // 목표값 초과 방지
            //
            // Overshoot 발생 시
            // 목표값으로 제한
            // ==================================================

            if (m_currentBodyX > targetX)
                m_currentBodyX = targetX;

            if (m_currentBodyY > targetY)
                m_currentBodyY = targetY;


            // 현재 동체 각도 출력
            std::cout 
                << " [기동중] 동체 절대 각도 -> X: "
                << m_currentBodyX
                << "도, Y: "
                << m_currentBodyY
                << "도\n";


            // 제어 주기 유지
            std::this_thread::sleep_for(
                std::chrono::milliseconds(20)
            );

        }

        // ==================================================
        // 목표 사분면 도달
        //
        // 다음 단계:
        // 시커 활성화 및 종말 유도 진입
        // ==================================================

        std::cout 
            << " -> [목표 사분면 도달] 시커 센서를 활성화하고 [종말 유도 단계]로 진입합니다.\n";


        // Phase 3 상태 변경
        m_currentState = SystemState::TERMINAL_LOCKON;

    }    
    
    // ==================================================
    // Phase 3:
    // 종말 유도 단계
    //
    // 목적:
    // - 시커(Seeker)를 이용하여 목표 추적
    // - 목표와 현재 시커 방향의 오차 계산
    // - 오차 기반 짐벌 제어 수행
    // - 일정 시간 안정적으로 추적 시 Lock-On 판단
    //
    // 현재 구현:
    // P 제어 + Gyro 각속도 역보상 구조
    //
    // 실제 구현 시:
    // - 초음파 센서 입력
    // - IMU Gyro 데이터
    // - Servo PWM 출력
    // 연결 필요
    // ==================================================

    void processPhase3_TerminalLockOn() {

        std::cout << "\n[Phase 3] 종말 유도 단계 진입: 시커 고속 추적 및 역보정 가동" << std::endl;

        // ==================================================
        // 제어 주기 변환
        //
        // ms → sec 변환
        //
        // 각속도(deg/s)를 각도 변화량으로 변환할 때 사용
        // ==================================================

        double dt = ControlConstants::LOOP_TIME_MS / 1000.0;


        // 목표 추적 루프
        while (true) {

            // ==================================================
            // 시커 기준 목표 오차 계산
            //
            // 목표 위치 - 현재 시커 방향
            //
            // 오차가 크면 더 큰 방향 보정 필요
            // ==================================================

            double seekerErrorX = m_mockTargetX - m_seekerYawAngle;
            double seekerErrorY = m_mockTargetY - m_seekerPitchAngle;

            // ==================================================
            // 추적 Gain
            //
            // 현재는 P 제어 계수
            //
            // 최적화 단계에서는:
            // Gain 변화에 따른
            // 응답 속도 / Overshoot / 안정시간 비교 가능
            // ==================================================

            double trackingGain = 0.3;

            // ==================================================
            // 시커 이동량 계산
            //
            // 오차 보정:
            // Error × Gain
            //
            // Gyro 역보상:
            // 외란으로 발생하는 회전을 상쇄
            // ==================================================

            double deltaYaw =
                (seekerErrorX * trackingGain)
                - (m_gyroYawRate * dt);

            double deltaPitch =
                (seekerErrorY * trackingGain)
                - (m_gyroPitchRate * dt);


            // 계산된 보정량 적용

            m_seekerYawAngle += deltaYaw;
            m_seekerPitchAngle += deltaPitch;

            // ==================================================
            // Servo 출력용 절대각 변환
            //
            // Servo 중앙 위치:
            // 90도
            //
            // 상대각 + 중심각 = 실제 출력각
            // ==================================================

            double finalScaleYaw =
                ControlConstants::SEEKER_YAW_CENTER
                + m_seekerYawAngle;

            double finalScalePitch =
                ControlConstants::SEEKER_PITCH_CENTER
                + m_seekerPitchAngle;


            std::cout 
                << " [추적중] 시커 오차 -> X: "
                << seekerErrorX
                << "도, Y: "
                << seekerErrorY 
                << "도 | 짐벌 출력 -> Yaw: "
                << finalScaleYaw
                << "도, Pitch: "
                << finalScalePitch
                << "도\n";

            // ==================================================
            // Lock-On 판단
            //
            // 오차가 기준값 이하일 경우
            // 안정화 카운터 증가
            //
            // 일정 횟수 이상 유지:
            // → 실제 추적 성공 판단
            // ==================================================

            if (std::abs(seekerErrorX) <= ControlConstants::LOCKON_THRESHOLD && 
                std::abs(seekerErrorY) <= ControlConstants::LOCKON_THRESHOLD) {

                m_stableLockCount++;

                // 5회 연속 조건 만족 시 Lock-On 성공
                if (m_stableLockCount >= 5) {

                    std::cout << "\n -> 🎯 [LOCK-ON SUCCESS] 타겟 록온 안착! 제어권 이관.\n";

                    // ==================================================
                    // Phase 4 전달 데이터 저장
                    //
                    // 시커 절대 방향 정보를 저장하여
                    // 이후 동체 정렬에 사용
                    // ==================================================

                    m_seekerAbsYaw = finalScaleYaw;
                    m_seekerAbsPitch = finalScalePitch;

                    break;

                }
            }

            else {

                // 오차가 다시 증가하면
                // 안정화 조건 초기화
                m_stableLockCount = 0;

            }


            // 20ms 제어 주기 유지
            std::this_thread::sleep_for(
                std::chrono::milliseconds(20)
            );

        }


        // Phase 4 이동
        m_currentState = SystemState::BODY_TRACKING;

    }    
    
    // ==================================================
    // Phase 4:
    // 최종 동체 고정 추종 단계
    //
    // 목적:
    // - 시커가 바라보는 방향과 동체 방향 정렬
    // - 최종적으로 동체 축과 목표 방향 일치
    //
    // 현재 구현:
    // P 제어(Proportional Control)
    //
    // 제어식:
    //
    // 이동량 = 오차 × Kp
    //
    // 추후 개선:
    // P → PD → PID 제어 비교 가능
    // ==================================================

    void processPhase4_BodyTracking() {

        std::cout << "\n[Phase 4] 최종 동체 추종 단계 진입: 시커-동체 정렬 가동" << std::endl;

        // ==================================================
        // 비례 제어 Gain
        //
        // Kp가 클수록:
        // - 빠른 응답
        // - Overshoot 가능성 증가
        //
        // Kp가 작을수록:
        // - 안정성 증가
        // - 응답 속도 감소
        //
        // 최적화 과정에서 튜닝 대상
        // ==================================================

        double bodyKpX = 0.25;
        double bodyKpY = 0.20;


        while (true) {

            // ==================================================
            // 시커와 동체 사이 방향 오차 계산
            //
            // 시커 중심각(90도)을 기준으로
            // 현재 얼마나 틀어져 있는지 계산
            //
            // 예:
            // seekerAbsYaw = 105도
            //
            // 105 - 90 = 15도
            //
            // → 동체가 15도 이동 필요
            // ==================================================

            double bodyErrorX =
                m_seekerAbsYaw - ControlConstants::SEEKER_YAW_CENTER;

            double bodyErrorY =
                m_seekerAbsPitch - ControlConstants::SEEKER_PITCH_CENTER;

            // ==================================================
            // 최종 정렬 완료 판단
            //
            // Yaw/Pitch 오차가 모두 허용범위 이하이면
            // 정렬 완료
            // ==================================================

            if (std::abs(bodyErrorX) <= ControlConstants::TRACKING_TOLERANCE && 
                std::abs(bodyErrorY) <= ControlConstants::TRACKING_TOLERANCE) {

                std::cout << "\n🎯 [ALIGNMENT COMPLETE] 동체-시커 정렬 최종 완료. 돌격!\n";

                break;

            }

            // ==================================================
            // P 제어 입력 계산
            //
            // 오차 크기에 비례하여
            // 이번 제어 루프에서 이동할 양 결정
            //
            // 이동량 = Error × Kp
            // ==================================================

            double bodyMoveX = bodyErrorX * bodyKpX;
            double bodyMoveY = bodyErrorY * bodyKpY;


            // 계산된 제어량만큼 동체 각도 변경

            m_currentBodyX += bodyMoveX;
            m_currentBodyY += bodyMoveY;

            // ==================================================
            // 동체가 이동하면 시커 기준 오차 감소
            //
            // 실제 시스템에서는:
            // - IMU
            // - Encoder
            // - Gimbal Angle Sensor
            //
            // 값을 이용해 계산
            // ==================================================

            m_seekerAbsYaw -= bodyMoveX;
            m_seekerAbsPitch -= bodyMoveY;


            std::cout 
                << " [정렬중] 동체 조향 오차 -> X: "
                << bodyErrorX
                << "도, Y: "
                << bodyErrorY 
                << "도 | 동체 절대각 -> X: "
                << m_currentBodyX
                << "도, Y: "
                << m_currentBodyY
                << "도\n";


            // 제어 주기 유지
            std::this_thread::sleep_for(
                std::chrono::milliseconds(20)
            );

        }





        // 모든 Phase 완료
        m_currentState = SystemState::MISSION_COMPLETE;

    }

};

// ==================================================
// Main 함수
//
// IntegratedMissileController 객체 생성 후
// 전체 상태 머신 실행
// ==================================================

int main() {

    IntegratedMissileController controller;

    // Phase 1 → Phase 4 순차 실행
    controller.runSystem();

    return 0;

}