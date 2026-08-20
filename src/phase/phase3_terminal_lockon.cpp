#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

// 제어 시스템 하드웨어 상수 정의
struct ControlConstants {
    static constexpr double SEEKER_YAW_CENTER   = 90.0; // 시커 짐벌 Yaw 서보모터 기하학적 중심각 (도)
    static constexpr double SEEKER_PITCH_CENTER = 90.0; // 시커 짐벌 Pitch 서보모터 기하학적 중심각 (도)
    static constexpr double LOOP_TIME_MS = 20.0;        // 제어 주기 (20ms = 실시간 50Hz 제어 루프 충족)
    static constexpr double LOCKON_THRESHOLD = 2.0;     // 록온(Lock-on) 판단을 위한 허용 잔류 오차 한계값 (도)
};

int main() {
    std::cout << "\n==================================================" << std::endl;
    std::cout << "[Phase 3] 종말 유도 단계: 시커 고속 추적 및 역보정 가동" << std::endl;
    std::cout << "==================================================" << std::endl;

    // 제어 변수 초기화
    double seekerYawAngle = 0.0, seekerPitchAngle = 0.0; // 중심각 기준 시커의 상대 변위각 (오프셋)
    double mockTargetX = 15.0, mockTargetY = 10.0;       // 시뮬레이션용 가상 표적 위치 (도)
    double gyroYawRate = -5.0, gyroPitchRate = 3.0;      // 자이로 센서가 계측한 외란에 의한 동체 각속도 (deg/s) 
    
    int stableLockCount = 0;                             // 제어계 안정을 확인하기 위한 록온 유지 횟수 카운터
    double dt = ControlConstants::LOOP_TIME_MS / 1000.0; // 수치 적분을 위한 이산 시간 변화량 (dt = 0.02초)

    // 실시간 유도 제어 루프 진입
    while (true) {
        // [1] 시커 오차(Error) 연산: 표적 위치와 현재 시커 지향각의 편차 계산
        double seekerErrorX = mockTargetX - seekerYawAngle;
        double seekerErrorY = mockTargetY - seekerPitchAngle;

        // [2] 복합 제어 입력 산출 (피드백 + 피드포워드 외란 상쇄)
        double trackingGain = 0.3; // 표적 추종을 위한 비례 제어 게인 (P-Gain)
        
        // (오차 * 게인) = 표적을 쫓아가는 피드백 항
        // (각속도 * dt) = 동체 회전(외풍 등 외란)을 즉각 지워버리는 자이로 피드포워드(Feed-forward) 역보정 항
        double deltaYaw   = (seekerErrorX * trackingGain) - (gyroYawRate * dt);
        double deltaPitch = (seekerErrorY * trackingGain) - (gyroPitchRate * dt);

        // [3] 상태 업데이트: 산출된 제어 입력을 누적 적분하여 시커 변위 갱신
        seekerYawAngle   += deltaYaw;
        seekerPitchAngle += deltaPitch;

        // [4] 구동기 신호 매핑: 상대 변위를 실제 서보모터 구동 물리각(90도 중심)으로 변환
        double finalScaleYaw   = ControlConstants::SEEKER_YAW_CENTER + seekerYawAngle;
        double finalScalePitch = ControlConstants::SEEKER_PITCH_CENTER + seekerPitchAngle;

        // 디버깅 데이터 출력: 실시간 추종 상태 모니터링
        std::cout << " [추적중] 시커 오차 -> X: " << seekerErrorX << "도, Y: " << seekerErrorY 
                  << "도 | 짐벌 출력 -> Yaw: " << finalScaleYaw << "도, Pitch: " << finalScalePitch << "도\n";

        // [5] finite State Machine (FSM) 상태 천이 조건 검증
        // 오차가 허용 임계값(2.0도) 이내로 들어왔는지 판별
        if (std::abs(seekerErrorX) <= ControlConstants::LOCKON_THRESHOLD && std::abs(seekerErrorY) <= ControlConstants::LOCKON_THRESHOLD) {
            stableLockCount++;
            // 20ms 간격으로 5회 연속(총 0.1초 동안) 오차를 유지하면 제어계가 완전히 안정화(안착)되었다고 판단
            if (stableLockCount >= 5) {
                std::cout << "\n -> 🎯 [LOCK-ON SUCCESS] 타겟 추적 록온 완벽 안착!" << std::endl;
                break; // Phase 3 루프 종료 후 탈출
            }
        } else {
            // 한 번이라도 임계값을 벗어나면 카운터를 초기화하여 채터링(불안정 상태) 방지
            stableLockCount = 0;
        }

        // 50Hz 실시간 동기화를 위한 젠틀한 대기 (OS에게 CPU 자원을 반납하는 표준 C++ Sleep 구조)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // 다음 FSM 상태(Phase 4: 질량 이동 동체 조향)로 전환 트리거 발생
    std::cout << "[전환 트리거] 시커 고속 고정 완료. [최종 동체 추종 단계]로 전환합니다.\n";
    return 0;
}