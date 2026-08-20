#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

// 제어에 사용되는 고정된 상수들을 모아둔 구조체입니다.
struct ControlConstants {
    // 제어 루프의 주기 (20ms -> 1초에 50번 돌도록 설정)
    static constexpr double LOOP_TIME_MS = 20.0;
    
    // 안전을 위한 최대 각속도 제한 (루프당 최대 2도씩만 회전 가능)
    static constexpr double MAX_ANGULAR_VELOCITY = 2.0; 
};

int main() {
    // -------------------------------------------------------------------------
    // 1. 초기화 및 대기 상태 시뮬레이션
    // -------------------------------------------------------------------------
    std::cout << "\n==================================================" << std::endl;
    std::cout << "[Phase 2] 중간 유도 단계: Putty 명령 대기 중..." << std::endl;
    std::cout << "==================================================" << std::endl;

    // Putty 터미널로부터 '1'이라는 명령어가 들어왔다고 가정 (가상 입력값)
    char mockPuttyInput = '1';
    
    // 이동할 최종 목표 각도 (X축, Y축) 초기화
    double targetX = 0.0, targetY = 0.0;

    // -------------------------------------------------------------------------
    // 2. 명령어 해석 및 목표 각도 설정
    // -------------------------------------------------------------------------
    std::cout << "[Putty 수신] 사용자 입력 발견 -> '" << mockPuttyInput << "' (1분면 기동 명령)" << std::endl;
    
    // 입력된 명령어가 '1'일 경우, 1분면의 특정 목표 지점(X: 22.5도, Y: 15.0도)으로 설정
    if (mockPuttyInput == '1') {
        targetX = 22.5; 
        targetY = 15.0;
    }
    std::cout << " -> 목표 사분면 확정: [1분면] 목표 각도 -> X: " << targetX << "도, Y: " << targetY << "도" << std::endl;

    // -------------------------------------------------------------------------
    // 3. 구동 제어 루프 (중간 유도 수행)
    // -------------------------------------------------------------------------
    // 현재 동체의 초기 각도 (0도에서 시작)
    double currentX = 0.0, currentY = 0.0;
    std::cout << "[구동 시작] 동체 기동 시동... (안전 속도 제한 적용)" << std::endl;

    // X축 또는 Y축 중 하나라도 목표 각도에 도달하지 못했다면 계속 반복 수행
    while (currentX < targetX || currentY < targetY) {
        
        // [속도 제한 적용] 현재 각도가 목표 각도보다 작다면, 최대 각속도(2.0도)만큼 증가시킴
        if (currentX < targetX) currentX += ControlConstants::MAX_ANGULAR_VELOCITY;
        if (currentY < targetY) currentY += ControlConstants::MAX_ANGULAR_VELOCITY;

        // [오버슈트(Overshoot) 방지] 
        // 각속도를 더하다가 목표 각도를 살짝 초과해버린 경우, 목표 각도에 딱 맞게 고정함
        if (currentX > targetX) currentX = targetX;
        if (currentY > targetY) currentY = targetY;

        // 현재 실시간 구동 상태를 출력
        std::cout << " [기동중] 현재 동체 각도 -> X: " << currentX << "도, Y: " << currentY << "도" << std::endl;
        
        // 실제 하드웨어 제어 주기(20ms)를 모사하기 위해 루프마다 20밀리초 동안 대기(Sleep)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // -------------------------------------------------------------------------
    // 4. 목표 도달 및 다음 단계(종말 유도)로의 전환
    // -------------------------------------------------------------------------
    // While 루프를 빠져나왔다는 것은 X, Y축 모두 목표 각도에 안전하게 안착했음을 의미함
    std::cout << " -> [목표 안착] 동체가 지시 사분면 중심점에 도달했습니다." << std::endl;
    
    // 중간 유도가 끝났으므로, 자체 센서(시커)를 켜고 마지막 추적 단계인 [종말 유도]로 넘어가는 트리거를 작동
    std::cout << "[전환 트리거] 중간 유도 성공. 시커를 활성화하고 [종말 유도 단계]로 진입합니다.\n";
    
    return 0;
}