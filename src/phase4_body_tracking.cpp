#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>


// 제어 시스템에서 사용하는 고정 상수 정의
struct ControlConstants {

    // 동체(Body) 기준 Yaw 초기 중심각
    static constexpr double BODY_YAW_CENTER = 90.0;

    // 시커(탐색 센서) 짐벌 Yaw 중심각
    static constexpr double SEEKER_YAW_CENTER = 90.0;

    // 시커(탐색 센서) 짐벌 Pitch 중심각
    static constexpr double SEEKER_PITCH_CENTER = 90.0;

    // 제어 루프 주기
    // 20ms = 50Hz 제어 주파수
    static constexpr double LOOP_TIME_MS = 20.0;

    // 최종 정렬 완료 판단 오차 범위
    // 0.5도 이내면 정렬 완료로 판단
    static constexpr double TRACKING_TOLERANCE = 0.5;
};


int main() {

    std::cout << "\n==================================================" << std::endl;
    std::cout << "[Phase 4] 최종 동체 추종 단계: 시커-동체 물아일체 정렬 가동" << std::endl;
    std::cout << "==================================================" << std::endl;


    /*
        현재 동체 자세 초기값

        Phase 2~3을 통해:
        - 동체가 목표 방향으로 기동 완료
        - 시커가 목표를 추적한 상태

        라고 가정한 초기 조건
    */
    double currentBodyX = 22.0;   // 현재 동체 Yaw 각도
    double currentBodyY = 15.0;   // 현재 동체 Pitch 각도


    /*
        시커가 바라보고 있는 절대 방향

        현재 시커는 동체 기준 중심각(90도)에서
        Yaw +15도, Pitch +9.5도 만큼 벗어난 상태

        즉 동체가 시커 방향으로 회전해야 함
    */
    double seekerAbsYaw = 105.0;
    double seekerAbsPitch = 99.5;


    /*
        비례 제어 게인(Kp)

        오차 크기에 비례하여
        동체 이동량을 결정

        이동량 = 오차 × Kp

        Kp가 크면:
        - 빠른 응답
        - 오버슈트 가능

        Kp가 작으면:
        - 안정적
        - 응답 느림
    */
    double bodyKpX = 0.25;
    double bodyKpY = 0.20;



    // 실시간 제어 루프
    while (true) {


        /*
            시커와 동체 사이의 방향 오차 계산

            시커 중심각(90도)을 기준으로
            얼마나 벗어나 있는지 계산

            예)
            seekerAbsYaw = 105도

            105 - 90 = +15도

            → 동체가 15도 회전해야 함
        */
        double bodyErrorX =
            seekerAbsYaw - ControlConstants::SEEKER_YAW_CENTER;


        double bodyErrorY =
            seekerAbsPitch - ControlConstants::SEEKER_PITCH_CENTER;



        /*
            정렬 완료 조건

            Yaw와 Pitch 오차가 모두
            0.5도 이하이면 목표 방향과 일치했다고 판단
        */
        if (std::abs(bodyErrorX) <= ControlConstants::TRACKING_TOLERANCE &&
            std::abs(bodyErrorY) <= ControlConstants::TRACKING_TOLERANCE) {


            std::cout << "\n🎯 [ALIGNMENT COMPLETE] "
                      << "동체와 시커가 타겟 정면에 완벽 일치했습니다. 돌격!"
                      << std::endl;

            break;
        }



        /*
            비례 제어(P Control)

            현재 오차에 비례하여
            이번 제어 주기에서 이동할 각도를 계산

            bodyMoveX = 오차 × Kp
        */
        double bodyMoveX = bodyErrorX * bodyKpX;
        double bodyMoveY = bodyErrorY * bodyKpY;



        /*
            동체 조향 명령 적용

            계산된 이동량만큼
            동체 방향 변경
        */
        currentBodyX += bodyMoveX;
        currentBodyY += bodyMoveY;



        /*
            시커-동체 상대 오차 감소

            동체가 회전하면
            시커 기준으로 바라보는 오차도 감소

            실제 시스템에서는:
            - 짐벌 각도
            - IMU 자세값
            - 엔코더 값

            등을 이용해 계산
        */
        seekerAbsYaw   -= bodyMoveX;
        seekerAbsPitch -= bodyMoveY;



        // 현재 제어 상태 출력
        std::cout << " [정렬중] "
                  << "동체 조향 오차 -> X: " << bodyErrorX
                  << "도, Y: " << bodyErrorY
                  << "도 | 동체 절대각 -> X: "
                  << currentBodyX
                  << "도, Y: "
                  << currentBodyY
                  << "도\n";


        /*
            20ms 대기

            실제 임베디드 시스템에서는:
            Timer Interrupt 또는 RTOS Task 주기로 대체
        */
        std::this_thread::sleep_for(
            std::chrono::milliseconds(20)
        );
    }


    return 0;
}