#include "SeekerControl.h"

#include <iostream>

/*
 * ============================================================
 * File Name : seeker_control_test.cpp
 *
 * Description
 * ------------------------------------------------------------
 * seeker_tracking_v1을 SeekerControl 라이브러리로 분리한 뒤
 * 기존 V1과 동일하게 동작하는지 확인하기 위한 단독 시험.
 *
 * 이 테스트에서는:
 * - 질량이동 사용 X
 * - Body Yaw Stepper 사용 X
 * - Main State Machine 사용 X
 * - SeekerControl만 단독 실행
 *
 * 종료:
 * Ctrl + C
 * ============================================================
 */

int main()
{
    std::cout
        << "========================================\n"
        << "     SeekerControl V1 Library Test\n"
        << "========================================\n";


    SeekerControl seeker;


    if (!seeker.initialize())
    {
        std::cerr
            << "[ERROR] SeekerControl 초기화 실패\n";

        return 1;
    }


    std::cout
        << "[INIT COMPLETE]\n"
        << "V1 tracking start\n"
        << "Ctrl + C to quit\n"
        << "========================================\n";


    while (true)
    {
        seeker.update();
    }


    return 0;
}

