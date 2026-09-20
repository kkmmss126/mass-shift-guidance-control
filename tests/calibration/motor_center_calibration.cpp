/*
 * File: motor_center_calibration.cpp
 *
 * Description:
 * 질량 이동용 리니어 액추에이터의 중심 위치(CENTER_STEP)를
 * 측정하기 위한 테스트 프로그램이다.
 *
 * 키보드 입력으로 액추에이터를 수동 이동시키며,
 * 기계적 중심에 도달했을 때의 Step 값을 확인한다.
 *
 * Project:
 * 질량 이동 기반 중간 및 종말 유도 모사 시스템
 *
 * 주요 기능:
 * 1. 액추에이터 수동 이동
 * 2. 현재 Step 위치 확인
 * 3. 원점(0 Step) 설정
 * 4. 중심 위치(CENTER_STEP) 측정
 */

#include "StepperMotor.h"

#include <iostream>

/*
 * BCM GPIO 번호를 사용한다.
 *
 * 실제 배선과 일치하도록 수정해야 한다.
 */
constexpr int STEP_PIN = 17;
constexpr int DIR_PIN = 27;
constexpr int ENABLE_PIN = 22;

/*
 * STEP 펄스의 HIGH/LOW 유지시간(us)
 *
 * 값이 클수록 모터는 천천히 움직이며,
 * 초기 테스트는 1000~2000us 정도를 권장한다.
 */
constexpr int PULSE_DELAY_US = 1500;

/*
 * 프로그램에서 사용할 조작 방법을 출력한다.
 */
void printMenu()
{
    std::cout
        << "\n========== 액추에이터 중심 위치 탐색 ==========\n"
        << "1 : 정방향 1000 step\n"
        << "2 : 정방향 100 step\n"
        << "3 : 정방향 10 step\n"
        << "4 : 정방향 1 step\n"
        << "5 : 역방향 1000 step\n"
        << "6 : 역방향 100 step\n"
        << "7 : 역방향 10 step\n"
        << "8 : 역방향 1 step\n"
        << "z : 현재 위치를 0으로 지정\n"
        << "c : 현재 위치를 중심값으로 출력\n"
        << "p : 현재 위치 출력\n"
        << "q : 종료\n"
        << "==============================================\n";
}

int main()
{
    /*
     * StepperMotor 객체 생성
     *
     * STEP, DIR, ENABLE GPIO 번호를 전달한다.
     */
    StepperMotor actuator(
        STEP_PIN,
        DIR_PIN,
        ENABLE_PIN
    );

    // 모터 속도(펄스 간격) 설정
    actuator.setPulseDelay(PULSE_DELAY_US);

    // GPIO 초기화
    if (!actuator.initialize())
    {
        std::cerr << "모터 GPIO 초기화 실패\n";
        return 1;
    }

    /*
     * 프로그램 시작 시 현재 위치를
     * 기준 위치(0 Step)로 설정한다.
     */
    actuator.setCurrentPosition(0);

    std::cout
        << "액추에이터 중심 위치 탐색 프로그램\n\n"
        << "주의: 이 프로그램에는 리미트 스위치가 없습니다.\n"
        << "기계적 끝에 도달하기 전에 반드시 정지하십시오.\n\n"
        << "액추에이터를 기준 시작 위치에 둔 뒤\n"
        << "z를 입력하여 현재 위치를 0으로 지정하세요.\n";

    printMenu();

    char command;

    while (true)
    {
        std::cout
            << "\n현재 위치: "
            << actuator.getCurrentPosition()
            << " step\n"
            << "명령 입력: ";

        std::cin >> command;

        bool moveResult = true;

        switch (command)
        {
            // 정방향 이동
            case '1':
                moveResult = actuator.moveSteps(1000);
                break;

            case '2':
                moveResult = actuator.moveSteps(100);
                break;

            case '3':
                moveResult = actuator.moveSteps(10);
                break;

            case '4':
                moveResult = actuator.moveSteps(1);
                break;

            // 역방향 이동
            case '5':
                moveResult = actuator.moveSteps(-1000);
                break;

            case '6':
                moveResult = actuator.moveSteps(-100);
                break;

            case '7':
                moveResult = actuator.moveSteps(-10);
                break;

            case '8':
                moveResult = actuator.moveSteps(-1);
                break;

            /*
             * 현재 위치를 원점(0 Step)으로 다시 설정한다.
             *
             * 리미트 스위치가 없는 환경에서
             * 기준 위치를 지정할 때 사용한다.
             */
            case 'z':
                actuator.setCurrentPosition(0);

                std::cout
                    << "현재 위치를 0 step으로 지정했습니다.\n";
                break;

            /*
             * 현재 위치를 중심 위치로 사용하기 위해
             * Step 값을 출력한다.
             */
            case 'c':
                std::cout
                    << "\n================================\n"
                    << "CENTER_STEP = "
                    << actuator.getCurrentPosition()
                    << "\n"
                    << "================================\n";
                break;

            // 현재 논리적 위치 출력
            case 'p':
                std::cout
                    << "현재 위치: "
                    << actuator.getCurrentPosition()
                    << " step\n";
                break;

            // 프로그램 종료
            case 'q':
                actuator.disable();

                std::cout
                    << "\n최종 위치: "
                    << actuator.getCurrentPosition()
                    << " step\n"
                    << "프로그램을 종료합니다.\n";

                return 0;

            default:
                std::cout << "잘못된 명령입니다.\n";
                printMenu();
                continue;
        }

        // 모터 이동 중 오류가 발생하면 프로그램 종료
        if (!moveResult)
        {
            std::cerr
                << "모터 이동 중 오류가 발생했습니다.\n";
            return 1;
        }
    }
}