#include "StepperMotor.h"

#include <chrono>
#include <iostream>
#include <thread>

/*
 * ============================================================
 * File Name : body_diagonal_motion_test.cpp
 *
 * Purpose
 * ------------------------------------------------------------
 * Body Yaw + Mass Shift Pitch의 동시 구동을 확인하기 위한
 * 대각 이동 전용 테스트 프로그램.
 *
 * 이 파일은 main.cpp와 분리된 하드웨어 검증용 테스트 파일이다.
 *
 * Axis Definition
 * ------------------------------------------------------------
 * Body Yaw
 *   STEP = GPIO19
 *   DIR  = GPIO26
 *
 * Mass Shift Pitch
 *   STEP = GPIO17
 *   DIR  = GPIO27
 *
 * A4988 ENABLE
 *   GND 고정
 *   -> 코드에서 Enable GPIO 사용 안 함
 *
 * Test Sequence
 * ------------------------------------------------------------
 * 1. Right + Up
 * 2. 원점 복귀
 * 3. Left + Up
 * 4. 원점 복귀
 * 5. Right + Down
 * 6. 원점 복귀
 * 7. Left + Down
 * 8. 원점 복귀
 *
 * 주의
 * ------------------------------------------------------------
 * +step이 실제 Right / Up 방향인지 아직 확정되지 않았다면,
 * 아래 방향 상수만 +1 <-> -1로 변경한다.
 * ============================================================
 */


// ============================================================
// Hardware Pin
// ============================================================

constexpr int BODY_YAW_STEP_PIN = 19;
constexpr int BODY_YAW_DIR_PIN  = 26;

constexpr int MASS_SHIFT_STEP_PIN = 17;
constexpr int MASS_SHIFT_DIR_PIN  = 27;

/*
 * A4988 ENABLE은 GND 고정.
 *
 * 기존 StepperMotor 라이브러리에서
 * enablePin < 0일 때 Enable GPIO를 사용하지 않는 구조를 전제로 한다.
 */
constexpr int UNUSED_ENABLE_PIN = -1;


// ============================================================
// Test Parameter
// ============================================================

/*
 * 첫 시험은 작은 이동량부터 시작.
 * 기구물 여유 확인 후 300 -> 500 -> 800 등으로 증가 가능.
 */
constexpr int BODY_YAW_TEST_STEP   = 300;
constexpr int MASS_SHIFT_TEST_STEP = 300;


/*
 * 실제 방향이 반대라면 여기만 변경.
 *
 * +1 : +step을 Right / Up으로 사용
 * -1 : -step을 Right / Up으로 사용
 */
constexpr int BODY_YAW_RIGHT_SIGN = +1;
constexpr int MASS_SHIFT_UP_SIGN  = +1;


/*
 * 각 시험 위치를 눈으로 확인할 시간.
 */
constexpr int OBSERVE_DELAY_MS = 1200;


// ============================================================
// Utility
// ============================================================

void waitForObservation()
{
    std::this_thread::sleep_for(
        std::chrono::milliseconds(OBSERVE_DELAY_MS)
    );
}


// ============================================================
// Zero Return
// ============================================================

bool returnToZero(
    StepperMotor& bodyYaw,
    StepperMotor& massShift)
{
    const long yawPosition =
        bodyYaw.getCurrentPosition();

    const long massPosition =
        massShift.getCurrentPosition();


    std::cout
        << "\n[RETURN TO ZERO]\n"
        << "Body Yaw   : "
        << yawPosition
        << " -> 0 step\n"
        << "Mass Shift : "
        << massPosition
        << " -> 0 step\n";


    bool yawResult = true;
    bool massResult = true;


    /*
     * 두 축을 동시에 원점으로 복귀시킨다.
     */
    std::thread yawThread(
        [&]()
        {
            if (yawPosition != 0)
            {
                yawResult =
                    bodyYaw.moveSteps(
                        -static_cast<int>(yawPosition)
                    );
            }
        }
    );


    std::thread massThread(
        [&]()
        {
            if (massPosition != 0)
            {
                massResult =
                    massShift.moveSteps(
                        -static_cast<int>(massPosition)
                    );
            }
        }
    );


    yawThread.join();
    massThread.join();


    std::cout
        << "Return Result\n"
        << "Body Yaw   : "
        << bodyYaw.getCurrentPosition()
        << " step\n"
        << "Mass Shift : "
        << massShift.getCurrentPosition()
        << " step\n";


    return yawResult && massResult;
}


// ============================================================
// Diagonal Movement
// ============================================================

bool moveDiagonal(
    StepperMotor& bodyYaw,
    StepperMotor& massShift,
    int yawSteps,
    int massSteps,
    const char* testName)
{
    std::cout
        << "\n==================================================\n"
        << "[DIAGONAL TEST] "
        << testName
        << '\n'
        << "==================================================\n"
        << "Body Yaw Command   : "
        << yawSteps
        << " step\n"
        << "Mass Shift Command : "
        << massSteps
        << " step\n";


    bool yawResult = false;
    bool massResult = false;


    /*
     * 기존 StepperMotor::moveSteps()를 그대로 사용하되,
     * 두 축을 각각 별도 std::thread에서 실행하여
     * 동시에 움직이게 한다.
     */
    std::thread yawThread(
        [&]()
        {
            yawResult =
                bodyYaw.moveSteps(yawSteps);
        }
    );


    std::thread massThread(
        [&]()
        {
            massResult =
                massShift.moveSteps(massSteps);
        }
    );


    yawThread.join();
    massThread.join();


    if (!yawResult || !massResult)
    {
        std::cerr
            << "[ERROR] 대각 이동 실패\n";

        return false;
    }


    std::cout
        << "Movement Complete\n"
        << "Body Yaw Position   : "
        << bodyYaw.getCurrentPosition()
        << " step\n"
        << "Mass Shift Position : "
        << massShift.getCurrentPosition()
        << " step\n";


    return true;
}


// ============================================================
// Main
// ============================================================

int main()
{
    std::cout
        << "==================================================\n"
        << "       Body Diagonal Motion Test\n"
        << "==================================================\n";


    StepperMotor bodyYaw(
        BODY_YAW_STEP_PIN,
        BODY_YAW_DIR_PIN,
        UNUSED_ENABLE_PIN
    );


    StepperMotor massShift(
        MASS_SHIFT_STEP_PIN,
        MASS_SHIFT_DIR_PIN,
        UNUSED_ENABLE_PIN
    );


    // --------------------------------------------------------
    // Initialization
    // --------------------------------------------------------

    if (!bodyYaw.initialize())
    {
        std::cerr
            << "[ERROR] Body Yaw Stepper 초기화 실패\n";

        return 1;
    }


    if (!massShift.initialize())
    {
        std::cerr
            << "[ERROR] Mass Shift Stepper 초기화 실패\n";

        return 1;
    }


    /*
     * 테스트 시작 당시 실제 위치를 프로그램상의 0점으로 정의.
     *
     * 반드시 테스트 시작 전에
     * 동체와 질량이동 장치를 원하는 기준 위치에 맞춰둘 것.
     */
    bodyYaw.setCurrentPosition(0);
    massShift.setCurrentPosition(0);


    std::cout
        << "[INIT] Complete\n"
        << "Body Yaw   : GPIO19 STEP / GPIO26 DIR\n"
        << "Mass Shift : GPIO17 STEP / GPIO27 DIR\n"
        << "Current Position = 0 step\n";


    const int rightStep =
        BODY_YAW_TEST_STEP *
        BODY_YAW_RIGHT_SIGN;

    const int leftStep =
        -rightStep;

    const int upStep =
        MASS_SHIFT_TEST_STEP *
        MASS_SHIFT_UP_SIGN;

    const int downStep =
        -upStep;


    // --------------------------------------------------------
    // 1. Right + Up
    // --------------------------------------------------------

    if (!moveDiagonal(
            bodyYaw,
            massShift,
            rightStep,
            upStep,
            "RIGHT + UP"
        ))
    {
        returnToZero(bodyYaw, massShift);
        return 1;
    }

    waitForObservation();

    if (!returnToZero(bodyYaw, massShift))
    {
        return 1;
    }

    waitForObservation();


    // --------------------------------------------------------
    // 2. Left + Up
    // --------------------------------------------------------

    if (!moveDiagonal(
            bodyYaw,
            massShift,
            leftStep,
            upStep,
            "LEFT + UP"
        ))
    {
        returnToZero(bodyYaw, massShift);
        return 1;
    }

    waitForObservation();

    if (!returnToZero(bodyYaw, massShift))
    {
        return 1;
    }

    waitForObservation();


    // --------------------------------------------------------
    // 3. Right + Down
    // --------------------------------------------------------

    if (!moveDiagonal(
            bodyYaw,
            massShift,
            rightStep,
            downStep,
            "RIGHT + DOWN"
        ))
    {
        returnToZero(bodyYaw, massShift);
        return 1;
    }

    waitForObservation();

    if (!returnToZero(bodyYaw, massShift))
    {
        return 1;
    }

    waitForObservation();


    // --------------------------------------------------------
    // 4. Left + Down
    // --------------------------------------------------------

    if (!moveDiagonal(
            bodyYaw,
            massShift,
            leftStep,
            downStep,
            "LEFT + DOWN"
        ))
    {
        returnToZero(bodyYaw, massShift);
        return 1;
    }

    waitForObservation();

    if (!returnToZero(bodyYaw, massShift))
    {
        return 1;
    }


    std::cout
        << "\n==================================================\n"
        << "          Diagonal Test Complete\n"
        << "==================================================\n"
        << "Final Body Yaw   : "
        << bodyYaw.getCurrentPosition()
        << " step\n"
        << "Final Mass Shift : "
        << massShift.getCurrentPosition()
        << " step\n";


    return 0;
}
