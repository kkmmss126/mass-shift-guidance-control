#include <pigpio.h>

#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <algorithm>

// ============================================================
// Target Mover Calibration
// CoreXY 방식 표적 이동장치 X/Y 한계 측정용
// ============================================================
//
// 사용 방법
//
// 1. 프로그램 실행 전에 표적을 프레임 중앙에 수동으로 위치
// 2. 실행 시 현재 위치를 X=0, Y=0으로 선언
// 3. A/D : X축 좌우 이동
// 4. W/S : Y축 상하 이동
// 5. 실제 프레임 끝에 가까워지면 현재 좌표를 기록
// 6. Q : 종료
//
// IMPORTANT
// - 처음에는 STEP_SIZE를 작게 설정할 것
// - 기계 끝까지 밀지 말고 안전 여유를 남길 것
// - 방향이 반대라면 DIR_INVERT만 수정
//
// ============================================================


// ============================================================
// USER SETTINGS
// ============================================================

// ------------------------------------------------------------
// GPIO PIN
// 실제 표적 이동장치 결선에 맞게 수정
// ------------------------------------------------------------

constexpr int MOTOR_A_STEP_PIN = 5;
constexpr int MOTOR_A_DIR_PIN  = 6;

constexpr int MOTOR_B_STEP_PIN = 13;
constexpr int MOTOR_B_DIR_PIN  = 16;


// ------------------------------------------------------------
// 모터 방향 반전
//
// 특정 모터 방향이 예상과 반대면 true로 변경
// ------------------------------------------------------------

constexpr bool MOTOR_A_DIR_INVERT = false;
constexpr bool MOTOR_B_DIR_INVERT = false;


// ------------------------------------------------------------
// 키 1회 입력당 이동량
//
// 처음에는 20~50 step 정도 권장
//
// 너무 크면 프레임 끝에서 충돌 위험 있음
// ------------------------------------------------------------

constexpr int STEP_SIZE = 50;


// ------------------------------------------------------------
// STEP 속도
//
// 값이 작을수록 빠름
//
// 초기 캘리브레이션은 느리게 하는 게 안전
// ------------------------------------------------------------

constexpr unsigned STEP_INTERVAL_US = 1200;


// ------------------------------------------------------------
// STEP HIGH 유지시간
// ------------------------------------------------------------

constexpr unsigned STEP_PULSE_US = 5;


// ============================================================
// USER SETTINGS END
// ============================================================



int currentX = 0;
int currentY = 0;



// ============================================================
// Terminal raw mode
// ============================================================

char getKey()
{
    termios oldt{};
    termios newt{};

    tcgetattr(STDIN_FILENO, &oldt);

    newt = oldt;

    newt.c_lflag &= ~(ICANON | ECHO);

    tcsetattr(
        STDIN_FILENO,
        TCSANOW,
        &newt
    );


    char ch = static_cast<char>(getchar());


    tcsetattr(
        STDIN_FILENO,
        TCSANOW,
        &oldt
    );


    return ch;
}



// ============================================================
// Motor Direction
// ============================================================

void setMotorADirection(bool positive)
{
    bool dir = positive;

    if (MOTOR_A_DIR_INVERT)
        dir = !dir;

    gpioWrite(
        MOTOR_A_DIR_PIN,
        dir ? 1 : 0
    );
}


void setMotorBDirection(bool positive)
{
    bool dir = positive;

    if (MOTOR_B_DIR_INVERT)
        dir = !dir;

    gpioWrite(
        MOTOR_B_DIR_PIN,
        dir ? 1 : 0
    );
}



// ============================================================
// CoreXY Motor Step
// ============================================================

void moveMotorSteps(
    int stepA,
    int stepB
)
{
    if (stepA == 0 && stepB == 0)
        return;


    setMotorADirection(stepA >= 0);
    setMotorBDirection(stepB >= 0);


    int absA = std::abs(stepA);
    int absB = std::abs(stepB);

    int totalSteps =
        std::max(absA, absB);


    int accumulatorA = 0;
    int accumulatorB = 0;


    for (int i = 0; i < totalSteps; ++i)
    {
        accumulatorA += absA;
        accumulatorB += absB;


        bool pulseA = false;
        bool pulseB = false;


        if (accumulatorA >= totalSteps)
        {
            accumulatorA -= totalSteps;
            pulseA = true;
        }


        if (accumulatorB >= totalSteps)
        {
            accumulatorB -= totalSteps;
            pulseB = true;
        }


        if (pulseA)
            gpioWrite(
                MOTOR_A_STEP_PIN,
                1
            );

        if (pulseB)
            gpioWrite(
                MOTOR_B_STEP_PIN,
                1
            );


        gpioDelay(
            STEP_PULSE_US
        );


        if (pulseA)
            gpioWrite(
                MOTOR_A_STEP_PIN,
                0
            );

        if (pulseB)
            gpioWrite(
                MOTOR_B_STEP_PIN,
                0
            );


        gpioDelay(
            STEP_INTERVAL_US
        );
    }
}



// ============================================================
// X/Y 이동
// ============================================================
//
// 논리 좌표 X/Y를 CoreXY Motor A/B로 변환
//
// A = dX + dY
// B = dX - dY
//
// ============================================================

void moveXY(
    int dx,
    int dy
)
{
    int stepA =
        dx + dy;

    int stepB =
        dx - dy;


    moveMotorSteps(
        stepA,
        stepB
    );


    currentX += dx;
    currentY += dy;
}



// ============================================================
// 위치 출력
// ============================================================

void printPosition()
{
    std::cout
        << "\r"
        << "X = "
        << currentX
        << " step"
        << "    "
        << "Y = "
        << currentY
        << " step"
        << "                "
        << std::flush;
}



// ============================================================
// Main
// ============================================================

int main()
{
    if (gpioInitialise() < 0)
    {
        std::cerr
            << "[ERROR] pigpio initialization failed\n";

        return 1;
    }


    gpioSetMode(
        MOTOR_A_STEP_PIN,
        PI_OUTPUT
    );

    gpioSetMode(
        MOTOR_A_DIR_PIN,
        PI_OUTPUT
    );

    gpioSetMode(
        MOTOR_B_STEP_PIN,
        PI_OUTPUT
    );

    gpioSetMode(
        MOTOR_B_DIR_PIN,
        PI_OUTPUT
    );


    gpioWrite(
        MOTOR_A_STEP_PIN,
        0
    );

    gpioWrite(
        MOTOR_B_STEP_PIN,
        0
    );


    std::cout << "\n";
    std::cout << "========================================\n";
    std::cout << " Target Mover Calibration\n";
    std::cout << "========================================\n";
    std::cout << "\n";

    std::cout << "IMPORTANT\n";
    std::cout << "Start with target positioned at CENTER.\n";
    std::cout << "Current center = X 0 / Y 0\n\n";

    std::cout << "Controls\n";
    std::cout << "----------------------------------------\n";

    std::cout
        << "W : +Y  (UP)\n";

    std::cout
        << "S : -Y  (DOWN)\n";

    std::cout
        << "A : -X  (LEFT)\n";

    std::cout
        << "D : +X  (RIGHT)\n";

    std::cout
        << "Q : Quit\n";

    std::cout << "----------------------------------------\n";

    std::cout
        << "STEP SIZE = "
        << STEP_SIZE
        << "\n\n";


    printPosition();


    bool running = true;


    while (running)
    {
        char key = getKey();


        switch (key)
        {
            // -------------------------
            // 위
            // -------------------------

            case 'w':
            case 'W':

                moveXY(
                    0,
                    STEP_SIZE
                );

                break;


            // -------------------------
            // 아래
            // -------------------------

            case 's':
            case 'S':

                moveXY(
                    0,
                    -STEP_SIZE
                );

                break;


            // -------------------------
            // 왼쪽
            // -------------------------

            case 'a':
            case 'A':

                moveXY(
                    -STEP_SIZE,
                    0
                );

                break;


            // -------------------------
            // 오른쪽
            // -------------------------

            case 'd':
            case 'D':

                moveXY(
                    STEP_SIZE,
                    0
                );

                break;


            // -------------------------
            // 종료
            // -------------------------

            case 'q':
            case 'Q':

                running = false;

                break;


            default:
                break;
        }


        if (running)
            printPosition();
    }


    gpioWrite(
        MOTOR_A_STEP_PIN,
        0
    );

    gpioWrite(
        MOTOR_B_STEP_PIN,
        0
    );


    gpioTerminate();


    std::cout << "\n\n";

    std::cout << "========================================\n";
    std::cout << " Calibration Finished\n";
    std::cout << "========================================\n";

    std::cout
        << "Final X = "
        << currentX
        << " step\n";

    std::cout
        << "Final Y = "
        << currentY
        << " step\n";


    return 0;
}