#include <pigpio.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>

// ============================================================
// Target Moving Device - CoreXY Control
// ============================================================
//
// 목적
//  1. 프로그램 시작 전 표적을 프레임 중앙에 수동 배치
//  2. 중앙 위치를 X=0, Y=0 으로 선언
//  3. 미리 설정한 코스에 따라 X/Y 목표좌표를 연속 생성
//  4. X/Y 좌표를 CoreXY Motor A/B 스텝으로 변환
//  5. 8자 / 타원 / Z 코스를 반복 실행
//
// CoreXY 기본 변환
//
//      Motor A = dX + dY
//      Motor B = dX - dY
//
// 실제 모터 장착 방향에 따라 DIR 방향은 반대일 수 있으므로
// MOTOR_A_DIR_INVERT / MOTOR_B_DIR_INVERT 값만 변경하면 된다.
//
// ============================================================



// ============================================================
//                   USER SETTINGS
// ============================================================
//
// 이 부분이 실제 실험할 때 주로 수정하는 영역이다.
// 아래 설정 외의 제어 로직은 가능하면 수정하지 않는다.
// ============================================================


// ------------------------------------------------------------
// [1] GPIO PIN 설정
// ------------------------------------------------------------
//
// BCM GPIO 번호 기준.
//
// !!! 아래 번호는 예시값 !!!
// 실제 표적 이동 장치 결선 후 반드시 실제 GPIO 번호로 수정할 것.
//
constexpr int MOTOR_A_STEP_PIN = 5;
constexpr int MOTOR_A_DIR_PIN  = 6;

constexpr int MOTOR_B_STEP_PIN = 13;
constexpr int MOTOR_B_DIR_PIN  = 16;


// ------------------------------------------------------------
// [2] 모터 회전 방향 보정
// ------------------------------------------------------------
//
// CoreXY를 실제 조립하면 모터 설치 방향 때문에
// 코드에서 생각한 +방향과 실제 방향이 반대로 움직일 수 있다.
//
// 해당 모터 방향이 반대라면 false -> true 로 변경.
//
// 예)
// Motor A만 반대라면
//
// MOTOR_A_DIR_INVERT = true;
// MOTOR_B_DIR_INVERT = false;
//
constexpr bool MOTOR_A_DIR_INVERT = false;
constexpr bool MOTOR_B_DIR_INVERT = false;


// ------------------------------------------------------------
// [3] X / Y 이동 한계
// ------------------------------------------------------------
//
// 표적을 중앙에 놓고 X=0, Y=0으로 설정한 후,
// 실제 시험을 통해 최대 이동 가능한 스텝을 측정하여 입력한다.
//
// 예)
// 중앙에서 왼쪽  -2200 step
// 중앙에서 오른쪽 +2300 step
//
// X_MIN_STEPS = -2200
// X_MAX_STEPS = +2300
//
// 기계적 끝까지 사용하는 것보다 충돌 방지를 위해
// 실제 최대값보다 약간 안쪽 값을 사용하는 것을 권장.
//
// !!! 현재 숫자는 임시값 !!!
//
constexpr int X_MIN_STEPS = -2000;
constexpr int X_MAX_STEPS =  2000;

constexpr int Y_MIN_STEPS = -1200;
constexpr int Y_MAX_STEPS =  1200;


// ------------------------------------------------------------
// [4] 실제 코스에서 사용할 이동 범위
// ------------------------------------------------------------
//
// 1.0 = 설정된 최대 이동범위의 100% 사용
// 0.8 = 설정된 최대 이동범위의 80% 사용
// 0.5 = 설정된 최대 이동범위의 50% 사용
//
// 프레임 충돌 및 줄 장력 문제 때문에 처음에는
// 0.6 ~ 0.8 정도를 권장.
//
// 예)
// X_MAX = 2000
// X_RANGE_RATIO = 0.8
//
// 실제 코스에서 약 ±1600 step 범위를 사용.
//
constexpr double X_RANGE_RATIO = 0.80;
constexpr double Y_RANGE_RATIO = 0.75;


// ------------------------------------------------------------
// [5] 이동 코스 선택
// ------------------------------------------------------------
//
// 사용 가능한 코스
//
// FIGURE_EIGHT : 8자 운동
//                좌/우 + 상/하 + 대각 방향전환 포함
//                종합 추적 성능 시험용
//
// ELLIPSE      : 타원 운동
//                가장 부드럽고 연속적인 운동
//                안정적인 추적 성능 시험용
//
// Z_PATTERN    : Z자 왕복 운동
//                직선 + 대각선 + 방향전환 포함
//                급격한 방향 변화 대응 시험용
//
enum class MotionCourse
{
    FIGURE_EIGHT,
    ELLIPSE,
    Z_PATTERN
};


// ------------------------------------------------------------
// ★ 여기서 원하는 코스 선택
// ------------------------------------------------------------

constexpr MotionCourse SELECTED_COURSE =
    MotionCourse::FIGURE_EIGHT;


// ------------------------------------------------------------
// [6] 코스 1회 진행 시간
// ------------------------------------------------------------
//
// 단위 : 초
//
// 값이 작을수록 표적이 빨리 움직이고,
// 값이 클수록 천천히 움직인다.
//
// 예)
// 12.0 → 느림
//  8.0 → 보통
//  5.0 → 빠름
//
// 처음 시험할 때는 8~12초 정도 권장.
//
constexpr double COURSE_PERIOD_SEC = 10.0;


// ------------------------------------------------------------
// [7] 코스 반복 여부
// ------------------------------------------------------------
//
// true
//   → 코스를 계속 반복
//
// false
//   → 코스를 한 번 실행한 후 종료
//
constexpr bool REPEAT_COURSE = true;


// ------------------------------------------------------------
// [8] 시작 대기시간
// ------------------------------------------------------------
//
// 프로그램 실행 직후 모터가 바로 움직이지 않도록 하는 시간.
//
// 표적 확인 / 손 제거 / 시커 준비 등을 위한 시간.
//
// 단위 : ms
//
constexpr int START_DELAY_MS = 3000;


// ------------------------------------------------------------
// [9] 제어 갱신 주기
// ------------------------------------------------------------
//
// 몇 ms마다 새로운 목표 X/Y 좌표를 계산할지 결정.
//
// 값이 작을수록 곡선이 더 부드러워지지만
// CPU 및 STEP 명령량은 증가.
//
// 10~20 ms 권장.
//
constexpr int CONTROL_PERIOD_MS = 10;


// ------------------------------------------------------------
// [10] STEP 펄스 관련 설정
// ------------------------------------------------------------
//
// A4988 STEP 신호 HIGH 유지시간.
//
// 일반적으로 몇 us 정도면 충분.
//
constexpr unsigned STEP_PULSE_US = 5;


// CoreXY에서 다음 스텝 출력까지의 간격.
//
// 값이 작을수록 모터가 더 빠르게 움직인다.
//
// 처음에는 너무 빠르게 설정하지 말고
// 500~1500 us 정도에서 시험 후 조절.
//
constexpr unsigned STEP_INTERVAL_US = 800;


// ============================================================
//          USER SETTINGS END
// ============================================================



constexpr double PI = 3.14159265358979323846;


// 종료 플래그
volatile std::sig_atomic_t running = 1;


// 현재 논리적 X/Y 위치
//
// 프로그램 시작 시 표적을 중앙에 두기 때문에
// 반드시 0,0에서 시작한다.
//
int currentX = 0;
int currentY = 0;



// ============================================================
// Signal Handler
// ============================================================

void signalHandler(int)
{
    running = 0;
}



// ============================================================
// Utility
// ============================================================

int clampValue(int value, int minValue, int maxValue)
{
    return std::max(minValue, std::min(value, maxValue));
}


// smoothstep
//
// Z 코스에서 구간 시작/끝의 움직임을 조금 부드럽게 만든다.
//
double smoothStep(double t)
{
    t = std::clamp(t, 0.0, 1.0);

    return t * t * (3.0 - 2.0 * t);
}



// ============================================================
// Motor Direction
// ============================================================

void setMotorADirection(bool positive)
{
    bool dir = positive;

    if (MOTOR_A_DIR_INVERT)
        dir = !dir;

    gpioWrite(MOTOR_A_DIR_PIN, dir ? 1 : 0);
}


void setMotorBDirection(bool positive)
{
    bool dir = positive;

    if (MOTOR_B_DIR_INVERT)
        dir = !dir;

    gpioWrite(MOTOR_B_DIR_PIN, dir ? 1 : 0);
}



// ============================================================
// CoreXY synchronized step control
// ============================================================
//
// stepA / stepB 만큼 두 모터를 가능한 한 동시에 움직인다.
//
// 두 모터의 스텝 수가 서로 다른 경우 DDA 방식으로
// 두 모터의 스텝 비율을 자동 분배한다.
//
// 예)
//
// Motor A = 1000 step
// Motor B = 500 step
//
// → A가 약 2 step 움직일 때 B가 1 step 움직이는 식으로
//   동기화된다.
//
// ============================================================

void moveMotorSteps(int stepA, int stepB)
{
    if (stepA == 0 && stepB == 0)
        return;


    setMotorADirection(stepA >= 0);
    setMotorBDirection(stepB >= 0);


    int absA = std::abs(stepA);
    int absB = std::abs(stepB);


    int totalSteps = std::max(absA, absB);

    if (totalSteps == 0)
        return;


    int accumulatorA = 0;
    int accumulatorB = 0;


    for (int i = 0; i < totalSteps && running; ++i)
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


        // 두 모터가 동시에 STEP이 필요한 경우
        // 동시에 HIGH로 만든다.

        if (pulseA)
            gpioWrite(MOTOR_A_STEP_PIN, 1);

        if (pulseB)
            gpioWrite(MOTOR_B_STEP_PIN, 1);


        gpioDelay(STEP_PULSE_US);


        if (pulseA)
            gpioWrite(MOTOR_A_STEP_PIN, 0);

        if (pulseB)
            gpioWrite(MOTOR_B_STEP_PIN, 0);


        gpioDelay(STEP_INTERVAL_US);
    }
}



// ============================================================
// Target X/Y → CoreXY motor conversion
// ============================================================

void updateTargetPosition(int targetX, int targetY)
{
    // -------------------------------
    // 안전범위 제한
    // -------------------------------

    targetX = clampValue(
        targetX,
        X_MIN_STEPS,
        X_MAX_STEPS
    );

    targetY = clampValue(
        targetY,
        Y_MIN_STEPS,
        Y_MAX_STEPS
    );


    // 현재 위치로부터 이동해야 할 양

    int dx = targetX - currentX;
    int dy = targetY - currentY;


    if (dx == 0 && dy == 0)
        return;


    // -------------------------------
    // CoreXY 좌표 변환
    // -------------------------------

    int motorASteps = dx + dy;
    int motorBSteps = dx - dy;


    moveMotorSteps(
        motorASteps,
        motorBSteps
    );


    // 현재 논리적 위치 갱신

    currentX = targetX;
    currentY = targetY;
}



// ============================================================
// FIGURE EIGHT
// ============================================================
//
// 8자 궤적
//
// X = A sin(t)
// Y = B sin(2t)
//
// 좌/우 이동뿐 아니라
// 상/하 및 대각 방향 전환이 모두 발생.
//
// 종합 추적 시험용으로 사용할 예정.
//
// ============================================================

void generateFigureEight(
    double phase,
    double xRange,
    double yRange,
    int& x,
    int& y)
{
    x = static_cast<int>(
        xRange * std::sin(phase)
    );

    y = static_cast<int>(
        yRange * std::sin(2.0 * phase)
    );
}



// ============================================================
// ELLIPSE
// ============================================================
//
// 타원 운동.
//
// 방향전환이 매우 부드러워
// 연속 추적 안정성을 확인하기 좋다.
//
// ============================================================

void generateEllipse(
    double phase,
    double xRange,
    double yRange,
    int& x,
    int& y)
{
    x = static_cast<int>(
        xRange * std::cos(phase)
    );

    y = static_cast<int>(
        yRange * std::sin(phase)
    );
}



// ============================================================
// Z PATTERN
// ============================================================
//
// 한 사이클:
//
//  LEFT_TOP
//      ↓
//  RIGHT_TOP
//      ↓
//  LEFT_BOTTOM
//      ↓
//  RIGHT_BOTTOM
//
// 이후 역방향으로 돌아오는 형태.
//
// 각 구간은 smoothStep을 이용해
// 시작/끝의 급격한 충격을 완화한다.
//
// ============================================================

void generateZPattern(
    double progress,
    double xRange,
    double yRange,
    int& x,
    int& y)
{
    // progress : 0.0 ~ 1.0

    // 왕복을 위해 전체를 6구간으로 나눈다.

    constexpr int SEGMENTS = 6;

    double scaled = progress * SEGMENTS;

    int segment =
        static_cast<int>(scaled);

    if (segment >= SEGMENTS)
        segment = SEGMENTS - 1;


    double localT =
        scaled - std::floor(scaled);

    localT = smoothStep(localT);


    // Z자 제어점

    const double px[] =
    {
        -xRange,
         xRange,
        -xRange,
         xRange,
        -xRange,
         xRange,
        -xRange
    };


    const double py[] =
    {
         yRange,
         yRange,
        -yRange,
        -yRange,
        -yRange,
         yRange,
         yRange
    };


    double startX = px[segment];
    double startY = py[segment];

    double endX = px[segment + 1];
    double endY = py[segment + 1];


    x = static_cast<int>(
        startX +
        (endX - startX) * localT
    );


    y = static_cast<int>(
        startY +
        (endY - startY) * localT
    );
}



// ============================================================
// Course Generator
// ============================================================

void generateTargetPosition(
    MotionCourse course,
    double progress,
    double xRange,
    double yRange,
    int& targetX,
    int& targetY)
{
    // progress
    //
    // 0.0 → 코스 시작
    // 1.0 → 코스 1회 종료


    double phase =
        progress * 2.0 * PI;


    switch (course)
    {
        case MotionCourse::FIGURE_EIGHT:

            generateFigureEight(
                phase,
                xRange,
                yRange,
                targetX,
                targetY
            );

            break;


        case MotionCourse::ELLIPSE:

            generateEllipse(
                phase,
                xRange,
                yRange,
                targetX,
                targetY
            );

            break;


        case MotionCourse::Z_PATTERN:

            generateZPattern(
                progress,
                xRange,
                yRange,
                targetX,
                targetY
            );

            break;
    }
}



// ============================================================
// Main
// ============================================================

int main()
{
    std::signal(SIGINT, signalHandler);


    // pigpio 초기화

    if (gpioInitialise() < 0)
    {
        std::cerr
            << "[ERROR] pigpio initialization failed\n";

        return 1;
    }


    // GPIO 출력 설정

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


    gpioWrite(MOTOR_A_STEP_PIN, 0);
    gpioWrite(MOTOR_B_STEP_PIN, 0);


    // --------------------------------------------------------
    // 사용할 실제 X/Y 이동범위 계산
    // --------------------------------------------------------

    double xNegativeRange =
        std::abs(X_MIN_STEPS);

    double xPositiveRange =
        std::abs(X_MAX_STEPS);

    double yNegativeRange =
        std::abs(Y_MIN_STEPS);

    double yPositiveRange =
        std::abs(Y_MAX_STEPS);


    // 좌우 / 상하 중 더 작은 한계를 기준으로 사용한다.
    //
    // 예)
    //
    // 왼쪽  = 2000
    // 오른쪽 = 2200
    //
    // → 안전하게 ±2000 기준 사용

    double xRange =
        std::min(
            xNegativeRange,
            xPositiveRange
        ) * X_RANGE_RATIO;


    double yRange =
        std::min(
            yNegativeRange,
            yPositiveRange
        ) * Y_RANGE_RATIO;


    std::cout << "\n";
    std::cout << "=====================================\n";
    std::cout << " Target Moving Device\n";
    std::cout << "=====================================\n";

    std::cout
        << "Center position : X=0, Y=0\n";

    std::cout
        << "X range : +/- "
        << xRange
        << " step\n";

    std::cout
        << "Y range : +/- "
        << yRange
        << " step\n";

    std::cout
        << "Course period : "
        << COURSE_PERIOD_SEC
        << " sec\n";

    std::cout
        << "Start delay : "
        << START_DELAY_MS
        << " ms\n";

    std::cout
        << "Ctrl+C : Stop\n";

    std::cout << "=====================================\n\n";


    // --------------------------------------------------------
    // 시작 전 대기
    // --------------------------------------------------------

    std::this_thread::sleep_for(
        std::chrono::milliseconds(
            START_DELAY_MS
        )
    );


    // 프로그램 시작 시간을 저장

    auto courseStart =
        std::chrono::steady_clock::now();


    while (running)
    {
        auto now =
            std::chrono::steady_clock::now();


        double elapsed =
            std::chrono::duration<double>(
                now - courseStart
            ).count();


        // 현재 코스 진행도
        //
        // 0.0 → 시작
        // 1.0 → 한 바퀴 종료

        double progress =
            elapsed / COURSE_PERIOD_SEC;


        // 한 사이클 완료

        if (progress >= 1.0)
        {
            if (REPEAT_COURSE)
            {
                courseStart =
                    std::chrono::steady_clock::now();

                continue;
            }

            else
            {
                break;
            }
        }


        int targetX = 0;
        int targetY = 0;


        generateTargetPosition(
            SELECTED_COURSE,
            progress,
            xRange,
            yRange,
            targetX,
            targetY
        );


        updateTargetPosition(
            targetX,
            targetY
        );


        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                CONTROL_PERIOD_MS
            )
        );
    }


    // --------------------------------------------------------
    // 종료
    // --------------------------------------------------------

    gpioWrite(MOTOR_A_STEP_PIN, 0);
    gpioWrite(MOTOR_B_STEP_PIN, 0);


    gpioTerminate();


    std::cout
        << "\n[STOP] Target mover stopped.\n";


    std::cout
        << "Logical position : X="
        << currentX
        << ", Y="
        << currentY
        << "\n";


    return 0;
}