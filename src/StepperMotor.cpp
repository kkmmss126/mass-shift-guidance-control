/*
 * File: StepperMotor.cpp
 *
 * Description:
 * A4988 스텝모터 드라이버를 이용하여 스텝모터 또는
 * 리니어 스텝모터 액추에이터를 제어하는 클래스 구현 파일이다.
 *
 * libgpiod를 이용해 라즈베리파이 GPIO를 제어하며,
 * STEP 펄스 생성, 회전 방향 설정, 드라이버 활성화 및
 * 현재 위치값 관리를 수행한다.
 *
 * Project:
 * 질량 이동 기반 중간 및 종말 유도 모사 시스템
 *
 * 주요 기능:
 * 1. GPIO 칩 및 GPIO Line 초기화
 * 2. A4988 STEP/DIR/ENABLE 신호 제어
 * 3. 지정한 Step 수만큼 모터 이동
 * 4. 이동 명령을 기준으로 현재 위치 계산
 * 5. 프로그램 종료 시 GPIO 자원 해제
 */

#include "StepperMotor.h"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

/*
 * 사용할 GPIO 번호와 기본 상태를 저장한다.
 *
 * 생성자에서는 GPIO를 실제로 점유하지 않으며,
 * GPIO 초기화는 initialize() 함수에서 수행한다.
 */
StepperMotor::StepperMotor(
    int stepPin,
    int dirPin,
    int enablePin,
    const char* chipName
)
    : chipName_(chipName),
      stepPin_(stepPin),
      dirPin_(dirPin),
      enablePin_(enablePin),
      pulseDelayUs_(1000),
      currentPosition_(0),
      initialized_(false),
      chip_(nullptr),
      stepLine_(nullptr),
      dirLine_(nullptr),
      enableLine_(nullptr)
{
}

/*
 * 객체가 소멸될 때 모터 드라이버를 비활성화하고
 * 점유하고 있던 GPIO 자원을 해제한다.
 */
StepperMotor::~StepperMotor()
{
    disable();
    releaseGPIO();
}

/*
 * STEP, DIR, ENABLE GPIO를 출력 모드로 설정하고
 * A4988을 제어할 수 있는 상태로 초기화한다.
 */
bool StepperMotor::initialize()
{
    // 지정한 GPIO 칩을 연다. 라즈베리파이에서는 일반적으로 gpiochip0을 사용한다.
    chip_ = gpiod_chip_open_by_name(chipName_);

    if (chip_ == nullptr)
    {
        std::cerr << "GPIO chip open failed: "
                  << chipName_ << '\n';
        return false;
    }

    // GPIO 칩에서 STEP 및 DIR 핀에 해당하는 GPIO Line을 가져온다.
    stepLine_ = gpiod_chip_get_line(chip_, stepPin_);
    dirLine_ = gpiod_chip_get_line(chip_, dirPin_);

    if (stepLine_ == nullptr || dirLine_ == nullptr)
    {
        std::cerr << "STEP 또는 DIR GPIO line 획득 실패\n";
        releaseGPIO();
        return false;
    }

    // STEP 핀을 초기값 LOW인 출력 핀으로 설정한다.
    if (gpiod_line_request_output(
            stepLine_,
            "stepper-step",
            0) < 0)
    {
        std::cerr << "STEP GPIO 출력 설정 실패\n";
        releaseGPIO();
        return false;
    }

    // DIR 핀을 초기값 LOW인 출력 핀으로 설정한다.
    if (gpiod_line_request_output(
            dirLine_,
            "stepper-dir",
            0) < 0)
    {
        std::cerr << "DIR GPIO 출력 설정 실패\n";
        releaseGPIO();
        return false;
    }

    /*
     * enablePin이 -1이면 ENABLE 핀을 사용하지 않는다.
     * ENABLE 핀을 연결한 경우에만 GPIO를 초기화한다.
     */
    if (enablePin_ >= 0)
    {
        enableLine_ = gpiod_chip_get_line(chip_, enablePin_);

        if (enableLine_ == nullptr)
        {
            std::cerr << "ENABLE GPIO line 획득 실패\n";
            releaseGPIO();
            return false;
        }

        /*
         * A4988의 ENABLE 핀은 Active Low 방식이다.
         *
         * HIGH: 드라이버 비활성화
         * LOW : 드라이버 활성화
         *
         * 초기값을 HIGH로 설정하여 GPIO 초기화 중
         * 모터가 의도하지 않게 움직이는 것을 방지한다.
         */
        if (gpiod_line_request_output(
                enableLine_,
                "stepper-enable",
                1) < 0)
        {
            std::cerr << "ENABLE GPIO 출력 설정 실패\n";
            releaseGPIO();
            return false;
        }
    }

    initialized_ = true;

    // GPIO 설정이 완료된 뒤 A4988 드라이버를 활성화한다.
    enable();

    return true;
}

/*
 * 지정한 Step 수만큼 모터를 이동시킨다.
 *
 * 양수: DIR 핀을 HIGH로 설정하고 정방향 이동
 * 음수: DIR 핀을 LOW로 설정하고 역방향 이동
 *
 * A4988은 STEP 핀의 상승 에지마다 모터를 한 Step 이동시킨다.
 */
bool StepperMotor::moveSteps(int steps)
{
    if (!initialized_)
    {
        std::cerr << "StepperMotor가 초기화되지 않았습니다.\n";
        return false;
    }

    // 이동량이 0이면 GPIO 신호를 출력하지 않고 정상 종료한다.
    if (steps == 0)
    {
        return true;
    }

    const bool positiveDirection = steps > 0;
    const int stepCount = std::abs(steps);

    // 이동 방향에 따라 DIR 핀의 출력값을 설정한다.
    if (gpiod_line_set_value(
            dirLine_,
            positiveDirection ? 1 : 0) < 0)
    {
        std::cerr << "DIR GPIO 출력 실패\n";
        return false;
    }

    /*
     * DIR 신호가 변경된 직후 STEP 펄스를 출력하면
     * 드라이버가 방향 신호를 안정적으로 인식하지 못할 수 있다.
     */
    std::this_thread::sleep_for(
        std::chrono::microseconds(10)
    );

    /*
     * STEP 핀에 HIGH와 LOW를 반복 출력한다.
     *
     * LOW → HIGH 상승 에지가 한 번 발생할 때마다
     * A4988이 모터를 한 Step 이동시킨다.
     */
    for (int i = 0; i < stepCount; ++i)
    {
        if (gpiod_line_set_value(stepLine_, 1) < 0)
        {
            std::cerr << "STEP HIGH 출력 실패\n";
            return false;
        }

        std::this_thread::sleep_for(
            std::chrono::microseconds(pulseDelayUs_)
        );

        if (gpiod_line_set_value(stepLine_, 0) < 0)
        {
            std::cerr << "STEP LOW 출력 실패\n";
            return false;
        }

        std::this_thread::sleep_for(
            std::chrono::microseconds(pulseDelayUs_)
        );
    }

    /*
     * 센서나 엔코더로 측정한 실제 위치가 아니라,
     * 출력한 Step 수를 누적하여 논리적 현재 위치를 계산한다.
     */
    if (positiveDirection)
    {
        currentPosition_ += stepCount;
    }
    else
    {
        currentPosition_ -= stepCount;
    }

    return true;
}

/*
 * STEP 펄스의 HIGH 및 LOW 유지시간을 설정한다.
 *
 * 값이 작을수록 모터가 빠르게 움직이지만,
 * 너무 빠르면 탈조나 토크 부족이 발생할 수 있다.
 */
void StepperMotor::setPulseDelay(int microseconds)
{
    // 지나치게 짧은 펄스 설정을 방지하기 위한 최소 제한값이다.
    if (microseconds < 100)
    {
        std::cerr
            << "펄스 지연시간이 너무 짧습니다. "
            << "100 us로 설정합니다.\n";

        pulseDelayUs_ = 100;
        return;
    }

    pulseDelayUs_ = microseconds;
}

/*
 * A4988 드라이버를 활성화한다.
 *
 * ENABLE 핀은 Active Low이므로 LOW를 출력한다.
 */
void StepperMotor::enable()
{
    if (enableLine_ != nullptr)
    {
        gpiod_line_set_value(enableLine_, 0);
    }
}

/*
 * A4988 드라이버를 비활성화한다.
 *
 * ENABLE 핀에 HIGH를 출력하면 모터 전류가 차단되며,
 * 발열과 소비 전력을 줄일 수 있다.
 */
void StepperMotor::disable()
{
    if (enableLine_ != nullptr)
    {
        gpiod_line_set_value(enableLine_, 1);
    }
}

/*
 * 프로그램 내부에서 누적 계산한 현재 Step 위치를 반환한다.
 */
long StepperMotor::getCurrentPosition() const
{
    return currentPosition_;
}

/*
 * 현재 논리적 위치값을 지정한 값으로 변경한다.
 *
 * 원점 설정이나 중심 위치 보정 시 사용한다.
 */
void StepperMotor::setCurrentPosition(long position)
{
    currentPosition_ = position;
}

/*
 * GPIO 초기화 완료 여부를 반환한다.
 */
bool StepperMotor::isInitialized() const
{
    return initialized_;
}

/*
 * 점유하고 있던 GPIO Line과 GPIO 칩을 안전하게 해제한다.
 *
 * 초기화 중 오류가 발생하거나 객체가 소멸될 때 호출된다.
 */
void StepperMotor::releaseGPIO()
{
    initialized_ = false;

    /*
     * GPIO 칩을 닫기 전에 개별 GPIO Line을 먼저 해제한다.
     * ENABLE, DIR, STEP 순서는 기능상 큰 차이는 없지만,
     * 초기화 과정의 역순으로 정리한다.
     */
    if (enableLine_ != nullptr)
    {
        gpiod_line_release(enableLine_);
        enableLine_ = nullptr;
    }

    if (dirLine_ != nullptr)
    {
        gpiod_line_release(dirLine_);
        dirLine_ = nullptr;
    }

    if (stepLine_ != nullptr)
    {
        gpiod_line_release(stepLine_);
        stepLine_ = nullptr;
    }

    if (chip_ != nullptr)
    {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
    }
}