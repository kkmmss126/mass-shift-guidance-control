/*
 * File: StepperMotor.cpp
 *
 * Description:
 * A4988 스텝모터 드라이버를 이용하여 스텝모터 또는
 * 리니어 스텝모터 액추에이터를 제어하는 클래스 구현 파일이다.
 *
 * libgpiod 2.x API를 사용하여 Raspberry Pi GPIO를 제어한다.
 *
 * 주요 기능:
 * 1. STEP, DIR, ENABLE GPIO 출력 설정
 * 2. STEP 펄스 생성
 * 3. 이동 방향 설정
 * 4. A4988 활성화 및 비활성화
 * 5. 논리적 현재 위치 관리
 */

#include "StepperMotor.h"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <thread>

/*
 * 생성자
 *
 * GPIO 번호와 기본 설정값만 저장한다.
 * 실제 GPIO 점유는 initialize() 함수에서 수행한다.
 */
StepperMotor::StepperMotor(
    int stepPin,
    int dirPin,
    int enablePin,
    const char* chipPath
)
    : chipPath_(chipPath),
      stepPin_(stepPin),
      dirPin_(dirPin),
      enablePin_(enablePin),
      pulseDelayUs_(1000),
      currentPosition_(0),
      initialized_(false),
      chip_(nullptr),
      request_(nullptr),
      offsets_{0, 0, 0},
      offsetCount_(0)
{
}

/*
 * 소멸자
 *
 * 객체가 제거될 때 드라이버를 비활성화하고
 * GPIO 요청 및 GPIO 칩 자원을 해제한다.
 */
StepperMotor::~StepperMotor()
{
    disable();
    releaseGPIO();
}

/*
 * GPIO 초기화
 *
 * libgpiod 2.x에서는 개별 gpiod_line 객체를 가져오는 대신,
 * 사용할 GPIO offset들을 하나의 line_request로 묶어서 요청한다.
 */
bool StepperMotor::initialize()
{
    /*
     * 이미 초기화된 경우 중복 요청을 방지한다.
     */
    if (initialized_)
    {
        return true;
    }

    /*
     * GPIO 칩 장치 파일을 연다.
     *
     * 일반적인 Raspberry Pi 4에서는:
     * /dev/gpiochip0
     */
    chip_ = gpiod_chip_open(chipPath_);

    if (chip_ == nullptr)
    {
        std::cerr
            << "GPIO chip open failed: "
            << chipPath_
            << '\n';

        return false;
    }

    /*
     * 사용할 GPIO offset 배열을 구성한다.
     *
     * BCM GPIO 번호와 gpiochip offset이 같은 환경을 기준으로 한다.
     */
    offsets_[0] = static_cast<unsigned int>(stepPin_);
    offsets_[1] = static_cast<unsigned int>(dirPin_);
    offsetCount_ = 2;

    /*
     * ENABLE 핀을 사용하는 경우 세 번째 offset으로 추가한다.
     */
    if (enablePin_ >= 0)
    {
        offsets_[2] = static_cast<unsigned int>(enablePin_);
        offsetCount_ = 3;
    }

    /*
     * 모든 GPIO에 공통으로 적용할 출력 설정 객체를 생성한다.
     */
    gpiod_line_settings* settings =
        gpiod_line_settings_new();

    if (settings == nullptr)
    {
        std::cerr << "GPIO line settings 생성 실패\n";
        releaseGPIO();
        return false;
    }

    /*
     * STEP, DIR, ENABLE 핀을 출력 모드로 설정한다.
     */
    if (gpiod_line_settings_set_direction(
            settings,
            GPIOD_LINE_DIRECTION_OUTPUT) < 0)
    {
        std::cerr << "GPIO 출력 방향 설정 실패\n";

        gpiod_line_settings_free(settings);
        releaseGPIO();

        return false;
    }

    /*
     * 요청 직후 기본 출력값은 LOW로 설정한다.
     *
     * STEP = LOW
     * DIR = LOW
     *
     * ENABLE은 Active Low이므로, 요청 완료 후 별도로 HIGH를 출력해
     * 초기화 과정에서 모터가 활성화되는 시간을 최소화한다.
     */
    if (gpiod_line_settings_set_output_value(
            settings,
            GPIOD_LINE_VALUE_INACTIVE) < 0)
    {
        std::cerr << "GPIO 초기 출력값 설정 실패\n";

        gpiod_line_settings_free(settings);
        releaseGPIO();

        return false;
    }

    /*
     * GPIO offset과 설정을 연결하기 위한 line config를 생성한다.
     */
    gpiod_line_config* lineConfig =
        gpiod_line_config_new();

    if (lineConfig == nullptr)
    {
        std::cerr << "GPIO line config 생성 실패\n";

        gpiod_line_settings_free(settings);
        releaseGPIO();

        return false;
    }

    /*
     * STEP, DIR, ENABLE에 동일한 출력 설정을 적용한다.
     */
    if (gpiod_line_config_add_line_settings(
            lineConfig,
            offsets_,
            offsetCount_,
            settings) < 0)
    {
        std::cerr
            << "GPIO offset 설정 추가 실패: "
            << std::strerror(errno)
            << '\n';

        gpiod_line_config_free(lineConfig);
        gpiod_line_settings_free(settings);
        releaseGPIO();

        return false;
    }

    /*
     * GPIO 요청의 이름을 설정한다.
     *
     * gpioinfo 명령에서 어떤 프로그램이 GPIO를 사용 중인지
     * 확인할 때 표시된다.
     */
    gpiod_request_config* requestConfig =
        gpiod_request_config_new();

    if (requestConfig == nullptr)
    {
        std::cerr << "GPIO request config 생성 실패\n";

        gpiod_line_config_free(lineConfig);
        gpiod_line_settings_free(settings);
        releaseGPIO();

        return false;
    }

    gpiod_request_config_set_consumer(
        requestConfig,
        "stepper-motor"
    );

    /*
     * 설정된 GPIO들을 하나의 요청 객체로 점유한다.
     */
    request_ = gpiod_chip_request_lines(
        chip_,
        requestConfig,
        lineConfig
    );

    /*
     * line request가 생성된 뒤 설정 객체는 더 이상 필요하지 않다.
     */
    gpiod_request_config_free(requestConfig);
    gpiod_line_config_free(lineConfig);
    gpiod_line_settings_free(settings);

    if (request_ == nullptr)
    {
        std::cerr
            << "GPIO line request 실패: "
            << std::strerror(errno)
            << '\n';

        releaseGPIO();
        return false;
    }

    initialized_ = true;

    /*
     * ENABLE 핀을 사용한다면 먼저 비활성화 상태로 만든다.
     *
     * A4988 ENABLE:
     * HIGH = 비활성화
     * LOW  = 활성화
     */
    disable();

    /*
     * GPIO 초기화가 완료된 뒤 드라이버를 활성화한다.
     */
    enable();

    return true;
}

/*
 * 지정한 Step 수만큼 이동한다.
 *
 * 양수: DIR HIGH
 * 음수: DIR LOW
 */
bool StepperMotor::moveSteps(int steps)
{
    if (!initialized_ || request_ == nullptr)
    {
        std::cerr
            << "StepperMotor가 초기화되지 않았습니다.\n";

        return false;
    }

    /*
     * 이동량이 0이면 신호를 출력하지 않는다.
     */
    if (steps == 0)
    {
        return true;
    }

    const bool positiveDirection = steps > 0;
    const int stepCount = std::abs(steps);

    /*
     * 이동 방향 설정
     */
    const gpiod_line_value directionValue =
        positiveDirection
            ? GPIOD_LINE_VALUE_ACTIVE
            : GPIOD_LINE_VALUE_INACTIVE;

    if (gpiod_line_request_set_value(
            request_,
            static_cast<unsigned int>(dirPin_),
            directionValue) < 0)
    {
        std::cerr
            << "DIR GPIO 출력 실패: "
            << std::strerror(errno)
            << '\n';

        return false;
    }

    /*
     * DIR 값이 안정된 뒤 STEP 신호를 출력한다.
     */
    std::this_thread::sleep_for(
        std::chrono::microseconds(10)
    );

    /*
     * STEP 핀의 LOW → HIGH 상승 에지마다
     * A4988이 한 스텝씩 이동한다.
     */
    for (int i = 0; i < stepCount; ++i)
    {
        /*
         * STEP HIGH
         */
        if (gpiod_line_request_set_value(
                request_,
                static_cast<unsigned int>(stepPin_),
                GPIOD_LINE_VALUE_ACTIVE) < 0)
        {
            std::cerr
                << "STEP HIGH 출력 실패: "
                << std::strerror(errno)
                << '\n';

            return false;
        }

        std::this_thread::sleep_for(
            std::chrono::microseconds(pulseDelayUs_)
        );

        /*
         * STEP LOW
         */
        if (gpiod_line_request_set_value(
                request_,
                static_cast<unsigned int>(stepPin_),
                GPIOD_LINE_VALUE_INACTIVE) < 0)
        {
            std::cerr
                << "STEP LOW 출력 실패: "
                << std::strerror(errno)
                << '\n';

            return false;
        }

        std::this_thread::sleep_for(
            std::chrono::microseconds(pulseDelayUs_)
        );
    }

    /*
     * 실제 엔코더 위치가 아니라,
     * 출력한 Step 수를 기준으로 논리적 위치를 누적한다.
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
 * STEP 펄스 HIGH 및 LOW 유지시간 설정
 */
void StepperMotor::setPulseDelay(int microseconds)
{
    /*
     * 너무 빠른 펄스는 탈조와 토크 부족을 유발할 수 있다.
     */
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
 * A4988 드라이버 활성화
 *
 * ENABLE은 Active Low이다.
 */
void StepperMotor::enable()
{
    /*
     * ENABLE 핀을 사용하지 않는 경우에는 아무 작업도 하지 않는다.
     */
    if (!initialized_ ||
        request_ == nullptr ||
        enablePin_ < 0)
    {
        return;
    }

    if (gpiod_line_request_set_value(
            request_,
            static_cast<unsigned int>(enablePin_),
            GPIOD_LINE_VALUE_INACTIVE) < 0)
    {
        std::cerr
            << "ENABLE LOW 출력 실패: "
            << std::strerror(errno)
            << '\n';
    }
}

/*
 * A4988 드라이버 비활성화
 *
 * ENABLE HIGH를 출력하면 모터 코일 전류가 차단된다.
 */
void StepperMotor::disable()
{
    if (!initialized_ ||
        request_ == nullptr ||
        enablePin_ < 0)
    {
        return;
    }

    if (gpiod_line_request_set_value(
            request_,
            static_cast<unsigned int>(enablePin_),
            GPIOD_LINE_VALUE_ACTIVE) < 0)
    {
        std::cerr
            << "ENABLE HIGH 출력 실패: "
            << std::strerror(errno)
            << '\n';
    }
}

/*
 * 현재 논리적 Step 위치 반환
 */
long StepperMotor::getCurrentPosition() const
{
    return currentPosition_;
}

/*
 * 논리적 현재 위치 강제 지정
 */
void StepperMotor::setCurrentPosition(long position)
{
    currentPosition_ = position;
}

/*
 * GPIO 초기화 상태 반환
 */
bool StepperMotor::isInitialized() const
{
    return initialized_;
}

/*
 * GPIO 요청 및 GPIO 칩 자원 해제
 */
void StepperMotor::releaseGPIO()
{
    initialized_ = false;

    /*
     * GPIO line request를 먼저 해제한다.
     */
    if (request_ != nullptr)
    {
        gpiod_line_request_release(request_);
        request_ = nullptr;
    }

    /*
     * GPIO 칩을 닫는다.
     */
    if (chip_ != nullptr)
    {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
    }

    offsetCount_ = 0;
}