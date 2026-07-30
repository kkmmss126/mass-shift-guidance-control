/*
 * File: PCA9685.cpp
 *
 * Description:
 * Linux I2C 인터페이스를 이용하여 PCA9685 PWM 드라이버를
 * 제어하는 클래스의 구현 파일이다.
 *
 * PCA9685의 동작 모드와 PWM 주파수를 설정하고,
 * 각 채널의 PWM 시작·종료 카운트를 제어한다.
 *
 * Project:
 * 질량 이동 기반 중간 및 종말 유도 모사 시스템
 *
 * 주요 기능:
 * 1. I2C 장치 연결
 * 2. PCA9685 동작 모드 초기화
 * 3. PWM 주파수 설정
 * 4. 채널별 PWM 출력 설정
 */

#include "PCA9685.h"

#include <cmath>
#include <cstdint>
#include <iostream>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

/* PCA9685 register addresses */
namespace
{
constexpr uint8_t MODE1     = 0x00;
constexpr uint8_t MODE2     = 0x01;
constexpr uint8_t LED0_ON_L = 0x06;
constexpr uint8_t PRESCALE  = 0xFE;

constexpr float OSCILLATOR_FREQUENCY = 25'000'000.0F;
constexpr float PWM_RESOLUTION       = 4096.0F;
}

/*
 * I2C 버스를 열고 사용할 PCA9685 슬레이브 주소를 설정한다.
 */
PCA9685::PCA9685(uint8_t addr)
    : fd(-1),
      address(addr)
{
    fd = open("/dev/i2c-1", O_RDWR);

    if (fd < 0)
    {
        std::cerr << "Failed to open I2C bus\n";
        return;
    }

    if (ioctl(fd, I2C_SLAVE, address) < 0)
    {
        std::cerr << "Failed to select PCA9685 I2C address\n";

        close(fd);
        fd = -1;
        return;
    }

    if (!begin())
    {
        std::cerr << "Failed to initialize PCA9685\n";
    }
}

/*
 * 객체가 소멸될 때 열려 있는 I2C 파일 디스크립터를 닫는다.
 */
PCA9685::~PCA9685()
{
    if (fd >= 0)
    {
        close(fd);
    }
}

/*
 * PCA9685의 기본 동작 모드를 설정한다.
 *
 * MODE1의 AI 비트를 활성화하여 연속된 레지스터에
 * 순차적으로 접근할 수 있도록 한다.
 *
 * MODE2는 Totem Pole 출력 방식으로 설정한다.
 */
bool PCA9685::begin()
{
    if (fd < 0)
    {
        return false;
    }

    write8(MODE1, 0x20);
    write8(MODE2, 0x04);

    // 레지스터 설정이 안정화될 시간을 확보한다.
    usleep(5000);

    return true;
}

/*
 * 지정한 PCA9685 레지스터에 1바이트 데이터를 기록한다.
 */
void PCA9685::write8(uint8_t reg,
                     uint8_t data)
{
    if (fd < 0)
    {
        std::cerr << "I2C device is not initialized\n";
        return;
    }

    const uint8_t buffer[2] = {reg, data};

    if (write(fd, buffer, sizeof(buffer)) !=
        static_cast<ssize_t>(sizeof(buffer)))
    {
        std::cerr << "Failed to write PCA9685 register\n";
    }
}

/*
 * 지정한 PCA9685 레지스터에서 1바이트 데이터를 읽는다.
 */
uint8_t PCA9685::read8(uint8_t reg)
{
    if (fd < 0)
    {
        std::cerr << "I2C device is not initialized\n";
        return 0;
    }

    if (write(fd, &reg, 1) != 1)
    {
        std::cerr << "Failed to select PCA9685 register\n";
        return 0;
    }

    uint8_t value = 0;

    if (read(fd, &value, 1) != 1)
    {
        std::cerr << "Failed to read PCA9685 register\n";
        return 0;
    }

    return value;
}

/*
 * PCA9685의 PWM 출력 주파수를 설정한다.
 *
 * PRESCALE 레지스터는 Sleep 상태에서만 변경할 수 있으므로
 * Sleep 진입 → PRESCALE 설정 → Sleep 해제 순서로 진행한다.
 */
void PCA9685::setPWMFreq(float freq)
{
    if (fd < 0)
    {
        std::cerr << "I2C device is not initialized\n";
        return;
    }

    if (freq <= 0.0F)
    {
        std::cerr << "PWM frequency must be greater than zero\n";
        return;
    }

    const float prescaleValue =
        OSCILLATOR_FREQUENCY /
        (PWM_RESOLUTION * freq) - 1.0F;

    const uint8_t prescale =
        static_cast<uint8_t>(
            std::floor(prescaleValue + 0.5F)
        );

    const uint8_t oldMode = read8(MODE1);

    /*
     * MODE1의 RESTART 비트는 제거하고,
     * SLEEP 비트를 활성화한다.
     */
    const uint8_t sleepMode =
        static_cast<uint8_t>((oldMode & 0x7F) | 0x10);

    write8(MODE1, sleepMode);
    write8(PRESCALE, prescale);
    write8(MODE1, oldMode);

    usleep(5000);

    /*
     * RESTART와 Auto Increment를 활성화한다.
     *
     * 0x80: RESTART
     * 0x20: Auto Increment
     */
    write8(MODE1, 0xA0);

    usleep(5000);
}

/*
 * 지정한 채널의 PWM 시작점과 종료점을 설정한다.
 *
 * 각 채널은 ON_L, ON_H, OFF_L, OFF_H의
 * 네 개 레지스터를 사용한다.
 */
void PCA9685::setPWM(uint8_t channel,
                     uint16_t on,
                     uint16_t off)
{
    if (fd < 0)
    {
        std::cerr << "I2C device is not initialized\n";
        return;
    }

    if (channel > 15)
    {
        std::cerr << "Invalid PCA9685 channel\n";
        return;
    }

    if (on > 4095 || off > 4095)
    {
        std::cerr << "PWM count must be between 0 and 4095\n";
        return;
    }

    // 각 채널은 네 개의 연속된 레지스터를 사용한다.
    const uint8_t reg =
        static_cast<uint8_t>(LED0_ON_L + 4 * channel);

    write8(reg,     static_cast<uint8_t>(on & 0xFF));
    write8(reg + 1, static_cast<uint8_t>((on >> 8) & 0x0F));

    write8(reg + 2, static_cast<uint8_t>(off & 0xFF));
    write8(reg + 3, static_cast<uint8_t>((off >> 8) & 0x0F));
}