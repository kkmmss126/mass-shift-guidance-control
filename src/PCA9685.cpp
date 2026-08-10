#include "PCA9685.h"

#include <cmath>
#include <iostream>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <linux/i2c-dev.h>

/*
 * PCA9685 주요 레지스터 주소
 */
#define MODE1      0x00
#define MODE2      0x01
#define PRESCALE   0xFE
#define LED0_ON_L  0x06

/*
 * MODE1 비트 설정
 */
#define MODE1_RESTART  0x80
#define MODE1_AI       0x20
#define MODE1_SLEEP    0x10

/*
 * MODE2 비트 설정
 *
 * OUTDRV = 1:
 * PCA9685 출력을 토템폴 방식으로 설정한다.
 */
#define MODE2_OUTDRV   0x04


PCA9685::PCA9685(uint8_t addr)
    : address(addr),
      fd(-1)
{
    /*
     * Raspberry Pi의 기본 I2C 장치 파일을 연다.
     */
    fd = open("/dev/i2c-1", O_RDWR);

    if (fd < 0)
    {
        std::cerr << "I2C Open Failed\n";
        return;
    }

    /*
     * 통신할 PCA9685의 I2C 주소를 지정한다.
     * 일반적인 기본 주소는 0x40이다.
     */
    if (ioctl(fd, I2C_SLAVE, address) < 0)
    {
        std::cerr << "I2C Address Failed\n";

        close(fd);
        fd = -1;

        return;
    }

    begin();
}


PCA9685::~PCA9685()
{
    if (fd >= 0)
    {
        close(fd);
    }
}


bool PCA9685::begin()
{
    if (fd < 0)
    {
        return false;
    }

    /*
     * MODE1 설정
     *
     * AI(Auto Increment)를 활성화해야 setPWM()에서
     * 여러 레지스터에 연속으로 데이터를 쓸 수 있다.
     */
    write8(MODE1, MODE1_AI);

    /*
     * 출력 드라이버를 토템폴 방식으로 설정한다.
     */
    write8(MODE2, MODE2_OUTDRV);

    /*
     * PCA9685가 설정을 적용할 시간을 준다.
     */
    usleep(5000);

    return true;
}


void PCA9685::write8(uint8_t reg, uint8_t data)
{
    if (fd < 0)
    {
        return;
    }

    uint8_t buffer[2];

    buffer[0] = reg;
    buffer[1] = data;

    /*
     * 레지스터 주소와 데이터를 한 번에 전송한다.
     */
    if (write(fd, buffer, 2) != 2)
    {
        std::cerr << "PCA9685 register write failed\n";
    }
}


uint8_t PCA9685::read8(uint8_t reg)
{
    if (fd < 0)
    {
        return 0;
    }

    /*
     * 먼저 읽고자 하는 레지스터 주소를 지정한다.
     */
    if (write(fd, &reg, 1) != 1)
    {
        std::cerr << "PCA9685 register select failed\n";
        return 0;
    }

    uint8_t value = 0;

    /*
     * 지정한 레지스터의 값을 읽는다.
     */
    if (read(fd, &value, 1) != 1)
    {
        std::cerr << "PCA9685 register read failed\n";
        return 0;
    }

    return value;
}


void PCA9685::setPWMFreq(float freq)
{
    if (fd < 0 || freq <= 0.0f)
    {
        return;
    }

    /*
     * PCA9685 프리스케일 계산식
     *
     * prescale =
     * oscillator / (4096 × PWM 주파수) - 1
     */
    float prescaleValue = 25000000.0f;

    prescaleValue /= 4096.0f;
    prescaleValue /= freq;
    prescaleValue -= 1.0f;

    uint8_t prescale =
        static_cast<uint8_t>(std::floor(prescaleValue + 0.5f));

    /*
     * 기존 MODE1 설정을 읽는다.
     * 여기에는 AI 비트가 포함되어 있다.
     */
    uint8_t oldMode = read8(MODE1);

    /*
     * PRESCALE 레지스터는 SLEEP 상태에서만 변경할 수 있다.
     */
    uint8_t sleepMode =
        static_cast<uint8_t>((oldMode & 0x7F) | MODE1_SLEEP);

    write8(MODE1, sleepMode);
    write8(PRESCALE, prescale);

    /*
     * SLEEP 상태를 해제하면서 기존 설정을 복원한다.
     */
    write8(MODE1, oldMode);

    usleep(5000);

    /*
     * RESTART 비트를 설정해 PWM 동작을 재시작한다.
     * AI 비트도 유지한다.
     */
    write8(MODE1, oldMode | MODE1_RESTART);

    usleep(5000);
}


void PCA9685::setPWM(uint8_t channel,
                     uint16_t on,
                     uint16_t off)
{
    if (fd < 0)
    {
        return;
    }

    /*
     * PCA9685는 0~15번까지 총 16개 채널을 지원한다.
     */
    if (channel >= 16)
    {
        std::cerr << "Invalid PCA9685 channel\n";
        return;
    }

    /*
     * PWM 카운트는 12비트이므로 최대값은 4095이다.
     */
    on &= 0x0FFF;
    off &= 0x0FFF;

    /*
     * 각 채널은 4개의 연속된 레지스터를 사용한다.
     *
     * LEDn_ON_L
     * LEDn_ON_H
     * LEDn_OFF_L
     * LEDn_OFF_H
     */
    uint8_t reg =
        static_cast<uint8_t>(LED0_ON_L + 4 * channel);

    uint8_t buffer[5];

    /*
     * 첫 바이트는 시작 레지스터 주소이다.
     */
    buffer[0] = reg;

    /*
     * PWM 시작 카운트
     */
    buffer[1] = static_cast<uint8_t>(on & 0xFF);
    buffer[2] = static_cast<uint8_t>((on >> 8) & 0x0F);

    /*
     * PWM 종료 카운트
     */
    buffer[3] = static_cast<uint8_t>(off & 0xFF);
    buffer[4] = static_cast<uint8_t>((off >> 8) & 0x0F);

    /*
     * MODE1의 AI 비트가 활성화되어 있으므로
     * 네 개의 PWM 레지스터가 순서대로 기록된다.
     */
    if (write(fd, buffer, 5) != 5)
    {
        std::cerr << "PCA9685 PWM write failed\n";
    }
}