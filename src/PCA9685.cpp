#include "PCA9685.h"

#include <cmath>
#include <iostream>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <linux/i2c-dev.h>

/*=============================
    PCA9685 레지스터 주소
=============================*/

// 동작 모드 레지스터
#define MODE1      0x00

// 출력 모드 레지스터
#define MODE2      0x01

// PWM 주파수 설정 레지스터
#define PRESCALE   0xFE

// 채널0 PWM 시작 레지스터
// 채널마다 4바이트씩 증가
#define LED0_ON_L  0x06


/*====================================================
    생성자
====================================================*/
PCA9685::PCA9685(uint8_t addr)
    : fd(-1), address(addr)
{
    // I2C 버스(/dev/i2c-1) 열기
    fd = open("/dev/i2c-1", O_RDWR);

    if (fd < 0)
    {
        std::cerr << "I2C Open Failed\n";
        return;
    }

    // 사용할 I2C 슬레이브 주소 선택(기본 0x40)
    if (ioctl(fd, I2C_SLAVE, address) < 0)
    {
        std::cerr << "I2C Address Failed\n";

        close(fd);
        fd = -1;

        return;
    }

    // PCA9685 초기화
    begin();
}


/*====================================================
    소멸자
====================================================*/
PCA9685::~PCA9685()
{
    // 열린 I2C 파일 닫기
    if(fd >= 0)
        close(fd);
}


/*====================================================
    PCA9685 초기 설정
====================================================*/
bool PCA9685::begin()
{
    /*
        MODE1

        0x20

        AI(Auto Increment) 활성화

        여러 레지스터를 연속 접근할 수 있게 설정
    */
    write8(MODE1, 0x20);

    /*
        MODE2

        0x04

        Totem Pole 출력

        일반적인 서보모터 구동 방식
    */
    write8(MODE2, 0x04);

    // 설정 안정화 대기
    usleep(5000);

    return true;
}


/*====================================================
    레지스터 1바이트 쓰기
====================================================*/
void PCA9685::write8(uint8_t reg,
                     uint8_t data)
{
    uint8_t buffer[2];

    // 첫 번째 바이트 : 레지스터 주소
    buffer[0] = reg;

    // 두 번째 바이트 : 기록할 데이터
    buffer[1] = data;

    // I2C 전송
    write(fd, buffer, 2);
}


/*====================================================
    레지스터 1바이트 읽기
====================================================*/
uint8_t PCA9685::read8(uint8_t reg)
{
    // 읽고 싶은 레지스터 주소 전송
    write(fd, &reg, 1);

    uint8_t value;

    // 해당 레지스터 값 읽기
    read(fd, &value, 1);

    return value;
}


/*====================================================
    PWM 주파수 설정

    서보모터 : 50Hz
====================================================*/
void PCA9685::setPWMFreq(float freq)
{
    /*
        PCA9685 내부 클럭

        25MHz
    */
    float prescaleValue = 25000000.0F;

    // 12bit 분해능(4096단계)
    prescaleValue /= 4096.0F;

    // 원하는 주파수 적용
    prescaleValue /= freq;

    // 데이터시트 공식
    prescaleValue -= 1.0F;

    uint8_t prescale =
        static_cast<uint8_t>(std::floor(prescaleValue + 0.5F));

    // 현재 MODE1 값 읽기
    uint8_t oldMode = read8(MODE1);

    /*
        PRESCALE 변경 시

        반드시 Sleep 상태로 진입해야 한다.
    */
    uint8_t sleepMode = (oldMode & 0x7F) | 0x10;

    // Sleep 진입
    write8(MODE1, sleepMode);

    // 새로운 Prescale 값 적용
    write8(PRESCALE, prescale);

    // Sleep 해제
    write8(MODE1, oldMode);

    usleep(5000);

    /*
        RESTART

        Auto Increment 활성화
    */
    write8(MODE1, 0xA0);

    usleep(5000);
}


/*====================================================
    특정 채널 PWM 출력

    channel : 0~15

    on  : HIGH 시작 시점

    off : HIGH 종료 시점
====================================================*/
void PCA9685::setPWM(uint8_t channel,
                     uint16_t on,
                     uint16_t off)
{
    // PCA9685는 16채널만 지원
    if(channel > 15)
    {
        std::cerr << "Invalid PCA9685 Channel\n";
        return;
    }

    /*
        채널0

        LED0_ON_L
        LED0_ON_H
        LED0_OFF_L
        LED0_OFF_H

        채널마다 4바이트씩 증가
    */
    uint8_t reg = LED0_ON_L + 4 * channel;

    /*
        HIGH 시작 시점
    */
    write8(reg,
           on & 0xFF);

    write8(reg + 1,
           (on >> 8) & 0x0F);

    /*
        HIGH 종료 시점

        서보모터는 이 값(off)이
        실제 각도를 결정한다.
    */
    write8(reg + 2,
           off & 0xFF);

    write8(reg + 3,
           (off >> 8) & 0x0F);
}