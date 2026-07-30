#ifndef PCA9685_H
#define PCA9685_H

/*
 * File: PCA9685.h
 *
 * Description:
 * PCA9685 16채널 PWM 드라이버를 제어하기 위한 클래스 선언 파일이다.
 *
 * I2C 통신을 이용하여 PWM 주파수 및 Duty Cycle을 설정하며,
 * MG90S 서보모터의 각도 제어에 사용한다.
 *
 * Project:
 * 질량 이동 기반 중간 및 종말 유도 모사 시스템
 *
 * 주요 기능:
 * 1. I2C 초기화
 * 2. PCA9685 초기 설정
 * 3. PWM 주파수 설정
 * 4. 채널별 PWM 출력
 *
 * 적용 대상:
 * - MG90S 2축 시커 짐벌
 * - PCA9685 PWM 드라이버
 */

#include <cstdint>

/*
 * PCA9685 PWM Driver 클래스
 *
 * PCA9685와의 I2C 통신을 담당하며
 * 각 채널의 PWM 출력을 제어한다.
 */
class PCA9685
{
public:

    // PCA9685 객체 생성 (기본 I2C 주소 : 0x40)
    PCA9685(uint8_t address = 0x40);

    // 객체 소멸 시 I2C 장치 종료
    ~PCA9685();

    // PCA9685 초기화
    bool begin();

    // PWM 주파수 설정 (예: 서보모터 50Hz)
    void setPWMFreq(float freq);

    // 지정한 채널의 PWM 출력 설정
    void setPWM(uint8_t channel,
                uint16_t on,
                uint16_t off);

private:

    // I2C 파일 디스크립터
    int fd;

    // PCA9685 I2C 주소
    uint8_t address;

    // PCA9685 레지스터에 1Byte 데이터 쓰기
    void write8(uint8_t reg,
                uint8_t data);

    // PCA9685 레지스터에서 1Byte 데이터 읽기
    uint8_t read8(uint8_t reg);
};

#endif