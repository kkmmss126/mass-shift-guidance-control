/*
 * File: StepperMotor.h
 *
 * Description:
 * A4988 스텝모터 드라이버를 이용해 스텝모터 또는
 * 리니어 스텝모터 액추에이터를 제어하기 위한 클래스 선언 파일이다.
 *
 * 이 파일에는 클래스의 구조와 외부에서 사용할 수 있는 함수만 선언한다.
 * 실제 GPIO 제어 동작은 StepperMotor.cpp에 구현한다.
 *
 * Project:
 * 질량 이동 기반 중간 및 종말 유도 모사 시스템
 *
 * 주요 기능:
 * 1. GPIO 초기화
 * 2. 모터 이동 방향 설정
 * 3. STEP 펄스 생성
 * 4. A4988 드라이버 활성화 및 비활성화
 * 5. 현재 모터 위치를 Step 단위로 관리
 *
 * 적용 대상:
 * - 질량 이동용 리니어 액추에이터
 * - 기체 자세 제어용 스텝모터
 * - A4988과 STEP/DIR 방식으로 연결된 모터
 */
#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

/*
 * libgpiod를 사용해 라즈베리파이 GPIO 칩과
 * 개별 GPIO 라인에 접근한다.
 */
#include <gpiod.h>

/*
 * A4988과 연결된 하나의 스텝모터를 객체로 표현한다.
 */
class StepperMotor
{
public:
    StepperMotor(
        int stepPin,
        int dirPin,
        int enablePin = -1,
        const char* chipName = "gpiochip0"
    );

    ~StepperMotor();

    // GPIO 초기화
    bool initialize();

    // 지정한 스텝 수만큼 이동
    // 양수: 정방향, 음수: 역방향
    bool moveSteps(int steps);

    // 펄스의 HIGH/LOW 유지시간 설정
    void setPulseDelay(int microseconds);

    // 드라이버 활성화/비활성화
    void enable();
    void disable();

    // 현재 프로그램 내부에서 계산한 위치
    long getCurrentPosition() const;

    // 현재 위치값 강제 지정
    void setCurrentPosition(long position);

    bool isInitialized() const;

private:
    const char* chipName_;

    int stepPin_;
    int dirPin_;
    int enablePin_;

    int pulseDelayUs_;
    long currentPosition_;

    bool initialized_;

    gpiod_chip* chip_;
    gpiod_line* stepLine_;
    gpiod_line* dirLine_;
    gpiod_line* enableLine_;

    void releaseGPIO();
};

#endif