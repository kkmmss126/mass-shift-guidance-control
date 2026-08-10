/*
 * File: StepperMotor.h
 *
 * Description:
 * A4988 스텝모터 드라이버를 이용해 스텝모터 또는
 * 리니어 스텝모터 액추에이터를 제어하기 위한 클래스 선언 파일이다.
 *
 * libgpiod 2.x API를 기준으로 작성한다.
 *
 * Project:
 * 질량 이동 기반 중간 및 종말 유도 모사 시스템
 */

#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

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
        const char* chipPath = "/dev/gpiochip0"
    );

    ~StepperMotor();

    // GPIO 초기화
    bool initialize();

    // 지정한 스텝 수만큼 이동
    // 양수: 정방향, 음수: 역방향
    bool moveSteps(int steps);

    // STEP 펄스의 HIGH/LOW 유지시간 설정
    void setPulseDelay(int microseconds);

    // A4988 드라이버 활성화/비활성화
    void enable();
    void disable();

    // 현재 프로그램 내부에서 계산한 위치
    long getCurrentPosition() const;

    // 현재 위치값 강제 지정
    void setCurrentPosition(long position);

    // GPIO 초기화 여부 확인
    bool isInitialized() const;

private:
    /*
     * libgpiod 2.x에서는 GPIO 칩 이름이 아니라
     * "/dev/gpiochip0" 같은 장치 경로를 사용한다.
     */
    const char* chipPath_;

    int stepPin_;
    int dirPin_;
    int enablePin_;

    int pulseDelayUs_;
    long currentPosition_;

    bool initialized_;

    /*
     * libgpiod 2.x에서는 gpiod_line을 직접 보관하지 않고
     * line_request 객체 하나를 통해 여러 GPIO를 제어한다.
     */
    gpiod_chip* chip_;
    gpiod_line_request* request_;

    /*
     * GPIO 요청에 사용할 offset 목록을 저장한다.
     *
     * enablePin이 없는 경우에는 STEP과 DIR 두 개만 사용하고,
     * enablePin이 있는 경우에는 세 개를 사용한다.
     */
    unsigned int offsets_[3];
    unsigned int offsetCount_;

    /*
     * GPIO 자원 해제 함수
     */
    void releaseGPIO();
};

#endif