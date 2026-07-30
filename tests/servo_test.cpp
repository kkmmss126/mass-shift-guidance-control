/*
 * File: servo_center_calibration.cpp
 *
 * Description:
 * PCA9685 PWM 드라이버에 연결된 서보모터를 키보드로 직접 조작하여
 * 기계적 중심 위치에 해당하는 PWM 값을 측정하는 테스트 프로그램이다.
 *
 * Project:
 * 질량 이동 기반 중간 및 종말 유도 모사 시스템
 *
 * 주요 기능:
 * 1. PCA9685 PWM 주파수 설정
 * 2. 키보드 입력을 이용한 PWM 미세 조정
 * 3. 서보모터 중심 위치 탐색
 * 4. 중심 PWM 값 출력
 *
 * 조작 방법:
 * a / d : PWM 값을 1씩 감소 / 증가
 * z / c : PWM 값을 10씩 감소 / 증가
 * q     : 현재 PWM 값을 출력하고 종료
 */

#include "PCA9685.h"

#include <iostream>
#include <cstdio>
#include <termios.h>
#include <unistd.h>

namespace
{
constexpr uint8_t SERVO_CHANNEL = 0;

// 중심 탐색을 시작할 초기 PWM 값
constexpr int INITIAL_PWM = 300;

// 서보모터에 과도한 펄스가 입력되는 것을 방지하기 위한 제한값
constexpr int MIN_PWM = 100;
constexpr int MAX_PWM = 600;
}

/*
 * 엔터 입력 없이 키 하나를 즉시 읽는다.
 *
 * ICANON을 비활성화하면 줄 단위가 아닌 문자 단위로 입력받고,
 * ECHO를 비활성화하면 입력한 문자가 터미널에 표시되지 않는다.
 */
char getKey()
{
    termios oldTerminal{};
    termios newTerminal{};

    // 현재 터미널 설정을 저장한다.
    if (tcgetattr(STDIN_FILENO, &oldTerminal) < 0)
    {
        std::cerr << "Failed to read terminal settings\n";
        return '\0';
    }

    newTerminal = oldTerminal;

    // 엔터 입력 대기와 입력 문자 표시를 비활성화한다.
    newTerminal.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));

    if (tcsetattr(STDIN_FILENO, TCSANOW, &newTerminal) < 0)
    {
        std::cerr << "Failed to change terminal settings\n";
        return '\0';
    }

    const char key = static_cast<char>(std::getchar());

    // 키 입력이 끝난 뒤 원래 터미널 설정으로 복구한다.
    tcsetattr(STDIN_FILENO, TCSANOW, &oldTerminal);

    return key;
}

int main()
{
    // 기본 I2C 주소 0x40을 사용하는 PCA9685 객체를 생성한다.
    PCA9685 pwm;

    // 일반적인 아날로그 서보모터의 PWM 주파수인 50 Hz로 설정한다.
    pwm.setPWMFreq(50.0F);

    int pwmValue = INITIAL_PWM;

    // 프로그램 시작 시 서보모터를 초기 PWM 위치로 이동시킨다.
    pwm.setPWM(
        SERVO_CHANNEL,
        0,
        static_cast<uint16_t>(pwmValue)
    );

    std::cout
        << "======================================\n"
        << " Servo Center Calibration Program\n"
        << "======================================\n"
        << "a : PWM -1\n"
        << "d : PWM +1\n"
        << "z : PWM -10\n"
        << "c : PWM +10\n"
        << "q : Save & Exit\n"
        << "======================================\n";

    while (true)
    {
        const char key = getKey();

        switch (key)
        {
            case 'a':
                --pwmValue;
                break;

            case 'd':
                ++pwmValue;
                break;

            case 'z':
                pwmValue -= 10;
                break;

            case 'c':
                pwmValue += 10;
                break;

            case 'q':
                std::cout
                    << "\n==============================\n"
                    << "Center PWM : " << pwmValue << '\n'
                    << "==============================\n";

                return 0;

            default:
                // 정의되지 않은 키는 무시하고 다시 입력받는다.
                continue;
        }

        /*
         * 설정한 안전 범위를 벗어나지 않도록 PWM 값을 제한한다.
         *
         * 실제 사용 범위는 서보모터와 기구물에 따라 달라지므로,
         * 테스트 결과에 맞춰 MIN_PWM과 MAX_PWM을 조정해야 한다.
         */
        if (pwmValue < MIN_PWM)
        {
            pwmValue = MIN_PWM;
        }
        else if (pwmValue > MAX_PWM)
        {
            pwmValue = MAX_PWM;
        }

        // 변경된 PWM 값을 PCA9685에 전달하여 서보 위치를 갱신한다.
        pwm.setPWM(
            SERVO_CHANNEL,
            0,
            static_cast<uint16_t>(pwmValue)
        );

        // '\r'을 사용하여 같은 줄에서 현재 값을 계속 갱신한다.
        std::cout
            << "\rCurrent PWM : "
            << pwmValue
            << "      "
            << std::flush;
    }
}