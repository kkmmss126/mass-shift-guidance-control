#include <iostream>
#include <unistd.h>
#include <termios.h>
#include "PCA9685.h"

using namespace std;

/*
 * 엔터를 누르지 않고 키 하나만 입력받는 함수
 */
char getKey()
{
    struct termios oldt, newt;
    char ch;

    // 현재 터미널 설정 저장
    tcgetattr(STDIN_FILENO, &oldt);

    newt = oldt;

    // 엔터 입력 대기 및 입력 문자 출력(ECHO) 비활성화
    newt.c_lflag &= ~(ICANON | ECHO);

    // 변경된 설정 적용
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // 키 하나 입력
    ch = getchar();

    // 원래 설정 복구
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}

int main()
{
    // PCA9685 객체 생성
    PCA9685 pwm;

    // 서보모터 주파수(50Hz)
    pwm.setPWMFreq(50);

    // 테스트할 PCA9685 채널 번호
    const int SERVO_CHANNEL = 0;

    // 시작 PWM 값
    int pwmValue = 300;

    // 처음 위치로 이동
    pwm.setPWM(SERVO_CHANNEL, 0, pwmValue);

    cout << "======================================" << endl;
    cout << " Servo Center Calibration Program" << endl;
    cout << "======================================" << endl;
    cout << "a : PWM -1" << endl;
    cout << "d : PWM +1" << endl;
    cout << "z : PWM -10" << endl;
    cout << "c : PWM +10" << endl;
    cout << "q : Save & Exit" << endl;
    cout << "======================================" << endl;

    while (true)
    {
        // 키 입력
        char key = getKey();

        switch (key)
        {
            // PWM 1 감소
            case 'a':
                pwmValue--;
                break;

            // PWM 1 증가
            case 'd':
                pwmValue++;
                break;

            // PWM 10 감소
            case 'z':
                pwmValue -= 10;
                break;

            // PWM 10 증가
            case 'c':
                pwmValue += 10;
                break;

            // 종료
            case 'q':

                cout << endl;
                cout << "==============================" << endl;
                cout << "Center PWM : " << pwmValue << endl;
                cout << "==============================" << endl;

                return 0;
        }

        // 서보 이동
        pwm.setPWM(SERVO_CHANNEL, 0, pwmValue);

        // 현재 PWM 출력
        cout << "\rCurrent PWM : "
             << pwmValue
             << "      "
             << flush;
    }

    return 0;
}