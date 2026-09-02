#include "PCA9685.h"

#include <iostream>
#include <termios.h>
#include <unistd.h>

/*
 * ============================================================
 * Seeker Center Calibration
 * ============================================================
 *
 * W / S : Tilt 서보 이동
 * A / D : Pan 서보 이동
 * Q     : 종료
 *
 * 직각 Pan-Tilt 구조에서 센서부가 동체 정면을 바라보는
 * PWM 값을 찾기 위한 캘리브레이션 프로그램이다.
 */

constexpr int TILT_CHANNEL = 0;
constexpr int PAN_CHANNEL  = 1;

/*
 * 시작 PWM
 */
int tiltPWM = 330;
int panPWM  = 312;

/*
 * 안전 범위
 */
constexpr int TILT_MIN_PWM = 200;
constexpr int TILT_MAX_PWM = 450;

constexpr int PAN_MIN_PWM = 200;
constexpr int PAN_MAX_PWM = 450;


/*
 * Enter 없이 키 하나를 입력받는다.
 */
char getKey()
{
    struct termios oldt;
    struct termios newt;

    char ch;

    tcgetattr(STDIN_FILENO, &oldt);

    newt = oldt;

    newt.c_lflag &= ~(ICANON | ECHO);

    tcsetattr(
        STDIN_FILENO,
        TCSANOW,
        &newt);

    ch = getchar();

    tcsetattr(
        STDIN_FILENO,
        TCSANOW,
        &oldt);

    return ch;
}


int main()
{
    /*
     * PCA9685 초기화
     */
    PCA9685 pwm;

    if (!pwm.begin())
    {
        std::cerr
            << "[ERROR] PCA9685 초기화 실패\n";

        return 1;
    }

    /*
     * 서보 PWM 주파수
     */
    pwm.setPWMFreq(50);


    /*
     * 초기 위치 적용
     */
    pwm.setPWM(
        TILT_CHANNEL,
        0,
        tiltPWM);

    pwm.setPWM(
        PAN_CHANNEL,
        0,
        panPWM);


    std::cout
        << "========================================\n"
        << "   Seeker Center Calibration\n"
        << "========================================\n"
        << "W : Tilt PWM +1\n"
        << "S : Tilt PWM -1\n"
        << "A : Pan PWM -1\n"
        << "D : Pan PWM +1\n"
        << "Q : Quit\n"
        << "========================================\n";


    while (true)
    {
        std::cout
            << "Tilt PWM = "
            << tiltPWM
            << " | Pan PWM = "
            << panPWM
            << '\n';


        char key = getKey();


        /*
         * Tilt +
         */
        if (key == 'w' || key == 'W')
        {
            if (tiltPWM < TILT_MAX_PWM)
            {
                tiltPWM++;

                pwm.setPWM(
                    TILT_CHANNEL,
                    0,
                    tiltPWM);
            }
        }


        /*
         * Tilt -
         */
        else if (key == 's' || key == 'S')
        {
            if (tiltPWM > TILT_MIN_PWM)
            {
                tiltPWM--;

                pwm.setPWM(
                    TILT_CHANNEL,
                    0,
                    tiltPWM);
            }
        }


        /*
         * Pan -
         */
        else if (key == 'a' || key == 'A')
        {
            if (panPWM > PAN_MIN_PWM)
            {
                panPWM--;

                pwm.setPWM(
                    PAN_CHANNEL,
                    0,
                    panPWM);
            }
        }


        /*
         * Pan +
         */
        else if (key == 'd' || key == 'D')
        {
            if (panPWM < PAN_MAX_PWM)
            {
                panPWM++;

                pwm.setPWM(
                    PAN_CHANNEL,
                    0,
                    panPWM);
            }
        }


        /*
         * 종료
         */
        else if (key == 'q' || key == 'Q')
        {
            break;
        }
    }


    std::cout
        << "\n========================================\n"
        << "Calibration Result\n"
        << "Tilt PWM = "
        << tiltPWM
        << '\n'
        << "Pan PWM  = "
        << panPWM
        << '\n'
        << "========================================\n";


    return 0;
}
