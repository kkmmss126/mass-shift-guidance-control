#include <gpiod.h>
#include <chrono>
#include <iostream>
#include <thread>

using namespace std;

constexpr unsigned int LEFT_TRIG   = 5;
constexpr unsigned int LEFT_ECHO   = 20;

constexpr unsigned int RIGHT_TRIG  = 12;
constexpr unsigned int RIGHT_ECHO  = 13;

constexpr unsigned int BOTTOM_TRIG = 16;
constexpr unsigned int BOTTOM_ECHO = 6;

constexpr long TIMEOUT_US = 30000;


// 현재 시간을 us 단위로 반환
long long nowUs()
{
    return chrono::duration_cast<chrono::microseconds>(
        chrono::steady_clock::now().time_since_epoch()
    ).count();
}


// 초음파 센서 1개 거리 측정
double measureDistance(
    gpiod_line_request* trigReq,
    unsigned int trigPin,
    gpiod_line_request* echoReq,
    unsigned int echoPin
)
{
    // TRIG LOW
    gpiod_line_request_set_value(
        trigReq,
        trigPin,
        GPIOD_LINE_VALUE_INACTIVE
    );

    this_thread::sleep_for(chrono::microseconds(2));

    // TRIG HIGH 약 10us
    gpiod_line_request_set_value(
        trigReq,
        trigPin,
        GPIOD_LINE_VALUE_ACTIVE
    );

    this_thread::sleep_for(chrono::microseconds(10));

    // 다시 LOW
    gpiod_line_request_set_value(
        trigReq,
        trigPin,
        GPIOD_LINE_VALUE_INACTIVE
    );


    // ECHO HIGH 대기
    long long waitStart = nowUs();

    while (
        gpiod_line_request_get_value(
            echoReq,
            echoPin
        ) == GPIOD_LINE_VALUE_INACTIVE
    )
    {
        if (nowUs() - waitStart > TIMEOUT_US)
        {
            return -1.0;
        }
    }


    // ECHO HIGH 시작 시간
    long long echoStart = nowUs();


    // ECHO LOW 대기
    while (
        gpiod_line_request_get_value(
            echoReq,
            echoPin
        ) == GPIOD_LINE_VALUE_ACTIVE
    )
    {
        if (nowUs() - echoStart > TIMEOUT_US)
        {
            return -1.0;
        }
    }


    // ECHO HIGH 종료 시간
    long long echoEnd = nowUs();

    long long pulseWidth = echoEnd - echoStart;

    // cm 계산
    double distance = pulseWidth * 0.0343 / 2.0;

    return distance;
}


int main()
{
    cout << "========================================\n";
    cout << " 3-Channel Ultrasonic Sensor Test\n";
    cout << "========================================\n";
    cout << "LEFT   : TRIG 5  / ECHO 20\n";
    cout << "RIGHT  : TRIG 12 / ECHO 13\n";
    cout << "BOTTOM : TRIG 16 / ECHO 6\n";
    cout << "Ctrl+C : Exit\n\n";


    gpiod_chip* chip = gpiod_chip_open("/dev/gpiochip0");

    if (!chip)
    {
        cerr << "[ERROR] gpiochip0 open failed\n";
        return 1;
    }


    // ========================================================
    // TRIG 3개 설정
    // ========================================================

    unsigned int trigOffsets[3] = {
        LEFT_TRIG,
        RIGHT_TRIG,
        BOTTOM_TRIG
    };

    gpiod_line_settings* trigSettings =
        gpiod_line_settings_new();

    gpiod_line_config* trigConfig =
        gpiod_line_config_new();


    gpiod_line_settings_set_direction(
        trigSettings,
        GPIOD_LINE_DIRECTION_OUTPUT
    );

    gpiod_line_settings_set_output_value(
        trigSettings,
        GPIOD_LINE_VALUE_INACTIVE
    );

    gpiod_line_config_add_line_settings(
        trigConfig,
        trigOffsets,
        3,
        trigSettings
    );

    gpiod_line_request* trigReq =
        gpiod_chip_request_lines(
            chip,
            nullptr,
            trigConfig
        );

    if (!trigReq)
    {
        cerr << "[ERROR] TRIG request failed\n";
        return 1;
    }


    // ========================================================
    // ECHO 3개 설정
    // ========================================================

    unsigned int echoOffsets[3] = {
        LEFT_ECHO,
        RIGHT_ECHO,
        BOTTOM_ECHO
    };

    gpiod_line_settings* echoSettings =
        gpiod_line_settings_new();

    gpiod_line_config* echoConfig =
        gpiod_line_config_new();


    gpiod_line_settings_set_direction(
        echoSettings,
        GPIOD_LINE_DIRECTION_INPUT
    );

    gpiod_line_config_add_line_settings(
        echoConfig,
        echoOffsets,
        3,
        echoSettings
    );

    gpiod_line_request* echoReq =
        gpiod_chip_request_lines(
            chip,
            nullptr,
            echoConfig
        );

    if (!echoReq)
    {
        cerr << "[ERROR] ECHO request failed\n";
        return 1;
    }


    // ========================================================
    // 반복 측정
    // ========================================================

    while (true)
    {
        double left =
            measureDistance(
                trigReq,
                LEFT_TRIG,
                echoReq,
                LEFT_ECHO
            );

        // 초음파 잔향 방지
        this_thread::sleep_for(
            chrono::milliseconds(50)
        );


        double right =
            measureDistance(
                trigReq,
                RIGHT_TRIG,
                echoReq,
                RIGHT_ECHO
            );

        this_thread::sleep_for(
            chrono::milliseconds(50)
        );


        double bottom =
            measureDistance(
                trigReq,
                BOTTOM_TRIG,
                echoReq,
                BOTTOM_ECHO
            );

        this_thread::sleep_for(
            chrono::milliseconds(50)
        );


        cout
            << "L = " << left << " cm"
            << " | R = " << right << " cm"
            << " | B = " << bottom << " cm"
            << '\n';
    }


    return 0;
}
