#include <gpiod.h>
#include <iostream>
#include <chrono>
#include <thread>

/*
 * ============================================================
 * SRF-05 Single Ultrasonic Sensor Test
 * ============================================================
 *
 * 목적
 * ------------------------------------------------------------
 * 기존 seeker_tracking_test.cpp와 완전히 분리하여
 * 왼쪽 SRF-05 한 개만 단독으로 테스트한다.
 *
 * 핀 구성 (BCM GPIO 번호)
 * ------------------------------------------------------------
 * TRIG : GPIO5
 * ECHO : GPIO20
 *
 * 동작 과정
 * ------------------------------------------------------------
 * 1. TRIG를 LOW로 유지
 * 2. TRIG에 약 10us HIGH 펄스 출력
 * 3. ECHO가 HIGH가 될 때까지 대기
 * 4. ECHO HIGH 유지시간 측정
 * 5. 펄스 폭으로 거리 계산
 *
 * 거리 계산
 * ------------------------------------------------------------
 * 음속 ≈ 343 m/s = 0.0343 cm/us
 *
 * 거리 = ECHO HIGH 시간 × 0.0343 / 2
 *
 * ============================================================
 */

constexpr unsigned int TRIG_PIN = 16;
constexpr unsigned int ECHO_PIN = 6;

/*
 * ECHO가 들어오지 않을 경우 무한 대기하지 않도록
 * 최대 대기시간을 30ms로 제한한다.
 *
 * SRF-05의 일반적인 측정 범위를 고려하면 충분한 시간이다.
 */
constexpr long TIMEOUT_US = 30000;


/*
 * 현재 시각을 microsecond 단위로 반환한다.
 */
long long nowMicroseconds()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count();
}


int main()
{
    std::cout << "========================================\n";
    std::cout << " SRF-05 Single Sensor Test\n";
    std::cout << " TRIG : GPIO" << TRIG_PIN << "\n";
    std::cout << " ECHO : GPIO" << ECHO_PIN << "\n";
    std::cout << " Ctrl+C : 종료\n";
    std::cout << "========================================\n\n";


    /*
     * Raspberry Pi GPIO chip 열기
     */
    gpiod_chip* chip = gpiod_chip_open("/dev/gpiochip0");

    if (!chip)
    {
        std::cerr << "[ERROR] gpiochip0 open failed\n";
        return 1;
    }


    /*
     * ========================================================
     * TRIG GPIO 설정
     * ========================================================
     */

    gpiod_line_settings* trigSettings = gpiod_line_settings_new();
    gpiod_line_config* trigConfig = gpiod_line_config_new();

    if (!trigSettings || !trigConfig)
    {
        std::cerr << "[ERROR] TRIG config allocation failed\n";
        return 1;
    }

    /*
     * TRIG는 출력으로 설정한다.
     */
    gpiod_line_settings_set_direction(
        trigSettings,
        GPIOD_LINE_DIRECTION_OUTPUT
    );

    /*
     * 시작할 때 TRIG = LOW
     */
    gpiod_line_settings_set_output_value(
        trigSettings,
        GPIOD_LINE_VALUE_INACTIVE
    );

    unsigned int trigOffset = TRIG_PIN;

    gpiod_line_config_add_line_settings(
        trigConfig,
        &trigOffset,
        1,
        trigSettings
    );

    gpiod_line_request* trigRequest =
        gpiod_chip_request_lines(chip, nullptr, trigConfig);

    if (!trigRequest)
    {
        std::cerr << "[ERROR] TRIG GPIO request failed\n";
        return 1;
    }


    /*
     * ========================================================
     * ECHO GPIO 설정
     * ========================================================
     */

    gpiod_line_settings* echoSettings = gpiod_line_settings_new();
    gpiod_line_config* echoConfig = gpiod_line_config_new();

    if (!echoSettings || !echoConfig)
    {
        std::cerr << "[ERROR] ECHO config allocation failed\n";
        return 1;
    }

    /*
     * ECHO는 입력으로 설정한다.
     */
    gpiod_line_settings_set_direction(
        echoSettings,
        GPIOD_LINE_DIRECTION_INPUT
    );

    unsigned int echoOffset = ECHO_PIN;

    gpiod_line_config_add_line_settings(
        echoConfig,
        &echoOffset,
        1,
        echoSettings
    );

    gpiod_line_request* echoRequest =
        gpiod_chip_request_lines(chip, nullptr, echoConfig);

    if (!echoRequest)
    {
        std::cerr << "[ERROR] ECHO GPIO request failed\n";
        return 1;
    }


    /*
     * 센서 단독 반복 측정
     */
    while (true)
    {
        /*
         * ----------------------------------------------------
         * 1. TRIG LOW
         * ----------------------------------------------------
         */
        gpiod_line_request_set_value(
            trigRequest,
            TRIG_PIN,
            GPIOD_LINE_VALUE_INACTIVE
        );

        std::this_thread::sleep_for(
            std::chrono::microseconds(2)
        );


        /*
         * ----------------------------------------------------
         * 2. TRIG HIGH 약 10us
         * ----------------------------------------------------
         */
        gpiod_line_request_set_value(
            trigRequest,
            TRIG_PIN,
            GPIOD_LINE_VALUE_ACTIVE
        );

        std::this_thread::sleep_for(
            std::chrono::microseconds(10)
        );


        /*
         * 다시 LOW
         */
        gpiod_line_request_set_value(
            trigRequest,
            TRIG_PIN,
            GPIOD_LINE_VALUE_INACTIVE
        );


        /*
         * ----------------------------------------------------
         * 3. ECHO HIGH 대기
         * ----------------------------------------------------
         */

        long long waitStart = nowMicroseconds();

        while (
            gpiod_line_request_get_value(
                echoRequest,
                ECHO_PIN
            ) == GPIOD_LINE_VALUE_INACTIVE
        )
        {
            if (nowMicroseconds() - waitStart > TIMEOUT_US)
            {
                std::cout
                    << "[TIMEOUT] ECHO never went HIGH\n";

                goto measurement_end;
            }
        }


        /*
         * ECHO가 HIGH가 된 시점
         */
        {
            long long echoStart = nowMicroseconds();


            /*
             * ------------------------------------------------
             * 4. ECHO LOW 대기
             * ------------------------------------------------
             */

            while (
                gpiod_line_request_get_value(
                    echoRequest,
                    ECHO_PIN
                ) == GPIOD_LINE_VALUE_ACTIVE
            )
            {
                if (nowMicroseconds() - echoStart > TIMEOUT_US)
                {
                    std::cout
                        << "[TIMEOUT] ECHO stayed HIGH\n";

                    goto measurement_end;
                }
            }


            /*
             * ECHO가 LOW로 내려간 시점
             */
            long long echoEnd = nowMicroseconds();


            /*
             * HIGH 펄스 폭
             */
            long long pulseWidth =
                echoEnd - echoStart;


            /*
             * 거리 계산
             *
             * 0.0343 cm/us : 음속
             * 왕복 거리이므로 2로 나눈다.
             */
            double distance =
                pulseWidth * 0.0343 / 2.0;


            std::cout
                << "Pulse = "
                << pulseWidth
                << " us"
                << " | Distance = "
                << distance
                << " cm\n";
        }


measurement_end:

        /*
         * 초음파 잔향 및 다음 측정을 고려해서
         * 100ms 간격으로 천천히 측정한다.
         *
         * 단독 센서 테스트이므로 간섭 가능성을 최대한 제거한다.
         */
        std::this_thread::sleep_for(
            std::chrono::milliseconds(100)
        );
    }


    /*
     * Ctrl+C로 종료되므로 일반적으로 아래까지 도달하지 않는다.
     */
    gpiod_line_request_release(echoRequest);
    gpiod_line_request_release(trigRequest);

    gpiod_line_config_free(echoConfig);
    gpiod_line_settings_free(echoSettings);

    gpiod_line_config_free(trigConfig);
    gpiod_line_settings_free(trigSettings);

    gpiod_chip_close(chip);

    return 0;
}
