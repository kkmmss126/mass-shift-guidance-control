#include <gpiod.h>
#include <iostream>
#include <unistd.h>

/*
 * ============================================================
 * Body Yaw Stepper Motor Test
 * ============================================================
 *
 * 목적:
 *   동체 좌우(Yaw) 제어용 스텝모터의 단독 동작 확인
 *
 * A4988 연결:
 *   DIR  -> GPIO26
 *   STEP -> GPIO19
 *   ENABLE -> GND 고정
 *
 * 동작:
 *   1. 정방향 500 step
 *   2. 1초 정지
 *   3. 역방향 500 step
 *
 * libgpiod 2.x API 기준
 * ============================================================
 */

constexpr unsigned int DIR_PIN  = 26;
constexpr unsigned int STEP_PIN = 19;

/* 테스트 이동량 */
constexpr int STEP_COUNT = 500;

/*
 * STEP 펄스 시간
 *
 * HIGH 1000 us + LOW 1000 us
 * → 한 스텝당 약 2 ms
 *
 * 최초 테스트이므로 비교적 천천히 구동한다.
 */
constexpr int STEP_DELAY_US = 1000;


/*
 * ============================================================
 * moveStepper()
 *
 * direction = true  : 정방향
 * direction = false : 역방향
 *
 * 지정된 횟수만큼 STEP 펄스를 발생시킨다.
 * ============================================================
 */
void moveStepper(gpiod_line_request* request,
                 bool direction,
                 int steps)
{
    /*
     * DIR 핀 설정
     */
    gpiod_line_request_set_value(
        request,
        DIR_PIN,
        direction ? GPIOD_LINE_VALUE_ACTIVE
                  : GPIOD_LINE_VALUE_INACTIVE
    );

    /*
     * 방향 변경 후 드라이버 안정화 시간
     */
    usleep(1000);


    /*
     * STEP 펄스 발생
     */
    for (int i = 0; i < steps; ++i)
    {
        /*
         * STEP HIGH
         */
        gpiod_line_request_set_value(
            request,
            STEP_PIN,
            GPIOD_LINE_VALUE_ACTIVE
        );

        usleep(STEP_DELAY_US);


        /*
         * STEP LOW
         */
        gpiod_line_request_set_value(
            request,
            STEP_PIN,
            GPIOD_LINE_VALUE_INACTIVE
        );

        usleep(STEP_DELAY_US);
    }
}


int main()
{
    /*
     * ========================================================
     * GPIO Chip 열기
     * Raspberry Pi 4 기준 /dev/gpiochip0
     * ========================================================
     */
    gpiod_chip* chip = gpiod_chip_open("/dev/gpiochip0");

    if (!chip)
    {
        std::cerr << "[ERROR] gpiochip0 open failed\n";
        return 1;
    }


    /*
     * ========================================================
     * GPIO 출력 설정 생성
     * ========================================================
     */
    gpiod_line_settings* settings = gpiod_line_settings_new();

    if (!settings)
    {
        std::cerr << "[ERROR] line settings creation failed\n";

        gpiod_chip_close(chip);
        return 1;
    }


    /*
     * GPIO를 OUTPUT으로 설정
     */
    gpiod_line_settings_set_direction(
        settings,
        GPIOD_LINE_DIRECTION_OUTPUT
    );


    /*
     * 초기 GPIO 출력 LOW
     */
    gpiod_line_settings_set_output_value(
        settings,
        GPIOD_LINE_VALUE_INACTIVE
    );


    /*
     * ========================================================
     * 사용할 GPIO 등록
     *
     * GPIO26 : DIR
     * GPIO19 : STEP
     * ========================================================
     */
    unsigned int offsets[] =
    {
        DIR_PIN,
        STEP_PIN
    };


    gpiod_line_config* lineConfig =
        gpiod_line_config_new();

    if (!lineConfig)
    {
        std::cerr << "[ERROR] line config creation failed\n";

        gpiod_line_settings_free(settings);
        gpiod_chip_close(chip);

        return 1;
    }


    if (gpiod_line_config_add_line_settings(
            lineConfig,
            offsets,
            2,
            settings) < 0)
    {
        std::cerr << "[ERROR] line settings configuration failed\n";

        gpiod_line_config_free(lineConfig);
        gpiod_line_settings_free(settings);
        gpiod_chip_close(chip);

        return 1;
    }


    /*
     * ========================================================
     * GPIO 요청 설정
     * ========================================================
     */
    gpiod_request_config* requestConfig =
        gpiod_request_config_new();


    gpiod_request_config_set_consumer(
        requestConfig,
        "body_yaw_stepper_test"
    );


    /*
     * 실제 GPIO 사용 요청
     */
    gpiod_line_request* request =
        gpiod_chip_request_lines(
            chip,
            requestConfig,
            lineConfig
        );


    if (!request)
    {
        std::cerr << "[ERROR] GPIO request failed\n";

        gpiod_request_config_free(requestConfig);
        gpiod_line_config_free(lineConfig);
        gpiod_line_settings_free(settings);
        gpiod_chip_close(chip);

        return 1;
    }


    /*
     * ========================================================
     * 스텝모터 테스트 시작
     * ========================================================
     */

    std::cout << "\n==============================\n";
    std::cout << " Body Yaw Stepper Motor Test\n";
    std::cout << " DIR  GPIO : " << DIR_PIN << "\n";
    std::cout << " STEP GPIO : " << STEP_PIN << "\n";
    std::cout << "==============================\n\n";


    /*
     * 정방향
     */
    std::cout << "[FORWARD] 500 steps\n";

    moveStepper(
        request,
        true,
        STEP_COUNT
    );


    /*
     * 잠시 정지
     */
    sleep(1);


    /*
     * 역방향
     */
    std::cout << "[REVERSE] 500 steps\n";

    moveStepper(
        request,
        false,
        STEP_COUNT
    );


    std::cout << "\n[COMPLETE] Test finished\n";


    /*
     * ========================================================
     * 자원 해제
     * ========================================================
     */
    gpiod_line_request_release(request);

    gpiod_request_config_free(requestConfig);
    gpiod_line_config_free(lineConfig);
    gpiod_line_settings_free(settings);

    gpiod_chip_close(chip);


    return 0;
}