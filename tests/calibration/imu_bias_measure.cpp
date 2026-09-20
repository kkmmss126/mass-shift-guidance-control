/*
 * ============================================================
 * File Name : imu_bias_measure.cpp
 *
 * Description
 * ------------------------------------------------------------
 * MPU6050 센서의 초기 기준값(Bias)을 측정하기 위한 테스트 프로그램이다.
 *
 * 주요 기능
 * 1. MPU6050 라이브러리를 이용하여 센서를 초기화한다.
 * 2. 일정 시간 동안 센서를 안정화한 후 데이터를 측정한다.
 * 3. 가속도(ACCEL)와 자이로(GYRO) Raw 데이터를 다회 측정한다.
 * 4. 각 축의 평균값을 계산하여 초기 기준값(Bias)을 도출한다.
 * 5. 계산된 자이로 기준값을 메인 프로젝트에서 사용할 수 있도록
 *    constexpr 형식으로 출력한다.
 *
 * 측정된 기준값은 이후 자세 제어 알고리즘에서
 * 센서 오차(Bias) 보정 및 초기 자세 기준값으로 사용한다.
 *
 * Project
 * ------------------------------------------------------------
 * Mass Shift Guidance Control System
 * ============================================================
 */

#include "MPU6050.h"
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <thread>

/*
 * 기준값 계산에 사용할 샘플 개수
 *
 * 샘플 수가 많을수록 평균값은 안정적이지만
 * 측정 시간은 길어진다.
 */
constexpr int SAMPLE_COUNT = 1000;

/*
 * 센서 측정 간격(ms)
 *
 * 너무 빠르게 연속 측정하면 동일한 데이터가
 * 반복될 수 있으므로 짧은 지연을 둔다.
 */
constexpr int SAMPLE_DELAY_MS = 2;

/*
 * 측정 시작 전 대기 시간
 *
 * 센서를 평평한 곳에 놓고
 * 진동이 안정될 시간을 확보한다.
 */
constexpr int START_DELAY_SECONDS = 3;

int main()
{
    /*
     * MPU6050 객체 생성 및 초기화
     */
    MPU6050 imu;

    if (!imu.begin())
    {
        std::cerr << "MPU6050 초기화 실패\n";
        return 1;
    }

    std::cout
        << "========================================\n"
        << "      MPU6050 기준값 측정 프로그램\n"
        << "========================================\n"
        << "센서를 움직이지 말고 평평한 곳에 놓으십시오.\n"
        << START_DELAY_SECONDS
        << "초 후 측정을 시작합니다.\n"
        << "========================================\n";

    /*
     * 센서가 안정될 때까지 대기
     */
    std::this_thread::sleep_for(
        std::chrono::seconds(START_DELAY_SECONDS));

    /*
     * 센서값 누적 변수
     *
     * 1000회 이상 누적되므로
     * 오버플로우를 방지하기 위해 int64_t를 사용한다.
     */
    int64_t accelXSum = 0;
    int64_t accelYSum = 0;
    int64_t accelZSum = 0;

    int64_t gyroXSum = 0;
    int64_t gyroYSum = 0;
    int64_t gyroZSum = 0;

    std::cout << "\n측정을 시작합니다.\n";

    /*
     * 지정한 횟수만큼 센서 데이터를 읽어
     * 각 축의 값을 누적한다.
     */
    for (int sample = 0; sample < SAMPLE_COUNT; sample++)
    {
        int16_t ax, ay, az;
        int16_t gx, gy, gz;

        /*
         * 센서 데이터 읽기
         */
        if (!imu.readRawData(ax, ay, az, gx, gy, gz))
        {
            std::cerr << "센서 데이터 읽기 실패\n";
            return 1;
        }

        /*
         * 평균 계산을 위해 모든 측정값을 누적한다.
         */
        accelXSum += ax;
        accelYSum += ay;
        accelZSum += az;

        gyroXSum += gx;
        gyroYSum += gy;
        gyroZSum += gz;

        /*
         * 진행 상황 출력
         *
         * 너무 자주 출력하면 터미널 출력 속도 때문에
         * 측정 속도가 느려질 수 있으므로
         * 100회마다 한 번씩만 출력한다.
         */
        if ((sample + 1) % 100 == 0)
        {
            std::cout
                << sample + 1
                << " / "
                << SAMPLE_COUNT
                << " 측정 완료\n";
        }

        /*
         * 다음 측정 전 잠시 대기
         */
        std::this_thread::sleep_for(
            std::chrono::milliseconds(SAMPLE_DELAY_MS));
    }

    /*
     * 각 축의 평균값 계산
     *
     * 자이로 평균값은 이후 메인 프로젝트에서
     * 기준값(Bias)으로 사용된다.
     */
    const double accelXAverage =
        static_cast<double>(accelXSum) / SAMPLE_COUNT;

    const double accelYAverage =
        static_cast<double>(accelYSum) / SAMPLE_COUNT;

    const double accelZAverage =
        static_cast<double>(accelZSum) / SAMPLE_COUNT;

    const double gyroXBias =
        static_cast<double>(gyroXSum) / SAMPLE_COUNT;

    const double gyroYBias =
        static_cast<double>(gyroYSum) / SAMPLE_COUNT;

    const double gyroZBias =
        static_cast<double>(gyroZSum) / SAMPLE_COUNT;

    /*
     * 소수 둘째 자리까지 출력
     */
    std::cout << std::fixed << std::setprecision(2);

    std::cout
        << "\n========================================\n"
        << "             측정 결과\n"
        << "========================================\n";

    /*
     * 가속도 평균값 출력
     *
     * 센서를 평평하게 놓았다면
     * X, Y축은 0에 가까운 값,
     * Z축은 ±16384 근처의 값이 출력된다.
     */
    std::cout << "[가속도 평균값]\n";
    std::cout << "ACCEL_X = " << accelXAverage << '\n';
    std::cout << "ACCEL_Y = " << accelYAverage << '\n';
    std::cout << "ACCEL_Z = " << accelZAverage << "\n\n";

    /*
     * 자이로 기준값 출력
     *
     * 메인 프로젝트에서는
     * 현재 측정값에서 아래 값을 빼서 사용한다.
     */
    std::cout << "[자이로 기준값]\n";
    std::cout << "GYRO_X_BIAS = " << gyroXBias << '\n';
    std::cout << "GYRO_Y_BIAS = " << gyroYBias << '\n';
    std::cout << "GYRO_Z_BIAS = " << gyroZBias << "\n\n";

    /*
     * 메인 프로젝트에 바로 복사하여 사용할 수 있도록
     * constexpr 형식으로 출력한다.
     */
    std::cout << "[메인 코드 복사용]\n";
    std::cout << "constexpr double GYRO_X_BIAS = "
              << gyroXBias << ";\n";

    std::cout << "constexpr double GYRO_Y_BIAS = "
              << gyroYBias << ";\n";

    std::cout << "constexpr double GYRO_Z_BIAS = "
              << gyroZBias << ";\n";

    std::cout
        << "========================================\n"
        << "기준값 측정이 완료되었습니다.\n";

    return 0;
}