/*
 * ============================================================
 * File Name : imu_reference_test.cpp
 *
 * Description
 * ------------------------------------------------------------
 * imu_bias_measure.cpp에서 측정한 MPU6050 자이로 기준값을
 * 실제 센서 데이터에 적용하여 보정 결과를 검증하는 프로그램이다.
 *
 * 주요 기능
 * 1. MPU6050 자체 작성 라이브러리를 이용하여 센서를 초기화한다.
 * 2. 센서의 가속도 및 자이로 Raw 데이터를 반복 측정한다.
 * 3. 사전에 측정한 자이로 Bias를 현재 Raw 데이터에서 제거한다.
 * 4. 보정된 자이로 값을 deg/s 단위로 변환한다.
 * 5. 가속도 값을 이용하여 Pitch와 Roll을 계산한다.
 * 6. 정지 상태에서 자이로 값이 0 deg/s 근처인지 확인한다.
 *
 * Project
 * ------------------------------------------------------------
 * Mass Shift Guidance Control System
 * ============================================================
 */

#include "MPU6050.h"

#include <chrono>   // std::chrono::seconds, milliseconds
#include <cmath>    // std::atan2, std::sqrt
#include <cstdint>  // std::int16_t
#include <iomanip>  // std::fixed, std::setprecision
#include <iostream> // std::cout, std::cerr
#include <thread>   // std::this_thread::sleep_for

/*
 * imu_bias_measure.cpp를 이용해 측정한
 * 자이로 Raw 기준값이다.
 *
 * 센서가 정지한 상태에서도 출력되는 고유 오차이며,
 * 현재 측정값에서 이 값을 빼서 보정한다.
 */
constexpr double GYRO_X_BIAS = 76.09;
constexpr double GYRO_Y_BIAS = -94.12;
constexpr double GYRO_Z_BIAS = -19.14;

/*
 * 가속도 센서 범위가 ±2g일 때의 감도이다.
 *
 * Raw 값 16384가 약 1g에 해당한다.
 */
constexpr double ACCEL_SCALE = 16384.0;

/*
 * 자이로 센서 범위가 ±250 deg/s일 때의 감도이다.
 *
 * Raw 값 131이 약 1 deg/s에 해당한다.
 */
constexpr double GYRO_SCALE = 131.0;

/*
 * 검증에 사용할 센서 데이터 개수
 *
 * 샘플 수가 많을수록 평균 결과가 안정적이지만
 * 전체 측정 시간은 길어진다.
 */
constexpr int SAMPLE_COUNT = 1000;

/*
 * 각 센서 측정 사이의 대기시간
 *
 * 너무 빠르게 읽으면 동일한 데이터가 반복될 수 있으므로
 * 2ms의 짧은 지연을 둔다.
 */
constexpr int SAMPLE_DELAY_MS = 2;

/*
 * 측정 시작 전 대기시간
 *
 * 센서를 평평한 곳에 놓고 손을 뗀 뒤
 * 진동이 안정될 시간을 확보한다.
 */
constexpr int START_DELAY_SECONDS = 3;

/*
 * 라디안 값을 도 단위로 변환하기 위한 상수
 */
constexpr double RAD_TO_DEG =
    180.0 / 3.14159265358979323846;

int main()
{
    /*
     * MPU6050 객체 생성
     *
     * 실제 I2C 통신과 레지스터 처리는
     * MPU6050.cpp 내부에서 수행된다.
     */
    MPU6050 imu;

    /*
     * MPU6050 센서 초기화
     */
    if (!imu.begin())
    {
        std::cerr << "MPU6050 초기화 실패\n";
        return 1;
    }

    std::cout
        << "========================================\n"
        << "      MPU6050 기준값 검증 프로그램\n"
        << "========================================\n"
        << "센서를 움직이지 말고 평평한 곳에 놓으십시오.\n"
        << START_DELAY_SECONDS
        << "초 후 측정을 시작합니다.\n"
        << "========================================\n";

    /*
     * 센서를 놓고 손을 뗄 수 있도록 대기한다.
     */
    std::this_thread::sleep_for(
        std::chrono::seconds(START_DELAY_SECONDS));

    /*
     * 평균 계산을 위한 누적 변수
     *
     * 자이로는 Bias를 제거한 뒤 deg/s 단위로 누적한다.
     * Pitch와 Roll은 가속도 데이터를 이용해 계산한 뒤 누적한다.
     */
    double correctedGyroXSum = 0.0;
    double correctedGyroYSum = 0.0;
    double correctedGyroZSum = 0.0;

    double pitchSum = 0.0;
    double rollSum  = 0.0;

    std::cout << "\n검증 측정을 시작합니다.\n";

    /*
     * 설정한 횟수만큼 MPU6050 데이터를 측정한다.
     */
    for (int sample = 0; sample < SAMPLE_COUNT; sample++)
    {
        /*
         * MPU6050에서 읽어올 Raw 데이터
         *
         * ax, ay, az : 가속도 Raw 데이터
         * gx, gy, gz : 자이로 Raw 데이터
         */
        std::int16_t ax, ay, az;
        std::int16_t gx, gy, gz;

        /*
         * MPU6050.cpp에 구현된 readRawData()를 이용해
         * 가속도와 자이로 값을 한 번에 읽는다.
         */
        if (!imu.readRawData(ax, ay, az, gx, gy, gz))
        {
            std::cerr
                << sample + 1
                << "번째 센서 데이터 읽기 실패\n";

            return 1;
        }

        /*
         * 가속도 Raw 값을 g 단위로 변환한다.
         */
        const double accelX =
            static_cast<double>(ax) / ACCEL_SCALE;

        const double accelY =
            static_cast<double>(ay) / ACCEL_SCALE;

        const double accelZ =
            static_cast<double>(az) / ACCEL_SCALE;

        /*
         * 자이로 Raw 데이터에서 사전에 측정한 Bias를 제거한다.
         *
         * 이후 GYRO_SCALE로 나누어
         * deg/s 단위의 각속도로 변환한다.
         *
         * 보정 각속도 =
         * (현재 Raw 값 - 기준 Raw 값) / 감도
         */
        const double correctedGyroX =
            (static_cast<double>(gx) - GYRO_X_BIAS)
            / GYRO_SCALE;

        const double correctedGyroY =
            (static_cast<double>(gy) - GYRO_Y_BIAS)
            / GYRO_SCALE;

        const double correctedGyroZ =
            (static_cast<double>(gz) - GYRO_Z_BIAS)
            / GYRO_SCALE;

        /*
         * 가속도 센서가 측정한 중력 방향을 이용하여
         * Pitch 각도를 계산한다.
         *
         * 센서의 X축이 기울어질 때 값이 변한다.
         */
        const double pitch =
            std::atan2(
                -accelX,
                std::sqrt(
                    accelY * accelY +
                    accelZ * accelZ))
            * RAD_TO_DEG;

        /*
         * 가속도 센서가 측정한 중력 방향을 이용하여
         * Roll 각도를 계산한다.
         *
         * 센서의 Y축이 기울어질 때 값이 변한다.
         */
        const double roll =
            std::atan2(
                accelY,
                accelZ)
            * RAD_TO_DEG;

        /*
         * 평균 계산을 위해 현재 측정값을 누적한다.
         */
        correctedGyroXSum += correctedGyroX;
        correctedGyroYSum += correctedGyroY;
        correctedGyroZSum += correctedGyroZ;

        pitchSum += pitch;
        rollSum  += roll;

        /*
         * 터미널 출력으로 인한 측정 지연을 줄이기 위해
         * 100회마다 진행 상황을 출력한다.
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
         * 다음 센서 측정 전 잠시 대기한다.
         */
        std::this_thread::sleep_for(
            std::chrono::milliseconds(SAMPLE_DELAY_MS));
    }

    /*
     * 누적한 값을 전체 샘플 개수로 나누어
     * 각 측정값의 평균을 계산한다.
     */
    const double correctedGyroXAverage =
        correctedGyroXSum / SAMPLE_COUNT;

    const double correctedGyroYAverage =
        correctedGyroYSum / SAMPLE_COUNT;

    const double correctedGyroZAverage =
        correctedGyroZSum / SAMPLE_COUNT;

    const double pitchAverage =
        pitchSum / SAMPLE_COUNT;

    const double rollAverage =
        rollSum / SAMPLE_COUNT;

    /*
     * 결과를 소수점 아래 4자리까지 출력한다.
     */
    std::cout
        << std::fixed
        << std::setprecision(4);

    std::cout
        << "\n========================================\n"
        << "             검증 결과\n"
        << "========================================\n"

        << "[보정된 자이로 평균값]\n"
        << "GYRO_X = "
        << correctedGyroXAverage
        << " deg/s\n"

        << "GYRO_Y = "
        << correctedGyroYAverage
        << " deg/s\n"

        << "GYRO_Z = "
        << correctedGyroZAverage
        << " deg/s\n\n"

        << "[가속도 기반 자세 기준값]\n"
        << "PITCH = "
        << pitchAverage
        << " deg\n"

        << "ROLL  = "
        << rollAverage
        << " deg\n"

        << "========================================\n";

    /*
     * 자이로 평균이 0 deg/s에 가까울수록
     * 앞에서 측정한 Bias가 적절하다는 의미이다.
     *
     * Pitch와 Roll은 센서 기판 및 설치면의 미세한 기울기로 인해
     * 완전히 0이 아닐 수 있다.
     */
    std::cout
        << "자이로 값이 0 deg/s 근처인지 확인하십시오.\n"
        << "Pitch와 Roll 값은 초기 자세 기준값으로 사용할 수 있습니다.\n";

    return 0;
}