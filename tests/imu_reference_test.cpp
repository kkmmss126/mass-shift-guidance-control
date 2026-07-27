#include <iostream>      // std::cout, std::cerr 출력
#include <iomanip>       // std::fixed, std::setprecision
#include <cmath>         // std::atan2, std::sqrt
#include <cstdint>       // std::uint8_t, std::int16_t
#include <thread>        // std::this_thread::sleep_for
#include <chrono>        // std::chrono::milliseconds

#include <fcntl.h>       // open(), O_RDWR
#include <unistd.h>      // close()
#include <sys/ioctl.h>   // ioctl()
#include <linux/i2c-dev.h> // I2C_SLAVE
#include <i2c/smbus.h>   // i2c_smbus_read/write 함수

// MPU6050 관련 주소와 변환 상수
namespace MPU6050
{
    // AD0 핀이 LOW일 때 기본 I2C 주소
    constexpr int ADDRESS = 0x68;

    // MPU6050 내부 레지스터 주소
    constexpr std::uint8_t PWR_MGMT_1   = 0x6B;
    constexpr std::uint8_t WHO_AM_I     = 0x75;
    constexpr std::uint8_t ACCEL_XOUT_H = 0x3B;

    // 초기 설정 기준 감도
    // 가속도 범위 ±2g
    constexpr double ACCEL_SCALE = 16384.0;

    // 자이로 범위 ±250 deg/s
    constexpr double GYRO_SCALE = 131.0;
}

// 기준값 측정 횟수
constexpr int SAMPLE_COUNT = 200;

// 각 측정 사이 대기시간
constexpr int SAMPLE_INTERVAL_MS = 5;

// 라디안 값을 도 단위로 변환하기 위한 상수
constexpr double RAD_TO_DEG =
    180.0 / 3.14159265358979323846;

// MPU6050이 전달하는 상위 바이트와 하위 바이트를
// 하나의 signed 16비트 값으로 결합
std::int16_t combineBytes(
    std::uint8_t highByte,
    std::uint8_t lowByte
)
{
    return static_cast<std::int16_t>(
        (static_cast<std::uint16_t>(highByte) << 8) |
        static_cast<std::uint16_t>(lowByte)
    );
}

int main()
{
    // 라즈베리파이의 기본 I2C 버스 파일 열기
    int i2cFile = open("/dev/i2c-1", O_RDWR);

    // I2C 버스를 열지 못한 경우 프로그램 종료
    if (i2cFile < 0)
    {
        std::cerr
            << "오류: /dev/i2c-1을 열 수 없습니다.\n"
            << "I2C 활성화 상태를 확인하세요.\n";

        return 1;
    }

    // 현재 통신 대상으로 MPU6050 주소 0x68 지정
    if (ioctl(
            i2cFile,
            I2C_SLAVE,
            MPU6050::ADDRESS
        ) < 0)
    {
        std::cerr
            << "오류: MPU6050 주소 0x68을 선택할 수 없습니다.\n";

        close(i2cFile);
        return 1;
    }

    // WHO_AM_I 레지스터를 읽어서 센서가 응답하는지 확인
    int whoAmI = i2c_smbus_read_byte_data(
        i2cFile,
        MPU6050::WHO_AM_I
    );

    if (whoAmI < 0)
    {
        std::cerr
            << "오류: WHO_AM_I 레지스터를 읽지 못했습니다.\n";

        close(i2cFile);
        return 1;
    }

    std::cout
        << "WHO_AM_I: 0x"
        << std::hex
        << whoAmI
        << std::dec
        << '\n';

    // MPU6050의 기본 절전 모드 해제
    if (i2c_smbus_write_byte_data(
            i2cFile,
            MPU6050::PWR_MGMT_1,
            0x00
        ) < 0)
    {
        std::cerr
            << "오류: MPU6050 절전 모드를 해제하지 못했습니다.\n";

        close(i2cFile);
        return 1;
    }

    // 절전 모드 해제 직후 센서 안정화 대기
    std::this_thread::sleep_for(
        std::chrono::milliseconds(100)
    );

    // 200회 측정값 누적용 변수
    double gyroXSum = 0.0;
    double gyroYSum = 0.0;
    double pitchSum = 0.0;
    double rollSum  = 0.0;

    std::cout
        << "\n========================================\n"
        << "MPU6050 기준값 측정을 시작합니다.\n"
        << "센서를 지면과 평행하게 놓고 움직이지 마세요.\n"
        << "측정 횟수: " << SAMPLE_COUNT << "회\n"
        << "========================================\n";

    // 설정한 횟수만큼 실제 센서값 측정
    for (int i = 0; i < SAMPLE_COUNT; ++i)
    {
        // MPU6050의 가속도, 온도, 자이로 데이터를
        // 한 번에 읽기 위한 14바이트 버퍼
        std::uint8_t buffer[14]{};

        // ACCEL_XOUT_H부터 연속 14바이트 읽기
        int bytesRead = i2c_smbus_read_i2c_block_data(
            i2cFile,
            MPU6050::ACCEL_XOUT_H,
            sizeof(buffer),
            buffer
        );

        // 14바이트를 전부 읽지 못하면 측정 실패
        if (bytesRead != static_cast<int>(sizeof(buffer)))
        {
            std::cerr
                << "오류: "
                << i + 1
                << "번째 센서값을 읽지 못했습니다.\n";

            close(i2cFile);
            return 1;
        }

        // 가속도 Raw 데이터 변환
        std::int16_t rawAccelX =
            combineBytes(buffer[0], buffer[1]);

        std::int16_t rawAccelY =
            combineBytes(buffer[2], buffer[3]);

        std::int16_t rawAccelZ =
            combineBytes(buffer[4], buffer[5]);

        // 자이로 Raw 데이터 변환
        // buffer[6], buffer[7]은 온도 데이터이므로 건너뜀
        std::int16_t rawGyroX =
            combineBytes(buffer[8], buffer[9]);

        std::int16_t rawGyroY =
            combineBytes(buffer[10], buffer[11]);

        // 가속도 Raw 값을 g 단위로 변환
        double accelX =
            rawAccelX / MPU6050::ACCEL_SCALE;

        double accelY =
            rawAccelY / MPU6050::ACCEL_SCALE;

        double accelZ =
            rawAccelZ / MPU6050::ACCEL_SCALE;

        // 자이로 Raw 값을 deg/s 단위로 변환
        double gyroX =
            rawGyroX / MPU6050::GYRO_SCALE;

        double gyroY =
            rawGyroY / MPU6050::GYRO_SCALE;

        // 중력 방향을 이용해 Pitch 계산
        double pitch = std::atan2(
            -accelX,
            std::sqrt(
                accelY * accelY +
                accelZ * accelZ
            )
        ) * RAD_TO_DEG;

        // 중력 방향을 이용해 Roll 계산
        double roll = std::atan2(
            accelY,
            accelZ
        ) * RAD_TO_DEG;

        // 현재 측정값 누적
        gyroXSum += gyroX;
        gyroYSum += gyroY;
        pitchSum += pitch;
        rollSum  += roll;

        // 다음 측정 전 5ms 대기
        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                SAMPLE_INTERVAL_MS
            )
        );
    }

    // 200회 측정값의 평균 계산
    const double gyroXAverage =
        gyroXSum / SAMPLE_COUNT;

    const double gyroYAverage =
        gyroYSum / SAMPLE_COUNT;

    const double pitchAverage =
        pitchSum / SAMPLE_COUNT;

    const double rollAverage =
        rollSum / SAMPLE_COUNT;

    // 소수점 아래 6자리까지 출력
    std::cout
        << std::fixed
        << std::setprecision(6);

    // 최종 기준값 출력
    std::cout
        << "\n========== 기준값 측정 결과 ==========\n"
        << "Gyro X 평균 : "
        << gyroXAverage
        << " deg/s\n"

        << "Gyro Y 평균 : "
        << gyroYAverage
        << " deg/s\n"

        << "Pitch 평균  : "
        << pitchAverage
        << " deg\n"

        << "Roll 평균   : "
        << rollAverage
        << " deg\n"

        << "======================================\n";

    // I2C 장치 파일 닫기
    close(i2cFile);

    return 0;
}