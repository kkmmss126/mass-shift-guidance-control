#include "MPU6050.h"

#include <iostream>

#include <fcntl.h>          // open()
#include <unistd.h>         // close(), read(), write()
#include <sys/ioctl.h>      // ioctl()
#include <linux/i2c-dev.h>  // I2C_SLAVE

/*
 * MPU6050 내부 레지스터 주소
 */
constexpr uint8_t PWR_MGMT_1   = 0x6B;
constexpr uint8_t SMPLRT_DIV   = 0x19;
constexpr uint8_t CONFIG       = 0x1A;
constexpr uint8_t GYRO_CONFIG  = 0x1B;
constexpr uint8_t ACCEL_CONFIG = 0x1C;
constexpr uint8_t ACCEL_XOUT_H = 0x3B;
constexpr uint8_t WHO_AM_I     = 0x75;


/*
 * MPU6050 생성자
 *
 * Raspberry Pi의 I2C 장치 파일을 열고,
 * 통신 대상 주소를 MPU6050 주소로 설정한다.
 */
MPU6050::MPU6050(uint8_t address)
    : fd(-1),
      address(address)
{
    /*
     * Raspberry Pi에서는 일반적으로
     * /dev/i2c-1 장치를 사용한다.
     */
    fd = open("/dev/i2c-1", O_RDWR);

    if (fd < 0)
    {
        std::cerr << "I2C 장치를 열 수 없습니다.\n";
        return;
    }

    /*
     * 현재 I2C 통신 대상 장치를
     * MPU6050 주소로 설정한다.
     */
    if (ioctl(fd, I2C_SLAVE, address) < 0)
    {
        std::cerr << "MPU6050 I2C 주소 설정 실패\n";

        close(fd);
        fd = -1;

        return;
    }
}


/*
 * MPU6050 소멸자
 *
 * 객체가 사라질 때 열려 있는
 * I2C 장치 파일을 닫는다.
 */
MPU6050::~MPU6050()
{
    if (fd >= 0)
    {
        close(fd);
    }
}


/*
 * I2C 연결 상태 확인
 *
 * fd가 0 이상이면 I2C 장치가 정상적으로 열린 상태이다.
 */
bool MPU6050::isConnected() const
{
    return fd >= 0;
}


/*
 * MPU6050 특정 레지스터에 1바이트 값을 기록한다.
 */
bool MPU6050::writeRegister(uint8_t reg, uint8_t value)
{
    /*
     * buffer[0]에는 레지스터 주소,
     * buffer[1]에는 기록할 값을 저장한다.
     */
    uint8_t buffer[2] = {reg, value};

    /*
     * 정확히 2바이트가 기록되어야 성공이다.
     */
    if (write(fd, buffer, 2) != 2)
    {
        std::cerr << "MPU6050 레지스터 기록 실패\n";
        return false;
    }

    return true;
}


/*
 * 특정 레지스터부터 연속된 여러 바이트를 읽는다.
 */
bool MPU6050::readRegisters(uint8_t startReg,
                            uint8_t* buffer,
                            std::size_t length)
{
    /*
     * 먼저 읽기를 시작할 레지스터 주소를
     * MPU6050에 전달한다.
     */
    if (write(fd, &startReg, 1) != 1)
    {
        std::cerr << "MPU6050 레지스터 주소 전송 실패\n";
        return false;
    }

    /*
     * 지정된 바이트 수만큼 데이터를 읽는다.
     */
    if (read(fd, buffer, length) != static_cast<ssize_t>(length))
    {
        std::cerr << "MPU6050 데이터 읽기 실패\n";
        return false;
    }

    return true;
}


/*
 * 상위 바이트와 하위 바이트를 결합하여
 * 부호 있는 16비트 센서값으로 변환한다.
 *
 * MPU6050은 센서값을 2바이트로 나누어 저장한다.
 */
int16_t MPU6050::combineBytes(uint8_t highByte,
                              uint8_t lowByte) const
{
    return static_cast<int16_t>(
        (static_cast<uint16_t>(highByte) << 8) |
        static_cast<uint16_t>(lowByte)
    );
}


/*
 * MPU6050 초기화
 */
bool MPU6050::begin()
{
    /*
     * 생성자에서 I2C 장치 열기에 실패했다면
     * 초기화를 진행할 수 없다.
     */
    if (!isConnected())
    {
        std::cerr << "MPU6050이 I2C에 연결되지 않았습니다.\n";
        return false;
    }

    /*
     * WHO_AM_I 레지스터를 읽어
     * 실제 MPU6050이 연결되어 있는지 확인한다.
     */
    uint8_t whoAmI = 0;

    if (!readRegisters(WHO_AM_I, &whoAmI, 1))
    {
        return false;
    }

    /*
     * 일반적인 MPU6050의 WHO_AM_I 값은 0x68이다.
     */
    if (whoAmI != 0x68)
    {
        std::cerr << "MPU6050 장치 확인 실패\n";
        std::cerr << "WHO_AM_I 값: 0x"
                  << std::hex
                  << static_cast<int>(whoAmI)
                  << std::dec
                  << '\n';

        return false;
    }

    /*
     * MPU6050은 초기 상태에서 절전 모드이다.
     *
     * PWR_MGMT_1에 0x00을 기록하여
     * 절전 모드를 해제한다.
     */
    if (!writeRegister(PWR_MGMT_1, 0x00))
    {
        return false;
    }

    /*
     * 샘플링 속도 분주값 설정
     *
     * 현재 기준값 측정 용도에서는
     * 일반적인 설정값인 0x07을 사용한다.
     */
    if (!writeRegister(SMPLRT_DIV, 0x07))
    {
        return false;
    }

    /*
     * 디지털 저역 통과 필터 설정
     *
     * 0x03은 고주파 노이즈를 어느 정도 줄이면서
     * 응답 속도도 확보하는 설정이다.
     */
    if (!writeRegister(CONFIG, 0x03))
    {
        return false;
    }

    /*
     * 자이로 측정 범위를 ±250 deg/s로 설정한다.
     *
     * 이 범위에서 자이로 감도는
     * 131 LSB/(deg/s)이다.
     */
    if (!writeRegister(GYRO_CONFIG, 0x00))
    {
        return false;
    }

    /*
     * 가속도 측정 범위를 ±2g로 설정한다.
     *
     * 이 범위에서 가속도 감도는
     * 16384 LSB/g이다.
     */
    if (!writeRegister(ACCEL_CONFIG, 0x00))
    {
        return false;
    }

    return true;
}


/*
 * MPU6050의 가속도 및 자이로 Raw 데이터 읽기
 */
bool MPU6050::readRawData(int16_t& ax,
                          int16_t& ay,
                          int16_t& az,
                          int16_t& gx,
                          int16_t& gy,
                          int16_t& gz)
{
    /*
     * 0x3B부터 14바이트를 연속으로 읽는다.
     *
     * 데이터 순서:
     * 0~1   : 가속도 X
     * 2~3   : 가속도 Y
     * 4~5   : 가속도 Z
     * 6~7   : 온도
     * 8~9   : 자이로 X
     * 10~11 : 자이로 Y
     * 12~13 : 자이로 Z
     */
    uint8_t buffer[14];

    if (!readRegisters(ACCEL_XOUT_H,
                       buffer,
                       sizeof(buffer)))
    {
        return false;
    }

    /*
     * 가속도 Raw 데이터 변환
     */
    ax = combineBytes(buffer[0], buffer[1]);
    ay = combineBytes(buffer[2], buffer[3]);
    az = combineBytes(buffer[4], buffer[5]);

    /*
     * buffer[6], buffer[7]은 온도 데이터이므로
     * 현재 함수에서는 사용하지 않는다.
     */

    /*
     * 자이로 Raw 데이터 변환
     */
    gx = combineBytes(buffer[8], buffer[9]);
    gy = combineBytes(buffer[10], buffer[11]);
    gz = combineBytes(buffer[12], buffer[13]);

    return true;
}