#ifndef MPU6050_H
#define MPU6050_H
#include <cstddef>
#include <cstdint>

/*
 * MPU6050 클래스
 *
 * 역할:
 * - Raspberry Pi의 I2C 장치를 열고 MPU6050과 연결
 * - MPU6050 초기화
 * - 가속도 및 자이로 Raw 데이터 읽기
 *
 * 실제 기준값 계산은 이 클래스에서 하지 않고,
 * 별도의 실행 파일인 imu_bias_measure.cpp에서 수행한다.
 */
class MPU6050
{
public:
    /*
     * 생성자
     *
     * MPU6050 객체가 생성될 때
     * I2C 장치를 열고 센서 주소를 설정한다.
     *
     * 기본 I2C 주소는 0x68이다.
     */
    explicit MPU6050(uint8_t address = 0x68);

    /*
     * 소멸자
     *
     * 객체가 사라질 때 열려 있는 I2C 장치를 닫는다.
     */
    ~MPU6050();

    /*
     * MPU6050 초기화 함수
     *
     * 수행 내용:
     * - 센서 연결 상태 확인
     * - 절전 모드 해제
     * - 자이로 측정 범위 설정
     * - 가속도 측정 범위 설정
     * - 디지털 저역 통과 필터 설정
     *
     * 초기화 성공 시 true,
     * 실패 시 false를 반환한다.
     */
    bool begin();

    /*
     * MPU6050의 가속도와 자이로 Raw 데이터를 읽는다.
     *
     * ax, ay, az:
     * 가속도 센서의 X, Y, Z축 Raw 값
     *
     * gx, gy, gz:
     * 자이로 센서의 X, Y, Z축 Raw 값
     *
     * 읽기 성공 시 true,
     * 실패 시 false를 반환한다.
     */
    bool readRawData(int16_t& ax,
                     int16_t& ay,
                     int16_t& az,
                     int16_t& gx,
                     int16_t& gy,
                     int16_t& gz);

    /*
     * I2C 장치가 정상적으로 열렸는지 확인한다.
     *
     * 정상 상태이면 true,
     * 장치 열기 또는 주소 설정에 실패했다면 false를 반환한다.
     */
    bool isConnected() const;

private:
    /*
     * Linux I2C 장치 파일 디스크립터
     *
     * 정상적으로 열리면 0 이상의 값을 가지며,
     * 실패하거나 닫힌 상태에서는 -1이다.
     */
    int fd;

    /*
     * MPU6050의 I2C 주소
     *
     * 일반적으로 AD0 핀이 GND이면 0x68,
     * VCC이면 0x69이다.
     */
    uint8_t address;

    /*
     * MPU6050의 특정 레지스터에 1바이트 값을 기록한다.
     */
    bool writeRegister(uint8_t reg, uint8_t value);

    /*
     * 특정 레지스터부터 연속된 여러 바이트를 읽는다.
     */
    bool readRegisters(uint8_t startReg,
                       uint8_t* buffer,
                       std::size_t length);

    /*
     * 상위 바이트와 하위 바이트를 결합하여
     * 부호 있는 16비트 센서값으로 변환한다.
     */
    int16_t combineBytes(uint8_t highByte,
                         uint8_t lowByte) const;
};

#endif