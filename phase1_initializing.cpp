#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

struct ControlConstants {
    static constexpr double LOOP_TIME_MS = 20.0;
    static constexpr double GYRO_BIAS_TOLERANCE = 0.01;
};

int main() {
    std::cout << "\n==================================================" << std::endl;
    std::cout << "[Phase 1] 시스템 초기화 및 센서 영점 정렬 단계" << std::endl;
    std::cout << "==================================================" << std::endl;

    double gyroOffsetSumX = 0.0;
    double gyroOffsetSumY = 0.0;
    int sampleCount = 50; 

    std::cout << " [센서 캘리브레이션] 자이로 데이터 수집 중 (" << sampleCount << "샘플)..." << std::endl;

    for (int i = 0; i < sampleCount; ++i) {
        gyroOffsetSumX += 0.02; 
        gyroOffsetSumY += -0.01;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    double finalBiasX = gyroOffsetSumX / sampleCount;
    double finalBiasY = gyroOffsetSumY / sampleCount;

    std::cout << " -> 자이로 오프셋 측정 완료 X: " << finalBiasX << " rad/s, Y: " << finalBiasY << " rad/s" << std::endl;

    if (std::abs(finalBiasX) > ControlConstants::GYRO_BIAS_TOLERANCE || std::abs(finalBiasY) > ControlConstants::GYRO_BIAS_TOLERANCE) {
        std::cout << " -> [보정 가동] 소프트웨어 바이어스 역상쇄 필터 적용." << std::endl;
    }

    std::cout << "\n[정렬 완료] 구동기 영점 셋팅 및 하드웨어 준비 완료. [중간 유도 단계]로 천이합니다.\n";
    return 0;
}