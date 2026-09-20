

붙여넣은 코드(1)(20260917-052825).cpp
파일
이거 일거야 아마도 cpp도 새로 만들어야 해? 

좋아. V1 로직은 건드리지 않고 라이브러리 구조로만 분리해놨어. 기준은 네가 올린 동결 V1 그대로야. 


파일은 3개야.

SeekerControl.h

SeekerControl.cpp

seeker_control_test.cpp

프로젝트에는 이렇게 넣으면 돼.

include/SeekerControl.h
src/SeekerControl.cpp
tests/seeker_control_test.cpp
이번에 한 일은 간단히 말하면, 기존 seeker_tracking_test.cpp의 while(true) 내부를 SeekerControl::update() 1회 동작으로 바꾼 거야. 그래서 메인이나 테스트에서는 이제:

SeekerControl seeker;

seeker.initialize();

while (true)
{
    seeker.update();
}
이렇게만 쓰면 기존 V1 추적이 돌아가.

외부에서 조회 가능한 값도 추가했어.

getYawPWM();
getPitchPWM();

getLeftDistance();
getRightDistance();
getBottomDistance();

getYawError();
getPitchError();
특히 나중에 Phase 4에서는

panError =
    seeker.getYawPWM()
    - Calibration::SEEKER_YAW_CENTER_PWM;

tiltError =
    seeker.getPitchPWM()
    - Calibration::SEEKER_PITCH_CENTER_PWM;
처럼 바로 사용할 수 있게 돼.

중요하게 아직 isTracking()은 넣지 않았어. 현재 V1 자체에는 명확한 Lock-On 상태변수가 없기 때문에, 지금 억지로 추가하면 “라이브러리화하면서 알고리즘까지 바꾸는 것”이 돼버려. 먼저 이 라이브러리가 기존 V1과 똑같이 움직이는지 확인한 뒤 Phase 3 단계에서 Lock-On 조건을 정의해서 추가하자.

센서 핀도 네가 수정한 V1 기준으로 통일했어.

LEFT   = GPIO12 / 13
RIGHT  = GPIO5 / 20
BOTTOM = GPIO16 / 6
그리고 중심 PWM은 하드코딩하지 않고 calibration_data.h에서 가져오게 했어.

Calibration::SEEKER_YAW_CENTER_PWM
Calibration::SEEKER_PITCH_CENTER_PWM
다만 V1의 제어 파라미터는 아직 그대로 SeekerControl.cpp 안에 유지했어.

Yaw Dead Zone    = 2.5 cm
Yaw PWM Step     = 4
Yaw PWM Range    = 200~400

Pitch Dead Zone  = 1.0 cm
Pitch PWM Step   = 3
Pitch Max Error  = 15 cm
Pitch PWM Range  = 200~450

Sensor interval  = 15 ms
Control wait     = 50 ms
이건 일부러 그래. 지금은 V1 보존이 우선이고, 나중에 안정적으로 동작하는 거 확인한 다음 control_config.h로 빼면 돼.

컴파일은:

rm -f seeker_control_test
g++ -std=c++17 \
tests/seeker_control_test.cpp \
src/SeekerControl.cpp \
src/PCA9685.cpp \
-Iinclude \
-lgpiod \
-o seeker_control_test
실행:

./seeker_control_test
이걸 기존 seeker_tracking_v1과 비교해서 좌/우/상/하 추적 반응이 동일하면 라이브러리화 성공이야.

그다음 단계가 정확히 Phase 3에 SeekerControl 연결이야. 거기서 처음으로 LOCK-ON → BODY_TRACKING 전환 조건을 정하면 된다. 


SeekerControl.h
코드


SeekerControl.cpp
코드


seeker_control_test.cpp
코드


오늘 오후 2:20
테스트 코드는 뭐임?

테스트 코드는 라이브러리화한 SeekerControl이 기존 동결 V1이랑 똑같이 동작하는지 확인하는 전용 파일이야.

즉 역할은 이거야.

seeker_tracking_v1.cpp
        ↓
라이브러리화
        ↓
SeekerControl.h
SeekerControl.cpp
        ↓
seeker_control_test.cpp로 단독 검증
seeker_control_test.cpp 안에서는 그냥:

SeekerControl seeker;

seeker.initialize();

while (true)
{
    seeker.update();
}
이렇게만 돌려.

즉 메인 상태머신, 질량이동, Body Yaw 스텝모터는 전혀 안 건드리고, 오직 시커만 단독으로 돌려서 기존 V1과 같은지 보는 용도야.

확인할 건:

좌측 표적 → 기존처럼 좌측 추적

우측 표적 → 기존처럼 우측 추적

위/아래 → Pitch 반응 동일

표적 놓치면 HOLD

PWM 증가/감소 방향 동일

이거 통과하면 SeekerControl 라이브러리화 성공으로 보고, 그다음 메인 Phase 3에 붙이는 거야.

원하면 테스트 코드 없이 바로 Phase 3에 붙일 수도 있는데, 지금까지 우리가 잡은 방식대로라면 단독 테스트 한 번 하고 들어가는 게 더 안전해.


라이브러리화 한 v1 코드가 정상 동작하는지 확인?

맞아. 딱 그 용도야.

seeker_control_test.cpp는 라이브러리화한 V1이 기존 seeker_tracking_v1.cpp와 동일하게 동작하는지 확인하는 검증용 코드야.

확인 포인트는:

초기 PWM이 Yaw=300, Pitch=321로 들어가는지

LEFT / RIGHT 반응이 기존과 같은지

Pitch가 LEFT - BOTTOM 기준으로 같은 방향으로 움직이는지

Dead Zone 동작이 같은지

Pitch 이상치 15 cm 이상일 때 HOLD 되는지

센서 LOST 시 중심 복귀하지 않고 HOLD 되는지

PWM Step이 기존처럼 Yaw 4, Pitch 3인지

즉 기능을 추가하는 테스트가 아니라, 코드 분리 때문에 기존 V1 동작이 깨지지 않았는지 확인하는 회귀 테스트라고 보면 돼.

이거 정상 동작 확인되면 seeker_tracking_v1.cpp 역할은 사실상 SeekerControl 라이브러리가 대체한 거고, 다음 단계로 메인 Phase 3에 연결하면 된다.


라이브러리
/
질량 이동 기반 중간 및 종말 유도 모사 시스템
/
seeker_control_test.cpp


1
2
3
4
5
6
7
8
9
10
11
12
13
14
15
16
17
18
19
20
21
22
23
24
25
26
27
28
29
30
31
32
33
34
35
36
37
38
39
40
41
42
43
44
45
46
47
48
49
50
51
52
53
54
55
56
57
58
59
60
#include "SeekerControl.h"

#include <iostream>

/*
 * ============================================================
 * File Name : seeker_control_test.cpp
 *
 * Description
 * ------------------------------------------------------------
 * seeker_tracking_v1을 SeekerControl 라이브러리로 분리한 뒤
 * 기존 V1과 동일하게 동작하는지 확인하기 위한 단독 시험.
 *
 * 이 테스트에서는:
 * - 질량이동 사용 X
 * - Body Yaw Stepper 사용 X
 * - Main State Machine 사용 X
 * - SeekerControl만 단독 실행
 *
 * 종료:
 * Ctrl + C
 * ============================================================
 */

int main()
{
    std::cout
        << "========================================\n"
        << "     SeekerControl V1 Library Test\n"
        << "========================================\n";


    SeekerControl seeker;


    if (!seeker.initialize())
    {
        std::cerr
            << "[ERROR] SeekerControl 초기화 실패\n";

        return 1;
    }


    std::cout
        << "[INIT COMPLETE]\n"
        << "V1 tracking start\n"
        << "Ctrl + C to quit\n"
        << "========================================\n";


    while (true)
    {
        seeker.update();
    }


    return 0;
}

