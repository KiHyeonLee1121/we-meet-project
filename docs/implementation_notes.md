# 재구성 근거와 검증 범위

## 읽은 자료

제공된 `02-dadaka-REPORT.md`, 소스와 분석이 포함된 `05-_-.md`, 진단 tar의 현재/as-flown 코드·YAML·로그, 자율 `04-px4-ulog.ulg`(log154), 수동 log155~167 13개 ULog를 읽었습니다. raw ULog는 pyulog로 직접 열어 비행 모드와 주요 PX4 설정을 다시 확인했습니다. 원본 전체와 실제 시험 코드 1/2는 이 브랜치에 없으므로 복원본과 동일한 코드라고 주장하지 않습니다. 대용량 로그·사설 호스트 정보는 커밋하지 않았습니다.

- 현재 진단 자료: Pi 5, ROS 2 Jazzy는 Docker, MAVROS 2.14.0, Pixhawk FMUv5, IMX708, `/distance/filtered` Range 입력.
- 제공 `distance_controller.py`의 `yaw_rate_command()`를 그대로 유지: 각도 wrap, P 제어, rate clamp. YAML의 `kp=1.0`, `max_rate=0.35rad/s`도 유지했습니다.
- 기존 Z 제어의 `kp=0.6`, `kd=0.15`, 최대 `0.25m/s`, 가속도 `0.5m/s²`를 사용했습니다. 기존 대규모 mission 전체를 이식하지는 않았습니다.
- 제공 log154에서 `cs_gnss_pos`와 `cs_gnss_vel` 활성, optical-flow/EV-position/range-height fusion 비활성을 확인했습니다. Range 토픽을 mission Z 제어에 쓰는 것과 PX4 내부 range-height fusion은 다른 기능입니다.
- 수동 log155~167에서는 OFFBOARD가 없고 POSCTL 위주, 일부 ALTCTL/AUTO 구간이 있었으며 `MPC_POS_MODE=4`였습니다. RC 축 값은 바람·기체 상태에 무관한 일정 거리 단위가 아닙니다. 로그로 RC-거리 변환표를 만들어 제어하지 않았습니다.
- 이전 분석 자료의 상태 전환 위치 setpoint 점프를 피하려고 위치 setpoint 없이 한 개의 velocity stream과 XY vector slew를 사용했습니다.
- log154의 `COM_RC_IN_MODE=1`, `COM_RC_OVERRIDE=1`이 현재 현장 설정인지 여기서 확인할 수 없습니다. 실제 모드 변경을 관측하면 제어를 놓지만 RC 스틱만 움직여도 자동 해제된다고 보장하지 않습니다.

선별 결과는 [evidence_summary.json](evidence_summary.json)에 있습니다.

MAVROS 2.14.0 [setpoint_velocity.cpp](https://github.com/mavlink/mavros/blob/2.14.0/mavros/src/plugins/setpoint_velocity.cpp)를 확인했습니다. 기본 `mav_frame`은 문자열 `LOCAL_NED`, 입력은 ENU xyz와 yaw rate, position/acceleration/yaw angle은 ignore mask로 제외합니다. ROS local pose yaw도 ENU/FLU 규약을 사용합니다.

## 구현상 선택

전진 속도 0.35m/s, 수평 가속도 0.25m/s²는 새 시험의 보수적인 초깃값입니다. 제공 로그로 이 값에서 실제 5m 도달을 교정한 것은 아닙니다. 명령 적분 거리와 실제 거리의 차이는 수동/자율 시험 로그로 별도 분석해야 합니다.

상승/대기/접근/감속/정렬/5초 정지 모두 고정 yaw_ref에 대한 같은 함수를 20Hz로 계산합니다. 정상 구간에서 yaw_ref를 현재 yaw로 덮어쓰지 않습니다. 영상 변환에만 현재 yaw를 사용합니다.

heading reset 카운터를 MAVROS에서 직접 구독하지 못하므로 pose yaw 변화에서 IMU 기반 물리 회전을 뺀 innovation을 이용합니다. 기존 자료의 1.5도 threshold를 유지하되, 이전 코드처럼 reference를 재설정하지 않고 이번 시험에서는 중단합니다. 이것은 추정기 reset의 완전한 검출기가 아니며 IMU/pose 시간 정렬과 실기체에서 민감도를 확인해야 합니다.

영상 중심 ±15%의 사각 허용 영역은 미터 단위 위치 오차가 아닙니다. 카메라 화각·높이·장착 방향에 따라 실제 범위가 달라집니다. 목표 panel이 중앙 영역을 떠나거나 검출이 끊기면 5초 성공 타이머를 초기화합니다. heading/고도도 안정되어야 타이머를 진행합니다. 실제 XY ground speed는 이번 제어/성공 판정의 입력으로 사용하지 않습니다. 따라서 이 시험의 '정지'는 중앙 영역 유지와 XY 명령 0이며, 실제 무속도/정확한 좌표 도달 증명은 ULog와 외부 측정이 필요합니다.

검출은 밝은 배경의 어두운 사각형을 대상으로 합니다. 학습 모델이나 패널 셀 무늬 인식은 없습니다. 충분히 화면 안에 들어온 사각형을 여러 프레임 확인한 뒤 감속하며, 잘린 패널 중심을 추정해서 정렬하지 않습니다. 후보 선택 뒤에는 거리/면적 연관으로 하나를 잠그고, 다른 큰 후보로 전환하지 않습니다. 가림/빠른 움직임으로 원래 패널을 잃으면 중단할 수 있습니다.

영상 해석은 queue size 1의 worker에서 수행해 FC 발행 timer를 막지 않습니다. transport/source age와 실제 분석 프레임 나이를 기록합니다. 처리 지연·중복 stamp는 유효한 새 관측으로 인정하지 않습니다. 로그도 bounded queue로 기록하고 쓰기 오류를 controller에 전달합니다.

## 현장 적용 범위

기존 MAVROS와 라이다 드라이버를 재사용합니다. optional camera_source는 Picamera2 또는 V4L2 입력을 raw Image로 발행합니다. 이 환경에 ROS/Pi/libcamera 장치가 없으므로 colcon/ROS 런타임 및 실제 CSI 카메라 연결은 검증하지 못했습니다. 시스템 설치·컨테이너 실행·FC 파라미터 변경·arming·실제 비행은 수행하지 않았습니다.

최초 현장 적용 시 실제 camera image로 threshold와 변환 방향을 확인하고, FC 속도 plugin의 frame·setpoint 단일 발행·RC/모드 스위치·OFFBOARD-loss 회수 경로를 확인한 뒤 baseline 시험을 먼저 실행하세요. 실기체 설정을 확인하지 않은 상태에서 verified 플래그를 단순히 true로 바꾸는 것은 교정이나 비행 검증을 대신하지 않습니다.


추가 검증: FC EstimatorStatus의 attitude/XY velocity/Z velocity 유효성·stale 검사를 별도로 적용했습니다. 위치/GPS 좌표는 거리 피드백에 사용하지 않습니다. 추정기 무효 시 LAND를 요청할 수 있지만 zero velocity로 안전 정지가 보장된다고 가정해 stream을 유지하지 않습니다. 라이다 수직높이 선보정 입력과 raw body-down 입력을 구분하고, raw인 경우 장착 오프셋·roll/pitch 투영을 한 번만 적용합니다. 입력 max_range가 목표 3m를 지원하지 않으면 차단합니다. 실제 지면/패널 높이 변화와 장착 보정은 현장 확인이 필요합니다.
