# v1/v2 공통 비행 로직

현재 코어는 **0.3.0**입니다. [2026-10-02 실패 후 수정과 필수 현장 연결](failure_followup_20261002.md)을 먼저 확인하세요.

2026-10-02: v1의 발행 시각 기반 거리 적분, 안정 조건, 라이다 보정, 단계 제한시간과 LAND 인계 처리를 공통 기준으로 채택했습니다. 두 브랜치의 `ros2_ws/src/we_meet_flight_core/`는 동일한 파일입니다. 각 브랜치를 단독 다운로드해도 실행할 수 있고 v1은 OpenCV를 설치할 필요가 없습니다.

## 공통 실행 흐름과 기본값

`PRESTREAM → OFFBOARD_WAIT → ARM_WAIT → ASCEND → SETTLE → ADVANCE → BRAKE`

| 항목 | 양쪽에 적용되는 로직/기본값 |
|---|---|
| 시작 | fresh 지상/disarm·센서·추정기 확인, 안정된 yaw를 한 번 저장, 1.2초 실제 prestream 후 모드/ARM 요청과 FC 관측을 구분 |
| 명령 | 20Hz 단일 `TwistStamped`에 ENU xyz 속도와 yaw rate. MAVROS LOCAL_NED 변환·mask 1479 |
| yaw | 같은 wrap/P/clamp 함수, kp 1.0, 최대 0.35rad/s, 고정 yaw_ref. 전진 방향도 yaw_ref 기준 |
| 상승/Z | 목표 3m, kp 0.6·kd 0.15, 속도 ±0.25m/s·가속 0.5m/s², 상승 중 수평 0속도 |
| 상승 안정 | 높이 오차 ±0.10m·높이 변화율 ±0.05m/s·yaw 오차 ±5도·추정 수평 속도≤0.15m/s·추정 상방 속도≤0.10m/s, 연속 2초 |
| 라이다 | 기본 raw 거리(false). `(거리+센서 오프셋)×cos roll×cos pitch`. 이미 수직높이로 보정한 입력은 true로 표시하여 이중 보정 방지. 변화율 창 0.5초의 90% 확보 필요 |
| 전진 | 최대 0.35m/s·수평 가속/감속 0.25m/s², 측방 명령 0 |
| 거리 | 성공한 local publish 시각 사이에 이전 최종 속도의 고정 전방 투영을 부호 그대로 적분. 다음 명령 결정용 예측은 누적값에 중복 반영하지 않음 |
| 감속 | 제동 명령 면적 `v²/(2a)` 및 발행 간격 여유를 남겨 감속. 명령 적분 목표 5m·허용 ±0.15m. 실제 공간 거리의 허용 오차가 아님 |
| 건강/시간 | yaw·IMU·라이다 age 0.3초, state/estimator 2.5초, landed/battery 2초. timer/publish gap 0.2초, 상승35·안정15·전진35·감속5·정지10·전체110·착륙35초 |
| 제어권/중단 | 수동 모드·disarm·critical은 제어 해제, 재진입 없음. 제어권이 유효할 때 LAND 요청. invalid 추정기/frame/발행 실패 이후 zero stream 재개 없음 |
| LAND | AUTO.LAND 관측 즉시 xyz/yaw 발행 중단. landed+disarm 관측 후 COMPLETE. 공중 DISARM 없음 |
| 로그 | 공통 run_id·명령·actual publish 시각/적분 구간·GPS/추정 XY 기록·서비스 결과·중단 원인·요약. XY/GPS는 수동적 중단 감시 및 추정 속도 유지 판정에 사용; 수평 조향 명령은 만들지 않음 |

## v2 확장 경계

v1과 v2의 `baseline.yaml`(vision_enabled=false)은 `BRAKE → ZERO_VELOCITY_HOLD → LANDING → COMPLETE`가 같습니다. 고도·yaw·추정 속도 조건이 안정되고 실제 수평 0속도 발행을 시작한 때부터 연속 5초를 셉니다. 조건 이탈 시 초기화합니다. 상승 단독에서는 ADVANCE/BRAKE를 생략합니다.

v2의 `panel_approach.yaml`은 **건강한 카메라** 입력과 live 시작 시 확인된 카메라 축을 추가로 요구합니다. 이 조건을 만족하고 패널이 아직 잠기지 않았으면 공통 구간의 명령·기본값·적분이 v1과 같습니다. 카메라가 고장나면 전환 전에도 v2만 중단합니다.

잠긴 패널을 ADVANCE 중 인식하면 공통 BRAKE를 일찍 시작할 수 있습니다. 감속이 끝난 뒤 `ALIGN → VISUAL_HOLD → LANDING → COMPLETE`로 분기합니다. XY 목표만 카메라 오차로 만들며 공통 가속 제한, 라이다 Z, 고정 yaw_ref, FC 발행과 중단/LAND 처리를 계속 사용합니다. 영상 XY를 ENU로 변환할 때에는 현재 yaw를 사용합니다.

중앙 가로30%×세로30% 영역, 고도와 yaw 및 추정 속도 안정, 수평 명령0, 신선한 영상이 모두 유지되어야 첫 실제 0명령 발행부터 연속5초를 완료합니다. 한 조건이라도 깨지면 타이머를 초기화합니다. 영상 정렬의 이동은 접근 명령거리 예산에 더하지 않습니다. 패널 없는 5m 예산 소진은 v2 영상 프로필에서 실패입니다. 카메라 stale 0.3초/잠긴 목표 상실 0.6초는 착륙 중단 사유이며 전진을 재개하지 않습니다.

## 기존 설정의 이전

- ROS 패키지/서비스 경로는 v1 `we_meet_flight_v1`, `/test_flying_v1/…`; v2 `we_meet_flight`, `/test_flying_v2/…`를 유지합니다.
- **공통 패키지를 함께 빌드**하고 workspace를 source해야 합니다. v1: `colcon build --symlink-install --packages-select we_meet_flight_core we_meet_flight_v1`; v2: `colcon build --symlink-install --packages-select we_meet_flight_core we_meet_flight`.
- v2 기존 YAML 키는 `approach_enabled→advance_enabled`, `target_range_m→target_height_m`, `approach_distance_m→commanded_target_distance_m`, `cruise_speed_mps→forward_speed_mps`, `hover_s→zero_velocity_hold_s`로 읽습니다. 양 이름이 함께 있으면 충돌로 거부합니다. 저장소 프로필은 새 이름을 사용합니다.
- 이전 로컬 YAML에 명시한 값은 보존됩니다. 특히 기존 v2의 `lidar_input_is_vertical_height=true`가 실제 보정과 맞는지 다시 확인하세요. 저장소 새 기본은 양쪽 false입니다. 이미 교정한 설정을 통일한다는 이유로 덮어쓰지 않습니다.
- 기존 v2 상태 이름은 TAKEOFF→ASCEND, CRUISE→ADVANCE, HOLD→ZERO_VELOCITY_HOLD 또는 VISUAL_HOLD, DONE→COMPLETE로 바뀝니다. 결과 baseline은 `expected_5m_zero_command_5s`, 영상은 `panel_centered_5s`입니다. 로그 분석 스크립트가 이전 이름에 의존하면 갱신하세요.
- 기존 build/install과 함께 새 코드를 섞어 쓰지 말고 두 패키지를 다시 빌드합니다. 로컬 변경·현장 교정은 보존하고 `reset --hard`로 지우지 않습니다. 두 브랜치의 노드를 동시에 FC에 연결해 실행하지 않습니다.

## 유지보수와 동일성 검증

알고리즘과 ROS/FC 처리의 원본은 공통 패키지입니다. v1 패키지는 호환 import/실행 경로만 제공하고, v2는 Config/Sensors 및 camera hook만 추가합니다. 공통 파일을 수정하면 양쪽 브랜치에 같은 변경을 반영하고 `docs/shared_core_manifest.json`의 파일 SHA256을 갱신합니다.

```bash
python3 tools/verify_shared_core.py
# 두 브랜치를 다른 폴더에 내려받았을 때, 반대 브랜치의 저장소 root 지정
python3 tools/verify_shared_core.py --other /path/to/other-branch
```

공통 단위시험은 양쪽 동일하며, v2에는 별도 controller 인스턴스의 command/state/distance 일치 시험이 추가되어 있습니다. vision=false 전체 시험 및 건강한 영상+미검출 상태의 감속 완료 직전까지를 비교하고, 불규칙 timer와 실제 발행 지연도 비교합니다. 이 검증은 코드 동일성과 합성 실행의 검증이며 실기체 비행 성공의 근거는 아닙니다.
