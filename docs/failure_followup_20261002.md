# 2026-10-02 비행 실패 후 공통 코어 수정 — 0.3.0

이 변경은 `test-flying-v1`과 `test-flying-v2`의 공통 코어에 동일하게 적용합니다. 카메라 전환 전 비행 명령은 같고, v2 영상 정렬 중에도 공통 감시를 계속합니다. **원래 Pi 코드의 복원이거나 실비행 성공 검증은 아닙니다.**

## 어떤 근거로 바꿨나

제공 문서 `DA_DAKA_2026-10-02_비행_외부분석_통합.md`의 원본 ULog 및 ROS CSV를 확인했습니다. ULog SHA256은 `f198b8df20d16fb425e13649dc125ff80727cf621890aaa2ca3e16eb4294c6bf`입니다. 당시 실제 실행은 전진 없는 `velocity_only/vertical_only` 상승 시험이었고, ADVANCE/5m/5초 유지 단계에 도달하지 않았습니다. 따라서 이번 변경은 기본 상승과 감시를 보강하며 5m 비행 성공을 주장하지 않습니다.

| 관찰 | 적용 |
|---|---|
| 상승 초기에 yaw 오차 약 24도, 그 뒤 FC yaw 재정렬 | 15도 오차 또는 35도/s 회전이 0.5초 지속되면 중단. gyro 기반 점프 휴리스틱에 FC 리셋 카운터 감시 추가 |
| FC quaternion reset counter 2→3, 약 7.67도 재정렬 | ODOMETRY의 결합 리셋 카운터 변화도 즉시 중단. 저장한 yaw 기준을 자동 이동하지 않음 |
| GPS 수평 sigma 약 1.8m, 추정 XY 변화와 속도 적분의 큰 불일치 | 시작/실행 중 품질 검사, 명령 경로 대비 추정 XY 이탈 감시, FC 위치·속도 자체 일관성 검사. GPS 보정 명령은 추가하지 않음 |
| 라이다 높이 감소와 FC 상승속도가 반대 방향 | 같은 평탄한 반사면에서 높이 변화율과 FC ENU 상방 속도를 비교. 지속 불일치 시 중단 |
| 기존 v1은 고도·yaw가 벗어나도 0속도 발행 5초로 완료 가능 | v1/v2 모두 연속 고도·yaw·추정 속도 안정과 실제 0속도 발행을 요구. 조건 이탈 시 5초 타이머 초기화 |
| MAVROS const_pos=true와 ULog cs_constant_pos=false | 해당 PX4 소스의 legacy 플래그에 vehicle_at_rest도 포함됨을 확인. fresh landed 상태에서는 허용하고 공중 또는 landed stale이면 거부 |
| 수동 AUTO.LAND와 요약 모드의 불일치 | 관측 모드 전환 이벤트, 최종 관측 모드/armed, 중단 원인 기록. LAND 요청과 FC 관측을 구분 |

위 수치는 **새 시험의 임시 허용/중단 기준**입니다. 실측 기체 스펙이나 실제 지면 위치 정확도 보장이 아닙니다. 초기 회전의 물리 원인은 확정하지 못했으므로 모터·프로펠러·FC 장착·자력계·yaw 제어 응답은 현장에서 별도로 확인해야 합니다. FC/PID/자력계/EKF 파라미터를 이 코드가 자동 변경하지 않습니다.

## 드론 Codex가 먼저 연결할 것

1. 기존 MAVROS 연결의 **MAVLink 2** 및 `mavros_msgs/msg/Mavlink` raw 수신 토픽을 확인합니다. ROS 2 router에서는 보통 `/mavros/mavlink_source`이며, ROS 1 계열의 `/mavlink/from` 이름을 추정해서 사용하지 않습니다. `ros2 topic list -t`로 실제 토픽을 찾고 launch `fc_mavlink_topic`에 지정합니다. 새 serial consumer나 FC setpoint publisher는 만들지 않습니다.
2. 실제 FC의 `sysid/compid`를 관찰해 `fc_system_id`, `fc_component_id`에 지정합니다. 기본값 1/1은 예시입니다. GCS/외부 비전의 ODOMETRY는 입력으로 인정하지 않습니다.
3. **FC가 내보내는 ODOMETRY(메시지 331)를 20Hz로 요청/확인**합니다. PX4의 LOCAL_NED pose/velocity, estimator_type=AUTOPILOT(8), reset_counter 및 위치/속도 분산이 필요합니다. quality=0은 해당 PX4의 unknown 값이므로 그것만으로 좋은 품질로 판정하지 않습니다. 0/음수/NaN/누락 분산은 시작을 차단합니다.
4. 메시지가 없다면 정지·disarm 상태에서 기존 MAVROS의 telemetry interval 서비스 또는 FC 콘솔을 통해 해당 **기존 연결**의 스트림을 설정합니다. 다음은 namespace와 서비스 타입을 확인한 후 사용하는 telemetry 요청 예시입니다. OFFBOARD/arming/setpoint 명령이 아닙니다.

```bash
ros2 topic list -t
ros2 service type /mavros/cmd/command
ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong \
  '{broadcast: false, command: 511, confirmation: 0, param1: 331.0, param2: 50000.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}'
```

응답 성공만으로 수신을 보장하지 않습니다. raw 토픽에서 msgid=331, sysid/compid, MAVLink-v2 magic=253, framing_status=1, payload가 반복 수신되는지 확인합니다. `topic hz`는 모든 raw 메시지의 합산 주기이므로 ODOMETRY 자체의 20Hz 증거가 아닙니다. dry-run `/test_flying_v1/status` 또는 `/test_flying_v2/status` 및 로그의 `sensors.odometry_age`, `fc_odometry_guard.timestamp_us` 진행, `fc_reset_counter`, 분산을 함께 확인하세요. 시작 서비스가 거부한 이유도 events 로그에 남습니다. PX4 콘솔을 쓰면 `mavlink status`로 Pi 연결의 실제 인스턴스/장치를 확인한 후 그 기존 링크에 `mavlink stream -d <실제_FC_장치> -s ODOMETRY -r 20`를 적용합니다. 새 MAVLink 인스턴스를 만들거나 임의 포트를 가정하지 않습니다.

**수신 경로가 없거나 firmware에서 스트림/분산을 지원하지 않으면 이 버전은 실행을 거부합니다.** 감시를 삭제하지 말고 실제 환경의 지원 여부를 보고하세요. MAVROS 로컬 Odometry 토픽이 있다는 것만으로 reset_counter를 받는 것은 아닙니다.

## 현장에서 검토할 YAML

모든 저장소 프로필에 다음 항목을 명시했습니다. 변경 전 현장 교정값을 보존하고, 로드 결과 `config_snapshot.yaml`을 확인합니다. live 시작의 `flight_settings_verified=true`에는 이 새 감시 한계·센서 주기·FC 스트림 확인도 포함합니다. 타이머가 정상이어도 실제 setpoint local publish 간격이 0.2초를 넘으면 5초 완료를 인정하지 않고 중단합니다.

| 항목 | 기본값 | 뜻 |
|---|---:|---|
| `fc_odometry_timeout_s` | 0.3s | raw FC 추정 수신 감시. 중복 payload timestamp는 age를 갱신하지 않음. 시작 후 clock rollback은 중단 |
| `maximum_position_sigma_m` / `maximum_velocity_sigma_mps` | 0.5m / 0.2m/s | FC 공분산의 수평 축별 최대 sigma. 실제 오차 보장이 아님 |
| `gps_accuracy_required` / `maximum_gps_sigma_m` | true / 0.5m | 현재 GNSS 의존 FC의 추가 품질 검사. fix뿐 아니라 알려진 수평 공분산의 최대 고유값 사용 |
| `gps_timeout_s` | 2s | GNSS 품질 데이터 age |
| `horizontal_tracking_limit_m` / `horizontal_tracking_dwell_s` | 1m / 0.5s | 시작 XY + 전체 최종 XY 속도명령 적분 경로에서 추정 위치 이탈. ASCEND부터 영상 정렬까지 적용 |
| `position_velocity_window_s` / `position_velocity_residual_limit_m` | 1s / 0.5m | 같은 FC 추정에서 위치 변화와 속도 사다리꼴 적분의 불일치 |
| `yaw_abort_error_rad` / `yaw_abort_rate_rad_s` / `yaw_abort_dwell_s` | 15도 / 35도/s / 0.5s | 지속 yaw 이탈/회전 감시. hold yaw 허용값 5도는 유지 |
| `lidar_velocity_check_enabled` | true | 고정 평탄한 반사면에서만 유효한 라이다/FC 상방 속도 비교 |
| `lidar_velocity_disagreement_mps` / `lidar_velocity_disagreement_s` | 0.25m/s / 0.75s | 라이다 0.5s 높이 변화율과 현재 FC 속도 차이가 지속되면 중단 |
| `hold_horizontal_speed_mps` / `hold_vertical_speed_mps` | 0.15 / 0.10m/s | 2초 상승 안정 및 5초 유지의 추정 속도 상한 |

GNSS 품질이 당시처럼 sigma≈1.8m이면 기본 설정으로 시작하지 않습니다. 수신 위성 수나 fix만 보고 허용치를 늘리지 않습니다. 검증한 optical flow/EV 등의 독립적인 FC 속도/위치 입력으로 GNSS 의존을 실제 해소한 경우에만 `gps_accuracy_required=false`의 근거를 기록합니다. 카메라 패널 XY 명령만 추가한 현재 v2는 FC 위치 추정기에 카메라 위치를 넣지 않으므로 이 예외의 근거가 아닙니다. GNSS 검사 여부와 무관하게 FC ODOMETRY·분산·리셋·일관성 검사는 필수입니다.

라이다 반사면이 패널/지면 사이에서 바뀌거나 경사지면이면 높이 변화가 기체 상방 속도와 같지 않습니다. 우선 일정한 평면에서 시험하고 `lidar_geometry_verified`에 반사면·오프셋·필터 지연 확인을 포함하세요. 다른 지형 시험으로 `lidar_velocity_check_enabled=false`를 선택할 때는 제외 사유와 대체 높이 검증을 기록해야 하며, 코드가 자동 제외하지 않습니다.

FC reset counter는 이 firmware에서 quaternion/XY/Z/속도 reset count의 uint8 합입니다. **yaw-only 원인 구분이나 heading_good_for_control 수신 기능은 없습니다.** 초기 정상 정렬이라도 시작 이후 counter가 바뀌면 고정 기준 시험을 중단합니다. 현장 FC 정렬과 이륙 전 절차를 검증해야 하며, 리셋 후 기준을 옮겨 비행을 계속하지 않습니다. counter가 wrap되어도 값이 바뀌면 중단하지만, 수신 사이 총 256회 변화 등 modulo counter의 한계는 남습니다.

## 실행·검증과 해석

기존 패키지와 `we_meet_flight_core`를 함께 재빌드합니다. v1/v2 launch에 `fc_mavlink_topic:=<실제토픽> fc_system_id:=<실제ID> fc_component_id:=<실제ID>`를 추가할 수 있습니다. dry-run은 실제 센서를 읽지만 FC 쓰기/서비스 요청은 하지 않습니다. 실제 비행은 README의 현장 실행 절차와 별도 실행 지시를 따릅니다.

```bash
python3 -m unittest discover -s tests -v
python3 tools/replay_failure_guards.py
python3 tools/verify_shared_core.py --other /path/to/other-branch
```

`tests/fixtures/oct02_failure_signals.json`에는 원본 ROS CSV/ULog CSV에서 선택한 스칼라 신호와 출처 해시를 넣었습니다. 재생은 GPS 사전 차단, 초기 yaw 이탈, FC 리셋, 추정 위치·속도 불일치, 라이다·FC 속도 불일치를 **각각 다른 입력은 합성 정상값으로 놓고** 확인합니다. 마지막 두 감시는 초기 yaw/GPS 차단을 통과시킨 가정에서 시험하며 실제 코드가 당시 전체 비행을 그대로 수행했다는 뜻이 아닙니다. fixture의 첫 리셋 수신 시각은 CSV 샘플 간격을 반영하므로 고주기 ULog의 최초 리셋 시각과 다릅니다.

거리 완료는 여전히 명령 적분 기준입니다. 5초 완료는 고도·yaw·FC 추정 속도·영상(v2)이 연속 조건을 만족했다는 뜻이며 줄자 기준 5m 도착/정지 증명이 아닙니다. 자세/추정기 리셋·stale 입력에서는 명령 스트림을 멈추고 제어권이 유효하면 LAND를 요청합니다. 지속 yaw/경로/일관성 감시 중단도 LAND 요청 후 스트림을 중단합니다. pilot/FC 모드 변경이 먼저 관측되면 제어를 넘기고 LAND/ARM/OFFBOARD를 재요청하지 않습니다. FC가 LAND를 거절/미응답하면 기존 검증한 OFFBOARD-loss 절차에 의존하므로 실제 자동 착륙 보장을 주장하지 않습니다.

공통 데이터 수신·FSM 단위시험, byte 동일성, 합성 plant/OpenCV 실행은 개발 환경 검증입니다. ROS 2 executor·DDS·실제 MAVROS 스트림·FC 통합 및 비행은 드론 컴퓨터와 현장에서 확인해야 합니다.

## 플래그/메시지 근거

- [해당 PX4 EKF legacy 플래그 정의](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF/ekf_helper.cpp)
- [해당 PX4 ODOMETRY 스트림](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mavlink/streams/ODOMETRY.hpp)
- [해당 PX4 EKF2 결합 reset_counter 발행](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF2.cpp)
- [MAVLink ODOMETRY wire schema](https://github.com/mavlink/c_library_v2/blob/master/common/mavlink_msg_odometry.h)
- [MAVROS ROS 2 raw message](https://github.com/mavlink/mavros/blob/ros2/mavros_msgs/msg/Mavlink.msg) / [router raw topic](https://github.com/mavlink/mavros/blob/ros2/mavros/src/lib/mavros_router.cpp)
