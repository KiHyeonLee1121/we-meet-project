# we-meet-project — test-flying-v1

## 목적

**카메라 없이 이륙한 뒤, 출발 방향으로 속도명령을 보내 약 5m 이동을 예상하고 감속·정지명령·착륙을 시험하는 브랜치**입니다. 실제 도착 거리와 정지 상태를 측정해, 이후 `test-flying-v2`의 카메라 기반 패널 정렬 단계로 넘어가기 위한 기초 시험입니다.

시험 순서는 **라이다 기준 약 3m 상승 → 안정 대기 → 고정된 출발 방향으로 속도 전진 → 미리 감속 → 수평 0속도 명령 5초 → AUTO.LAND → 착륙·disarm 확인**입니다. 이륙 전에 저장한 `yaw_ref`를 전체 OFFBOARD 구간에서 유지하며 기존 `yaw_rate_command()`의 각도 wrap·P 제어·회전속도 제한을 재사용합니다.

**5m는 최종 발행 속도명령의 실제 시간 적분값입니다. 실제 위치를 측정한 값이 아닙니다.** GPS/추정 XY 위치로 남은 거리·경로·수평 보정을 계산하지 않습니다. FC의 기존 EKF/GNSS 설정과 안전장치는 유지하고, PX4 속도제어가 요구하는 자세·속도 추정 유효성은 확인합니다. 수평 0속도 명령이 실제 제자리 호버를 보장하지 않으므로 ULog·영상·줄자 측정으로 따로 평가합니다.

| 프로필 | 시험 |
|---|---|
| `ascent_only.yaml` | 3m 상승 → 수평 0속도 명령 5초 → 착륙. 전진 없음 |
| `velocity_trial.yaml` | 3m 상승 → 명령 기준 약 5m 전진·감속 → 수평 0속도 명령 5초 → 착륙 |

카메라·OpenCV·AI·패널 탐색·분사·귀환은 이 브랜치에 포함하지 않습니다. ROS 패키지 이름은 **`we_meet_flight_v1`**, 서비스·debug 경로는 **`/test_flying_v1/...`**입니다. v2 패키지와 이름을 구분하지만 FC 명령 토픽은 같으므로 두 시험을 동시에 실행하지 않습니다.

기존 라즈베리파이 시험 코드 1/2 원본은 가져오지 못했습니다. 첨부 진단·코드·수동/자율 ULog와 [기존 da-daka_Ai](https://github.com/KiHyeonLee1121/da-daka_Ai)를 읽어 재구성했습니다. 원본과 완전히 같은 코드이거나 현장 비행을 검증한 코드라고 주장하지 않습니다. 자세한 근거는 [implementation_notes.md](docs/implementation_notes.md), 개발 검증은 [validation.md](docs/validation.md)에 있습니다.

## 2026-10-02 실패 분석을 반영한 변경 (공통 코어 0.3.0)

**FC 리셋 카운터·지속 yaw 이탈·추정 품질/경로/속도 불일치·라이다/FC 수직속도 불일치 감시와 연속 5초 유지 판정을 보강했습니다.** v1/v2 공통 코어는 동일합니다. GPS/추정 XY로 이동 명령을 보정하지 않으며, 품질이 나쁘거나 상태가 어긋나면 시험을 중단합니다. 고도·방향·추정 속도가 유지 조건을 벗어나면 5초 타이머를 처음부터 다시 셉니다.

**다운로드한 드론 Codex는 [실패 후 수정·현장 연결 안내](docs/failure_followup_20261002.md)를 먼저 읽어야 합니다.** 기존 MAVROS 링크의 MAVLink 2 / FC ODOMETRY(331) 20Hz 수신, 실제 FC ID, GPS/FC 분산과 새 YAML 한계를 확인해야 합니다. launch에 `fc_mavlink_topic`, `fc_system_id`, `fc_component_id`를 추가했습니다. 수신 누락이나 기본 GPS sigma 0.5m 한계 초과 시 시작을 거부합니다. 당시 약 1.8m 품질은 이 기본값으로 차단됩니다. 새 한계는 실측 기체값이 아니라 현장 검토가 필요한 임시 시험 기준입니다.

카메라를 쓰는 v2도 FC의 속도 추정 의존이 남습니다. 실기체 설정·교정·스트림 연결은 Pi에서 확인하고, 준비 작업에서 비행 명령을 실행하지 않습니다. 원본 실패 신호의 개별 감시 재생은 `python3 tools/replay_failure_guards.py`로 확인할 수 있습니다.

## v1/v2 공통 로직 통일

두 브랜치 모두 동일한 **`we_meet_flight_core` 공통 패키지**를 사용합니다. v1의 실제 발행 시각 기반 명령 적분·라이다 raw 보정·상승 안정 조건·단계 제한시간·중단/착륙 인계를 기준으로 통일했습니다. 카메라가 정상이고 목표가 아직 검출되지 않았다면 같은 입력에서 같은 명령을 냅니다. v2는 카메라 상태 확인 및 패널 인식 이후 감속 분기·영상 정렬·5초 중앙 유지만 추가합니다.

**새 공통 패키지를 함께 빌드해야 합니다.** 기존 실행/서비스 이름은 유지됩니다. 세부 비교·기존 v2 설정 키/상태 이름 변경·코어 동일성 확인은 [shared_flight_logic.md](docs/shared_flight_logic.md)를 읽으세요.

## 내려받은 드론 컴퓨터의 Codex에게

현재 준비 작업은 **다운로드 → 기존 환경 조사 → 설정·연결 → 빌드 → 비행 없는 검증 → 미확인 항목 보고**입니다. 준비 중 실제 FC setpoint, OFFBOARD, arming, 분사를 실행하지 마세요. 비행은 현장 확인 후 사용자의 별도 실행 지시에 따라 수행합니다.

### 1. 다운로드와 환경 확인

```bash
git clone --branch test-flying-v1 --single-branch https://github.com/KiHyeonLee1121/we-meet-project.git
cd we-meet-project
git branch --show-current
git rev-parse HEAD
git status --short
```

기존 폴더가 있으면 로컬 변경을 먼저 확인·보존합니다. 기존 코드에 `reset --hard`를 하지 마세요. 기존 `da-daka_Ai`의 compose·launch·센서 드라이버·교정 자료를 확인하고 ROS가 실행되는 호스트/컨테이너와 실제 mount 경로를 찾으세요. 자료에서는 Pi 5·ROS 2 Jazzy Docker·MAVROS 2.14·FMUv5/Pixhawk·TF-Luna 조합을 확인했습니다. 현재 기체 환경은 다시 읽어야 합니다.

```bash
# Pi 호스트에서 조회
uname -m
cat /etc/os-release
docker ps -a

# 실제 ROS 환경에서 조회
printenv ROS_DISTRO
ros2 node list
ros2 topic list -t
ros2 service list
```

예전 전체 미션 launch는 arming/분사 경로가 있을 수 있으므로 센서 확인용으로 실행하지 마세요. 기존 MAVROS·TF-Luna 드라이버·거리 필터의 센서 전용 경로를 재사용합니다. 이 패키지는 FC 또는 라이다 serial port를 직접 열지 않습니다. serial port, baudrate, Docker 장치 매핑은 기존 드라이버 환경에서 확인합니다.

### 2. 자동 입력과 현장 설정 구분

| 구분 | 값 | 적용 방식 |
|---|---|---|
| 실행 중 자동 수신 | FC 연결·모드·armed/landed, yaw·자세·IMU, 라이다 거리, 추정기 유효성, 배터리 | ROS 상태를 읽음. 누락/무효값을 0이나 가짜 센서값으로 채우지 않음 |
| 수동적 감시/기록 | FC ODOMETRY 위치·속도·분산·reset, GPS fix·공분산 | 품질·이탈·일관성 감시 및 추정 속도 유지 판정. 남은 거리나 수평 보정 명령에는 쓰지 않음 |
| 기체 파일·환경에서 확인 | ROS namespace, 토픽·타입·주기, MAVROS frame, 드라이버 보정 여부, 기존 yaw/Z 설정 | 기존 코드·실행 설정을 읽어 연결·설정에 반영 |
| 사람이 측정/확인 | 라이다 설치 방향·기체 기준점에서 센서까지 거리, 반사면/지면 높이, 전진 방향, 실제 이동·정지 오차 | 현재 장착에 맞는 교정 자료가 없으면 운영자에게 확인·측정 요청 |

### 3. 무엇을 어디에 설정할지

YAML 디렉터리는 `ros2_ws/src/we_meet_flight_v1/config/`입니다. 두 프로필은 독립 파일입니다. 생략한 값은 `we_meet_flight_core/config.py` 공통 기본값이 적용되며, 실제 로드 결과는 실행별 `config_snapshot.yaml`에 기록됩니다.

| 설정 위치 | 이름·기본값 | 확인/설정할 내용 |
|---|---|---|
| launch 인자 | `mavros_namespace=/mavros` | 실제 MAVROS prefix와 관련 상태·모드·arming 서비스 |
| launch 인자 | `lidar_topic=/distance/filtered` | 실제 `sensor_msgs/msg/Range` 입력과 stamp·측정 주기 |
| launch 인자 | `estimator_topic=/mavros/estimator_status` | 실제 `mavros_msgs/msg/EstimatorStatus`. 없으면 원인을 조사하고 검사 삭제 금지 |
| launch 인자 | `log_directory=~/flight_logs/test_flying_v1` | 실행 계정의 쓰기 권한·공간, 컨테이너이면 영속 host mount |
| YAML, 두 프로필 각각 | `lidar_input_is_vertical_height=false` | 기존 distance_filter는 평활화만 하므로 기본은 raw 사선거리. 현장 드라이버가 이미 높이/오프셋까지 보정하면 true |
| YAML, 두 프로필 각각 | `lidar_body_down_offset_m=0.0` | raw 입력일 때 기체 기준점 아래 센서까지의 거리(m). 0은 미측정 예시이며 현장 측정값 반영 |
| YAML, 두 프로필 각각 | `maximum_tilt_rad=0.35` | 약 20도 이내 roll/pitch 허용. 사선 장착축을 이 값으로 교정할 수 없음 |
| YAML / Config | `yaw_kp=1.0`, `yaw_max_rate_rad_s=0.35` | 기존 helper와 yaw 계수 유지. 현재 기체 설정과 다르면 근거 확인 후 반영 |
| YAML / Config | `vertical_kp=0.6`, `vertical_kd=0.15`, `vertical_speed_mps=0.25`, `vertical_accel_mps2=0.5` | 기존 거리 제어의 계수·속도/가속도 상한 재사용 |
| YAML / Config | `lidar_rate_window_s=0.5` | 거리 변화율 계산 창. 창의 90%가 확보되기 전에는 준비 완료로 처리하지 않음 |
| YAML | `target_height_m=3.0`, `commanded_target_distance_m=5.0`, `zero_velocity_hold_s=5.0` | 이번 시험 사양 |
| YAML / Config | `forward_speed_mps=0.35`, `horizontal_accel_mps2=0.25` | 새 전진 시험의 초기값. 실기체에서 5m·제동을 교정한 값이 아님 |
| YAML / Config | `commanded_distance_tolerance_m=0.15` | 명령 적분 약 5m의 검증 허용치. 실제 위치 오차 허용치나 지오펜스가 아님 |
| YAML / Config | `control_hz=20`, `sensor_timeout_s=0.3`, `state_timeout_s=2.5`, `maximum_tick_gap_s=0.2` | 현재 토픽 주기·전송/처리 지연·시계 동기 확인. 실행을 통과시키려고 검사 삭제 금지 |

라이다 보정은 **하향 body 축과 정렬된 센서 + 평탄한 반사면 + 그 축 위의 기준점/센서 오프셋**을 전제로 합니다. raw 입력에서는 `(range + offset) × cos(roll) × cos(pitch)`를 한 번 적용합니다. 이미 보정된 입력을 다시 보정하지 않습니다. 설치축이 다르거나 수평 센서 오프셋이 중요한 경우 추가 외부 보정이 필요하며 현재 기본식으로 처리했다고 표시하지 마세요.

`target_height_m`은 라이다가 보는 면에서 기체 기준점까지의 수직 높이입니다. 패널 위를 지나면 반사면이 바뀌므로 이번 v1은 평탄하고 일정한 지면의 전진 시험부터 수행하세요. 라이다 미장착·무효·목표 3m 측정 불가인 경우 실행을 차단하며 GPS/기압계 고도로 대체하지 않습니다.

기존 저장소 main의 FC serial 예시는 57600 baud, 첨부 현장 진단은 921600 baud입니다. 과거 값 중 하나를 추측해 강제 적용하지 말고 현재 MAVROS 실행 설정을 확인하세요. TF-Luna의 과거 설정은 115200 baud, 0.2~8m입니다. 현재 장치/드라이버가 동일한지도 확인합니다.

### 4. 상태·frame·제어권을 읽기 방식으로 확인

실제 namespace·토픽이 다르면 아래 조회 대상도 바꾸세요. `topic hz`는 관찰 후 Ctrl+C로 끝냅니다.

```bash
ros2 param get /mavros/setpoint_velocity mav_frame
ros2 topic info /mavros/setpoint_velocity/cmd_vel --verbose
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/extended_state --once
ros2 topic echo /mavros/estimator_status --once
ros2 topic info /distance/filtered --verbose
ros2 topic hz /distance/filtered
```

MAVROS frame은 `LOCAL_NED`여야 합니다. 이 코드에 넣는 xyz는 **ROS local ENU**이고 MAVROS가 NED로 변환합니다. 추가 수동 ENU→NED 변환이나 body frame 설정을 적용하지 않습니다. MAVROS 2.14 velocity plugin의 mask는 1479이며 **위치·가속도·yaw angle은 비활성, xyz 속도·yaw rate만 활성**입니다. `0` 속도는 축 무시가 아니라 해당 축의 0속도 명령입니다.

기존 mission·고도·yaw·v2 노드가 FC setpoint 토픽에 따로 발행하지 않도록 구성합니다. 노드는 position/raw/velocity/attitude 발행 충돌을 검사합니다. 운영 중인 임무를 임의 종료하지 말고 어떤 프로세스가 제어권을 가진지 먼저 확인하세요.

현재 RC 입력 소스·모드 스위치·OFFBOARD-loss 회수 절차와 ULog 기록 가능 상태를 확인·기록하세요. log154의 `COM_RC_IN_MODE=1`, `COM_RC_OVERRIDE=1`을 현재값으로 단정하거나 RC 스틱을 움직이면 자동 탈취된다고 보장하지 않습니다. 기존 PX4 안전 설정은 변경·완화하지 않습니다.

### 5. 준비 완료 후 전달할 내용

Codex는 적용한 설정·근거, 실제 repository/ROS/container 경로, 센서 토픽, 테스트·dry-run 결과, 자동 로그 위치와 미확인 항목을 보고합니다. 두 시험의 실제 실행 명령도 작성해 전달하되 준비 작업에서 비행 명령을 실행하지 않습니다.

`flight_settings_verified`와 `lidar_geometry_verified`는 운영자가 실제 설정·장착 보정을 확인했다는 표시입니다. 자동 교정 기능이 아니므로 준비 중에는 false로 유지하고, 별도 현장 비행 지시와 확인 후 true로 넘깁니다.

## 설치와 비행 없는 검증

기존 ROS 2 Jazzy 환경/컨테이너 안에서 실행합니다. 새 저장소의 ROS workspace가 실제로 컨테이너에 mount되어 있어야 합니다. 기존 mount가 옛 da-daka 폴더만 포함하면 현장 환경에 맞춰 소스 경로를 연결하세요.

```bash
sudo apt-get install python3-yaml ros-jazzy-mavros-msgs
cd we-meet-project/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select we_meet_flight_core we_meet_flight_v1
source install/setup.bash
```

새 셸에서는 ROS와 이 workspace의 `install/setup.bash`를 다시 source합니다. 파일 수정 후 다시 빌드하고 선택된 프로필·실제 snapshot을 확인합니다. v1은 카메라/OpenCV/NumPy가 필요 없습니다.

저장소 root에서 오프라인 검증:

```bash
python3 -m pip install -r requirements-test.txt
python3 tools/verify_shared_core.py
python3 -m unittest discover -s tests -v
python3 tools/offline_demo.py --output /tmp/v1_trial.jsonl
python3 tools/offline_demo.py --ascent-only
python3 tools/offline_demo.py --irregular
python3 tools/replay_commands.py /tmp/v1_trial.jsonl
```

현장 dry-run (FC 명령·모드/arming 서비스 쓰기 없음):

```bash
ros2 launch we_meet_flight_v1 trial.launch.py dry_run:=true
# 다른 ROS 셸, 센서가 준비된 지상 disarm 상태에서
ros2 service call /test_flying_v1/start std_srvs/srv/Trigger '{}'
ros2 topic echo /test_flying_v1/status
ros2 topic echo /test_flying_v1/debug/cmd_vel
```

지상 dry-run에서는 FC가 실제 OFFBOARD/armed로 바뀌지 않으므로 전체 비행 단계가 진행되지 않고 모드 전환 대기 후 종료됩니다. 전체 흐름은 합성 오프라인 데모로 확인합니다. 현장 dry-run은 입력/로그/시작 조건을 확인하며 가짜 FC 상태를 실제 토픽에 발행하지 않습니다.

## 현장 시험 실행 명령

아래 live 옵션은 실제 비행용입니다. 기체 설정/회수 절차·라이다 보정을 확인하고 사용자가 해당 시험을 실행하도록 지시한 뒤 사용합니다. 모든 프로세스는 시작 서비스 호출 전에는 자동 이륙하지 않습니다.

1차, 상승·수평 0속도 명령 5초·착륙:

```bash
ros2 launch we_meet_flight_v1 trial.launch.py profile:=ascent_only.yaml \
  dry_run:=false flight_settings_verified:=true lidar_geometry_verified:=true
```

2차, 약 5m 속도 전진·감속·정지명령·착륙:

```bash
ros2 launch we_meet_flight_v1 trial.launch.py profile:=velocity_trial.yaml \
  dry_run:=false flight_settings_verified:=true lidar_geometry_verified:=true \
  mavros_namespace:=/mavros lidar_topic:=/distance/filtered
```

기체 전방을 시험 방향에 맞추고 안정된 yaw·라이다·추정기·배터리·지상 disarm 상태에서 다음을 호출합니다. 이 호출은 live 모드에서 **OFFBOARD·arming·상승을 시작**합니다.

```bash
ros2 service call /test_flying_v1/start std_srvs/srv/Trigger '{}'
ros2 topic echo /test_flying_v1/status
# 제어권이 시험 노드에 있을 때 시험 중단 요청
ros2 service call /test_flying_v1/abort std_srvs/srv/Trigger '{}'
```

## 제어와 중단 처리

- prestream 후 OFFBOARD 요청과 실제 FC 모드 전환을 구분합니다. ARM 요청도 실제 armed 상태 관측과 구분합니다.
- `yaw_ref`는 이륙 전에 한 번 저장하며 상승·대기·전진·감속·수평 0속도 유지에서 같은 helper로 20Hz마다 계산합니다. 상태 전환 때 목표를 다시 저장하지 않습니다.
- 전진 ENU 방향은 **출발 yaw**로 고정합니다. 현재 yaw 변화에 맞춰 전진 방향을 매번 회전시키지 않습니다. yaw와 xyz는 한 `TwistStamped`에서 발행합니다.
- 감속 명령의 적분 면적까지 약 5m에 포함합니다. 5m를 채운 뒤 감속을 시작하지 않습니다. 이 계산은 명령 기준이며 실제 제동거리·공간 이탈을 제한하는 지오펜스가 아닙니다.
- 명령거리는 실제 local publish 시각의 monotonic dt로, 이전에 발행한 최종 ENU 명령을 고정 전방에 투영해 누적합니다. 부호를 유지하고 수직 속도는 제외합니다. 로컬 publish 성공은 FC 실행 ACK가 아닙니다.
- 5초 타이머는 고도·yaw·FC 추정 속도 조건을 만족하고 수평 0속도 명령을 실제로 로컬 발행한 때 시작합니다. 조건 이탈 시 초기화합니다. 추정 XY로 수평 보정 명령은 만들지 않으며 라이다·yaw 제어를 유지합니다.
- 라이다/yaw/추정기 invalid·stale, heading 점프, 통신/발행 실패, 긴 제어 공백, 단계/전체 제한시간에 중단 사유를 남깁니다. 긴 공백을 적분으로 따라잡거나 정상 도착으로 처리하지 않습니다.
- 수동/다른 모드·FC critical 상태가 관측되면 제어를 놓고 OFFBOARD를 다시 요청하지 않습니다. heading reset은 pose-yaw 변화와 IMU를 비교한 휴리스틱이며 모든 EKF reset을 완벽히 검출하는 것은 아닙니다.
- 시험 노드가 제어권을 가진 유효 상태에서 LAND를 요청합니다. 추정기 무효·통신/발행 실패 때 0속도가 안전 정지라고 가정하지 않습니다. LAND가 확인되면 속도·yaw 발행을 중단하고 FC에 넘깁니다.
- LAND 거부·무응답·연결 상실은 착륙 미확인/실패로 기록하고 stream을 중단해 기존 FC 회수 경로에 맡깁니다. 공중 DISARM은 보내지 않습니다. Ctrl+C/프로세스 예외도 stream을 끝내므로 현장 OFFBOARD-loss 절차가 먼저 검증돼 있어야 합니다.

## 자동 로그와 시험 후 가져올 파일

매 실행 즉시 `~/flight_logs/test_flying_v1/<run_id>/`를 만들고 준비 실패·수동 전환·중단·정상 종료를 기록합니다. 로그는 bounded queue의 별도 writer에 기록하며 쓰기 오류/누락 위험을 제어 루프에 전달합니다. 파일은 line buffering으로 저장하고 종료 시 요약을 남깁니다.

| 파일 | 내용 |
|---|---|
| `run_metadata.json` | run_id·UTC/monotonic 기준·코드 commit/변경·dry/live·통신 namespace·확인 플래그 |
| `config_snapshot.yaml` | 실제 로드 설정 |
| `telemetry.csv` | 모드·armed·라이다 높이·yaw·속도명령·센서/FC telemetry |
| `commands.jsonl` | 최종 명령·실제 publish 시각/간격·적분 구간·누적거리·frame/mask |
| `events.jsonl` | 단계·모드 상태 로그·service 요청/응답·중단·예외 |
| `summary.json` | 종료 결과·단계 시간·최종 명령거리·최대 속도/주기/발행 간격·5초 명령 유지·착륙 확인·기록 가능한 추정 이동량 비교 |
| `console.log`, `all.jsonl` | 주요 콘솔 사건/traceback 및 전체 기록 |
| `field_result.md` | 운영자가 잰 이동·착륙·호버 편차, run_id와 ULog 연결 정보 |

MAVROS 메시지 source stamp·수신 age를 기록합니다. 독립된 FC 원본 timestamp, live FW 버전, 전체 PX4 failsafe 플래그, ULog 기록 상태를 자동 조회하지 못한 값은 null/미확인으로 표시합니다. `MAV_STATE` critical/emergency 표시는 전체 failsafe 플래그와 같지 않습니다. FC 추정 이동량 역시 지면 기준 정답이 아닙니다.

```bash
python3 tools/replay_commands.py ~/flight_logs/test_flying_v1/<run_id>/commands.jsonl
```

비행 전에 QGroundControl MAVLink Console의 `logger status` 조회로 SD 기록 가능 상태를 확인하고 FC 버전·확인 결과를 현장 양식에 적습니다. run_id 로그의 UTC·arm/mode 전환과 ULog 구간을 연결하세요. FC 파일명 날짜가 잘못될 수 있으므로 파일명만으로 연결하지 않습니다.

착륙·disarm 후 QGroundControl **Analyze Tools → Log Download**에서 해당 ULog를 회수합니다. 비행 중 다운로드하지 않습니다. 또는 기체 전원을 끄고 SD 카드의 `log/`에서 원본 `.ulg`를 복사합니다. 원본은 보존하고 복사본에 `sha256sum <file>.ulg`를 실행합니다.

시험 후에는 **run_id 폴더 전체 + 원본 ULog와 해시 + 작성한 field_result.md + 현장 비행 영상**을 가져오세요. 실제 전방/좌우 이동량, 착륙 오차, 5초 정지 중 움직임을 별도로 측정해야 합니다.

## 변경 기록

- 2026-10-02: v1 전용 카메라 없는 속도/yaw 시험 구현. 상승 단독/약 5m 프로필, 실제 publish 시각 기반 명령 적분, 감속·5초 정지명령·LAND 인계, 자동 로그·사후 적분·무비행 테스트와 드론 Codex 적용 안내 추가.

- 2026-10-02: v1/v2 공통 제어를 we_meet_flight_core 0.2.0으로 통일. actual publish 적분·raw 라이다 기본값·안정/중단 조건·LAND 인계 공유. v2 카메라 hook, 명령 일치 검증 및 설정 이전 안내 추가.

- 2026-10-03: 실패 분석 기반 공통 코어 0.3.0. FC ODOMETRY 결합 reset·품질/이탈/일관성/yaw/라이다 감시, v1/v2 연속 안정 유지와 관측 모드 기록, 원본 스칼라 재생 및 Pi 설정 안내 추가.
