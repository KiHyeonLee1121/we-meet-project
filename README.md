# we-meet-project — test-flying-v2

## 이 브랜치의 목적

**기존 yaw 보정이 적용된 속도 시험을 바탕으로, 카메라에 패널이 보이면 감속하고 영상 중앙에 넓게 정렬해 5초간 유지하는 다음 단계 시험**을 위한 브랜치입니다.

흐름은 **약 3m 상승 → 출발 방향으로 속도 이동 → OpenCV 패널 검출 → 감속 → 카메라 기준 XY 정렬 → 중앙 영역에서 5초 유지 → 그 자리 AUTO.LAND**입니다. 카메라를 사용한 XY 보정은 패널 검출 이후에 적용하고, 고도는 라이다, 방향은 이륙 전 저장한 yaw 기준을 계속 사용합니다.

| 시험 프로필 | 목적 | 적용 순서 |
|---|---|---|
| `ascent_only.yaml` | 상승 → 수평 0속도 명령 5초 → 착륙 | 고도·yaw·모드 전환 확인 |
| `baseline.yaml` | 상승 → 명령 적분 약 5m 전진·감속 → 5초 유지 → 착륙 | 영상 시험 전에 기존 속도/yaw 시험 확인 |
| `panel_approach.yaml` | 속도 접근 중 패널 검출 → 감속 → 영상 중앙 정렬·5초 유지 → 착륙 | 앞선 시험이 현장에서 성공한 후 적용 |

이번 목표는 단일 패널에 대한 쉬운 정렬 검증입니다. 분사, 오염 인식, 여러 패널 탐색, 패널맵, 귀환은 구현 범위에 없습니다. 영상 중앙 유지와 수평 0속도 명령은 실제 지면 기준 위치·정지 정확도를 보장하지 않으며, ULog와 현장 측정으로 따로 평가합니다.

라즈베리파이 5 / ROS 2 Jazzy / MAVROS 2.14 / Pixhawk 구성을 제공 자료에서 확인해 재구성했습니다. **기존 시험 코드 원본과 현재 기체에 접속해서 만든 코드는 아니므로, 다운로드한 드론 컴퓨터의 Codex가 아래 현장 적용 절차를 수행해야 합니다.**

## 2026-10-02 실패 분석을 반영한 변경 (공통 코어 0.3.0)

**FC 리셋 카운터·지속 yaw 이탈·추정 품질/경로/속도 불일치·라이다/FC 수직속도 불일치 감시와 연속 5초 유지 판정을 보강했습니다.** v1/v2 공통 코어는 동일합니다. GPS/추정 XY로 이동 명령을 보정하지 않으며, 품질이 나쁘거나 상태가 어긋나면 시험을 중단합니다. 고도·방향·추정 속도가 유지 조건을 벗어나면 5초 타이머를 처음부터 다시 셉니다.

**다운로드한 드론 Codex는 [실패 후 수정·현장 연결 안내](docs/failure_followup_20261002.md)를 먼저 읽어야 합니다.** 기존 MAVROS 링크의 MAVLink 2 / FC ODOMETRY(331) 20Hz 수신, 실제 FC ID, GPS/FC 분산과 새 YAML 한계를 확인해야 합니다. launch에 `fc_mavlink_topic`, `fc_system_id`, `fc_component_id`를 추가했습니다. 수신 누락이나 기본 GPS sigma 0.5m 한계 초과 시 시작을 거부합니다. 당시 약 1.8m 품질은 이 기본값으로 차단됩니다. 새 한계는 실측 기체값이 아니라 현장 검토가 필요한 임시 시험 기준입니다.

카메라를 쓰는 v2도 FC의 속도 추정 의존이 남습니다. 실기체 설정·교정·스트림 연결은 Pi에서 확인하고, 준비 작업에서 비행 명령을 실행하지 않습니다. 원본 실패 신호의 개별 감시 재생은 `python3 tools/replay_failure_guards.py`로 확인할 수 있습니다.

## v1/v2 공통 로직 통일

두 브랜치 모두 동일한 **`we_meet_flight_core` 공통 패키지**를 사용합니다. v1의 실제 발행 시각 기반 명령 적분·라이다 raw 보정·상승 안정 조건·단계 제한시간·중단/착륙 인계를 기준으로 통일했습니다. 카메라가 정상이고 목표가 아직 검출되지 않았다면 같은 입력에서 같은 명령을 냅니다. v2는 카메라 상태 확인 및 패널 인식 이후 감속 분기·영상 정렬·5초 중앙 유지만 추가합니다.

**새 공통 패키지를 함께 빌드해야 합니다.** 기존 실행/서비스 이름은 유지됩니다. 세부 비교·기존 v2 설정 키/상태 이름 변경·코어 동일성 확인은 [shared_flight_logic.md](docs/shared_flight_logic.md)를 읽으세요.

## 내려받은 드론 컴퓨터의 Codex가 먼저 할 일

이 섹션은 라즈베리파이에서 이 저장소를 읽는 Codex에게 전달하는 작업 지침입니다. 기본 작업 범위는 **기존 환경 조사 → 현장 설정 반영 → 빌드 → 비행 없는 검증 → 미확인 항목 보고**입니다. 실제 비행은 이후 사용자의 별도 실행 지시가 있을 때 수행합니다. 확인용이라고 arming, OFFBOARD 전환, FC setpoint 발행, 분사를 실행하지 마세요.

### 1. 브랜치와 기존 환경 확인

처음 내려받는 경우 다음을 사용합니다. 기존 작업 폴더가 있으면 먼저 변경 내역을 확인하고 보존하세요. 기존 파일을 덮어쓰거나 `reset --hard`하지 마세요.

```bash
git clone --branch test-flying-v2 --single-branch https://github.com/KiHyeonLee1121/we-meet-project.git
cd we-meet-project
git branch --show-current
git rev-parse HEAD
git status --short
```

[재구성 근거](docs/implementation_notes.md)와 아래 설정 표를 읽고, 기존 `da-daka_Ai` 프로젝트의 compose/launch/YAML/카메라·라이다 드라이버를 확인하세요. 예전 전체 미션 launch는 자동 시동·분사 경로가 있을 수 있으므로 센서 확인용으로 실행하지 마세요.

과거 자료에서는 ROS가 Pi 호스트가 아니라 Docker 내부에 있었지만 현재도 동일한지 확인해야 합니다. 현재 컨테이너, ROS 배포판, 실제 소스 mount와 장치 매핑, MAVROS namespace·연결 방식, 센서 드라이버와 토픽을 조사하세요. 필요한 센서 프로세스는 기존 실행 내용을 확인해 센서 전용 경로로 연결하고, 이미 실행 중인 프로세스와 장치를 중복 점유하지 마세요.

```bash
# Pi 호스트에서 환경 조회
uname -m
cat /etc/os-release
docker ps -a

# 실제 ROS 환경/컨테이너에서 조회
printenv ROS_DISTRO
ros2 node list
ros2 topic list -t
ros2 service list
```

아래 기본 namespace는 `/mavros`입니다. 실제 namespace가 다르면 이후 명령도 그에 맞춰 읽으세요. MAVROS 연결 설정은 기존 운용 환경을 재사용하며 이 패키지에 FC 포트·baudrate를 새로 넣는 구조가 아닙니다.

### 2. 자동 입력과 현장 설정을 구분

| 구분 | 항목 | Codex가 해야 할 일 |
|---|---|---|
| 실행 중 자동 수신 | 현재 yaw·자세·라이다 값·배터리·FC 모드·armed/landed·추정기 유효성 | 실제 토픽과 신선한 데이터가 들어오는지 확인. 값을 상수나 가짜 센서로 채우지 않음 |
| 드론에서 조사 가능 | ROS/Docker 환경, 장치·드라이버, 토픽/타입/주기, MAVROS frame, 기존 yaw/Z 설정 | 현재 파일·노드·상태를 읽어 연결 및 설정에 반영하고 근거를 기록 |
| 현장 측정·관찰 필요 | 카메라 장착 방향, 영상 회전, 라이다 설치축·오프셋, 패널 높이·크기·영상 대비 | 기존 교정 자료가 있으면 현재 장착과 일치하는지 확인. 없으면 운영자에게 측정·사진·관찰 요청 |
| 시험으로 확인 필요 | 실제 제동·5m 편차·중앙 유지·RC 제어권 회수 | 무비행 검증 결과와 구분해 미검증으로 보고. 로그만으로 물리 정확도를 보장하지 않음 |

**드론 컴퓨터에 있다고 해서 카메라 장착 거리·방향 같은 물리값을 자동으로 아는 것은 아닙니다.** 확인되지 않은 값을 추측해 넣거나 실행 확인 플래그를 임의로 켜지 마세요.

### 3. 무엇을 어디에 설정하는가

YAML 경로의 공통 앞부분은 `ros2_ws/src/we_meet_flight/config/`입니다. 각 프로필은 독립 파일이며, 생략한 값은 `we_meet_flight_core/config.py` 공통 기본값과 `we_meet_flight/config.py` 영상 기본값을 사용합니다. 한 프로필을 수정해도 다른 프로필에 자동 반영되지 않습니다.

| 설정 위치 | 정확한 이름·기본값 | 현장에서 확인·반영할 내용 |
|---|---|---|
| launch 인자 | `mavros_namespace=/mavros` | 실제 MAVROS prefix. 예상 상태 토픽·서비스가 이 prefix 아래 존재해야 함 |
| launch 인자 | `estimator_topic=/mavros/estimator_status` | 실제 추정기 상태 토픽. 기본 경로는 MAVROS namespace에 맞춰 연결 |
| launch 인자 | `lidar_topic=/distance/filtered` | 실제 `sensor_msgs/msg/Range` 입력. 목표 3m 측정 가능 여부와 메시지 min/max 범위 확인 |
| launch 인자 | `image_topic=/camera/image_raw` | 실제 `sensor_msgs/msg/Image` raw 입력. BGR/RGB/mono8, stamp, 주기 확인. compressed 전용이면 raw 변환 연결 필요 |
| launch 인자 | `log_directory=~/flight_logs/test_flying_v2` | ROS 실행 계정의 실제 저장 위치·쓰기 권한·공간. Docker라면 호스트에 보존되는 mount인지 확인 |
| 선택적 `camera_source` 노드 인자 | `backend=picamera2`, `device=0`, `rotation_degrees=180` | 실제 CSI/USB 경로·접근 권한·회전. 기존 영상에서 이미 회전했다면 중복 회전하지 않음 |
| YAML | `image_to_body: [[0,-1],[-1,0]]` | 영상 오차를 body 전방/좌측으로 바꾸는 행렬. 실제 최종 영상 방향을 확인해 조정 |
| YAML, 세 프로필 각각 | `lidar_input_is_vertical_height: false` | 기존 필터는 raw 거리 평활화만 함. 실제 드라이버가 기울기·오프셋까지 이미 보정한 높이면 `true` |
| YAML, 세 프로필 각각 | `lidar_body_down_offset_m: 0.0` | raw 입력일 때 기체 기준점 아래의 센서 장착 거리(m). 0은 측정 결과가 아닌 예시 기본값 |
| YAML, 세 프로필 각각 | `maximum_tilt_rad: 0.35` | 입력 처리를 허용하는 roll/pitch 범위. 비스듬히 설치된 센서를 이 값으로 교정하지 않음 |
| YAML / Config 기본값 | `yaw_kp=1.0`, `yaw_max_rate_rad_s=0.35` | 기존 `yaw_rate_command()`와 설정을 유지. 현재 기체 코드의 값과 다르면 근거를 확인해 반영 |
| YAML / Config 기본값 | Z의 `vertical_kp=0.6`, `vertical_kd=0.15`, `vertical_speed_mps=0.25`, `vertical_accel_mps2=0.5` | 기존 고도 제어 설정과 센서 단위를 확인. 코드의 기본값을 현장 검증값으로 표시하지 않음 |
| YAML | `target_height_m=3.0`, `commanded_target_distance_m=5.0`, `zero_velocity_hold_s=5.0` | 시험 사양. 5m는 명령 적분 예산, 3m는 라이다 반사면 기준 보정 높이 |
| YAML / Config 기본값 | `forward_speed_mps=0.35`, `horizontal_accel_mps2=0.25`, `visual_max_speed_mps=0.12`, `visual_gain_mps=0.35` | 새 시험의 초기값. 실제 이동거리·제동 성능을 교정한 값이 아님 |
| YAML | `center_half_width=0.15`, `center_half_height=0.15` | 화면 중앙 가로·세로 30% 허용 영역 유지. 실제 미터 단위 허용 오차가 아님 |
| YAML | `panel_dark_threshold=130`, 면적/종횡비/어두운 픽셀 비율, `acquisition_frames=3` | 실제 패널 사진·영상으로 검출과 오인식을 점검해 조정 |
| YAML / Config 기본값 | `sensor_timeout_s=0.3`, `camera_timeout_s=0.3`, `state_timeout_s=2.5`, `maximum_tick_gap_s=0.2` | 수신·처리 지연과 시계 동기 확인. 실행 오류를 없애기 위해 freshness 검사를 삭제하거나 무작정 완화하지 않음 |

launch 인자는 실행 시 넣고, 기체 교정값·제어값은 선택할 YAML에 넣으세요. 기존 드라이버의 이미 적용된 보정과 중복되지 않도록 확인하고, 실제 적용 파일·값·측정 근거를 기록하세요. 설치 후 YAML/launch 파일을 수정했다면 아래 빌드를 다시 실행하고 실제 로드 설정을 `config_snapshot.yaml`에서 확인하세요.

### 4. 비행 없이 확인할 항목

실제 ROS 환경에서 상태와 타입을 조회하세요. 아래는 읽기 명령이며 서비스를 통한 모드/시동 변경은 포함하지 않습니다.

```bash
ros2 param get /mavros/setpoint_velocity mav_frame
ros2 topic info /mavros/setpoint_velocity/cmd_vel --verbose
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/extended_state --once
ros2 topic echo /mavros/estimator_status --once
ros2 topic info /distance/filtered --verbose
ros2 topic hz /distance/filtered
ros2 topic info /camera/image_raw --verbose
ros2 topic hz /camera/image_raw
```

`topic hz`는 관찰 후 Ctrl+C로 끝냅니다. 필요한 토픽이 없으면 원인·빠진 드라이버·연결을 보고하세요. GPS 위치로 거리 판단을 대체하거나 센서 검사를 삭제해서 실행시키지 마세요.

- MAVROS 속도 plugin frame은 **LOCAL_NED**여야 합니다. 이 패키지 입력은 ROS ENU이고 MAVROS가 변환하므로 수동 NED 변환을 더하지 않습니다.
- 기존 mission·고도·yaw·visual-servo 등이 FC 명령 토픽에 직접 발행하는지 확인합니다. 새 노드만 단일 xyz+yaw-rate 명령을 발행하도록 구성합니다. 기존 노드를 무작정 실행하거나 운영 중인 임무를 임의 종료하지 마세요.
- 라이다 보정식은 body-down 축과 정렬된 센서·평탄한 면을 전제로 합니다. 다른 설치축이나 지면/패널 높이 차이가 있으면 필요한 추가 보정과 현장 전제를 보고합니다.
- 영상 위/아래/좌/우에 패널을 놓고 **실제 최종 영상에서** 기체 전방/후방/좌측/우측 관계를 확인합니다. 기본 행렬에서는 영상 위→기체 전방, 영상 오른쪽→기체 우측입니다. 저장 영상과 오프라인 함수로 변환 부호를 검사하고 FC에는 보내지 않습니다.
- `tools/inspect_panel.py`는 검출 후보와 중앙 영역을 그리는 도구입니다. 이동명령이나 기체 축 화살표를 출력하는 도구는 아닙니다.
- 예전 RC 설정을 현재값으로 단정하지 않습니다. 현재 FC 파라미터와 RC 모드 스위치·OFFBOARD-loss 회수 계획을 확인하고 기록합니다. 설정을 임의 변경·완화하지 않습니다.

### 5. 빌드·오프라인 시험·dry-run 후 전달

아래 설치/검증 명령을 실제 ROS 환경에 맞춰 실행하세요. `dry_run=true`는 FC 명령 토픽과 모드/arming 서비스에 쓰지 않습니다. **지상 dry-run은 실제 FC가 OFFBOARD/armed가 되지 않으므로 전체 비행 단계가 진행되는 시험이 아닙니다.** 전체 상태 전환·영상 제어는 오프라인 데모/테스트로 확인하고, 현장 dry-run은 입력 연결·로그·시작 조건 확인으로 사용하세요. 정렬 부호 확인을 위해 실제 FC 상태를 위조하지 마세요.

완료 시 사용자에게 다음을 전달하세요.

1. 확인한 OS/ROS/컨테이너·저장소 경로·commit, 실제 센서 토픽/드라이버/MAVROS 연결.
2. 바꾼 파일과 적용값, 기존 설정에서 가져온 근거와 현장 측정 근거.
3. 빌드·테스트·실제 영상 검출·dry-run 결과와 자동 로그의 실제 경로.
4. 코덱스가 확인할 수 없어 운영자가 측정/확인할 항목, 아직 미검증인 비행 성능.
5. 해당 기체의 경로·토픽·프로필에 맞춘 실행 명령. 비행 명령을 만들어 전달하되 현재 작업 중 실행하지 않음.

`flight_settings_verified`, `lidar_geometry_verified`, `camera_axes_verified`는 각각 비행 설정, 라이다 보정/설치, 카메라 축을 확인했다는 운영자 표시입니다. **코드가 자동 교정했다는 뜻이 아니며, 준비 작업에서는 false로 유지합니다.** 이후 현장 확인을 마치고 사용자가 해당 시험 실행을 지시할 때 필요한 항목만 true로 넘깁니다.

## 동작

| 구간 | 동작 |
|---|---|
| 시작 | 지상·disarm·센서 상태를 확인하고 안정된 yaw를 한 번 저장 |
| 상승 | 라이다 기반 보정 높이 약 3m까지 상승, XY 속도 0, 같은 yaw 유지 |
| 안정 대기 | 높이오차 ±0.10m·변화율 ±0.05m/s·yaw오차 ±5도가 연속 2초 안정되면 전진 |
| 접근 | 출발 yaw를 기준으로 최대 0.35m/s, 최종 발행 명령 적분으로 약 5m의 전진 예산 관리 |
| 패널 검출 | 서로 다른 3개 영상에서 같은 어두운 사각형 패널을 확인하면 전진 감속 |
| 영상 정렬 | 현재 yaw로 영상 오차를 ENU 속도로 변환, 최대 0.12m/s |
| 정지 | 패널 중심이 화면 중앙 **가로 30% × 세로 30%** 안에 있고 고도·yaw가 안정된 상태를 연속 5초 유지 |
| 착륙 | AUTO.LAND를 요청하고 FC가 전환하면 속도 발행을 중단; 착륙·disarm을 확인 |

패널 중앙은 검출한 사각형의 영상 중심입니다. 하향 카메라의 영상 중심 아래에 패널을 정렬하는 시험이며, 노즐 위치 보정·분사·여러 패널 탐색·패널맵·귀환은 포함하지 않습니다. 카메라가 아래를 보는 배치를 전제로 합니다. 사선/전방 카메라의 경우 이 XY 제어식을 그대로 사용하면 안 됩니다.

`baseline.yaml`은 카메라 없이 **상승 → 속도 명령 적분 5m → 감속 → 5초 대기 → 착륙**을 실행합니다. 먼저 기존 yaw 시험을 이 프로필로 확인할 수 있습니다.

**5m는 실제 이동거리의 측정값이 아닙니다.** `commanded_distance_m`은 실제 발행한 이전 최종 전방 속도를 실제 local publish 시각 사이의 monotonic dt로 적분한 값입니다. 감속 거리를 포함하며, 영상 전환 뒤 정렬 거리는 이 값에 추가하지 않습니다. FC 추정 XY/속도와 GPS 품질은 중단 감시·추정 속도 유지 판정·기록에 사용하며 이동 명령을 보정하지 않습니다. FC의 EKF/GNSS 설정은 변경하지 않으며, 속도 제어 자체도 FC의 추정 속도에 의존합니다. 상승 중 XY 속도 0 역시 GPS 없는 실제 위치 고정을 보장하지 않습니다.

3m는 **라이다가 보는 면에서 기체 기준점까지의 수직 높이 목표**입니다. `lidar_input_is_vertical_height: true`는 입력 드라이버에서 기울기·장착 오프셋이 이미 보정됐다는 뜻입니다. 기본은 `false`이며 기존 필터의 raw 거리 입력에 맞췄습니다. raw 입력이라면 `lidar_body_down_offset_m`(기체 기준점 아래의 센서 거리)을 측정하세요. raw 입력은 `(거리 + 오프셋) × cos(roll) × cos(pitch)`로 한 번만 보정합니다. 하향 body 축과 정렬된 라이다·평탄한 면을 전제로 하며 다른 설치각이면 별도 외부 보정이 필요합니다. `lidar_geometry_verified`는 이 확인을 마쳤다는 표시입니다. 지면에서 패널 위로 넘어갈 때 라이다 반사면 높이가 바뀌면 고도 제어에도 영향이 생깁니다. 이번 시험은 평탄한 지면의 낮고 고정된 단일 패널, 하향 카메라·라이다 배치부터 확인하는 구성입니다.

## 구성

- `ros2_ws/src/we_meet_flight_core/`: v1/v2 동일한 명령 적분·비행 제어·센서/FC 어댑터·로그
- `ros2_ws/src/we_meet_flight/`: ROS 2 패키지, 단일 속도 발행 노드, 선택적 카메라 입력 노드
- `config/baseline.yaml`, `config/panel_approach.yaml`: 시험 설정
- `tools/inspect_panel.py`: 저장된 사진/영상에서 OpenCV 검출 결과 확인
- `tools/offline_demo.py`: FC 연결 없는 합성 영상·간단한 이동 모델 시험
- `tests/`: 상태 전환, 명령 적분, yaw, 영상 검출·추적, 중단 조건 시험
- [자료 해석과 검증 범위](docs/implementation_notes.md)

## 설치

기존 MAVROS와 `/distance/filtered` 라이다 발행기는 재사용합니다. 예전 `distance_controller`, mission, visual-servo 등 **FC에 setpoint를 발행하는 노드들은 함께 실행하지 않습니다.** 이 패키지가 xyz 속도와 yaw rate를 하나의 메시지에 합쳐 20Hz로 발행합니다.

기체의 **ROS 2 Jazzy 환경/컨테이너 내부**에서:

```bash
sudo apt-get install python3-opencv python3-numpy python3-yaml ros-jazzy-mavros-msgs
cd we-meet-project/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select we_meet_flight_core we_meet_flight
source install/setup.bash
```

ROS 패키지 의존성은 `package.xml`에 있습니다. Picamera2는 기본 flight 노드의 필수 의존성이 아닙니다. 기존 카메라 파이프라인에서 `sensor_msgs/msg/Image`를 발행할 수 있으면 그대로 사용합니다.

## 카메라 입력과 검출 확인

기본 입력은 `/camera/image_raw`의 BGR/RGB/mono8 raw Image입니다. compressed 전용이면 raw Image로 변환하거나 기존 발행기의 raw 토픽을 지정합니다. 영상의 source stamp가 오래되거나 반복되면 정상 영상으로 인정하지 않습니다. 검출은 어두운 볼록 사각형·면적·종횡비·어두운 픽셀 비율을 사용합니다. 어두운 상자나 바닥 무늬를 패널로 오인할 수 있으므로 실제 촬영 영상으로 조정해야 합니다.

저장된 사진으로 먼저 확인:

```bash
python3 tools/inspect_panel.py panel_photo.jpg --output panel_overlay.png
```

중앙의 녹색 사각형은 정지 허용 영역, 노란 테두리는 검출 후보입니다. 밝은 패널·반사광·그림자 조건에 따라 `panel_dark_threshold`, 면적, 종횡비 설정을 조정합니다. 잘린 패널은 중심이 불확실하므로 검출 후보에서 제외합니다. 단일 패널이 충분히 화면 안에 들어오는 높이와 출발 거리를 먼저 확인해야 합니다.

직접 CSI 카메라를 사용할 수 있는 ROS 환경이라면 선택적 발행기:

```bash
ros2 run we_meet_flight camera_source --ros-args \
  -p backend:=picamera2 -p rotation_degrees:=180
```

이 경우 Picamera2/libcamera와 IMX708 접근 권한이 실행 환경에 있어야 합니다. Pi 호스트에만 Picamera2가 있고 ROS는 컨테이너에만 있다면 기존 호스트-컨테이너 영상 연결을 사용하세요. 이 저장소가 Docker 카메라 접근 환경까지 자동 구성하지는 않습니다. USB 카메라는 `-p backend:=v4l2 -p device:=0`을 사용할 수 있습니다.

제공 자료의 180도 회전 영상 기준 기본 행렬은 `image_to_body: [[0,-1],[-1,0]]`입니다. **실제 영상에서 오른쪽이 기체 오른쪽, 아래가 기체 뒤쪽인지 확인하세요.** 출발 방향은 현장에서 기체 전방을 목표물 쪽으로 맞춥니다. 저장 영상의 목표를 화면 위/아래/좌/우에 놓고 오프라인 변환 함수로 생성되는 방향을 확인합니다. 지상 dry-run의 전체 정렬 단계가 자동 진행되는 것은 아닙니다. 영상 XY는 body FLU로 변환한 뒤 현재 yaw로 ENU에 변환하고, 전진 구간은 저장한 출발 yaw만 사용합니다.

## 실행

기본은 `dry_run=true`입니다. FC 속도 토픽과 모드/arming 서비스에는 쓰지 않고 `/test_flying_v2/debug/cmd_vel`에만 출력합니다. 실제 센서 입력이 있어야 시작할 수 있고, dry run이 FC의 OFFBOARD/arm 상태를 가짜로 만들어 주지는 않습니다. 전체 흐름은 아래 오프라인 데모로 확인합니다.

```bash
ros2 launch we_meet_flight trial.launch.py
ros2 service call /test_flying_v2/start std_srvs/srv/Trigger '{}'
```

실기체에서는 먼저 FC의 OFFBOARD-loss/RC 모드 스위치·착륙 절차, 라이다와 카메라 방향, 센서 주기, 비행 구역과 설정을 확인해야 합니다. `flight_settings_verified`와 `camera_axes_verified`는 그 확인을 했다는 실행자 표시이며 센서를 자동 교정하는 기능이 아닙니다.

MAVROS 2.14의 `/mavros/setpoint_velocity` 노드에서 `mav_frame`이 문자열 `LOCAL_NED`여야 합니다:

```bash
ros2 param get /mavros/setpoint_velocity mav_frame
ros2 topic info /mavros/setpoint_velocity/cmd_vel --verbose
```

`LOCAL_NED`는 MAVLink 출력 프레임입니다. 이 코드가 넣는 xyz 값은 ROS ENU이며 MAVROS가 NED로 변환합니다. `header.frame_id=map`을 쓰는 것만으로 plugin 프레임이 바뀌지는 않습니다. BODY_NED 설정으로 이 ENU 명령을 보내면 안 됩니다. 코드는 parameter service로 이를 읽고 다른 setpoint 발행기도 확인합니다.

1차 상승·정지·착륙 시험 (전진 없음):

```bash
ros2 launch we_meet_flight trial.launch.py profile:=ascent_only.yaml \
  dry_run:=false flight_settings_verified:=true lidar_geometry_verified:=true
```

2차 카메라 없는 기존 5m 시험:

```bash
ros2 launch we_meet_flight trial.launch.py profile:=baseline.yaml \
  dry_run:=false flight_settings_verified:=true lidar_geometry_verified:=true
```

기존 시험 성공 후 영상 시험:

```bash
ros2 launch we_meet_flight trial.launch.py profile:=panel_approach.yaml \
  dry_run:=false flight_settings_verified:=true lidar_geometry_verified:=true camera_axes_verified:=true \
  image_topic:=/camera/image_raw
```

각 실행은 자동 출발하지 않습니다. 지상 disarm 상태, 안정된 yaw·라이다·배터리·영상 입력이 준비된 뒤 `/test_flying_v2/start`를 호출하면 **OFFBOARD와 arming을 요청하고 이륙**합니다. 한 프로세스에서 시험은 한 번만 시작할 수 있습니다. 다시 하려면 노드를 재시작합니다.

```bash
ros2 service call /test_flying_v2/start std_srvs/srv/Trigger '{}'
ros2 topic echo /test_flying_v2/status
# 시험 중 중단 요청
ros2 service call /test_flying_v2/abort std_srvs/srv/Trigger '{}'
```

패널을 잃으면 XY 명령을 0으로 감속하고, 0.6초 이상 잃거나 카메라 자체가 0.3초 이상 오래되면 중단·착륙으로 전환합니다. 5m 전진 예산 안에 패널을 잡지 못하면 성공 처리하지 않습니다. yaw/라이다 오류, heading 점프, 제어 주기 지연도 중단 사유입니다. 모드가 수동/다른 모드로 바뀌거나 disarm되면 제어를 놓고 OFFBOARD를 다시 요청하지 않습니다.

AUTO.LAND는 FC가 수행합니다. 서비스 거부/무응답 또는 연결 상실 시 정상 착륙을 보장할 수 없으므로 실패를 기록하고 재진입 없이 FC의 검증된 OFFBOARD-loss 절차에 맡깁니다. Ctrl+C도 setpoint 송출을 끝내므로 정상 중단은 abort 서비스/현장 RC 모드 스위치 절차를 사용합니다. 제공 log154의 RC 설정만으로 스틱 움직임이 OFFBOARD를 해제한다고 가정할 수 없습니다.

## 로그와 오프라인 검증

매 실행마다 `~/flight_logs/test_flying_v2/<run_id>/`에 설정·dry/live 여부, 20Hz 명령과 실제 dt, 상태 전환, FC 모드/arm, 고정 yaw·오차·yaw rate, 명령 적분 거리, 라이다 거리/변화율/나이, 카메라 source stamp/나이, 패널 중심 오차·추적 정보, 영상 후보, 서비스 요청/응답, 결과·중단 사유를 자동 기록합니다. local position/velocity는 분석용으로만 기록합니다. 로그 쓰기 실패도 중단 사유입니다.

각 폴더에 `run_metadata.json`, `config_snapshot.yaml`, `telemetry.csv`, `commands.jsonl`, `events.jsonl`, `summary.json`, `console.log`, `all.jsonl`, 현장 측정용 `field_result.md`를 저장합니다. 버전/FC 로깅 상태처럼 자동 조회하지 않은 값은 null로 둡니다. 명령 publish는 로컬 ROS 전송 성공을 뜻하며 FC의 실행 확인이 아닙니다.

```bash
python3 tools/replay_commands.py ~/flight_logs/test_flying_v2/<run_id>/commands.jsonl
```

`/mavros/estimator_status`의 자세·수평/수직 속도 유효성과 data age를 필수 검사합니다. 토픽이 없으면 이 시험은 실행되지 않으며, GPS 위치로 거리 계산을 대체하지 않습니다. 실제 PX4/MAVROS에서 해당 상태 메시지가 전달되는지 확인해야 합니다.

원본 FC ULog도 기존 수집 방식으로 함께 보관해야 실제 궤적·속도·추정기 영향을 비교할 수 있습니다. 이 노드가 PX4 ULog를 다운로드하거나 FC 로깅 설정을 바꾸지는 않습니다.

```bash
python3 -m pip install -r requirements-test.txt
python3 tools/verify_shared_core.py
python3 -m unittest discover -s tests -v
python3 tools/offline_demo.py --output /tmp/visual_trial.jsonl
python3 tools/offline_demo.py --baseline --output /tmp/baseline_trial.jsonl
```

오프라인 시험은 코드 동작을 확인하는 합성 시험입니다. 실제 패널 영상과 ROS 2/기체에서의 통합·비행 성공은 별도 확인이 필요합니다.

## 시험 후 가져올 자료와 ULog 회수

비행 전에 QGroundControl의 MAVLink Console에서 `logger status`를 읽어 SD 기록 가능 상태를 확인하고 run_metadata.json/field_result.md에 관찰 결과·FC 버전을 적습니다. 노드 로그의 UTC 시작/모드·arm 전환과 FC ULog의 arm/mode 구간을 함께 비교해 연결하세요. FC의 UTC가 잘못된 경우 파일명 날짜만으로 연결하지 않습니다.

착륙·disarm 확인 후 QGroundControl **Analyze Tools → Log Download**에서 해당 ULog를 다운로드합니다. 비행 중 다운로드하지 않습니다. QGC에서 회수가 안 되면 기체 전원을 끈 뒤 SD 카드 `log/`에서 원본 `.ulg`를 복사합니다. 원본은 수정·삭제하지 말고 복사본에 `sha256sum <file>.ulg`를 실행해 해시를 남깁니다.

팀에 가져올 자료는 해당 run_id 폴더 전체, 원본 ULog와 해시, 측정값을 적은 field_result.md, 실제 패널·카메라 장착 사진, 시험 영상입니다. 이 코드로 FC 로깅 활성 여부를 자동 확인하거나 firmware version을 live query한 것은 아닙니다.

## 변경 기록

- 2026-10-02: README 서두에 브랜치 목적·시험 순서 추가. 드론 컴퓨터 Codex용 환경 조사, 설정값/경로 표, 현장 측정 구분, 무비행 검증 및 전달 절차 정리.

- 2026-10-02: 진단·수동/자율 ULog 근거로 속도/yaw 시험 재구성, OpenCV 단일 패널 감속·정렬·5초 유지 추가. 상승 단독/5m/영상 시험 프로필, 자동 로그·명령 적분 재현, 무비행 검증 추가.

- 2026-10-02: v1/v2 공통 제어를 we_meet_flight_core 0.2.0으로 통일. actual publish 적분·raw 라이다 기본값·안정/중단 조건·LAND 인계 공유. v2 카메라 hook, 명령 일치 검증 및 설정 이전 안내 추가.

- 2026-10-03: 실패 분석 기반 공통 코어 0.3.0. FC ODOMETRY 결합 reset·품질/이탈/일관성/yaw/라이다 감시, v1/v2 연속 안정 유지와 관측 모드 기록, 원본 스칼라 재생 및 Pi 설정 안내 추가.
