# test-flying-prototype — 141번 원본 비행 재현

이 브랜치의 목적은 **첨부 압축파일의 141번 비행을 가능한 한 그대로 재현하는 것**이다. 육안상 조사 경로가 거의 완벽했던 당시의 이륙, yaw 유지, 조사 경로, 패널 접근 흐름을 독립적으로 복원했다. 구현 근거는 `DA_DAKA_flights_141_153_external_review_20261002_v2.zip` 안의 **141 원본 ULog·141 현장 기록·당시 참고 코드**뿐이다. v1/v2 코드와 비행 정책을 가져오지 않았다.

**141의 성공 범위는 조사 경로다.** 이후에는 첫 패널을 재획득하지 못하고 `REACQUIRE timeout`으로 착륙했다. 이 브랜치는 그 이후 흐름과 당시 알려진 문제도 보존한다. 141 전체 임무가 성공했다고 가정하거나, 패널 위 정지/청소 성공을 강제로 만들지 않는다.

실제 배포 소스가 완전히 확인된 자료는 없다. 첨부파일도 세 참고 스냅샷 모두 `exact_as_flown_verified:false`로 표시한다. 따라서 이것은 **원본 로그에 맞춘 재구현**이며 당시 실행 파일의 완전한 복사본은 아니다. 아래에서 확인된 값과 재구성한 부분을 구분한다.

## 비행 과정

1. 시작 요청 시 local ENU 위치와 Home을 저장하고, 안정된 이륙 전 yaw를 고정한다.
2. 기존 수직 속도 제어기로 LiDAR 기준 **3m**까지 이륙한다. XY 속도 명령은 0이고 yaw rate로 기수 방향을 유지한다.
3. `AUTO.LOITER`를 거쳐 위치 제어 `OFFBOARD`로 전환한다. Home을 따라 보정한 이륙 기준점에서 위치·속도·yaw 안정 조건을 만족할 때까지 기다린다.
4. 기수 방향에 정렬된 **가로 3m × 세로 2m**, **전방 추가 오프셋 0m** 조사 사각형을 돈다. 네 모서리를 순서대로 방문하고 중심으로 돌아온다. 최대 진행 속도는 **0.50m/s**, 위치 setpoint를 점진적으로 이동시킨다. 카메라 관측으로 패널 지도를 만든다.
5. 조사 종료 후 패널 방문 순서를 정하고, 기존 수직 속도 제어기로 LiDAR 기준 **1m**까지 하강한다. 다시 위치 제어로 첫 패널 중심에 접근한다.
6. 패널을 재획득할 때까지 반경 **0.25m**, 주기 **5초** 원운동을 수행한다. 재획득 시간 제한은 **20초**다. 재획득되면 당시 FSM의 다음 상태로 진행하고, 실패하면 원래의 abort/착륙 경로를 따른다. 분사 출력은 mock/비활성이다.

수직 구간은 `/mavros/setpoint_velocity/cmd_vel`, 조사·접근 구간은 `/mavros/setpoint_position/local`을 사용한다. GPS는 시작 기준을 교차 검산하며, 조사 명령 자체는 MAVROS local ENU 위치 setpoint다. FC의 local 추정에는 당시 GPS/EKF가 사용됐다. 고정 시간으로 로그를 재생하거나 과거의 절대 좌표를 새 기체에 명령하는 코드가 아니다.

## 141에 맞춘 값과 근거

| 항목 | 이 브랜치 | 근거/확실성 |
|---|---|---|
| 이륙/하강 거리 | LiDAR 3m / 1m | 141 현장 기록과 참고 launch |
| 조사 경로 | 3×2m, yaw 정렬, 전방·좌측 오프셋 0m | **원본 141 FC 목표점으로 확인**. 참고 wrapper의 전방 +1m 기본값을 0으로 덮어씀 |
| 조사 yaw | 시작 전 안정된 yaw를 계속 유지 | 원본 141은 ENU −120.8676529°. 이 숫자는 검증용이며 실행 시 현재 yaw를 읽음 |
| 조사 진행 속도/제한 | 0.50m/s / 65초 | 첨부 시험 launch; 총 조사 시간 제한은 안정화 후부터 계산하는 당시 로직 |
| 기준점 보정 | `현재 Home XY + (시작 local XY − 시작 Home XY)` | 141 현장 기록을 구현. 원본 최종 기준점과 **0.52mm** 이내 일치(현장 입력이 mm로 반올림됨) |
| 시작 GPS/Home 검산 | 1.5m, Home 최대 이동 10m | 첨부 현장 기록의 당시 설정 |
| 패널 방문 순서 | 기록된 ID 우선순위 `(3,1,2,6)` | **재구성**. 참고 경로 계획기는 같은 지도에 `(6,2,1,3)`을 반환하므로 명시적 우선순위로 당시 결과를 맞춤 |
| 지도 투영 | 1m에서 0.52×0.31m, 카메라 전방 +0.07/좌측 +0.05/위 −0.16m | 당시 참고 `panel_survey.yaml` 유지 |
| 패널 병합 | 반경 0.45m, confidence weighted mean | 141 현장 기록과 참고 코드. 이후의 0.55m·median 변경을 적용하지 않음 |
| 관측 조건 | confidence ≥0.65, 관측 ≥8회, pose 동기화 0.15초, tilt 제한 7° | 참고 설정 유지 |
| 카메라 | 640×480, 15fps, 180° 회전, shutter 50000µs, gain 32 | 첨부 야간 설정과 메타데이터. 아래의 `night_asfound`는 과거 기본 camera flags를 재현하는 프로필 |
| takeoff 제어 | kp 1.0, 최대 1.50m/s, 최대 가속 4.0m/s² | 참고 launch 그대로. 설치 파일 기본값보다 launch override가 우선 |
| 하강 제어 | kp 0.70 / ki 0 / kd 0.15, 최대 0.25m/s | 참고 launch/설정 그대로 |
| 조사 전 안정화 | XY 0.30m, 속도 0.35m/s, yaw 3°, 2초 | 참고 코드 그대로 |
| 접근 도착 조건 | XY 0.25m, 속도 0.10m/s, 1초 | 참고 코드 그대로 |
| 기존 점검 | 배터리·LiDAR·MAVROS·EKF·AI 상태, 고도 상한 등 | 첨부 소스의 기존 점검 유지 |

**Home 보정은 당시 동작을 재현하기 위해 유지했다.** Home 재고정은 실제 local 원점 이동과 같지 않지만, 141에서는 그 변화만큼 목표 기준점이 옮겨졌다. 이를 올바른 위치 오차 보정으로 주장하지 않는다. 141의 heading reset 약 −3.185°에도 기존 yaw 목표를 유지한다. 이후 스냅샷의 origin/yaw reset 보정이나 v1/v2 중단 규칙을 추가하지 않았다.

패널 ID 토픽 불일치도 당시 상태로 남겼다. mission은 `/panel_localization_test/current_panel_id`를 발행하고 원본 sender는 `/autonomous_cleaning/current_panel_id`를 구독한다. 이로 인한 `clean mode requires a positive panel ID`는 141의 재획득 실패 설명과 일치한다. 성공적인 후속 패널 정렬을 만드는 변경은 이 브랜치의 목적에 포함되지 않는다. 참고 nozzle servo의 별도 투영 기본값도 수정하지 않았다.

## 파일 구조와 재구성 범위

- `ros2_ws/src/da_daka_control`, `da_daka_interfaces`, `laptop_ai`, 루트 `config`, `tools/pi_panel_cpu_runtime.py`: 첨부 **33dcf0703de41b7225171e7540c7429e08bc99fb** 스냅샷에서 138개 파일을 수정 없이 복사했다. 원래 FSM, 제어기, 필터, OpenCV 검출기와 UDP 프로토콜을 사용한다.
- `ros2_ws/src/we_meet_prototype141`: 새 ROS 패키지. 원본 mission을 상속해 기록에 있는 Home 보정과 패널 순서를 복원하고, 141 프로필과 로그 기록을 추가한다.
- `evidence/flight141_reference.json`: 원본 141 목표점·Home·yaw reset·현장 패널 지도. **검증 전용**으로 live mission에 과거 좌표/가짜 패널을 넣지 않는다.
- `evidence/px4_parameters_141.json`: 141 원본의 초기 PX4 파라미터 **1,146개**와 firmware/hash. FC에 자동으로 쓰는 코드가 아니다.
- `evidence/archive_source_manifest.json`: 수정하지 않은 원본 파일의 SHA256 목록.
- `tools/verify_141.py`: ROS/FC 연결 없이 원본 무결성, 경로, Home 기준점, yaw와 재획득 원운동을 검증한다.

추정이 남는 부분: 정확한 배포 커밋·설치된 의존성 버전·원래 경로 계획 알고리즘·일부 Home/GPS callback 시점·카메라의 모든 런타임 flag는 증명할 수 없다. 시작 GPS 신선도는 참고 `status_timeout_s`에 맞춰 6초로 선택했다. Home은 지속 보관되는 기준점이라 latched 토픽으로 받고 나이가 들었다는 이유로 무효화하지 않는다. 이미 생성된 조사 모서리는 이후 Home 변화에 다시 옮기지 않는다. **141에서는 조사 시작 후 Home XY 변화가 없었으므로** 이 경우의 원래 동작은 확인되지 않았다.

`historical_panel_order_ids: []`로 바꾸면 첨부 원본 경로 계획기를 그대로 사용한다. 기본 `(3,1,2,6)`은 과거 지도 ID에 대한 재현 옵션이며, 새 비행에서도 같은 물리 패널에 같은 ID가 부여된다는 보장은 없다. 현재 검출된 ID 중 존재하는 것만 우선 배치하고, 나머지는 원본 계획기 순서에 추가한다.

## 라즈베리파이의 Codex가 내려받은 후 할 일

**먼저 이 README를 읽고 다음 환경 정보를 확인한다.** 로컬 장치 경로·실제 보정값·FC 통신은 이 브랜치만으로 자동 결정할 수 없다.

| 현장에서 확인할 항목 | 설정/확인 위치 | 처리 |
|---|---|---|
| ROS 2 환경과 Python | 기존 MAVROS가 사용하는 ROS/컨테이너 환경 | 동일한 ROS와 Python에서 빌드. 다른 workspace의 같은 이름 패키지가 overlay되지 않게 확인 |
| FC 연결 | 기존 MAVROS의 serial/UDP 설정, `/mavros/state` | 이 repo는 MAVROS를 새로 띄우지 않음. 기존 실제 기체 연결을 사용 |
| LiDAR 실제 포트 | `/dev/serial/by-id`, launch `lidar_port` | 기본 Prolific USB 포트를 실제 연결 포트로 지정. baud 115200과 거리 단위/장착 방향 확인 |
| 카메라 | `rpicam-vid`, OpenCV worker | 실제 카메라로 프레임 수신 확인. 당시 180° 회전과 설치 방향을 맞춤 |
| 카메라/라이다 기하 | `da_daka_control/config/panel_survey.yaml`, `distance_filter.yaml`, `tf_luna_serial.yaml` | 기체가 당시와 같으면 첨부 값 유지. 장착이 다르면 현장에서 측정한 값으로 수정하고 차이 기록 |
| PX4 설정과 firmware | `evidence/px4_parameters_141.json` | 실제 FC와 대조해 차이를 기록. 다른 기체에 전체 파라미터를 일괄 적용하지 않음 |
| 위치·Home·GPS·EKF 토픽 | 아래 점검 명령 및 readiness | 필요한 MAVROS 플러그인/토픽과 발행을 확인 |
| 비행 명령 소유자 | 두 MAVROS setpoint 토픽 | v1/v2·이전 mission 프로세스를 정지해 중복 제어 발행자를 제거 |
| 시험 환경 | 141의 두 패널 배치와 yaw/조명 | 기록상 두 패널은 이륙점 전방 약 0.5m, 좌우 각각 약 0.5m. 새 환경 차이를 기록 |
| 저장 공간 | `~/flight141-logs`, `captures/panel_cpu_survey` | 쓰기 권한과 공간 확인. JSONL/파라미터·ROS bag·PX4 ULog를 함께 수집 |

환경 확인이 끝나기 전까지 `configuration_approved`와 `calibration_approved`는 false로 둔다. 이 두 값과 명시적인 start service는 **첨부 원본 소스의 시작 조건**이다. 실행만으로 ARM/비행을 시작하지 않는다.

### 내려받기와 빌드

새 디렉터리에서 받는다. 기존 기체의 checkout을 강제 초기화하거나 덮어쓰지 않는다.

```bash
git clone --branch test-flying-prototype --single-branch \
  https://github.com/KiHyeonLee1121/we-meet-project.git we-meet-prototype141
cd we-meet-prototype141
python3 tools/verify_141.py
```

Python 3.10 이상을 사용한다. 오프라인 테스트에는 PyYAML이 필요하다(카메라 의존성 목록에 포함). 설치된 ROS 환경을 먼저 source한다. ROS를 컨테이너에서 쓰고 있다면 다음 빌드/launch도 그 환경에서 수행하고, LiDAR 장치와 loopback UDP 연결이 접근 가능한지 확인한다. 카메라 worker와 ROS 수신기를 서로 다른 네트워크 namespace에서 실행하면 기본 `127.0.0.1`로 통신하지 못하므로 당시와 같은 host networking 또는 명시적 주소 설정이 필요하다.

```bash
# 실제 설치된 ROS 배포판의 setup.bash를 source한 상태에서
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-select da_daka_interfaces da_daka_control we_meet_prototype141
source install/setup.bash
ros2 pkg prefix da_daka_control
ros2 pkg prefix we_meet_prototype141
```

두 prefix가 이 checkout의 `ros2_ws/install`에 대응하는지 확인한다. 이미 기체에서 사용 중인 ROS/MAVROS 환경의 설치 방법은 이 브랜치가 추정해서 바꾸지 않는다.

카메라 worker는 CPU/OpenCV만 사용한다. `laptop_ai/pyproject.toml`의 GPU/ONNX 기본 의존성을 설치할 필요가 없다. 기존 기체의 검증된 라이브러리가 있으면 그대로 사용하고, 새 설치가 필요할 때만 CPU 목록을 사용한다. 아래 venv는 카메라 worker용이며 ROS Python 환경과 별도로 둔다.

```bash
# 저장소 루트에서
python3 -m venv --system-site-packages .venv-camera
.venv-camera/bin/python -m pip install -r requirements-prototype.txt
.venv-camera/bin/python -m unittest discover -s tests -v
.venv-camera/bin/python tools/pi_panel_cpu_runtime.py --profile night_asfound
```

`night_asfound`는 첨부 `camera_profiles.yaml`이 과거 야간 hardcoded camera flags로 설명한 설정이다. 이름이 `night`인 프로필에는 추가 `denoise: cdn_fast`가 들어 있으며 당시 비행 검증 여부가 불확실하다. 141 재현 실행 예시는 원래 rpicam 기본값을 유지하는 `night_asfound`를 사용한다. 카메라 worker가 `panel_only=True`로 실행되므로 ONNX 모델이나 CUDA가 필요 없다.

### 시작 전 확인과 실행

기존 MAVROS를 정상 연결한 상태에서, 다른 터미널에 현재 ROS와 이 workspace 환경을 source하고 launch한다.

```bash
ros2 launch we_meet_prototype141 flight141.launch.py \
  configuration_approved:=false calibration_approved:=false \
  lidar_port:=/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0
```

LiDAR 경로는 위 표의 현장 확인 결과로 바꾼다. 다음 토픽과 기체 상태를 확인한다.

```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/extended_state --once
ros2 topic echo /mavros/home_position/home --qos-durability transient_local --once
ros2 topic echo /mavros/global_position/global --once
ros2 topic echo /mavros/estimator_status --once
ros2 topic echo /panel_localization_test/readiness --once
ros2 topic info /mavros/setpoint_position/local --verbose
ros2 topic info /mavros/setpoint_velocity/cmd_vel --verbose
```

현장 확인을 완료한 뒤 위 launch를 정상 종료하고 **동일한 launch에서** 두 approval 값을 true로 바꿔 다시 실행한다. readiness가 `ready:true`이고 다른 비행 제어 프로세스가 없을 때 start를 호출한다. 설정은 시작 때 읽으므로 실행 중 `ros2 param set`으로 approval만 바꾸는 방법은 사용하지 않는다.

```bash
ros2 launch we_meet_prototype141 flight141.launch.py \
  configuration_approved:=true calibration_approved:=true \
  lidar_port:=/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0

# 별도 터미널: 명시적인 시작
ros2 service call /panel_localization_test/start std_srvs/srv/Trigger '{}'

# 필요한 경우: 원래 mission의 abort/착륙 요청
ros2 service call /panel_localization_test/abort std_srvs/srv/Trigger '{}'
```

기체의 실제 비행 시작 여부는 현장 운영자가 결정한다. 이 브랜치 작성 과정에서는 ARM/FC 변경/실비행을 수행하지 않았다.

기록 예시(비행 시작 전, 해당 ROS bag 플러그인이 설치된 환경):

```bash
ros2 bag record -o "$HOME/flight141-logs/rosbag-$(date +%Y%m%dT%H%M%S)" \
  /mavros/state /mavros/extended_state /mavros/local_position/pose \
  /mavros/local_position/velocity_local /mavros/home_position/home \
  /mavros/global_position/global /mavros/estimator_status /mavros/battery \
  /mavros/setpoint_position/local /mavros/setpoint_velocity/cmd_vel \
  /distance/filtered /panel_localization_test/state /panel_localization_test/result \
  /panel_localization_test/readiness /panel_localization_test/current_panel_id \
  /panel_survey/map /panel_survey/state /ai/perception /ai/health /ai/requested_mode
```

거리·AI 토픽 이름은 참고 YAML 및 실제 `ros2 topic list`와 대조한다. mission은 별도로 `~/flight141-logs`에 시작 파라미터 JSON과 상태·기준점·발행 명령 JSONL을 저장한다. `log_directory:=...`로 변경할 수 있다. JSONL은 모든 FC 센서 원본을 포함하지 않으므로 ROS bag과 PX4 ULog를 대체하지 않는다.

## 검증

```bash
python3 -m unittest discover -s tests -v
python3 tools/verify_141.py
python3 -m compileall -q ros2_ws/src tools laptop_ai/laptop_ai

# 압축파일의 원본 로그가 있을 때. numpy가 필요하다.
python3 tools/verify_141.py --ulog /실제/경로/px4_log_00141.ulg
```

작성 환경에서 **14개 오프라인 테스트, 원본 141 ULog 대조, Python 컴파일 검사가 통과했다**. 수치 결과는 `evidence/verification_141.json`에 저장했다. 오프라인 테스트는 실제 첨부 mission의 메서드와 격리된 ROS 메시지/Node 대역을 사용한다. 테스트는 기체에 연결하지 않는다. 원본 ULog 재검증에서는 SHA256, 다섯 목표점, Home, 고정 yaw, heading reset, 첫 패널 접근, 재획득 원의 반경/주기, PX4 파라미터를 대조한다. 원본 141 로그의 SHA256:

```text
56c3f79d9d2bade5ad945e62bbdf3fa035fb3fc798b0d7d4455e1ee97e948ad7
```

이 검증은 당시 명령과 기록의 일치를 확인한다. ROS 실제 빌드/통신·기체 안정성·GPS 지면 오차·새 환경의 비행 결과는 여기서 검증하지 않았다. 원본 firmware는 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`이며 현재 FC와 차이가 있으면 함께 기록한다.

첨부 원본 소스의 Apache-2.0 라이선스를 유지했고, 첨부 PyULog 파서는 `tools/vendor/pyulog/LICENSE.md`의 BSD 라이선스로 제공한다.
