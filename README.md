# we-meet-project — test-flying-v2

라즈베리파이 5 / ROS 2 Jazzy / MAVROS 2.14 / Pixhawk용 시험 프로세스입니다. 기존 시험 코드 원본을 가져오지 못했으므로, 제공된 진단 자료·소스 발췌·자율 및 수동 ULog를 바탕으로 별도 패키지를 재구성했습니다. 기체나 센서에 접속해서 설정을 바꾸지는 않았습니다.

**수직 상승 → 출발 방향으로 속도 이동 → OpenCV 패널 검출 → 감속 → 영상 중앙 정렬 → 5초 정지 → 그 자리 AUTO.LAND** 순서입니다. 앞선 시험의 착륙 단계는 유지했습니다.

## 동작

| 구간 | 동작 |
|---|---|
| 시작 | 지상·disarm·센서 상태를 확인하고 안정된 yaw를 한 번 저장 |
| 상승 | 하향 라이다 거리 약 3m까지 상승, XY 속도 0, 같은 yaw 유지 |
| 안정 대기 | 라이다 거리·거리 변화율·yaw가 2초간 안정되면 전진 |
| 접근 | 출발 yaw를 기준으로 최대 0.35m/s, 최종 발행 명령 적분으로 약 5m의 전진 예산 관리 |
| 패널 검출 | 서로 다른 3개 영상에서 같은 어두운 사각형 패널을 확인하면 전진 감속 |
| 영상 정렬 | 현재 yaw로 영상 오차를 ENU 속도로 변환, 최대 0.12m/s |
| 정지 | 패널 중심이 화면 중앙 **가로 30% × 세로 30%** 안에 있고 고도·yaw가 안정된 상태를 연속 5초 유지 |
| 착륙 | AUTO.LAND를 요청하고 FC가 전환하면 속도 발행을 중단; 착륙·disarm을 확인 |

패널 중앙은 검출한 사각형의 영상 중심입니다. 화면 중심 위에 카메라를 놓는 시험이며, 노즐 위치 보정·분사·여러 패널 탐색·패널맵·귀환은 포함하지 않습니다. 카메라가 아래를 보는 배치를 전제로 합니다. 사선/전방 카메라의 경우 이 XY 제어식을 그대로 사용하면 안 됩니다.

`baseline.yaml`은 카메라 없이 **상승 → 속도 명령 적분 5m → 감속 → 5초 대기 → 착륙**을 실행합니다. 먼저 기존 yaw 시험을 이 프로필로 확인할 수 있습니다.

**5m는 실제 이동거리의 측정값이 아닙니다.** `commanded_distance_m`은 실제 발행한 최종 전방 속도를 단조 증가 시각의 dt로 적분한 값입니다. 감속 거리를 포함하며, 영상 전환 뒤 정렬 거리는 이 값에 추가하지 않습니다. GPS/local XY 위치는 로그에만 기록합니다. FC의 EKF/GNSS 설정은 변경하지 않으며, 속도 제어 자체도 FC의 추정 속도에 의존할 수 있습니다. 상승 중 XY 속도 0 역시 GPS 없는 실제 위치 고정을 보장하지 않습니다.

3m는 **라이다가 보는 면에서 기체 기준점까지의 수직 높이 목표**입니다. `lidar_input_is_vertical_height: true`는 입력 드라이버에서 기울기·장착 오프셋이 이미 보정됐다는 뜻이며 기본 예시 설정입니다. 실제 `/distance/filtered`가 단순 거리 필터 값이면 `false`로 바꾸고 `lidar_body_down_offset_m`(기체 기준점 아래의 센서 거리)을 측정하세요. raw 입력은 `(거리 + 오프셋) × cos(roll) × cos(pitch)`로 한 번만 보정합니다. 하향 body 축과 정렬된 라이다·평탄한 면을 전제로 하며 다른 설치각이면 별도 외부 보정이 필요합니다. `lidar_geometry_verified`는 이 확인을 마쳤다는 표시입니다. 지면에서 패널 위로 넘어갈 때 라이다 반사면 높이가 바뀌면 고도 제어에도 영향이 생깁니다. 이번 시험은 평탄한 지면의 낮고 고정된 단일 패널, 하향 카메라·라이다 배치부터 확인하는 구성입니다.

## 구성

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
colcon build --symlink-install --packages-select we_meet_flight
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

제공 자료의 180도 회전 영상 기준 기본 행렬은 `image_to_body: [[0,-1],[-1,0]]`입니다. **실제 영상에서 오른쪽이 기체 오른쪽, 아래가 기체 뒤쪽인지 확인하세요.** 출발 방향은 현장에서 기체 전방을 목표물 쪽으로 맞춥니다. 검출 사진을 이용해 목표를 화면 위/아래/좌/우에 놓고 debug 속도가 목표 방향으로 생성되는지도 확인합니다. 영상 XY는 body FLU로 변환한 뒤 현재 yaw로 ENU에 변환하고, 전진 구간은 저장한 출발 yaw만 사용합니다.

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

- 2026-10-02: 진단·수동/자율 ULog 근거로 속도/yaw 시험 재구성, OpenCV 단일 패널 감속·정렬·5초 유지 추가. 상승 단독/5m/영상 시험 프로필, 자동 로그·명령 적분 재현, 무비행 검증 추가.
