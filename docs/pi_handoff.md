# Pi에서 내려받은 Codex의 적용 순서

현재 준비 작업은 **환경 조사 → 측정값 반영 → 빌드 → 비행 없는 검증 → 실행 명령과 남은 항목 보고**입니다. 아래 마지막 비행 절차를 준비 작업 중 실행하지 않습니다. 명령에 넣는 경로/장치/컨테이너 이름은 현장 현재값으로 바꾸고 변경을 보존합니다.

## 1. 기존 환경과 소유자 확인

Pi OS 호스트:

```bash
uname -m
cat /etc/os-release
docker ps -a
ls -l /dev/serial/by-id
pgrep -af 'camera|picamera|libcamera|rpicam|mavros|mission|distance'
```

ROS가 실제로 있는 컨테이너/환경:

```bash
printenv ROS_DISTRO
ros2 node list
ros2 topic list -t
ros2 service list
ros2 param get /mavros/setpoint_velocity mav_frame
ros2 topic info /mavros/odometry/out --verbose
ros2 topic info /mavros/setpoint_velocity/cmd_vel --verbose
```

FC 링크는 MAVROS 하나만 엽니다. 기존 미션·distance controller·visual servo·분사 노드를 새 시험과 함께 실행하지 않습니다. 기존 카메라 proxy/streamer는 CSI 장치를 점유할 수 있습니다. 소유자와 현재 임무를 확인하고 필요한 전환을 정리하며 운영 중 프로세스를 임의 종료하지 않습니다. GPS 없는 RC 회수 모드와 Offboard-loss 착륙 정책도 현장에 맞게 준비합니다.

## 2. ROS 빌드

기존 Jazzy 컨테이너를 우선 재사용합니다. ROS 2 Jazzy/Ubuntu 24.04 환경에서 필요한 패키지 예:

```bash
sudo apt-get update
sudo apt-get install -y git build-essential libeigen3-dev libboost-all-dev libceres-dev libopencv-dev python3-colcon-common-extensions python3-opencv python3-numpy python3-yaml python3-serial ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-tf2-ros-py
source /opt/ros/jazzy/setup.bash
python3 tools/fetch_openvins.py
cd ros2_ws
MAKEFLAGS=-j2 colcon build --packages-up-to ov_msckf we_meet_vio --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_ARUCO_TAGS=OFF -DDISABLE_MATPLOTLIB=ON
source install/setup.bash
cd ..
python3 tools/check.py
python3 tools/smoke_ros.py
```

root 컨테이너에서는 sudo를 생략합니다. Pi5 메모리/발열에 따라 `-j1`로 줄일 수 있습니다. OpenVINS 다운로드 스크립트는 기존 수정 checkout을 덮어쓰지 않습니다. 패치 manifest는 `ros2_ws/src/open_vins/WE_MEET_PATCH.json`입니다. OpenCV C++ 라이브러리는 cv_bridge가 사용하는 ROS 배포판 버전과 맞춰야 합니다. 컨테이너의 apt OpenCV에 별도 pip 바이너리를 섞어 로드하지 않습니다.

새 컨테이너가 필요한 경우 [Dockerfile](../Dockerfile)을 사용할 수 있습니다. 기존 컨테이너의 이름/volume/device 설정을 유지하고, 다음 자원을 반영하세요.

| 자원 | 배치 |
|---|---|
| MAVROS/ROS graph | 같은 DDS domain/RMW, `network_mode: host` 또는 검증한 DDS 연결 |
| FC | 실제 USB device 하나를 컨테이너에 매핑; 현재 FC URL/FC ID 사용 |
| LiDAR | 실제 USB device를 컨테이너에 매핑하고 settings.lidar.port를 **컨테이너 내부 경로**로 설정 |
| 카메라 | Pi OS 호스트의 distro Picamera2가 장치를 소유; 컨테이너에는 카메라 native 라이브러리 대신 IPC 디렉터리 공유 |
| IPC | 호스트 `/run/we-meet-vio`를 컨테이너 같은 절대 경로로 rw bind mount; 동일 UID 또는 setgid 디렉터리의 공통 group/권한 |
| 설정/로그 | 측정 설정 디렉터리와 로그를 호스트에 보존되는 volume으로 mount |

기존 MAVROS를 쓴다면 `start_mavros:=false`를 유지합니다. `mavros_overrides.yaml`을 해당 환경에 실제 로드했는지 확인하세요. 원래 px4.launch는 config_yaml을 내부 고정하므로 인자를 달기만 해서 적용됐다고 판단하면 안 됩니다. 이 브랜치의 `start_mavros:=true` 경로는 설치된 base config/pluginlists 다음에 override를 명시적으로 로드합니다.

카메라 소켓은 owner/group에만 쓰기 권한을 줍니다. 호스트 capture 사용자와 컨테이너 UID/group을 맞추거나 공유 디렉터리를 그 공통 group의 setgid(2770)로 준비하세요. root 컨테이너가 만든 소켓을 호스트 일반 사용자에게 전달하려면 group 상속이 필요합니다. 권한 오류를 world-writable로 우회하지 않습니다.

MAVROS를 새로 설치한 환경은 GeographicLib의 `egm96-5` geoid dataset도 필요합니다. 기존 운용 환경에 이미 있으면 재설치하지 않습니다. 없으면 해당 ROS 환경에서 `sudo geographiclib-get-geoids egm96-5` 또는 공식 `ros2 run mavros install_geographiclib_datasets.sh`로 준비합니다. GPS를 사용하지 않는 시험이어도 MAVROS의 기본 플러그인 초기화에 필요할 수 있습니다.

## 3. 카메라 수집과 교정

Pi OS 호스트에서 distro `python3-picamera2`, numpy를 사용합니다. `--probe`는 FC/ROS에 쓰지 않고 실제 metadata를 출력합니다. 카메라 점유 상태를 확인하고 실행하세요.

```bash
python3 tools/camera_host.py --probe --lens-position 0.33
```

0.33/4000µs는 벤치 시작 설정이지 측정 교정값이 아닙니다. 실제 3m 영상이 선명하고 충분히 밝도록 고정 focus/exposure/gain을 선택하고 그 설정으로 교정합니다. autofocus/autoexposure는 이 프로필에서 켜지 않습니다. `capture.example.json`을 별도 `capture.local.json`으로 복사해 실제 `LensPosition`, `ScalerCrop`, 선택한 controls와 clock을 기록합니다. IMX708 선택 sensor mode/crop가 다시 바뀌면 재교정합니다.

교정용 카메라 영상 수집은 **시험 launch와 별도**입니다. 실제 설정 profile과 공유 socket 디렉터리를 준비하고 다음 두 프로세스만 실행합니다.

```bash
# Pi OS 호스트 (실제 경로)
python3 tools/camera_host.py --profile /absolute/path/capture.local.json

# ROS 컨테이너: 비행/FC 연결을 추가하지 않는 raw 이미지 입력 모드
ros2 run we_meet_vio pi_camera --ros-args -p calibration_only:=true -p capture_profile:=/absolute/container/path/capture.local.json
```

이 모드는 intrinsics를 꾸며내지 않고 raw image만 발행합니다. OpenVINS/비행 노드를 실행하지 않습니다. 교정에 필요한 기존 센서 전용 MAVROS IMU와 함께 bag을 기록합니다.

```bash
ros2 bag record /camera/image_raw /mavros/imu/data_raw /mavros/timesync_status
```

실제 capture clock을 `CLOCK_BOOTTIME`/`CLOCK_MONOTONIC`과 비교하고 FC timestamp와 같은 ROS 시간 영역인지 확인합니다. 카메라 내부교정, 카메라–IMU 외부교정·시간차(Kalibr 등), 실제 IMU 스트림의 Allan noise를 얻습니다. 교정용 checkerboard/AprilGrid는 임시 교정 도구이며 비행 공간에 표식을 설치하는 위치 추정 방식이 아닙니다.

OpenVINS에 들어가는 IMU는 gravity-included acceleration SI와 gyro SI입니다. `/mavros/imu/data`의 자세나 중력 제거 가속도로 대체하지 않습니다. HIGHRES_IMU가 충분한 주기로 들어오는지, FC의 선택 IMU/보정이 유지되는지 확인합니다. 불가능하면 검증된 동기 IMU 소스를 별도 도입하고 그 토픽·프레임으로 다시 교정합니다. 저주기 데이터를 복제/보간해 rate를 속이지 않습니다.

## 4. 측정값 파일 생성

`mount.example.yaml`을 별도 파일로 복사해 다음을 채웁니다.

| 값 | 근거 |
|---|---|
| `T_body_imu` | MAVROS FLU IMU가 대표하는 실제 원점과 기체 기준점 관계. MAVROS가 이미 body 축으로 회전했다면 회전을 중복 적용하지 않음 |
| `lidar_position_body`, `lidar_geometry_verified` | [forward,left,up] m; sensor가 body-down과 정렬됐는지, raw 값인지 확인 |
| `scaler_crop`, `camera_controls`, `sensor_clock` | **교정 때의** 실제 640×480 촬영 profile과 동일 |
| `raw_imu_source_verified` | 실제 source/rate/units/gravity/clock 확인 결과 |
| `imu_intrinsics_already_corrected` | FC가 보정한 SI 데이터를 받는 경우에만 true. downstream identity는 추가 보정 없음이라는 뜻 |
| `verified`, `calibration_evidence` | 장착·교정 파일과 현장 확인 근거. 플래그는 자동교정을 의미하지 않음 |

```bash
python3 tools/configure_calibration.py --camera /path/actual-camchain-imucam.yaml --imu /path/actual-imu.yaml --mount /path/mount.local.yaml --out /path/vio-config
```

이미 존재하는 교정 파일은 덮어쓰지 않습니다. 새 디렉터리를 만들어 검토합니다. `vio-config`에는 공유 `calibration.json`, 실제 OpenVINS camera/IMU YAML, estimator 설정, `settings.yaml`, `mavros_overrides.yaml`이 생깁니다. camera info와 시각서보는 같은 calibration.json을 사용합니다. `settings.yaml`의 LiDAR port·transport delay·panel 높이·로그 경로·namespace·FC 기대값을 현장에 맞춰 조정합니다. 제어 수치의 전체 기본값은 `we_meet_vio/mission.py:Parameters`에 있습니다.

## 5. 무비행 dry-run

교정용 `pi_camera`를 종료해 socket을 해제한 뒤 실제 교정 profile로 호스트 촬영을 시작합니다. socket은 살아 있는 다른 소유자가 있으면 시작을 거부합니다. 비정상 종료로 남은 socket은 소유자가 없는지 확인한 후 현장에서 정리합니다.

```bash
# Pi OS 호스트
python3 tools/camera_host.py --profile /path/vio-config/calibration.json

# ROS 컨테이너 (기존 MAVROS 사용)
ros2 launch we_meet_vio test_vio.launch.py config_dir:=/container/path/vio-config dry_run:=true
```

기본 dry-run은 FC odometry와 setpoint 발행, mode/arming 서비스 쓰기를 하지 않습니다. FC 원래 설정에서는 융합 준비 flag가 false일 수 있어 정상적으로 시작 불가 이유가 표시됩니다. 전체 상태 전환을 보기 위해 FC 상태를 위조하지 않습니다. `tools/offline_simulation.py`는 완전히 별도 합성 입력 시험입니다.

```bash
ros2 topic echo /vio/status --once
ros2 topic echo /vio_flight/status --once
ros2 topic echo /work_map --once
python3 tools/bench_report.py --seconds 30 --out /path/logs/vio-bench.json
```

필수 판단: raw IMU ≥180Hz, camera ≥18Hz, 실제 VIO camera correction ≥12Hz, LiDAR ≥20Hz, detector ≥5Hz, 원 stamp 역행/반복 없음. source age와 max gap을 코드 한계와 비교합니다. p95/p99가 경계에 가까우면 원인을 먼저 개선합니다. Pi 호스트에서는 CPU·온도·`vcgencmd get_throttled`를 함께 기록합니다. 코드가 준비되었다고 이 처리량을 이미 실측한 것은 아닙니다.

## 6. FC 스트림과 외부 시각 융합의 지상 검증

다음 요청은 메시지 주기 설정이며 비행 명령이 아닙니다. 기존 링크 스트림과 대역폭을 확인한 후 해당 MAVROS namespace에서 요청하고 실제 주기를 측정하세요.

```bash
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval '{message_id: 105, message_rate: 200.0}'
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval '{message_id: 32, message_rate: 40.0}'
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval '{message_id: 31, message_rate: 40.0}'
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval '{message_id: 230, message_rate: 10.0}'
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval '{message_id: 245, message_rate: 4.0}'
ros2 service call /mavros/param/pull mavros_msgs/srv/ParamPull '{force_pull: true}'
```

105=HIGHRES_IMU, 32=LOCAL_POSITION_NED, 31=ATTITUDE_QUATERNION, 230=ESTIMATOR_STATUS, 245=EXTENDED_SYS_STATE. service 응답 성공은 실측 주기 확인을 대신하지 않습니다. 다른 GCS 요청이나 MAVLink bandwidth 제한으로 rate가 줄 수 있습니다.

현재 PX4 버전/param dump를 보존하고 `expected_fc_parameters` 표를 해당 버전과 비교합니다. 이 프로필의 의도는 EV XY/3D velocity/yaw(13), GNSS aiding off(0), magnetometer off(5), baro height reference(0)입니다. 이륙 가능한 yaw와 지상 손이동 VIO가 준비되기 전에 값을 맹목적으로 적용하지 않습니다. FC `EKF2_EV_DELAY`는 잔여 지연을 측정해 반영하고 기대값도 같은 값으로 둡니다. `EKF2_EV_POS_*`는 bridge의 body-origin 변환 때문에 0입니다.

**FC 융합 준비 단계에서는 외부 추정 입력을 FC에 쓰게 됩니다.** 해당 지상 점검이 지시된 상태에서만 다음 모드로 실행합니다. 비행 제어는 여전히 dry-run으로 비활성화됩니다.

```bash
ros2 launch we_meet_vio test_vio.launch.py config_dir:=/container/path/vio-config dry_run:=true feed_fc_vision:=true
```

시동 없이 손으로 기체를 앞/뒤/좌/우로 옮겨 VIO와 FC 추정 부호·거리·yaw·지연·covariance가 맞는지 확인합니다. OpenVINS static initialization 후 기체를 들고 적당한 3축 회전/병진을 가해 관측성을 확인하고 지상에 놓습니다. 시각 기준점은 한 번 고정되며 비행 중 재시작은 금지됩니다.

QGC 콘솔 `listener vehicle_visual_odometry`, `listener estimator_status_flags` 또는 ULog에서 외부 데이터 수신과 실제 `cs_ev_pos/vel/yaw` fusion을 각각 확인합니다. innovation 및 데이터 중단 시 거부 동작도 확인합니다. GPS/추정 quality flag만으로 실제 지면 오차를 확정하지 않습니다.

`verification.fc_fusion_verified`, `rc_recovery_verified`, `flight_settings_verified`와 `evidence`는 확인을 완료한 뒤 근거와 함께 반영합니다. FC 파라미터 읽기는 MAVROS cache이므로 QGC/FC의 현재값과 cache가 일치하도록 pull 완료도 확인합니다. RC와 Offboard-loss 회수 정책을 우회하는 값을 코드가 자동 쓰지 않습니다.

## 7. 별도 비행 지시가 있을 때

준비 결과와 남은 측정값을 먼저 보고합니다. 이후 현장 시험 실행 지시에서만 다음 흐름을 사용합니다.

```bash
# 호스트 카메라는 계속 한 인스턴스만 실행
ros2 launch we_meet_vio test_vio.launch.py config_dir:=/container/path/vio-config dry_run:=false

# 별도 ROS 터미널; 이 호출부터 prestream/OFFBOARD/arming/비행이 진행됨
ros2 service call /vio_flight/start std_srvs/srv/Trigger '{}'

# 중단 요청
ros2 service call /vio_flight/abort std_srvs/srv/Trigger '{}'
```

실패/완료 뒤 같은 노드에서 재시작하지 않습니다. 지상에서 원인·로그·sensor clock/초점/rig 변경을 확인하고 새 세션을 실행합니다. 로그는 settings.log_directory 아래 세션 폴더의 settings.json과 flight.jsonl이고 map 스냅샷도 JSONL에 남습니다. 각 시험의 ULog/bag와 지면 실제 이동 거리·하강 위치·영상 중심 유지 관찰을 함께 보존합니다.

보고할 내용: 실제 commit/환경·장치/토픽·주기, 바꾼 파일/값/측정 근거, 교정 및 fusion 증거, bench report/CPU/온도, 테스트 결과, 실제 로그 경로, 미검증 비행 항목, 현장 경로로 치환한 최종 실행 명령.
