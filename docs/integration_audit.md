# 센서–VIO–Pixhawk 연결 검토

## 근거와 조사 범위

제공한 실패 분석의 기체 자료에서 Pi5, Pi OS 호스트/Docker Jazzy, MAVROS 2.14, PX4 FMUv5/1.17 계열, USB 921600, TF-Luna/PL2303 115200, IMX708을 확인했습니다. 현재 실행 중인 드론에 접속한 결과가 아닙니다. `da-daka-REPORT`의 `EKF2_EV_CTRL=0`은 당시 자료이며 현장 현재값은 다시 조회해야 합니다.

실제 [OpenVINS 고정 소스](https://github.com/rpng/open_vins/tree/69488123ed9362dd44b6f28e7f4680abbff1442b)와 [MAVROS 2.14](https://github.com/mavlink/mavros/tree/2.14.0)를 읽어 다음 경로를 확인했습니다.

| 확인한 원본 | 코드에 반영한 내용 |
|---|---|
| OpenVINS `ROS2Visualizer::visualize_odometry`, `Propagator::fast_state_propagate` | odomimu pose=global/IMU origin, quaternion xyzw는 이미 ROS Hamilton IMU→global에 해당. twist=IMU local. 추가 conjugate 없음 |
| OpenVINS `callback_inertial`, `publish_state` | IMU 예측 odomimu와 카메라 보정 poseimu가 별개. odom만 살아 있어도 카메라 중단을 성공으로 판단하지 않음 |
| OpenVINS mono subscriber | upstream reliable/depth10과 Pi 영상 publisher QoS 불일치·대기 영상 큐 누적 가능성. SensorDataQoS depth2 및 pending camera queue 최대2로 패치 |
| MAVROS `odom.cpp` | out은 pose 부모·twist 자식 프레임을 사용하고 TF를 조회해 LOCAL_FRD/BODY_FRD ODOMETRY로 전송. TF 존재·회전값 검사 |
| MAVROS `odom.cpp` | reset_counter/quality를 nav_msgs/Odometry에서 채우지 않음. 재시작/좌표 점프를 비행 중 금지, 품질은 timestamp·rate·covariance·불일치로 감시 |
| MAVROS `local_position.cpp` | velocity_local의 값은 ENU인데 header는 base_link로 표기. 이 토픽을 좌표판단에 쓰지 않고 local_position/odom의 body twist를 map으로 변환 |
| MAVROS `imu.cpp` | data_raw는 FC 데이터를 FLU·SI로 바꿈. 실제 HIGHRES_IMU인지, sample rate/중력 포함/시계 동기/선택 IMU가 유지되는지 Pi에서 확인 필요 |
| MAVROS `setpoint_velocity.cpp` | LOCAL_NED 설정에서 ROS ENU xyz+yaw rate 변환. 실제 plugin parameter를 GetParameters로 조회하고 단일 발행자 검사 |
| MAVROS `param.cpp` | deprecated ParamGet 대신 `/mavros/param/get_parameters`의 ROS 2 GetParameters 사용. 이것은 FC 파라미터 cache 조회이므로 시작 전 pull/현재 cache 일치 확인 필요 |
| MAVROS `uas_tf.cpp`, `sys_time.cpp` | map↔map_ned, base_link↔base_link_frd 기본 TF와 `/mavros/timesync_status` 사용 |

원본 estimator 수학을 새로 흉내 내지 않습니다. 빌드 시 고정 OpenVINS를 가져옵니다. `.hpp`/Eigen/C++17 변경과 입력 QoS/큐 제한만 `patch_openvins.py`에서 재현하고 SHA256 manifest를 남깁니다. ArUco는 컴파일 및 런타임에서 끕니다. OpenVINS 자체 라이선스는 upstream 그대로입니다.

## 좌표와 시간

`T_A_B`는 `p_A = R_A_B p_B + t_A_B`입니다. Kalibr `T_cam_imu`와 측정 `T_body_imu`에서 `T_body_camera`를 유도합니다. 카메라는 오른쪽/아래/앞 optical, 기체는 앞/왼쪽/위 FLU, 제어 map은 Z-up입니다. OpenVINS 전역 XY 방향/원점은 임의이므로 disarm 상태에서 FC pose에 yaw와 원점을 한 번 맞추고 고정합니다. 비행 중 relatch나 Home 추종은 하지 않습니다.

IMU 위치에서 기체 위치로 옮길 때 위치뿐 아니라 `omega × lever_arm` 속도도 제거합니다. pose 공분산은 upstream의 IMU-local 자세 오차와 레버암 상관관계를 고려해 보수적인 spectral bound/Jacobian으로 변환합니다. twist 공분산은 전체 6×6 변환을 적용합니다. 공분산을 임의의 작은 상수나 0으로 덮어쓰지 않습니다. 0/음수/NaN 공분산은 준비 실패입니다.

Pi OS host의 `SensorTimestamp`를 패킷에 그대로 보존합니다. Docker와 호스트가 같은 BOOTTIME/MONOTONIC 시계를 공유하는지 확인하고 ROS 시각에 매핑합니다. 수신 시각을 촬영 시각이라고 붙이지 않습니다. 예측 odometry 역시 원 IMU 측정 stamp를 유지합니다. ROS 시계가 불연속으로 바뀌면 시작을 막고 비행 중에는 중단합니다.

OpenVINS poseimu stamp는 카메라 시각+교정한 camera-to-IMU 시간차입니다. 센서 시간차와 `EKF2_EV_DELAY`는 다른 보정입니다. Kalibr 결과를 쓰고 FC 잔여 지연을 별도로 측정하며 처리 시간을 양쪽에 중복 추가하지 않습니다. IMX708의 rolling shutter readout은 이 mono pinhole 모델에서 완전히 추정하지 않으므로 실제 기체 진동·회전에서 검증해야 합니다.

TF-Luna 기본 9-byte 출력에는 정밀 sensor clock이 없습니다. timestamp는 UART 수신과 측정한 transport_delay의 근사입니다. backlog는 폐기하고 최신 프레임만 내보냅니다. checksum·신호 세기·범위를 검사하고 raw range에 기울기·장착 오프셋을 한 번만 적용합니다. 높은 지형/패널로 range가 바뀌면 단순 지형 추종 대신 VIO Z와의 불일치로 중단합니다.

## PX4 융합

패널의 광선을 지도로 투영할 때도 카메라 stamp에 `timeshift_cam_imu`를 더한 시각의 VIO pose를 보간합니다. 새 이미지에 현재 pose를 붙이거나 카메라/IMU 시간차를 누락하지 않습니다. 패킷에는 원 카메라 stamp를 그대로 유지해 실제 영상 freshness를 검사합니다.

IMU는 OpenVINS 초기화 전에 `imu_gate`를 통과합니다. time sync·실제 source stamp·frame·rate·SI/gravity 기준이 안정되면 원 메시지를 `/vio/imu`로 전달합니다. 스트리밍 뒤 시각/원본 오류가 발생하면 재시작 전까지 막습니다. 낮은 주기 샘플을 복제하거나 나쁜 timestamp를 현재 시각으로 덮지 않습니다.

프로필은 외부 시각 XY+3D 속도+yaw(13), GNSS fusion off(0), magnetometer fusion off(5), FC barometric height reference(0), message covariance 사용(0)을 예상합니다. 실제 펌웨어의 파라미터 이름·비트·가용 yaw source를 확인해야 합니다. 코드가 이 값들을 FC에 자동 쓰지 않습니다.

수신 확인과 실제 융합 확인은 다릅니다. MAVROS EstimatorStatus의 위치/속도 유효 flag만으로 EV를 증명할 수 없습니다. 지상 QGC MAVLink 콘솔의 `listener estimator_status_flags` 또는 ULog에서 `cs_ev_pos`, `cs_ev_vel`, `cs_ev_yaw` 및 innovation을 확인한 뒤 `fc_fusion_verified`와 근거를 기록합니다. `EKF2_EV_QMIN=0`은 MAVROS 2.14 경로의 quality 필드 제약에 대한 값이며 저품질 입력을 허용한다는 뜻으로 사용하지 않습니다. 별도 stream/covariance/texture 감시가 남습니다.

VIO를 상위 거리 계산에만 쓰고 PX4가 계속 나쁜 GNSS 속도로 제어하면 목적을 달성하지 못합니다. 그래서 FC 융합 준비·현재 설정 조회·FC/VIO yaw/XY/속도 일치 검사를 포함했습니다. 이번 지도는 VIO 세션마다 새로 시작하며 drift-free SLAM, 지리 지도, 재시작 후 동일 좌표 재현을 주장하지 않습니다.

## Pi5 처리 예산

초기값은 640×480/25fps, mono KLT 120 points, 10 clones, SLAM-state landmarks 0, OpenCV 2 threads, 패널 검출 8Hz/1 thread, 제어·FC odometry 40Hz입니다. VIO는 MSCKF 창의 특징점으로 추정하며 영구 SLAM 지도는 만들지 않습니다. 640×480 mono8는 약 7.7MB/s 영상 payload이고 로컬 binary IPC를 사용합니다. FC 링크에는 영상이 들어가지 않습니다.

입력 큐·IPC는 최신 데이터를 선택하고 오래된 프레임을 소급 처리하지 않습니다. 디스크 기록은 bounded background queue로 분리했습니다. CPU affinity·realtime 우선순위·GPU/Hailo를 추정해서 고정하지 않습니다. Pi5에서 실제 feature tracking 주기와 P95/P99 capture age, IMU gap, CPU·온도·throttling을 측정한 뒤 조정합니다. 영상 계산 주기를 낮추더라도 IMU를 낮은 주기로 가짜 보간하거나 timestamp를 새로 붙여 준비 조건을 통과시키지 않습니다.

## 패널 검출 교체 계약

`/panels/detections`는 `std_msgs/String` JSON schema1입니다. root 필드는 `stamp`(원본 촬영 ROS 초), `frame_id=camera_optical`, `width=640`, `height=480`, `panels`(최대8), `texture_points`, `texture_cells`, `processing_ms`입니다. 각 panel은 `corners`(원 영상의 둘레 순서 4×2), `center`(대각선 교점 2D), `score`(0..1)를 갖습니다. 회전·resize·letterbox는 detector 내부에서 원래 좌표로 정확히 복원합니다.

AI로 바꿀 때도 VIO는 동일 raw image를 독립적으로 받습니다. 비동기 모델 결과에는 inference 완료 시각 대신 원 촬영 stamp를 넣습니다. 새로운 model이 여러 박스를 내면 평면에 투영한 위치로 지도의 ID를 연결하고 선택 ID를 유지합니다. bounding-box 중심만으로 실제 패널 중앙/기울기/3D 위치가 정확해지는 것은 아니므로 필요한 corner/keypoint 또는 segmentation 후처리를 추가해야 합니다.

## 공식 참고

- [OpenVINS sensor calibration](https://docs.openvins.com/gs-calibration.html)
- [OpenVINS Jazzy 관련 upstream PR](https://github.com/rpng/open_vins/pull/500)
- [PX4 external position estimation](https://docs.px4.io/main/en/ros/external_position_estimation)
- [PX4 parameter reference](https://docs.px4.io/main/en/advanced_config/parameter_reference)
- [Picamera2 manual](https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf)
