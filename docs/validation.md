# 검증 기록

## 이 작업 환경에서 실행한 검사

- 독립 제어·좌표·센서·지도·검출·교정 파일/시각 계약에 대한 unittest 52개: **51개 통과, 1개 환경 제한으로 skip**.
- 생략한 1개는 실제 Unix socket에 640×480 프레임을 보내는 검사입니다. 이 작업 환경은 socket 생성 syscall을 차단합니다. binary packet codec/크기/오류와 원 촬영 stamp 변환은 통과했습니다. GitHub의 Linux/Jazzy CI에 같은 실제 송수신 검사를 포함했습니다.
- 전체 Python/launch 문법 compile 확인.
- 단순 이동 모델에서 `PRESTREAM → OFFBOARD → ARM → TAKEOFF → STABILIZE → APPROACH → BLEND → ALIGN → HOLD → LAND → DONE`, 연속 5초 hold 확인. 초기 yaw 0/약90°/−160°와 작은 일정 외란 조건도 통과.
- 움직이지 않는 위치 입력에서 명령 시간이 지나도 5m 도착 성공으로 기록하지 않는 검사.
- camera correction 중단(odomimu 예측만 살아 있음), VIO jump/reset, clock gap, FC 속도 불일치, LiDAR 반사면 점프, target ID 교체/소실, 과도한 covariance, hold 조건 이탈, RC takeover, 착륙 인계·완료 timeout 검사.
- 실제 OpenCV를 이용한 원본 영상 사각형·기울어진 사각형 중심·clipped panel 거부·왜곡 보정 검사.
- 생성한 camera/IMU YAML을 **OpenCV FileStorage로 실제 열어** C++ parser가 읽을 구조인지 확인. 미교정 장착값/잘못된 camera 방향/미검증 source는 거부.
- 고정 OpenVINS의 실제 ROS2Visualizer/Propagator/config/parser와 MAVROS 2.14의 imu/odom/local_position/setpoint_velocity/param/time/TF 소스 검토.
- OpenVINS 입력 큐/QoS/Jazzy 패치를 실제 고정 소스 파일에 적용해 확인. fetch 시 변경 파일 hash manifest를 남김.

단순 모델 출력의 최종 XY 오차 약 0.039m/실행 약46.65초는 합성 pose와 단순 속도 응답을 사용한 **회귀 시험 수치**입니다. 실제 5m 오차·GPS 개선·기체 안정성을 증명하지 않습니다. LAND/긴급 중단의 0속도 인계는 일반 접근의 slew 제한과 별개입니다.

## GitHub CI

`.github/workflows/vio-checks.yml`은 두 경로를 제공합니다.

1. Python/OpenCV 회귀 시험 및 전체 단순 모델.
2. Ubuntu/Jazzy container에서 **고정 실제 OpenVINS + 새 ROS package 빌드**, 실제 ROS message 변환/격리 노드 구성/dry-run FC 명령 publisher 부재 검사.

게시 시점의 실행 결과는 Actions 탭에서 해당 commit을 확인하세요. 소스 검토/문법 검사를 실제 ROS build 통과로 바꿔 적지 않습니다. CI의 x86 빌드도 Pi5 aarch64의 처리속도나 실기체 비행을 검증한 것은 아닙니다.

## Pi에서 남은 검사

현재 mounted rig의 camera/IMU noise/extrinsic/time calibration, host/container clock/IPC permissions, 실제 HIGHRES_IMU ≥180Hz와 payload units, image/camera correction 지연·features·covariance, FC parameter cache/current firmware 일치, EV 실제 fusion/innovation, LiDAR geometry/반사면, RC/Offboard-loss 회수, CPU/발열/throttling, 현장 실제 이동거리/정렬/착륙입니다.

프로펠러/물 호스/분사 장치는 이 테스트/CI에서 동작하지 않습니다. 가짜 pose·가짜 calibration fixture는 `tests` 및 격리 smoke test에만 사용되며 실제 launch 설정에 설치하거나 자동 대체하지 않습니다.
