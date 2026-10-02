# v1 재구성 근거와 현장 미검증 항목

## 사용한 자료

- 첨부 `02-dadaka-REPORT.md`, `05-_-.md`, 진단 tar의 현재/as-flown 제어 코드·설정.
- 자율 실패 raw ULog(log154)와 수동 raw ULog(log155~167) 13개. pyulog로 모드/설정을 다시 확인했으며 결과는 [evidence_summary.json](evidence_summary.json)에 있습니다.
- 2026-10-02 현재 기존 [distance_controller.py](https://github.com/KiHyeonLee1121/da-daka_Ai/blob/main/ros2_ws/src/da_daka_control/da_daka_control/distance_controller.py), [distance_controller.yaml](https://github.com/KiHyeonLee1121/da-daka_Ai/blob/main/ros2_ws/src/da_daka_control/config/distance_controller.yaml), [TF-Luna 설정](https://github.com/KiHyeonLee1121/da-daka_Ai/blob/main/ros2_ws/src/da_daka_control/config/tf_luna_serial.yaml), [distance_filter.py](https://github.com/KiHyeonLee1121/da-daka_Ai/blob/main/ros2_ws/src/da_daka_control/da_daka_control/distance_filter.py), [compose](https://github.com/KiHyeonLee1121/da-daka_Ai/blob/main/deploy/pi-compose.yaml).
- MAVROS 2.14 [velocity plugin 원문](https://github.com/mavlink/mavros/blob/2.14.0/mavros/src/plugins/setpoint_velocity.cpp): LOCAL_NED 출력, ROS ENU 입력 변환, mask 1479로 위치/가속도/yaw angle 제외.

| 값 | 사용 근거/범위 |
|---|---|
| yaw helper·kp 1.0·rate 0.35rad/s | 기존 distance_controller 함수·YAML의 wrap/P/clamp 유지 |
| Z kp 0.6·kd 0.15·최대 0.25m/s·가속 0.5m/s² | 기존 거리 제어 설정에서 가져옴. 전체 legacy mission을 그대로 이식한 것은 아님 |
| TF-Luna 0.2~8m·115200 baud | 기존 설정. 이번 노드는 기존 Range 입력을 구독하며 serial driver를 새로 실행하지 않음 |
| 20Hz·0.3s sensor stale·0.5s rate window | 기존 제어 설정. 현장 실제 주기·time sync 검증 필요 |
| 3m·명령 5m·수평 0속도 5초 | 사용자의 이번 시험 사양. legacy의 1m/1.1m 고도 목표를 그대로 사용하지 않음 |
| 전진 0.35m/s·가속 0.25m/s²·명령거리 허용 0.15m | 새 시험 초기 설정. 로그로 실제 공간 이동/제동을 교정한 값은 아님 |
| 라이다 offset 0m | 미측정 placeholder. 운영자가 현재 장착값 측정 후 설정 |

기존 distance_filter는 median+moving-average이며 기울기/장착오프셋 보정을 하지 않습니다. 그래서 v1은 `lidar_input_is_vertical_height=false`가 기본입니다. 실제 현장 소스가 이미 보정했다면 true로 바꿔 중복 보정을 방지합니다. raw 높이 공식은 body-down 축 정렬·평탄한 지면을 전제로 합니다.

main compose에는 FC 57600 baud, 첨부 현장 진단에는 921600 baud가 남아 있으므로 현재값을 과거 자료에서 단정하지 않습니다. 현재 MAVROS 연결 설정을 재사용해야 합니다. 과거 자율 미션의 분사 feedforward·위치맵·position setpoint는 가져오지 않습니다.

## 로그를 제어 피드백으로 오용하지 않은 부분

log154의 `COM_RC_IN_MODE=1`, `COM_RC_OVERRIDE=1`은 현재 RC 탈취를 보장하는 근거가 아닙니다. 수동 로그는 POSCTL 위주이며 일부 ALTCTL/AUTO 구간이 있습니다. 스틱 수치를 고정 거리로 바꾸는 변환표를 만들지 않았습니다.

명령 적분은 이전 성공한 로컬 publish의 final ENU 속도를 출발 yaw 방향에 투영해 다음 publish까지 적분합니다. 불규칙 dt와 부호를 그대로 유지합니다. 다음 명령 계산에는 아직 적분하지 않은 held-command 구간을 예측하되 누적값에 중복 반영하지 않습니다. 발행 실패·긴 공백은 정상 완료를 막습니다. 5m를 채운 뒤 감속하는 대신 현재 명령속도의 제동 면적과 dispatch 여유를 남겨 BRAKE를 시작합니다. 기본 합성 일정 주기 시험의 총 명령거리는 4.97m이며 실제 이동 정확도의 증거가 아닙니다.

현재 yaw는 출발 기준을 다시 저장하는 데 쓰지 않습니다. XY 이동 방향도 고정 yaw_ref이고 lateral command는 0입니다. 추정 XY 위치/속도·GPS는 node의 telemetry 사전에만 들어가고 mission Sensors에는 해당 필드가 없습니다. 5초 타이머 역시 actual XY velocity를 검사하지 않습니다.

## 한계와 현장 확인

ROS 2/DDS/MAVROS·실제 Pi/라이다/FC에 접속해 검증하지 않았습니다. 실제 기체 파라미터·RC switch·OFFBOARD-loss·AUTO.LAND 경로·장착 오프셋은 드론 컴퓨터에서 확인해야 합니다. 코드 준비와 실비행 허용을 분리했고 이번 개발 중 arming/비행은 수행하지 않았습니다.

heading reset은 pose yaw 변화에서 IMU 기반 회전을 뺀 innovation으로 감지합니다. 모든 EKF reset counter를 직접 관측하는 방식이 아니며 시간 정렬/임계값을 현장에서 검증해야 합니다. State.system_status의 critical/emergency 표시는 전체 PX4 failsafe 상태가 아닙니다. ULog에서 이를 보완해야 합니다.

LAND가 실제로 관측되면 setpoint를 완전히 끊습니다. 중단 후 속도 추정이 무효이거나 publish/frame/제어권에 문제가 있으면 zero stream이 안전하다고 가정하지 않습니다. LAND 거부/통신 상실/프로세스 종료 이후의 회수는 기체에 이미 적용되고 현장에서 검증된 FC 정책에 의존합니다.

## 공통 로직 통일 (0.2.0)

현재 제어는 [shared_flight_logic.md](shared_flight_logic.md)의 동일한 공통 패키지를 사용합니다. 기존 v2의 timer 평가 시각 적분을 actual publish 시각 적분으로 바꿨고, raw 라이다 기본(false), rate 창 0.5초, 상승 안정 ±0.10m/±0.05m/s, 단계 제한시간과 critical/중단/LAND stream latch를 v1 기준으로 통일했습니다. 영상 검출기·target lock·이미지 변환과 현장 교정 요구는 유지합니다. 기존 속도명령 시험보다 실기체 정확도가 향상됐다는 주장은 하지 않습니다.
