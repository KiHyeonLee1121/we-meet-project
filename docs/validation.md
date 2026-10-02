# v1/v2 공통화 검증 (0.2.0)

2026-10-02, Python 3.12·OpenCV 5.0.0·NumPy 2.5.3 개발 환경에서 수행했습니다.

- v1 **37개**, v2 **59개** unittest 통과. 공통 flight/adapter 36개 및 manifest 검사를 양쪽에서 동일하게 실행했고, v2에는 command parity·visual mission·OpenCV/추적·설정 이전·camera adapter 검증을 추가했습니다.
- 공통 패키지 **12개 파일** SHA256 완전 일치: `verify_shared_core.py --other ...` 통과.
- vision=false 전체 trial의 command/state/distance, 건강한 영상과 목표 미검출 상태의 BRAKE 종료 직전까지 command/state/distance 일치. 독립 core/v2 controller 인스턴스에 같은 센서 이력·불규칙 timer·dispatch 지연 입력.
- 정상 20Hz camera-free 합성 시험: 명령거리 **4.970m**; 불규칙 **4.960325m**; 상승 단독 **0m**. 모두 5초 zero-command 유지 후 LAND 및 COMPLETE.
- 실제 OpenCV 사용 합성 영상: ADVANCE→BRAKE→ALIGN→VISUAL_HOLD→LANDING→COMPLETE, `panel_centered_5s`. 접근 명령거리 **4.8825m**이며 영상 전환 때문에 camera-free 5m와 다릅니다.
- v2 baseline 합성 시험도 **4.970m**, `expected_5m_zero_command_5s`, COMPLETE.
- 기록의 실제 publish 시각 재적분 결과와 누적 command distance 일치, invalid interval 0.
- 전체 Python 문법 컴파일, setup의 package/data_files, XML package 이름·공통 의존성, YAML 로드 및 launch 경로 정적 확인.

공통 검증은 yaw 고정/wrap/ENU 부호, 실제 발행 시각 적분·부호·예측 중복 방지·감속 면적, prestream/모드/arming 실관측 구분, raw 라이다/offset/기울기·중복 보정 방지·rate 창 준비, estimator/stale/critical/제어권 변경·phase timeout·발행 실패·긴 공백, 첫 실제 zero-command 발행 타이머, LAND 인계/거부·재진입 금지, run_id 로그와 GPS/XY가 제어 입력에 없는지입니다.

영상 검증은 패널 획득 후 공통 ramp 감속, 현재 yaw를 이용한 영상 변환, 중앙/고도/yaw/신선도 중 하나가 깨졌을 때 5초 초기화, 실제 zero-command 발행 후 타이머 시작, BRAKE·ALIGN 중 target loss, stale camera·invalid estimator·critical, 패널 없는 예산 소진, align phase timeout, target lock·중복 frame·stride·clipped rectangle 거부, live camera axes gate와 최신 영상 queue/처리 오류의 health 미갱신입니다.

어댑터 시험은 ROS 메시지/서비스/publisher test double입니다. 실제 ROS 2 executor/DDS/MAVROS/colcon 통합 시험이 아닙니다. Pi 카메라 장치·실제 ROS 환경이 없으므로 실제 카메라 입력·arming·비행은 수행하지 않았습니다. 현장 영상의 인식/오인식률·실제 5m 접근/제동/정지 정확도도 미측정입니다.

합성 plant는 명령속도를 바로 실제 속도로 가정하여 바람·관성·센서 오차를 재현하지 않습니다. 실기체 검증은 README의 기체 설정 확인과 상승 단독→baseline→영상 시험 순서, 원본 ULog·run_id·외부 측정으로 수행해야 합니다.
