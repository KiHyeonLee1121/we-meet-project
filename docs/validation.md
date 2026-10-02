# v1 무비행 검증

2026-10-02 개발 환경 Python 3.12에서 수행했습니다.

- `python3 -m unittest discover -s tests -v`: **37개 통과**.
- Python 문법 컴파일 성공. YAML 프로필 로드와 패키지/launch 경로 확인.
- 일정 20Hz 합성 command-response: 명령거리 **4.970m**, 수평 0속도 명령 5초·착륙/해제 종료.
- 불규칙 0.03/0.08/0.05/0.10/0.04s 합성 시험: 명령거리 **4.960325m**, 같은 단계 종료.
- 상승 단독 합성 시험: 전진 명령·적분거리 0, 5초 정지명령 후 착륙 종료.
- 오프라인 기록에서 실제 publish 시각을 재적분하면 코드 누적거리와 일치.

시험 범위는 고정 yaw helper/ENU 부호, 가속·감속 면적, 불규칙 dt·signed projection·중복 적분 방지, prestream/실제 모드·arming 관측, invalid/stale 센서·추정기, 수동/FC takeover, phase timeout, 발행 실패·긴 공백, zero 타이머, LAND handoff/거부, raw 라이다 보정·중복 보정 방지, 자동 run_id 파일·로그 summary, GPS/XY telemetry가 mission 입력에 없는지입니다.

어댑터 테스트는 ROS message/service/publisher의 test double입니다. 실제 ROS 2 executor, DDS, colcon, MAVROS 또는 기체 비행 통합 테스트는 아닙니다. 합성 plant는 속도명령을 즉시 실제 속도로 가정하므로 바람·관성·추정 오차·실제 제동거리를 재현하지 않습니다. 실제 공간 5m 접근·정지 성능은 미검증입니다.

현장 검증은 README 순서대로 기체 환경과 설정을 확인하고 상승 단독 → 속도 5m 시험으로 진행합니다. 원본 ULog·run_id 폴더·현장 측정·영상이 후속 분석 입력입니다.

공통화 후 v1은 37개, v2는 59개 무비행 테스트를 통과했습니다. 두 브랜치의 공통 패키지 12개 파일 SHA256이 완전히 일치하며 `verify_shared_core.py --other ...`로 확인했습니다. v2 baseline의 전체 command/state/integral, 영상 정상·패널 미검출의 BRAKE 종료 직전까지, 불규칙 dt와 actual dispatch 지연을 두 독립 controller 인스턴스로 비교해 일치했습니다. 패키지 setup data_files·XML 이름·공통 의존성 경로도 정적으로 확인했습니다. 새 공통화 후 실제 colcon/ROS 통합·실비행은 하지 않았습니다.
