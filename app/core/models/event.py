"""
이벤트 모델 모듈

이 모듈은 시뮬레이션에서 사용되는 이벤트 트리거와 조건을 정의합니다.
이벤트는 특정 조건이 충족될 때 선박의 동작을 변경하는 데 사용됩니다.

클래스:
    EventCondition: 복합 이벤트의 선행 조건
    EventTrigger: 이벤트 트리거 정의

이벤트 트리거 유형:
    - TIME: 특정 시간에 발동
    - AREA_ENTER: 영역 진입 시 발동
    - AREA_LEAVE: 영역 이탈 시 발동
    - CPA_UNDER: 최근접점(CPA)이 특정 거리 미만일 때 발동
    - CPA_OVER: 최근접점(CPA)이 특정 거리 이상일 때 발동
    - DIST_UNDER: 거리가 특정 값 미만일 때 발동
    - DIST_OVER: 거리가 특정 값 이상일 때 발동

이벤트 동작 유형:
    - STOP: 선박 정지
    - CHANGE_SPEED: 속도 변경
    - CHANGE_HEADING: 방향 변경
"""

from dataclasses import dataclass, field
from typing import List


@dataclass
class EventCondition:
    """
    복합 이벤트의 선행 조건을 정의하는 데이터 클래스입니다.

    다른 이벤트의 발동 여부를 조건으로 사용할 수 있습니다.
    여러 조건을 AND/OR 로직으로 연결할 수 있습니다.

    속성:
        event_id: 참조할 이벤트의 고유 ID
        mode: 조건 모드
            - "TRIGGERED": 해당 이벤트가 발동되었을 때
            - "NOT_TRIGGERED": 해당 이벤트가 발동되지 않았을 때
    """
    event_id: str  # 참조할 이벤트 ID
    mode: str = "TRIGGERED"  # 조건 모드 ("TRIGGERED" 또는 "NOT_TRIGGERED")


@dataclass
class EventTrigger:
    """
    이벤트 트리거를 정의하는 데이터 클래스입니다.

    특정 조건이 충족될 때 대상 선박에 동작(정지, 속도 변경, 방향 변경 등)을
    수행하도록 설정할 수 있습니다.

    속성:
        id: 이벤트 고유 식별자 (UUID)
        name: 이벤트 이름 (사용자 표시용)
        enabled: 이벤트 활성화 여부
        trigger_type: 트리거 조건 유형
            - "TIME": 시간 기반
            - "AREA_ENTER": 영역 진입
            - "AREA_LEAVE": 영역 이탈
            - "CPA_UNDER": 최근접점 거리 미만
            - "CPA_OVER": 최근접점 거리 이상
            - "DIST_UNDER": 거리 미만
            - "DIST_OVER": 거리 이상
        condition_value: 조건 값 (시간(초) 또는 거리(해리))
        action_type: 동작 유형
            - "STOP": 정지
            - "CHANGE_SPEED": 속도 변경
            - "CHANGE_HEADING": 방향 변경
        target_ship_idx: 대상 선박 인덱스
        action_value: 동작 값 (새 속도 또는 새 방향)
        is_relative_to_end: 시간 조건이 종료 시간 기준인지 여부
        reference_ship_idx: 거리 계산 기준 선박 인덱스
        prerequisite_events: 선행 이벤트 조건 목록
        prerequisite_logic: 선행 조건 연결 로직
            - "AND": 모든 조건 충족 필요
            - "OR": 하나라도 충족하면 됨
    """
    id: str  # 이벤트 고유 ID
    name: str  # 이벤트 이름
    enabled: bool = True  # 활성화 여부
    trigger_type: str = "TIME"  # 트리거 유형
    condition_value: float = 0.0  # 조건 값 (시간 또는 거리)
    action_type: str = "STOP"  # 동작 유형
    target_ship_idx: int = 0  # 대상 선박 인덱스
    action_value: float = 0.0  # 동작 값 (속도 또는 방향)
    is_relative_to_end: bool = False  # 종료 시간 기준 여부
    reference_ship_idx: int = 0  # 기준 선박 인덱스 (거리 조건용)

    # 조건부 이벤트: 선행 이벤트 조건
    prerequisite_events: List[EventCondition] = field(default_factory=list)  # 선행 조건 목록
    prerequisite_logic: str = "AND"  # 조건 연결 로직 ("AND" 또는 "OR")


# 별칭 (하위 호환성 유지)
SimEvent = EventTrigger
