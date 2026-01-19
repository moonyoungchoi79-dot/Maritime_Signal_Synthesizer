"""
시나리오 모델 모듈

이 모듈은 시뮬레이션 시나리오를 정의하고 관리합니다.
시나리오는 여러 이벤트를 묶어서 특정 상황을 재현하는 데 사용됩니다.

클래스:
    Scenario: 시뮬레이션 시나리오를 표현하는 클래스

시나리오 범위 모드:
    - ALL_SHIPS: 모든 선박에 적용
    - OWN_ONLY: 자선(OwnShip)에만 적용
    - TARGET_ONLY: 타선(Target)에만 적용
    - SELECTED_SHIPS: 선택된 선박에만 적용
"""

import json
import uuid
from app.core.models.project import EventTrigger
from app.core.models.event import EventCondition


class Scenario:
    """
    시뮬레이션 시나리오를 표현하는 클래스입니다.

    시나리오는 여러 이벤트를 포함하며, 특정 상황(예: 충돌 회피 시나리오)을
    재현하기 위해 사용됩니다. 파일로 저장하고 불러올 수 있습니다.

    속성:
        name: 시나리오 이름
        description: 시나리오 설명
        scope_mode: 이벤트 적용 범위
            - "ALL_SHIPS": 모든 선박
            - "OWN_ONLY": 자선만
            - "TARGET_ONLY": 타선만
            - "SELECTED_SHIPS": 선택된 선박만
        selected_ships: 선택된 선박 ID 목록 (SELECTED_SHIPS 모드용)
        events: 시나리오에 포함된 이벤트 목록
        enabled: 시나리오 활성화 여부
    """

    def __init__(self):
        """시나리오 인스턴스를 기본값으로 초기화합니다."""
        self.name = "New Scenario"  # 시나리오 이름
        self.description = ""  # 시나리오 설명
        self.scope_mode = "ALL_SHIPS"  # 적용 범위 모드
        self.selected_ships = []  # 선택된 선박 ID 목록
        self.events = []  # 이벤트 목록 (EventTrigger 객체)
        self.enabled = False  # 활성화 여부

    def to_dict(self):
        """
        시나리오를 딕셔너리로 변환합니다.

        JSON 직렬화를 위한 중간 형태로, 파일 저장에 사용됩니다.

        반환값:
            시나리오 데이터를 담은 딕셔너리
        """
        return {
            "name": self.name,
            "description": self.description,
            "scope_mode": self.scope_mode,
            "selected_ships": self.selected_ships,
            "events": [self._event_to_dict(e) for e in self.events]
        }

    def _event_to_dict(self, e):
        """
        이벤트 객체를 딕셔너리로 변환합니다.

        매개변수:
            e: EventTrigger 객체

        반환값:
            이벤트 데이터를 담은 딕셔너리
        """
        # 조건부 이벤트의 선행 조건 직렬화
        prereqs = getattr(e, 'prerequisite_events', [])
        prereq_list = []
        for cond in prereqs:
            if isinstance(cond, dict):
                prereq_list.append({
                    "event_id": cond.get('event_id', ''),
                    "mode": cond.get('mode', 'TRIGGERED')
                })
            else:
                prereq_list.append({
                    "event_id": cond.event_id,
                    "mode": cond.mode
                })

        return {
            "id": e.id,
            "name": e.name,
            "enabled": e.enabled,
            "trigger_type": e.trigger_type,
            "condition_value": e.condition_value,
            "action_type": e.action_type,
            "target_ship_idx": e.target_ship_idx,
            "reference_ship_idx": getattr(e, 'reference_ship_idx', -1),
            "action_value": e.action_value,
            "is_relative_to_end": getattr(e, 'is_relative_to_end', False),
            "action_option": getattr(e, 'action_option', ""),
            "prerequisite_events": prereq_list,
            "prerequisite_logic": getattr(e, 'prerequisite_logic', 'AND')
        }

    def save_to_file(self, filepath):
        """
        시나리오를 JSON 파일로 저장합니다.

        매개변수:
            filepath: 저장할 파일 경로
        """
        data = self.to_dict()
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=4, ensure_ascii=False)

    @classmethod
    def load_from_file(cls, filepath):
        """
        JSON 파일에서 시나리오를 로드합니다.

        매개변수:
            filepath: 로드할 파일 경로

        반환값:
            로드된 Scenario 객체
        """
        with open(filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)

        scen = cls()
        scen.name = data.get("name", "New Scenario")
        scen.description = data.get("description", "")
        scen.scope_mode = data.get("scope_mode", "ALL_SHIPS")
        scen.selected_ships = data.get("selected_ships", [])

        # 이벤트 목록 복원
        scen.events = []
        for e_data in data.get("events", []):
            # EventTrigger 객체 재구성
            evt = EventTrigger(
                id=e_data.get("id", str(uuid.uuid4())),
                name=e_data.get("name", "Unknown Event"),
                enabled=e_data.get("enabled", True),
                trigger_type=e_data.get("trigger_type", "TIME"),
                condition_value=e_data.get("condition_value", 0),
                action_type=e_data.get("action_type", "STOP"),
                target_ship_idx=e_data.get("target_ship_idx", 0),
                action_value=e_data.get("action_value", 0),
                is_relative_to_end=e_data.get("is_relative_to_end", False),
                reference_ship_idx=e_data.get("reference_ship_idx", -1)
            )
            evt.action_option = e_data.get("action_option", "")

            # 조건부 이벤트의 선행 조건 로드
            evt.prerequisite_logic = e_data.get("prerequisite_logic", "AND")
            evt.prerequisite_events = [
                EventCondition(event_id=c['event_id'], mode=c['mode'])
                for c in e_data.get("prerequisite_events", [])
            ]

            scen.events.append(evt)

        return scen
