from dataclasses import dataclass, field
from typing import List

@dataclass
class EventCondition:
    """복합 이벤트 조건 (AND/OR 연결)"""
    event_id: str                    # 참조할 이벤트 ID
    mode: str = "TRIGGERED"          # "TRIGGERED" (발동됨), "NOT_TRIGGERED" (발동 안됨)

@dataclass
class EventTrigger:
    id: str
    name: str
    enabled: bool = True
    trigger_type: str = "TIME" # "TIME", "AREA_ENTER", "AREA_LEAVE", "CPA_UNDER", "CPA_OVER", "DIST_UNDER", "DIST_OVER"
    condition_value: float = 0.0 # Time (sec) or Area ID
    action_type: str = "STOP" # "STOP", "CHANGE_SPEED", "CHANGE_HEADING"
    target_ship_idx: int = 0
    action_value: float = 0.0 # New Speed or New Heading
    is_relative_to_end: bool = False
    reference_ship_idx: int = 0 # For DIST_UNDER/OVER

    # 조건부 이벤트: 선행 이벤트 조건
    prerequisite_events: List[EventCondition] = field(default_factory=list)
    prerequisite_logic: str = "AND"  # "AND" (모든 조건 충족), "OR" (하나라도 충족)

# 별칭 (하위 호환성)
SimEvent = EventTrigger
