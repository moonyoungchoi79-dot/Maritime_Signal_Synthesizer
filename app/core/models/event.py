from dataclasses import dataclass

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
    reference_ship_idx: int = -1 # For DIST_UNDER/OVER
