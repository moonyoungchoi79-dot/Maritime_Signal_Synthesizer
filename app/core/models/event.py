from dataclasses import dataclass

import json

class Event:
    # Trigger Types
    TRIGGER_TIME = "TIME"
    TRIGGER_AREA_ENTER = "AREA_ENTER"
    TRIGGER_AREA_LEAVE = "AREA_LEAVE"
    TRIGGER_DISTANCE = "DISTANCE_TO_SHIP" # [추가]

    # Action Types
    ACTION_CHANGE_SPEED = "CHANGE_SPEED"
    ACTION_CHANGE_HEADING = "CHANGE_HEADING"
    ACTION_STOP = "STOP" # sog=0
    
    def __init__(self, name="New Event"):
        self.name = name
        self.enabled = True
        
        # Trigger
        self.trigger_type = Event.TRIGGER_TIME
        self.trigger_time = 60.0 # seconds
        self.trigger_area_id = -1
        
        # [추가] 거리 트리거 관련 속성
        self.trigger_target_ship_idx = -1 # 거리를 잴 대상 선박
        self.trigger_distance = 1000.0    # 거리 (미터)
        self.trigger_condition = "UNDER"  # "OVER" or "UNDER"

        # ActionTarget
        self.target_ship_idx = 0 
        
        # Action
        self.action_type = Event.ACTION_CHANGE_SPEED
        self.action_value = 0.0 # speed(kn) or heading(deg)

    def to_dict(self):
        return {
            "name": self.name,
            "enabled": self.enabled,
            "trigger_type": self.trigger_type,
            "trigger_time": self.trigger_time,
            "trigger_area_id": self.trigger_area_id,
            "trigger_target_ship_idx": self.trigger_target_ship_idx, # [추가]
            "trigger_distance": self.trigger_distance,               # [추가]
            "trigger_condition": self.trigger_condition,             # [추가]
            "target_ship_idx": self.target_ship_idx,
            "action_type": self.action_type,
            "action_value": self.action_value
        }

    @staticmethod
    def from_dict(d):
        e = Event(d.get("name", "New Event"))
        e.enabled = d.get("enabled", True)
        e.trigger_type = d.get("trigger_type", Event.TRIGGER_TIME)
        e.trigger_time = d.get("trigger_time", 60.0)
        e.trigger_area_id = d.get("trigger_area_id", -1)
        
        # [추가]
        e.trigger_target_ship_idx = d.get("trigger_target_ship_idx", -1)
        e.trigger_distance = d.get("trigger_distance", 1000.0)
        e.trigger_condition = d.get("trigger_condition", "UNDER")

        e.target_ship_idx = d.get("target_ship_idx", 0)
        e.action_type = d.get("action_type", Event.ACTION_CHANGE_SPEED)
        e.action_value = d.get("action_value", 0.0)
        return e

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
