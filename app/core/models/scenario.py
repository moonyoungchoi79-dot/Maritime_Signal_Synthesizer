import json
import uuid
from app.core.models.project import EventTrigger

class Scenario:
    def __init__(self):
        self.name = "New Scenario"
        self.description = ""
        self.scope_mode = "ALL_SHIPS" # ALL_SHIPS, OWN_ONLY, TARGET_ONLY, SELECTED_SHIPS
        self.selected_ships = [] # List of ship IDs (int)
        self.events = [] # List of EventTrigger objects
        self.enabled = False

    def to_dict(self):
        return {
            "name": self.name,
            "description": self.description,
            "scope_mode": self.scope_mode,
            "selected_ships": self.selected_ships,
            "events": [self._event_to_dict(e) for e in self.events]
        }

    def _event_to_dict(self, e):
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
            "is_relative_to_end": getattr(e, 'is_relative_to_end', False)
        }

    def save_to_file(self, filepath):
        data = self.to_dict()
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=4, ensure_ascii=False)

    @classmethod
    def load_from_file(cls, filepath):
        with open(filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)
        
        scen = cls()
        scen.name = data.get("name", "New Scenario")
        scen.description = data.get("description", "")
        scen.scope_mode = data.get("scope_mode", "ALL_SHIPS")
        scen.selected_ships = data.get("selected_ships", [])
        
        scen.events = []
        for e_data in data.get("events", []):
            # Reconstruct EventTrigger
            # Using the constructor signature compatible with EventTrigger
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
            scen.events.append(evt)
            
        return scen