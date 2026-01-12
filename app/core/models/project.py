import datetime
from dataclasses import dataclass, field
from typing import List, Optional

from app.core.models.ship import ShipData
from app.core.models.area import Area
from app.core.models.event import EventTrigger
from app.core.models.settings import ProjectSettings
from app.core.models.map_info import MapInfo

@dataclass
class Project:
    def __init__(self):
        self.project_name = "Untitled"
        self.project_path = ""
        self.start_time: datetime.datetime = datetime.datetime.now(datetime.timezone.utc)
        self.unit_time: float = 1.0 
        self.seed: int = int(datetime.datetime.now().timestamp())
        
        self.map_info = MapInfo()
        self.settings = ProjectSettings()
        defaults = [
            "AIVDM", "RATTM", "Camera"]
        for d in defaults:
           self.settings.dropout_probs[d] = 0.1

        self.ships: List[ShipData] = []
        self.areas: List[Area] = []
        self.events: List[EventTrigger] = []
        
    def reset(self):
        self.ships = []
        self.areas = []

    def get_ship_by_idx(self, idx) -> Optional[ShipData]:
        for s in self.ships:
            if s.idx == idx: return s
        return None
    
    def get_area_by_id(self, sid) -> Optional[Area]:
        for s in self.areas:
            if s.id == sid: return s
        return None

current_project = Project()