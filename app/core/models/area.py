from dataclasses import dataclass, field
from typing import List, Tuple

@dataclass
class Area:
    id: int
    name: str
    note: str = ""
    geometry: List[Tuple[float, float]] = field(default_factory=list) 
    color: str = "#808080"
