import numpy as np
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple

@dataclass
class ShipData:
    idx: int
    name: str
    note: str = ""
    mmsi: int = 0
    is_random_target: bool = False 
    
    variances: Dict[str, float] = field(default_factory=dict)
    signals_enabled: Dict[str, bool] = field(default_factory=dict)
    signal_intervals: Dict[str, float] = field(default_factory=dict)
    
    raw_points: List[Tuple[float, float]] = field(default_factory=list) 
    raw_speeds: Dict[int, float] = field(default_factory=dict)
    raw_notes: Dict[int, str] = field(default_factory=dict)
    
    resampled_points: List[Tuple[float, float]] = field(default_factory=list)
    
    node_velocities_kn: List[float] = field(default_factory=list)
    base_velocities_kn: List[float] = field(default_factory=list)
    
    cumulative_time: List[float] = field(default_factory=list)
    
    total_duration_sec: float = 0.0
    
    speed_keyframes: Dict[int, float] = field(default_factory=dict)
    speed_notes: Dict[int, str] = field(default_factory=dict)
    
    time_series: List[Dict] = field(default_factory=list)
    packed_data: Optional[np.ndarray] = None 
    
    is_generated: bool = False