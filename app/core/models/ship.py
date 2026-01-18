import numpy as np
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple

# Ship class dimensions: (length_m, beam_m, draft_m, air_draft_m)
SHIP_CLASS_DIMENSIONS = {
    "CONTAINER": (300.0, 40.0, 15.0, 60.0),
    "TANKER": (250.0, 45.0, 16.0, 50.0),
    "CARGO": (200.0, 32.0, 12.0, 45.0),
    "PASSENGER": (180.0, 28.0, 7.0, 55.0),
    "FISHING": (30.0, 8.0, 3.0, 12.0),
    "BUOY": (2.0, 2.0, 1.0, 2.0),
    "OTHER": (50.0, 12.0, 4.0, 20.0),
}

def get_ship_dimensions(ship_class: str) -> tuple:
    """Get ship dimensions by class. Returns (length, beam, draft, air_draft)."""
    return SHIP_CLASS_DIMENSIONS.get(ship_class, SHIP_CLASS_DIMENSIONS["OTHER"])


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

    # Ship class/type and dimensions (for camera bbox calculation)
    # Default values per specification: CONTAINER, 300m/40m/15m/60m
    ship_class: str = "CONTAINER"  # Options: CONTAINER, FISHING, TANKER, CARGO, PASSENGER, BUOY
    length_m: float = 300.0        # Ship length (meters)
    beam_m: float = 40.0           # Ship beam/width (meters)
    draft_m: float = 15.0          # Ship draft (meters)
    air_draft_m: float = 60.0      # Air draft above waterline (meters)
    height_m: float = 30.0         # Visible height above waterline for camera (meters)

    def get_display_segments(self) -> List[List[Tuple[float, float]]]:
        """
        날짜변경선(180도) 통과 시 지도가 가로질러 그려지는 현상을 방지하기 위해
        경로를 분할하여 반환합니다.
        """
        points = self.resampled_points if self.resampled_points else self.raw_points
        if not points:
            return []

        segments = []
        current_segment = [points[0]]

        for i in range(1, len(points)):
            prev_lat, prev_lon = points[i-1]
            curr_lat, curr_lon = points[i]

            if abs(curr_lon - prev_lon) > 180.0:
                segments.append(current_segment)
                current_segment = []
            
            current_segment.append((curr_lat, curr_lon))
        
        if current_segment:
            segments.append(current_segment)
            
        return segments