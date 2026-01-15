from dataclasses import dataclass, field
from typing import List, Dict

@dataclass
class ProjectSettings:
    select_color: str = "#0000FF"
    own_color: str = "#008000"
    target_color: str = "#FF0000"
    path_thickness: int = 3
    traveled_path_thickness: int = 4
    highlight_path_color: str = "#0000FF"
    select_thickness_mult: float = 1.5
    show_speed_notes: bool = True 
    
    btn_speed_color: str = "#FF9800"
    btn_sim_color: str = "#2196F3"
    btn_rtg_color: str = "#FFA500"
    rtg_zigzag_enabled: bool = True
    rtg_zigzag_turns: int = 2
    rtg_zigzag_angle_limit: float = 60.0
    random_color: str = "#B8860B"
    mask_color: str = "#404040"

    theme_mode: str = "System" # "System", "Light", "Dark"

    # Unified speed variance for wind/current effects (shared by RTG and simulation)
    speed_variance: float = 1.0
    
    own_ship_idx: int = 0
    include_gp_signal: bool = False 
    jitter_enabled: bool = False
    
    dropout_probs: Dict[str, float] = field(default_factory=lambda: {
        "AIVDM": 0.1, "RATTM": 0.1, "Camera": 0.1
    })
    ais_fragment_probs: List[float] = field(default_factory=lambda: [8.0, 1.0, 0.5, 0.4, 0.1])
