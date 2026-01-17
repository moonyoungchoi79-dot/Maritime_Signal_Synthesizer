from dataclasses import dataclass, field
from typing import List, Dict

@dataclass
class ReceptionModelConfig:
    """AIS/Radar/Camera 수신 모델 설정"""
    enabled: bool = True
    d0: float = 40.0          # Max reliable range (NM)
    d1: float = 50.0          # Fade-out end (NM)
    p0: float = 0.01          # Near-range dropout probability
    p1: float = 0.95          # At d1 dropout probability
    full_block_at_d1: bool = True  # d >= d1에서 완전 차단

    # Advanced
    curve_type: str = "smoothstep"  # "smoothstep", "linear", "logistic"

    # Burst loss (Advanced)
    burst_enabled: bool = False
    burst_min_sec: float = 1.0
    burst_max_sec: float = 5.0
    burst_trigger_mult: float = 2.0

@dataclass
class ARPATrackConfig(ReceptionModelConfig):
    """ARPA Track 전용 추가 설정"""
    coast_time_sec: float = 5.0      # 유실 후 트랙 유지 시간
    reacquire_prob: float = 0.8      # 재획득 확률

# 프리셋 정의
RECEPTION_PRESETS = {
    "realistic": {
        "ais": {"d0": 40, "d1": 50, "p0": 0.01, "p1": 0.95},
        "radar_detect": {"d0": 6, "d1": 24, "p0": 0.02, "p1": 0.80},
        "arpa_track": {"d0": 8, "d1": 18, "p0": 0.05, "p1": 0.90, "coast_time_sec": 5.0, "reacquire_prob": 0.8},
        "camera": {"d0": 1, "d1": 5, "p0": 0.01, "p1": 0.90}
    },
    "stable": {
        "ais": {"d0": 40, "d1": 80, "p0": 0.01, "p1": 0.30},
        "radar_detect": {"d0": 6, "d1": 48, "p0": 0.02, "p1": 0.30},
        "arpa_track": {"d0": 8, "d1": 36, "p0": 0.05, "p1": 0.40, "coast_time_sec": 10.0, "reacquire_prob": 0.9},
        "camera": {"d0": 2, "d1": 8, "p0": 0.01, "p1": 0.30}
    },
    "harsh": {
        "ais": {"d0": 20, "d1": 30, "p0": 0.05, "p1": 0.99},
        "radar_detect": {"d0": 4, "d1": 12, "p0": 0.10, "p1": 0.95},
        "arpa_track": {"d0": 6, "d1": 10, "p0": 0.10, "p1": 0.99, "coast_time_sec": 2.0, "reacquire_prob": 0.5},
        "camera": {"d0": 0.5, "d1": 2, "p0": 0.10, "p1": 0.99}
    }
}

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

    # Reception Model (거리 기반 수신 모델)
    reception_model_enabled: bool = True
    reception_preset: str = "realistic"  # "realistic", "stable", "harsh", "custom"

    ais_reception: ReceptionModelConfig = field(default_factory=lambda: ReceptionModelConfig(
        d0=40.0, d1=50.0, p0=0.01, p1=0.95
    ))
    radar_detect: ReceptionModelConfig = field(default_factory=lambda: ReceptionModelConfig(
        d0=6.0, d1=24.0, p0=0.02, p1=0.80
    ))
    arpa_track: ARPATrackConfig = field(default_factory=lambda: ARPATrackConfig(
        d0=8.0, d1=18.0, p0=0.05, p1=0.90, coast_time_sec=5.0, reacquire_prob=0.8
    ))
    camera_reception: ReceptionModelConfig = field(default_factory=lambda: ReceptionModelConfig(
        d0=1.0, d1=5.0, p0=0.01, p1=0.90
    ))
