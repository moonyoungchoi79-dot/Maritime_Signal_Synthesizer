"""
프로젝트 설정 모델 모듈

이 모듈은 프로젝트 전반의 설정값들을 관리합니다.

클래스:
    ReceptionModelConfig: 신호 수신 모델 설정
    ProjectSettings: 프로젝트 전체 설정

수신 모델 프리셋:
    - realistic: 실제 환경 모사 (기본값)
    - stable: 안정적인 환경 (낮은 유실률)
    - harsh: 열악한 환경 (높은 유실률)
"""

from dataclasses import dataclass, field
from typing import List, Dict


@dataclass
class ReceptionModelConfig:
    """
    신호 수신 모델 설정을 정의하는 데이터 클래스입니다.

    거리에 따른 신호 유실 확률을 모델링합니다.
    가까운 거리에서는 신호 수신이 양호하고,
    멀어질수록 유실 확률이 증가합니다.

    속성:
        enabled: 수신 모델 활성화 여부
        d0: 최대 신뢰 거리 (해리) - 이 거리까지는 p0 확률로 유실
        d1: 페이드아웃 종료 거리 (해리) - 이 거리 이상에서 p1 확률로 유실
        p0: 근거리 유실 확률 (0~1)
        p1: d1 거리에서의 유실 확률 (0~1)
        full_block_at_d1: d1 이상 거리에서 완전 차단 여부
        curve_type: 유실 확률 변화 곡선 유형
            - "smoothstep": 부드러운 S자 곡선
            - "linear": 선형 변화
            - "logistic": 로지스틱 곡선
        burst_enabled: 버스트 유실 활성화 여부
        burst_min_sec: 버스트 유실 최소 지속 시간 (초)
        burst_max_sec: 버스트 유실 최대 지속 시간 (초)
        burst_trigger_mult: 버스트 발생 확률 배수
    """
    enabled: bool = True  # 수신 모델 활성화 여부
    d0: float = 40.0  # 최대 신뢰 거리 (해리)
    d1: float = 50.0  # 페이드아웃 종료 거리 (해리)
    p0: float = 0.01  # 근거리 유실 확률
    p1: float = 0.95  # d1에서의 유실 확률
    full_block_at_d1: bool = True  # d >= d1에서 완전 차단

    # 고급 설정
    curve_type: str = "smoothstep"  # 유실 확률 변화 곡선 유형

    # 버스트 유실 설정 (고급)
    burst_enabled: bool = False  # 버스트 유실 활성화
    burst_min_sec: float = 1.0  # 버스트 최소 지속 시간
    burst_max_sec: float = 5.0  # 버스트 최대 지속 시간
    burst_trigger_mult: float = 2.0  # 버스트 발생 확률 배수


# 수신 모델 프리셋 정의
# 다양한 통신 환경을 사전 정의된 값으로 빠르게 설정할 수 있습니다.
RECEPTION_PRESETS = {
    # 실제 환경 모사 (기본값)
    "realistic": {
        "ais": {"d0": 40, "d1": 50, "p0": 0.01, "p1": 0.95},
        "radar_detect": {"d0": 6, "d1": 24, "p0": 0.02, "p1": 0.80},
        "camera": {"d0": 1, "d1": 5, "p0": 0.01, "p1": 0.90}
    },
    # 안정적인 환경 (테스트용)
    "stable": {
        "ais": {"d0": 40, "d1": 80, "p0": 0.01, "p1": 0.30},
        "radar_detect": {"d0": 6, "d1": 48, "p0": 0.02, "p1": 0.30},
        "camera": {"d0": 2, "d1": 8, "p0": 0.01, "p1": 0.30}
    },
    # 열악한 환경 (스트레스 테스트)
    "harsh": {
        "ais": {"d0": 20, "d1": 30, "p0": 0.05, "p1": 0.99},
        "radar_detect": {"d0": 4, "d1": 12, "p0": 0.10, "p1": 0.95},
        "camera": {"d0": 0.5, "d1": 2, "p0": 0.10, "p1": 0.99}
    }
}


@dataclass
class ProjectSettings:
    """
    프로젝트 전체 설정을 담는 데이터 클래스입니다.

    UI 표시 설정, 시뮬레이션 설정, 신호 생성 설정 등
    프로젝트 전반에 걸친 설정값들을 관리합니다.

    UI 표시 설정:
        select_color: 선택된 객체 색상
        own_color: 자선 경로 색상
        target_color: 타선 경로 색상
        path_thickness: 경로 선 두께
        highlight_path_color: 강조 경로 색상
        theme_mode: UI 테마 모드 (System/Light/Dark)

    시뮬레이션 설정:
        own_ship_idx: 자선으로 지정된 선박 인덱스
        speed_variance: 속도 변동 폭 (노트)

    신호 생성 설정:
        include_gp_signal: GP 신호 포함 여부
        jitter_enabled: 시간 지터 활성화 여부
        dropout_probs: 신호별 유실 확률
        ais_fragment_probs: AIS 프래그먼트 확률

    수신 모델 설정:
        reception_model_enabled: 거리 기반 수신 모델 활성화
        reception_preset: 수신 모델 프리셋
        ais_reception: AIS 수신 설정
        radar_detect: 레이더 탐지 설정
        camera_reception: 카메라 수신 설정
    """
    # UI 표시 색상 설정
    select_color: str = "#0000FF"  # 선택 색상 (파란색)
    own_color: str = "#008000"  # 자선 색상 (녹색)
    target_color: str = "#FF0000"  # 타선 색상 (빨간색)
    path_thickness: int = 3  # 경로 선 두께 (픽셀)
    traveled_path_thickness: int = 4  # 이동 경로 선 두께
    highlight_path_color: str = "#0000FF"  # 강조 경로 색상
    select_thickness_mult: float = 1.5  # 선택 시 두께 배수
    show_speed_notes: bool = True  # 속도 노트 표시 여부

    # 버튼 색상 설정
    btn_speed_color: str = "#FF9800"  # 속도 패널 버튼 색상
    btn_sim_color: str = "#2196F3"  # 시뮬레이션 버튼 색상
    btn_rtg_color: str = "#FFA500"  # RTG 버튼 색상
    random_color: str = "#B8860B"  # 랜덤 타겟 색상
    mask_color: str = "#404040"  # 마스크 색상

    # 테마 설정
    theme_mode: str = "System"  # 테마 모드: "System", "Light", "Dark"

    # 시뮬레이션 설정
    speed_variance: float = 1.0  # 속도 변동 폭 (노트) - 바람/해류 효과
    own_ship_idx: int = 0  # 자선 인덱스
    include_gp_signal: bool = False  # GP 신호 포함 여부
    jitter_enabled: bool = False  # 시간 지터 활성화

    # 신호 유실 확률 (0~1)
    dropout_probs: Dict[str, float] = field(default_factory=lambda: {
        "AIVDM": 0.1,  # AIS 신호 유실 확률
        "RATTM": 0.1,  # 레이더 신호 유실 확률
        "Camera": 0.1  # 카메라 신호 유실 확률
    })

    # AIS 프래그먼트 확률 (1~5개 프래그먼트)
    # 각 인덱스는 해당 개수의 프래그먼트가 생성될 상대적 확률
    ais_fragment_probs: List[float] = field(default_factory=lambda: [8.0, 1.0, 0.5, 0.4, 0.1])

    # 거리 기반 수신 모델 설정
    reception_model_enabled: bool = True  # 수신 모델 활성화
    reception_preset: str = "realistic"  # 프리셋: "realistic", "stable", "harsh", "custom"

    # 각 신호 유형별 수신 설정
    ais_reception: ReceptionModelConfig = field(default_factory=lambda: ReceptionModelConfig(
        d0=40.0, d1=50.0, p0=0.01, p1=0.95
    ))
    radar_detect: ReceptionModelConfig = field(default_factory=lambda: ReceptionModelConfig(
        d0=6.0, d1=24.0, p0=0.02, p1=0.80
    ))
    camera_reception: ReceptionModelConfig = field(default_factory=lambda: ReceptionModelConfig(
        d0=1.0, d1=5.0, p0=0.01, p1=0.90
    ))
