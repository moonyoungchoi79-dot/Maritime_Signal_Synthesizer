"""
선박 모델 모듈

이 모듈은 시뮬레이션에서 사용되는 선박 데이터 구조를 정의합니다.

클래스:
    ShipData: 선박 정보를 담는 데이터 클래스

선박 클래스 (유형):
    - CONTAINER: 컨테이너선
    - TANKER: 유조선
    - CARGO: 화물선
    - PASSENGER: 여객선
    - FISHING: 어선
    - BUOY: 부표
    - OTHER: 기타

상수:
    SHIP_CLASS_DIMENSIONS: 선박 유형별 기본 치수
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple


# 선박 유형별 기본 치수 정의
# 형식: (길이(m), 폭(m), 흘수(m), 공기흘수(m))
SHIP_CLASS_DIMENSIONS = {
    "CONTAINER": (300.0, 40.0, 15.0, 60.0),  # 컨테이너선: 대형
    "TANKER": (250.0, 45.0, 16.0, 50.0),      # 유조선: 폭이 넓음
    "CARGO": (200.0, 32.0, 12.0, 45.0),       # 화물선: 중형
    "PASSENGER": (180.0, 28.0, 7.0, 55.0),   # 여객선: 얕은 흘수, 높은 상부구조
    "FISHING": (30.0, 8.0, 3.0, 12.0),       # 어선: 소형
    "BUOY": (2.0, 2.0, 1.0, 2.0),            # 부표: 매우 작음
    "OTHER": (50.0, 12.0, 4.0, 20.0),        # 기타: 중소형
}


def get_ship_dimensions(ship_class: str) -> tuple:
    """
    선박 유형에 따른 기본 치수를 반환합니다.

    매개변수:
        ship_class: 선박 유형 문자열 (CONTAINER, TANKER 등)

    반환값:
        (길이, 폭, 흘수, 공기흘수) 튜플 (미터 단위)
    """
    return SHIP_CLASS_DIMENSIONS.get(ship_class, SHIP_CLASS_DIMENSIONS["OTHER"])


@dataclass
class ShipData:
    """
    선박 정보를 담는 데이터 클래스입니다.

    선박의 기본 정보, 경로, 속도 프로필, 신호 설정 등을 포함합니다.

    기본 정보:
        idx: 선박 고유 인덱스 (0~999: 수동 타겟, 1000~: 랜덤 타겟)
        name: 선박 이름
        note: 선박에 대한 메모
        mmsi: Maritime Mobile Service Identity (해상이동업무식별번호)
        is_random_target: 랜덤 생성된 타겟 여부

    변동성 설정:
        variances: 파라미터별 변동 폭
        signals_enabled: 신호 유형별 활성화 여부 (AIVDM, RATTM, Camera)
        signal_intervals: 신호 유형별 발신 주기 (초)

    경로 데이터:
        raw_points: 원본 경로 점 목록 [(픽셀x, 픽셀y), ...]
        raw_speeds: 노드별 속도 설정 {노드인덱스: 속도(노트)}
        raw_notes: 노드별 메모 {노드인덱스: 메모}
        resampled_points: 리샘플링된 경로 점 목록

    속도 프로필:
        node_velocities_kn: 노드별 속도 (노트)
        base_velocities_kn: 기본 속도 프로필 (노트)
        cumulative_time: 누적 시간 (초)
        total_duration_sec: 총 소요 시간 (초)
        speed_keyframes: 속도 키프레임 {인덱스: 속도}
        speed_notes: 속도 노트 {인덱스: 메모}

    시계열 데이터:
        time_series: 시간별 상태 데이터 목록
        packed_data: NumPy 배열로 압축된 데이터
        is_generated: 시계열 데이터 생성 완료 여부

    선박 치수:
        ship_class: 선박 유형 (CONTAINER, FISHING 등)
        length_m: 선박 길이 (미터)
        beam_m: 선박 폭 (미터)
        draft_m: 흘수 (미터) - 수면 아래 깊이
        air_draft_m: 공기흘수 (미터) - 수면 위 최고점 높이
        height_m: 가시 높이 (미터) - 카메라 탐지용
    """
    # 기본 정보
    idx: int  # 선박 고유 인덱스
    name: str  # 선박 이름
    note: str = ""  # 메모
    mmsi: int = 0  # MMSI (해상이동업무식별번호)
    is_random_target: bool = False  # 랜덤 타겟 여부

    # 변동성 및 신호 설정
    variances: Dict[str, float] = field(default_factory=dict)  # 파라미터별 변동 폭
    signals_enabled: Dict[str, bool] = field(default_factory=dict)  # 신호 활성화 여부
    signal_intervals: Dict[str, float] = field(default_factory=dict)  # 신호 발신 주기

    # 원본 경로 데이터
    raw_points: List[Tuple[float, float]] = field(default_factory=list)  # 원본 경로 점
    raw_speeds: Dict[int, float] = field(default_factory=dict)  # 노드별 속도
    raw_notes: Dict[int, str] = field(default_factory=dict)  # 노드별 메모

    # 처리된 경로 데이터
    resampled_points: List[Tuple[float, float]] = field(default_factory=list)  # 리샘플링된 경로

    # 속도 프로필
    node_velocities_kn: List[float] = field(default_factory=list)  # 노드별 속도
    base_velocities_kn: List[float] = field(default_factory=list)  # 기본 속도 프로필
    cumulative_time: List[float] = field(default_factory=list)  # 누적 시간
    total_duration_sec: float = 0.0  # 총 소요 시간
    speed_keyframes: Dict[int, float] = field(default_factory=dict)  # 속도 키프레임
    speed_notes: Dict[int, str] = field(default_factory=dict)  # 속도 노트

    # 시계열 데이터
    time_series: List[Dict] = field(default_factory=list)  # 시간별 상태 데이터
    packed_data: Optional[np.ndarray] = None  # 압축된 NumPy 배열
    is_generated: bool = False  # 생성 완료 여부

    # 선박 유형 및 치수
    # 기본값: 컨테이너선 규격 (300m/40m/15m/60m)
    ship_class: str = "CONTAINER"  # 선박 유형
    length_m: float = 300.0  # 선박 길이 (미터)
    beam_m: float = 40.0  # 선박 폭 (미터)
    draft_m: float = 15.0  # 흘수 (미터)
    air_draft_m: float = 60.0  # 공기흘수 (미터)
    height_m: float = 30.0  # 가시 높이 (미터) - 카메라 탐지용

    def get_display_segments(self) -> List[List[Tuple[float, float]]]:
        """
        지도 표시용으로 경로를 세그먼트로 분할하여 반환합니다.

        날짜변경선(경도 180도)을 통과할 때 경로가 지도를 가로질러
        그려지는 현상을 방지하기 위해 경로를 분할합니다.

        반환값:
            분할된 경로 세그먼트 목록
            각 세그먼트는 연속된 점들의 목록
        """
        points = self.resampled_points if self.resampled_points else self.raw_points
        if not points:
            return []

        segments = []
        current_segment = [points[0]]

        for i in range(1, len(points)):
            prev_lat, prev_lon = points[i-1]
            curr_lat, curr_lon = points[i]

            # 경도 차이가 180도 이상이면 날짜변경선 통과로 판단
            if abs(curr_lon - prev_lon) > 180.0:
                # 현재 세그먼트 저장 후 새 세그먼트 시작
                segments.append(current_segment)
                current_segment = []

            current_segment.append((curr_lat, curr_lon))

        # 마지막 세그먼트 추가
        if current_segment:
            segments.append(current_segment)

        return segments
