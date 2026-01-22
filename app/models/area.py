"""
영역 모델 모듈

이 모듈은 시뮬레이션에서 사용되는 지리적 영역을 정의합니다.
영역은 이벤트 트리거 조건(영역 진입/이탈)에 사용됩니다.

클래스:
    Area: 지리적 영역을 표현하는 데이터 클래스
"""

from dataclasses import dataclass, field
from typing import List, Tuple


@dataclass
class Area:
    """
    지리적 영역을 표현하는 데이터 클래스입니다.

    영역은 다각형 형태로 정의되며, 시뮬레이션에서 선박이
    특정 영역에 진입하거나 이탈할 때 이벤트를 발생시킬 수 있습니다.

    속성:
        id: 영역 고유 식별자
        name: 영역 이름 (사용자 표시용)
        note: 영역에 대한 메모/설명
        geometry: 다각형 꼭짓점 좌표 목록 [(픽셀x, 픽셀y), ...]
        color: 영역 표시 색상 (16진수 색상 코드)
    """
    id: int  # 영역 고유 ID
    name: str  # 영역 이름
    note: str = ""  # 영역 설명/메모
    geometry: List[Tuple[float, float]] = field(default_factory=list)  # 다각형 꼭짓점 좌표
    color: str = "#808080"  # 영역 표시 색상 (기본값: 회색)
