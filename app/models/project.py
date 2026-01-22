"""
프로젝트 모델 모듈

이 모듈은 애플리케이션의 핵심 데이터 구조인 프로젝트를 정의합니다.
프로젝트는 시뮬레이션에 필요한 모든 데이터(선박, 영역, 이벤트 등)를 포함합니다.

클래스:
    Project: 시뮬레이션 프로젝트를 표현하는 데이터 클래스

전역 인스턴스:
    current_project: 현재 활성화된 프로젝트 인스턴스
"""

import datetime
from dataclasses import dataclass, field
from typing import List, Optional

from app.models.ship import ShipData
from app.models.area import Area
from app.models.event import EventTrigger
from app.models.settings import ProjectSettings
from app.models.map_info import MapInfo


@dataclass
class Project:
    """
    시뮬레이션 프로젝트를 표현하는 데이터 클래스입니다.

    프로젝트는 시뮬레이션에 필요한 모든 정보를 담고 있으며,
    파일로 저장하고 불러올 수 있습니다.

    속성:
        project_name: 프로젝트 이름
        project_path: 프로젝트 파일 경로
        start_time: 시뮬레이션 시작 시간 (UTC)
        unit_time: 단위 시간 (초)
        seed: 랜덤 시드 (재현성을 위해)
        map_info: 지도 정보
        settings: 프로젝트 설정
        ships: 선박 목록
        areas: 영역 목록
        events: 이벤트 목록
    """

    def __init__(self):
        """프로젝트를 기본값으로 초기화합니다."""
        self.project_name = "Untitled"  # 프로젝트 이름
        self.project_path = ""  # 프로젝트 파일 경로
        self.start_time: datetime.datetime = datetime.datetime.now(datetime.timezone.utc)  # 시작 시간
        self.unit_time: float = 1.0  # 단위 시간 (초)
        self.seed: int = int(datetime.datetime.now().timestamp())  # 랜덤 시드

        # 지도 정보
        self.map_info = MapInfo()

        # 프로젝트 설정
        self.settings = ProjectSettings()

        # 신호별 기본 유실 확률 설정
        defaults = ["AIVDM", "RATTM", "Camera"]
        for d in defaults:
            self.settings.dropout_probs[d] = 0.1

        # 데이터 목록
        self.ships: List[ShipData] = []  # 선박 목록
        self.areas: List[Area] = []  # 영역 목록
        self.events: List[EventTrigger] = []  # 이벤트 목록

    def reset(self):
        """
        프로젝트 데이터를 초기화합니다.

        선박과 영역 목록을 비웁니다.
        설정과 지도 정보는 유지됩니다.
        """
        self.ships = []
        self.areas = []

    def get_ship_by_idx(self, idx) -> Optional[ShipData]:
        """
        인덱스로 선박을 검색합니다.

        매개변수:
            idx: 검색할 선박 인덱스

        반환값:
            해당 인덱스의 ShipData 객체, 없으면 None
        """
        for s in self.ships:
            if s.idx == idx:
                return s
        return None

    def get_area_by_id(self, sid) -> Optional[Area]:
        """
        ID로 영역을 검색합니다.

        매개변수:
            sid: 검색할 영역 ID

        반환값:
            해당 ID의 Area 객체, 없으면 None
        """
        for s in self.areas:
            if s.id == sid:
                return s
        return None


# 현재 활성화된 프로젝트의 전역 인스턴스
# 애플리케이션 전체에서 이 인스턴스를 통해 프로젝트 데이터에 접근합니다.
current_project = Project()
