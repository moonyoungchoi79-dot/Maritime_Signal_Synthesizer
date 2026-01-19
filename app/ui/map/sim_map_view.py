"""
시뮬레이션 지도 뷰 모듈

이 모듈은 시뮬레이션 패널에서 사용되는 지도 뷰를 제공합니다.
선박 마커 클릭 시 해당 선박의 상세 정보를 표시하는 기능을 추가합니다.

클래스:
    SimMapView: 시뮬레이션 전용 지도 뷰
"""

from PyQt6.QtCore import (
    Qt, QRect
)
from app.ui.map.map_view import MapView


class SimMapView(MapView):
    """
    시뮬레이션 패널에서 사용되는 지도 뷰입니다.

    기본 MapView를 상속하며, 선박 마커 클릭 시
    해당 선박의 타겟 정보 다이얼로그를 표시합니다.
    """

    def mousePressEvent(self, event):
        """
        마우스 클릭 이벤트를 처리합니다.

        좌클릭 시 클릭 위치에 선박 마커가 있는지 확인하고,
        있으면 해당 선박의 정보를 표시합니다.
        선박이 아닌 영역 클릭 시 기본 동작(패닝)을 수행합니다.

        매개변수:
            event: 마우스 이벤트 객체
        """
        if event.button() == Qt.MouseButton.LeftButton:
            click_pos = event.pos()
            tolerance = 10

            # 클릭 위치 주변의 아이템 검색
            rect = QRect(click_pos.x() - tolerance, click_pos.y() - tolerance, tolerance * 2, tolerance * 2)
            items = self.items(rect)

            # 클릭된 아이템이 선박 마커인지 확인
            if self.main_window:
                for item in items:
                    for idx, marker in self.main_window.ship_items.items():
                        if item == marker:
                            self.main_window.show_target_info(idx)
                            return  # 이벤트 소비 (패닝 안 함)

        super().mousePressEvent(event)
