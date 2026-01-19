"""
리플레이 지도 뷰 모듈

이 모듈은 로그 리플레이 윈도우에서 사용되는 지도 뷰를 제공합니다.
선박 아이템 클릭 시 해당 선박의 상세 정보를 표시하는 기능을 추가합니다.

클래스:
    ReplayMapView: 리플레이 전용 지도 뷰
"""

from PyQt6.QtCore import (
    Qt
)
from app.ui.map.map_view import MapView


class ReplayMapView(MapView):
    """
    로그 리플레이 윈도우에서 사용되는 지도 뷰입니다.

    기본 MapView를 상속하며, 선박 아이템 클릭 시
    해당 선박의 상세 정보를 표시합니다.
    컨텍스트 메뉴는 비활성화됩니다.
    """

    def showContextMenu(self, pos):
        """
        컨텍스트 메뉴를 표시합니다 (비활성화됨).

        리플레이 뷰에서는 컨텍스트 메뉴를 사용하지 않습니다.

        매개변수:
            pos: 마우스 위치
        """
        pass

    def mousePressEvent(self, event):
        """
        마우스 클릭 이벤트를 처리합니다.

        좌클릭 시 선박 아이템을 감지하고 상세 정보를 표시합니다.
        선박이 아닌 영역 클릭 시 패닝을 시작합니다.

        매개변수:
            event: 마우스 이벤트 객체
        """
        if event.button() == Qt.MouseButton.LeftButton:
            pos_scene = self.mapToScene(event.pos())
            items = self.scene().items(pos_scene)

            # 선박 아이템 클릭 감지
            if self.main_window:
                for item in items:
                    for sid, ship_item in self.main_window.ship_items.items():
                        if item == ship_item:
                            self.main_window.show_ship_details(sid)
                            return

            # 선박이 아닌 영역 클릭 시 패닝 시작
            self.start_pan(event)
        else:
            super().mousePressEvent(event)
