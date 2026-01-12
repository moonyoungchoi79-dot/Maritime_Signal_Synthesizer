from PyQt6.QtCore import (
    Qt
)
from app.ui.map.map_view import MapView

class ReplayMapView(MapView):
    def showContextMenu(self, pos):
        pass 
    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            pos_scene = self.mapToScene(event.pos())
            items = self.scene().items(pos_scene)
            if self.main_window:
                for item in items:
                    for sid, ship_item in self.main_window.ship_items.items():
                        if item == ship_item:
                            self.main_window.show_ship_details(sid)
                            return
            self.start_pan(event)
        else:
            super().mousePressEvent(event)