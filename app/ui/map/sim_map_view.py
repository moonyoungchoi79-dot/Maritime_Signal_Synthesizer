from PyQt6.QtCore import (
    Qt, QRect
)
from app.ui.map.map_view import MapView

class SimMapView(MapView):
    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            click_pos = event.pos()
            tolerance = 10
            rect = QRect(click_pos.x() - tolerance, click_pos.y() - tolerance, tolerance * 2, tolerance * 2)
            items = self.items(rect)
            # Check if any clicked item corresponds to a ship marker
            if self.main_window:
                for item in items:
                    for idx, marker in self.main_window.ship_items.items():
                        if item == marker:
                            self.main_window.show_target_info(idx)
                            return # Consume event (don't pan)
        
        super().mousePressEvent(event)
