import math

from PyQt6.QtWidgets import (
    QWidget
)
from PyQt6.QtCore import (
    Qt
)
from PyQt6.QtGui import (
    QColor, QPen, QFont, QPainter
)

from app.core.models.project import current_project
from app.core.geometry import get_optimal_grid_step, normalize_lon

class RulerOverlay(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.setAttribute(Qt.WidgetAttribute.WA_NoSystemBackground)
        self.view = parent
        self.setStyleSheet("background-color: transparent; color: black;")

    def paintEvent(self, event):
        painter = QPainter(self)

        painter.fillRect(0, 0, self.width(), 25, QColor(245, 245, 245, 230))
        painter.fillRect(0, 0, 25, self.height(), QColor(245, 245, 245, 230))
        

        painter.setPen(QPen(Qt.GlobalColor.gray, 1))
        painter.drawLine(0, 25, self.width(), 25)
        painter.drawLine(25, 0, 25, self.height())
        
        mi = current_project.map_info
        if not self.view or not mi: return
        
        scene_rect = self.view.mapToScene(self.view.viewport().rect()).boundingRect()
        
        scale = mi.pixels_per_degree
        current_zoom = self.view.transform().m11()
        total_scale = scale * current_zoom

        if total_scale <= 0: total_scale = 1
        
        deg_step = get_optimal_grid_step(total_scale)
        pixel_step = deg_step * scale


        width = self.width()
        
        start_lon_idx = math.floor((scene_rect.left() / scale) / deg_step)
        end_lon_idx = math.ceil((scene_rect.right() / scale) / deg_step)

        painter.setFont(QFont("Arial", 8))
        
        # Determine decimal places based on deg_step
        if deg_step < 0.1:
            decimals = 2
        elif deg_step < 1:
            decimals = 1
        else:
            decimals = 0

        def fmt(val):
            if decimals == 0:
                return f"{int(round(val))}"
            else:
                return f"{val:.{decimals}f}"

        for i in range(start_lon_idx, end_lon_idx + 1):
            lon_deg = i * deg_step
            sx = lon_deg * scale
            vx = self.view.mapFromScene(sx, scene_rect.top()).x()

            if 25 <= vx <= width:
                painter.setPen(QPen(Qt.GlobalColor.black, 1))
                painter.drawLine(vx, 0, vx, 5)

                norm_lon = normalize_lon(lon_deg)

                txt = f"{fmt(norm_lon)}({fmt(lon_deg)})"
                

                is_major = (abs(lon_deg % 180) < 1e-9)
                
                font = painter.font()
                font.setBold(is_major)
                painter.setFont(font)
                
                painter.drawText(vx + 4, 15, txt)


        height = self.height()
        start_lat_idx = math.floor((-scene_rect.bottom() / scale) / deg_step)
        end_lat_idx = math.ceil((-scene_rect.top() / scale) / deg_step)

        for i in range(start_lat_idx, end_lat_idx + 1):
            lat_deg = i * deg_step
            sy = -lat_deg * scale
            vy = self.view.mapFromScene(scene_rect.left(), sy).y()

            if 25 <= vy <= height:
                painter.setPen(QPen(Qt.GlobalColor.black, 1))
                painter.drawLine(0, vy, 5, vy)
                painter.save()
                painter.translate(15, vy + 4)

                txt = fmt(lat_deg)
                
                is_zero = (abs(lat_deg) < 0.001)
                
                font = painter.font()
                font.setBold(is_zero)
                painter.setFont(font)

                painter.drawText(0, 0, txt)
                painter.restore()
