import math

from PyQt6.QtWidgets import (
    QMenu, QGraphicsView, QGraphicsPathItem, QGraphicsPolygonItem, QGraphicsEllipseItem, 
    QGraphicsTextItem, QGraphicsItem, QDialog, QFormLayout, QLineEdit, QSpinBox, 
    QPushButton, QColorDialog, QDialogButtonBox, QDoubleSpinBox, QVBoxLayout, QLabel, 
    QMessageBox
)
from PyQt6.QtCore import (
    Qt, pyqtSignal, QPointF, QRectF
)
from PyQt6.QtGui import (
    QColor, QPen, QPainter, QPainterPath, QPolygonF, QBrush, QFont
)
from app.core.models.project import current_project
from app.core.geometry import get_optimal_grid_step, normalize_lon

from app.ui.map.ruler_overlay import RulerOverlay 

class MapView(QGraphicsView):
    coord_changed = pyqtSignal(float, float)
    view_changed = pyqtSignal()
    
    def __init__(self, scene, main_window):
        super().__init__(scene)
        self.main_window = main_window
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setMouseTracking(True)
        self.ruler_overlay = RulerOverlay(self)
        
        self.mode = "SELECT" 
        self.drawing_poly = []
        self.temp_item = None
        self.move_target_idx = None 
        
        self.is_panning = False
        self.last_pan_pos = None

        huge_val = 200000.0 * 2000.0 
        self.setSceneRect(-huge_val, -huge_val, huge_val * 2, huge_val * 2)

        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

    def resizeEvent(self, event):
        self.ruler_overlay.resize(event.size())
        super().resizeEvent(event)
    
    def get_hit_tolerance(self):
        screen_tolerance = 10.0
        current_scale = self.transform().m11()
        if current_scale == 0: return 10.0
        return screen_tolerance / current_scale

    def drawBackground(self, painter, rect):


        painter.fillRect(rect, Qt.GlobalColor.white)
        
        mi = current_project.map_info
        if not mi: return

        scale = mi.pixels_per_degree
        if scale <= 0: return

        current_zoom = self.transform().m11()
        total_scale = scale * current_zoom
        
        deg_step = get_optimal_grid_step(total_scale)
        pixel_step = deg_step * scale

        left = rect.left()
        right = rect.right()
        top = rect.top()
        bottom = rect.bottom()
        

        start_idx = math.floor(left / pixel_step)
        end_idx = math.ceil(right / pixel_step)
        
        pen_default = QPen(QColor(220, 220, 220), 0)
        

        pen_greenwich = QPen(QColor("blue"), 2) 
        pen_greenwich.setStyle(Qt.PenStyle.DotLine)
        pen_greenwich.setCosmetic(True)


        pen_dateline = QPen(QColor("red"), 2)
        pen_dateline.setStyle(Qt.PenStyle.DotLine)
        pen_dateline.setCosmetic(True)

        for i in range(start_idx, end_idx + 1):
            x = i * pixel_step
            lon_deg = i * deg_step
            

            k = round(lon_deg / 180.0)
            is_multiple_180 = (abs(lon_deg - k * 180.0) < 1e-5)
            
            if is_multiple_180:
                if k % 2 == 0:
                    painter.setPen(pen_greenwich) 
                else:
                    painter.setPen(pen_dateline)  
            else:
                painter.setPen(pen_default)
                
            painter.drawLine(QPointF(x, top), QPointF(x, bottom))


        start_idy = math.floor(top / pixel_step)
        end_idy = math.ceil(bottom / pixel_step)
        
        pen_equator = QPen(QColor(100, 100, 100), 2)
        pen_equator.setCosmetic(True)

        for i in range(start_idy, end_idy + 1):
            y = i * pixel_step
            lat_deg = - (y / scale)

            if abs(lat_deg) < 1e-5:
                painter.setPen(pen_equator)
            else:
                painter.setPen(pen_default)

            painter.drawLine(QPointF(left, y), QPointF(right, y))


        y_plus_90 = -90.0 * scale
        y_minus_90 = 90.0 * scale
        
        mask_color = QColor(current_project.settings.mask_color)
        
        if top < y_plus_90:
            fill_rect = QRectF(left, top, right - left, y_plus_90 - top)
            painter.fillRect(fill_rect, mask_color)
            
        if bottom > y_minus_90:
            fill_rect = QRectF(left, y_minus_90, right - left, bottom - y_minus_90)
            painter.fillRect(fill_rect, mask_color)

    def wheelEvent(self, event):
        factor = 1.1 if event.angleDelta().y() > 0 else 0.9
        
        if factor < 1.0:
            view_h = self.viewport().height()
            
            mi = current_project.map_info
            scene_pixels_per_degree = mi.pixels_per_degree if mi.pixels_per_degree > 0 else 1.0
            
            world_scene_height = 180.0 * scene_pixels_per_degree
            
            fit_scale = view_h / world_scene_height
            
            min_scale = fit_scale * 0.9 
            
            current_scale = self.transform().m11()
            new_scale = current_scale * factor
            
            if new_scale < min_scale:
                factor = min_scale / current_scale
        
        self.scale(factor, factor)
        self.ruler_overlay.update()
        self.view_changed.emit()

    def mousePressEvent(self, event):
        is_edit_mode = (self.mode in ["ADD_POINT", "ADD_AREA", "MOVE_POINT", "DELETE_POINT"])
        
        if event.button() == Qt.MouseButton.MiddleButton:
            self.start_pan(event)
            return
        if event.button() == Qt.MouseButton.LeftButton and not is_edit_mode:
            self.start_pan(event)
            return

        pt = self.mapToScene(event.pos())
        tolerance = self.get_hit_tolerance()
        
        if self.mode == "ADD_POINT":
            if event.button() == Qt.MouseButton.LeftButton:
                self.main_window.add_point_click(pt.x(), pt.y())
        elif self.mode == "ADD_AREA":
            if event.button() == Qt.MouseButton.LeftButton:
                self.drawing_poly = [pt]
                self.temp_item = QGraphicsPathItem()
                pen = QPen(Qt.GlobalColor.black, 2)
                pen.setCosmetic(True)
                self.temp_item.setPen(pen)
                self.scene().addItem(self.temp_item)
        elif self.mode == "MOVE_POINT":
             if event.button() == Qt.MouseButton.LeftButton:
                 for ship in current_project.ships:
                    for i, (px, py) in enumerate(ship.raw_points):
                          if abs(px - pt.x()) < tolerance and abs(py - pt.y()) < tolerance:
                              self.move_target_idx = (ship.idx, i)
                              return 
        elif self.mode == "DELETE_POINT":
            if event.button() == Qt.MouseButton.LeftButton:
                for ship in current_project.ships:
                      for i, (px, py) in enumerate(ship.raw_points):
                          if abs(px - pt.x()) < tolerance and abs(py - pt.y()) < tolerance:
                              self.main_window.delete_points(ship.idx, i)
                              return
        elif self.mode == "SELECT":
             super().mousePressEvent(event)
        
        if event.button() == Qt.MouseButton.RightButton:
            self.showContextMenu(event.pos())

    def start_pan(self, event):
        self.is_panning = True
        self.last_pan_pos = event.pos()
        self.setCursor(Qt.CursorShape.ClosedHandCursor)

    def mouseMoveEvent(self, event):
        if self.is_panning and self.last_pan_pos:
            delta = event.pos() - self.last_pan_pos
            self.last_pan_pos = event.pos()
            
            hs = self.horizontalScrollBar()
            vs = self.verticalScrollBar()
            hs.setValue(hs.value() - delta.x())
            vs.setValue(vs.value() - delta.y())
            
            self.view_changed.emit()
            return

        pt = self.mapToScene(event.pos())
        self.coord_changed.emit(pt.x(), pt.y())
        
        if self.mode == "ADD_AREA" and self.drawing_poly:
            self.drawing_poly.append(pt)
            path = self.temp_item.path()
            if path.elementCount() == 0: path.moveTo(self.drawing_poly[0])
            else: path.lineTo(pt)
            self.temp_item.setPath(path)
        elif self.mode == "MOVE_POINT" and self.move_target_idx:
            s_idx, p_idx = self.move_target_idx
            ship = current_project.get_ship_by_idx(s_idx)
            if ship:
                ship.raw_points[p_idx] = (pt.x(), pt.y())
                self.main_window.handle_path_change(ship)

        super().mouseMoveEvent(event)
        self.ruler_overlay.update()

    def mouseReleaseEvent(self, event):
        if self.is_panning:
            self.is_panning = False
            self.update_cursor()
            return

        if self.mode == "ADD_AREA" and self.drawing_poly:
            pts = [(p.x(), p.y()) for p in self.drawing_poly]
            self.scene().removeItem(self.temp_item)
            self.temp_item = None
            self.drawing_poly = []
            self.main_window.finish_add_area(pts)
        elif self.mode == "MOVE_POINT":
            self.move_target_idx = None
        
        super().mouseReleaseEvent(event)

    def update_cursor(self):
        if self.mode == "ADD_POINT": self.setCursor(Qt.CursorShape.CrossCursor)
        elif self.mode == "MOVE_POINT": self.setCursor(Qt.CursorShape.OpenHandCursor)
        else: self.setCursor(Qt.CursorShape.ArrowCursor)

    def showContextMenu(self, pos):
        pt = self.mapToScene(pos)
        tolerance = self.get_hit_tolerance()
        target_ship = None
        target_pt_idx = -1
        
        for ship in current_project.ships:
            for i, (px, py) in enumerate(ship.raw_points):
                if abs(px - pt.x()) < tolerance and abs(py - pt.y()) < tolerance:
                    target_ship = ship
                    target_pt_idx = i
                    break
            if target_ship: break
            
        if target_ship:
            menu = QMenu(self)
            a_add = menu.addAction("Add Point Here")
            a_move = menu.addAction("Move This Point")
            a_del = menu.addAction("Delete This Point")
            menu.addSeparator()
            a_trans = menu.addAction("Translate Path")
            
            action = menu.exec(self.mapToGlobal(pos))
            if action == a_add:
                self.main_window.obj_combo.setCurrentIndex(self.main_window.obj_combo.findData(target_ship.idx))
                self.main_window.add_point_click(pt.x(), pt.y())
                self.main_window.set_map_mode("ADD_POINT")
            elif action == a_move:
                self.main_window.obj_combo.setCurrentIndex(self.main_window.obj_combo.findData(target_ship.idx))
                self.main_window.set_map_mode("MOVE_POINT")
            elif action == a_del:
                self.main_window.delete_points(target_ship.idx, target_pt_idx)
            elif action == a_trans:
                self.main_window.translate_path_dialog(target_ship.idx)
        else:
            menu = QMenu(self)
            a_trans = menu.addAction("Translate Path (Selected Ship)")
            sel_txt = self.main_window.obj_combo.currentText()
            if "[Ship]" in sel_txt:
                sid = self.main_window.obj_combo.currentData()
                a_trans.triggered.connect(lambda: self.main_window.translate_path_dialog(sid))
            menu.exec(self.mapToGlobal(pos))