"""
지도 뷰 모듈

이 모듈은 해상 시뮬레이션을 위한 기본 지도 뷰 위젯을 제공합니다.
선박 경로 편집, 영역 추가, 포인트 이동/삭제 등의 인터랙티브 기능을 지원합니다.

클래스:
    MapView: 기본 지도 뷰 위젯

주요 기능:
    - 위도/경도 그리드 그리기
    - 마우스 휠 줌 인/아웃
    - 마우스 드래그 패닝
    - 경로 포인트 추가/이동/삭제
    - 영역 다각형 그리기
    - 컨텍스트 메뉴
"""

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
    """
    해상 시뮬레이션을 위한 기본 지도 뷰입니다.

    위도/경도 그리드를 배경에 그리고, 마우스 이벤트를 처리하여
    선박 경로 편집, 영역 추가 등의 기능을 제공합니다.

    시그널:
        coord_changed(float, float): 마우스 위치의 좌표가 변경될 때 발생
        view_changed(): 뷰가 변경(줌, 패닝)될 때 발생

    속성:
        main_window: 부모 메인 윈도우 참조
        ruler_overlay: 눈금자 오버레이 위젯
        mode: 현재 편집 모드 ("SELECT", "ADD_POINT", "MOVE_POINT", "DELETE_POINT", "ADD_AREA")
        drawing_poly: 영역 추가 시 임시 폴리곤 포인트 목록
        temp_item: 영역 추가 시 임시 그래픽스 아이템
        move_target_idx: 포인트 이동 시 대상 (선박 인덱스, 포인트 인덱스) 튜플
        is_panning: 패닝 중 여부
        last_pan_pos: 마지막 패닝 위치
    """

    coord_changed = pyqtSignal(float, float)
    view_changed = pyqtSignal()

    def __init__(self, scene, main_window):
        """
        MapView를 초기화합니다.

        매개변수:
            scene: 그래픽스 씬
            main_window: 부모 메인 윈도우
        """
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

        # 매우 큰 씬 영역 설정 (전 세계 + 여유)
        huge_val = 200000.0 * 2000.0
        self.setSceneRect(-huge_val, -huge_val, huge_val * 2, huge_val * 2)

        # 스크롤바 비활성화
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

    def resizeEvent(self, event):
        """
        리사이즈 이벤트를 처리합니다.

        눈금자 오버레이 크기를 뷰 크기에 맞게 조정합니다.

        매개변수:
            event: 리사이즈 이벤트 객체
        """
        self.ruler_overlay.resize(event.size())
        super().resizeEvent(event)

    def get_hit_tolerance(self):
        """
        현재 줌 레벨에 맞는 히트 테스트 허용 오차를 반환합니다.

        화면상 10픽셀에 해당하는 씬 좌표 거리를 계산합니다.

        반환값:
            씬 좌표 단위의 허용 오차
        """
        screen_tolerance = 10.0
        current_scale = self.transform().m11()
        if current_scale == 0: return 10.0
        return screen_tolerance / current_scale

    def drawBackground(self, painter, rect):
        """
        지도 배경을 그립니다.

        위도/경도 그리드, 본초 자오선, 날짜 변경선, 적도 등을 그립니다.
        위도 ±90° 외부 영역은 마스크 색상으로 채웁니다.

        매개변수:
            painter: QPainter 객체
            rect: 그리기 영역
        """
        # 흰색 배경
        painter.fillRect(rect, Qt.GlobalColor.white)

        mi = current_project.map_info
        if not mi: return

        scale = mi.pixels_per_degree
        if scale <= 0: return

        # 현재 줌 레벨에 따른 최적 그리드 간격 계산
        current_zoom = self.transform().m11()
        total_scale = scale * current_zoom

        deg_step = get_optimal_grid_step(total_scale)
        pixel_step = deg_step * scale

        left = rect.left()
        right = rect.right()
        top = rect.top()
        bottom = rect.bottom()

        # 경도선 그리기
        start_idx = math.floor(left / pixel_step)
        end_idx = math.ceil(right / pixel_step)

        # 기본 그리드 펜
        pen_default = QPen(QColor(220, 220, 220), 0)

        # 본초 자오선 펜 (파란색 점선)
        pen_greenwich = QPen(QColor("blue"), 2)
        pen_greenwich.setStyle(Qt.PenStyle.DotLine)
        pen_greenwich.setCosmetic(True)

        # 날짜 변경선 펜 (빨간색 점선)
        pen_dateline = QPen(QColor("red"), 2)
        pen_dateline.setStyle(Qt.PenStyle.DotLine)
        pen_dateline.setCosmetic(True)

        for i in range(start_idx, end_idx + 1):
            x = i * pixel_step
            lon_deg = i * deg_step

            # 180°의 배수인지 확인
            k = round(lon_deg / 180.0)
            is_multiple_180 = (abs(lon_deg - k * 180.0) < 1e-5)

            if is_multiple_180:
                if k % 2 == 0:
                    painter.setPen(pen_greenwich)  # 본초 자오선 (0°, 360°, ...)
                else:
                    painter.setPen(pen_dateline)  # 날짜 변경선 (180°, -180°, ...)
            else:
                painter.setPen(pen_default)

            painter.drawLine(QPointF(x, top), QPointF(x, bottom))

        # 위도선 그리기
        start_idy = math.floor(top / pixel_step)
        end_idy = math.ceil(bottom / pixel_step)

        # 적도 펜
        pen_equator = QPen(QColor(100, 100, 100), 2)
        pen_equator.setCosmetic(True)

        for i in range(start_idy, end_idy + 1):
            y = i * pixel_step
            lat_deg = - (y / scale)

            if abs(lat_deg) < 1e-5:
                painter.setPen(pen_equator)  # 적도
            else:
                painter.setPen(pen_default)

            painter.drawLine(QPointF(left, y), QPointF(right, y))

        # 위도 ±90° 외부 영역 마스크
        y_plus_90 = -90.0 * scale
        y_minus_90 = 90.0 * scale

        mask_color = QColor(current_project.settings.mask_color)

        # 북극 외부 (위도 > 90°)
        if top < y_plus_90:
            fill_rect = QRectF(left, top, right - left, y_plus_90 - top)
            painter.fillRect(fill_rect, mask_color)

        # 남극 외부 (위도 < -90°)
        if bottom > y_minus_90:
            fill_rect = QRectF(left, y_minus_90, right - left, bottom - y_minus_90)
            painter.fillRect(fill_rect, mask_color)

    def wheelEvent(self, event):
        """
        마우스 휠 이벤트를 처리합니다 (줌 인/아웃).

        줌 아웃 시 전 세계가 화면에 맞게 보이도록 최소 스케일을 제한합니다.

        매개변수:
            event: 휠 이벤트 객체
        """
        factor = 1.1 if event.angleDelta().y() > 0 else 0.9

        # 줌 아웃 시 최소 스케일 제한
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
        """
        마우스 클릭 이벤트를 처리합니다.

        편집 모드에 따라 포인트 추가, 이동, 삭제 또는 패닝을 시작합니다.

        매개변수:
            event: 마우스 이벤트 객체
        """
        is_edit_mode = (self.mode in ["ADD_POINT", "ADD_AREA", "MOVE_POINT", "DELETE_POINT"])

        # 가운데 버튼 또는 편집 모드가 아닐 때 좌클릭 = 패닝
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
                # 영역 그리기 시작
                self.drawing_poly = [pt]
                self.temp_item = QGraphicsPathItem()
                pen = QPen(Qt.GlobalColor.black, 2)
                pen.setCosmetic(True)
                self.temp_item.setPen(pen)
                self.scene().addItem(self.temp_item)
        elif self.mode == "MOVE_POINT":
             if event.button() == Qt.MouseButton.LeftButton:
                 # 클릭 위치에서 가장 가까운 포인트 찾기
                 for ship in current_project.ships:
                    for i, (px, py) in enumerate(ship.raw_points):
                          if abs(px - pt.x()) < tolerance and abs(py - pt.y()) < tolerance:
                              self.move_target_idx = (ship.idx, i)
                              return
        elif self.mode == "DELETE_POINT":
            if event.button() == Qt.MouseButton.LeftButton:
                # 클릭 위치의 포인트 삭제
                for ship in current_project.ships:
                      for i, (px, py) in enumerate(ship.raw_points):
                          if abs(px - pt.x()) < tolerance and abs(py - pt.y()) < tolerance:
                              self.main_window.delete_points(ship.idx, i)
                              return
        elif self.mode == "SELECT":
             super().mousePressEvent(event)

        # 우클릭 시 컨텍스트 메뉴
        if event.button() == Qt.MouseButton.RightButton:
            self.showContextMenu(event.pos())

    def start_pan(self, event):
        """
        패닝을 시작합니다.

        매개변수:
            event: 마우스 이벤트 객체
        """
        self.is_panning = True
        self.last_pan_pos = event.pos()
        self.setCursor(Qt.CursorShape.ClosedHandCursor)

    def mouseMoveEvent(self, event):
        """
        마우스 이동 이벤트를 처리합니다.

        패닝 중이면 뷰를 이동하고, 영역 그리기 중이면 경로를 업데이트하고,
        포인트 이동 중이면 포인트 위치를 업데이트합니다.

        매개변수:
            event: 마우스 이벤트 객체
        """
        # 패닝 처리
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

        # 영역 그리기 중
        if self.mode == "ADD_AREA" and self.drawing_poly:
            self.drawing_poly.append(pt)
            path = self.temp_item.path()
            if path.elementCount() == 0: path.moveTo(self.drawing_poly[0])
            else: path.lineTo(pt)
            self.temp_item.setPath(path)
        # 포인트 이동 중
        elif self.mode == "MOVE_POINT" and self.move_target_idx:
            s_idx, p_idx = self.move_target_idx
            ship = current_project.get_ship_by_idx(s_idx)
            if ship:
                ship.raw_points[p_idx] = (pt.x(), pt.y())
                self.main_window.handle_path_change(ship)

        super().mouseMoveEvent(event)
        self.ruler_overlay.update()

    def mouseReleaseEvent(self, event):
        """
        마우스 릴리즈 이벤트를 처리합니다.

        패닝을 종료하거나, 영역 그리기를 완료하거나, 포인트 이동을 종료합니다.

        매개변수:
            event: 마우스 이벤트 객체
        """
        # 패닝 종료
        if self.is_panning:
            self.is_panning = False
            self.update_cursor()
            return

        # 영역 그리기 완료
        if self.mode == "ADD_AREA" and self.drawing_poly:
            pts = [(p.x(), p.y()) for p in self.drawing_poly]
            self.scene().removeItem(self.temp_item)
            self.temp_item = None
            self.drawing_poly = []
            self.main_window.finish_add_area(pts)
        # 포인트 이동 종료
        elif self.mode == "MOVE_POINT":
            self.move_target_idx = None

        super().mouseReleaseEvent(event)

    def update_cursor(self):
        """현재 모드에 맞게 커서를 업데이트합니다."""
        if self.mode == "ADD_POINT": self.setCursor(Qt.CursorShape.CrossCursor)
        elif self.mode == "MOVE_POINT": self.setCursor(Qt.CursorShape.OpenHandCursor)
        else: self.setCursor(Qt.CursorShape.ArrowCursor)

    def showContextMenu(self, pos):
        """
        컨텍스트 메뉴를 표시합니다.

        클릭 위치에 선박 포인트가 있으면 포인트 관련 메뉴를,
        없으면 일반 메뉴를 표시합니다.

        매개변수:
            pos: 화면 좌표 위치
        """
        pt = self.mapToScene(pos)
        tolerance = self.get_hit_tolerance()
        target_ship = None
        target_pt_idx = -1

        # 클릭 위치에서 포인트 찾기
        for ship in current_project.ships:
            for i, (px, py) in enumerate(ship.raw_points):
                if abs(px - pt.x()) < tolerance and abs(py - pt.y()) < tolerance:
                    target_ship = ship
                    target_pt_idx = i
                    break
            if target_ship: break

        if target_ship:
            # 포인트를 클릭한 경우
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
            # 빈 영역을 클릭한 경우
            menu = QMenu(self)
            a_trans = menu.addAction("Translate Path (Selected Ship)")
            sel_txt = self.main_window.obj_combo.currentText()
            if "[Ship]" in sel_txt:
                sid = self.main_window.obj_combo.currentData()
                a_trans.triggered.connect(lambda: self.main_window.translate_path_dialog(sid))
            menu.exec(self.mapToGlobal(pos))
