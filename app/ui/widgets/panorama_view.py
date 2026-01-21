"""
파노라마 뷰 위젯 모듈

이 모듈은 카메라 탐지 시각화를 위한 파노라마 뷰 위젯을 제공합니다.
EO 카메라 (-90°~+90°)와 IR 카메라 (-55°~+55°) 두 개의 파노라마를 표시합니다.

클래스:
    PanoramaView: 단일 카메라 파노라마 뷰 위젯
    CombinedPanoramaView: EO/IR 통합 파노라마 뷰 위젯 (EO 기준, IR bbox 오버레이)
    DualPanoramaView: EO/IR 듀얼 카메라 파노라마 뷰 위젯 (탭 전환)

사양:
    C-1: 파노라마 뷰 표시
    C-2: 바운딩 박스 그리기
    C-3: 디바운싱을 통한 과도한 리페인트 방지

상수:
    EO_PANO_W_PX: EO 파노라마 이미지 너비 (3840 픽셀)
    EO_PANO_H_PX: EO 파노라마 이미지 높이 (1080 픽셀)
    IR_PANO_W_PX: IR 파노라마 이미지 너비 (1536 픽셀)
    IR_PANO_H_PX: IR 파노라마 이미지 높이 (512 픽셀)
"""

import time
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QTabWidget
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush, QFont, QFontMetrics


# 레거시 파노라마 상수 (하위 호환용)
PANO_W_PX = 1920  # 레거시 파노라마 이미지 너비
PANO_H_PX = 320   # 레거시 파노라마 이미지 높이

# EO 카메라 파노라마 상수
EO_PANO_W_PX = 3840  # EO 파노라마 이미지 너비
EO_PANO_H_PX = 1080  # EO 파노라마 이미지 높이
EO_FOV_DEG = 180.0   # EO 수평 FOV (±90°)

# IR 카메라 파노라마 상수
IR_PANO_W_PX = 1536  # IR 파노라마 이미지 너비
IR_PANO_H_PX = 512   # IR 파노라마 이미지 높이
IR_FOV_DEG = 110.0   # IR 수평 FOV (±55°)


class PanoramaView(QWidget):
    """
    카메라 탐지를 표시하는 파노라마 뷰 위젯입니다.

    -90°에서 +90° 시야각(FOV) 내에서 탐지된 선박을 바운딩 박스로 표시합니다.
    디바운싱을 적용하여 과도한 화면 갱신을 방지합니다.

    속성:
        detections: 현재 탐지 데이터 목록
        _last_update: 마지막 업데이트 시간 (밀리초)
        _debounce_ms: 디바운스 간격 (기본값: 50ms)

    색상 설정:
        _bg_color: 배경색 (어두운 파란색/회색)
        _grid_color: 그리드 색상
        _bbox_color: 바운딩 박스 테두리 색상 (녹색)
        _bbox_fill: 바운딩 박스 채우기 색상 (반투명 녹색)
        _text_color: 텍스트 색상 (흰색)
        _scale_color: 눈금 색상
    """

    def __init__(self, parent=None, camera_type="legacy"):
        """
        PanoramaView를 초기화합니다.

        매개변수:
            parent: 부모 위젯
            camera_type: 카메라 유형 ("legacy", "EO", "IR")
        """
        super().__init__(parent)
        self.detections = []  # 탐지 데이터 목록
        self.camera_type = camera_type
        self.setMinimumHeight(80)  # 최소 높이
        self.setMinimumWidth(300)   # 최소 너비
        self._last_update = 0       # 마지막 업데이트 시간
        self._debounce_ms = 50      # 디바운스 간격 (사양 C-3: 30-100ms)

        # 카메라 유형에 따른 파노라마 크기 및 FOV 설정
        if camera_type == "EO":
            self._pano_w = EO_PANO_W_PX
            self._pano_h = EO_PANO_H_PX
            self._fov_deg = EO_FOV_DEG
            self._half_fov = 90.0
        elif camera_type == "IR":
            self._pano_w = IR_PANO_W_PX
            self._pano_h = IR_PANO_H_PX
            self._fov_deg = IR_FOV_DEG
            self._half_fov = 55.0
        else:  # legacy
            self._pano_w = PANO_W_PX
            self._pano_h = PANO_H_PX
            self._fov_deg = 180.0
            self._half_fov = 90.0

        # 색상 설정
        self._bg_color = QColor(30, 30, 40)        # 배경색
        self._grid_color = QColor(60, 60, 80)      # 그리드 색상
        self._text_color = QColor(255, 255, 255)   # 텍스트 (흰색)
        self._scale_color = QColor(150, 150, 170)  # 눈금 색상

        # 카메라 유형별 색상 및 선 스타일 설정
        if camera_type == "EO":
            self._bbox_color = QColor(0, 255, 0)       # 녹색
            self._bbox_fill = QColor(0, 255, 0, 40)
            self._bbox_pen_style = Qt.PenStyle.SolidLine  # 실선
        elif camera_type == "IR":
            self._bbox_color = QColor(255, 100, 100)   # 빨간색
            self._bbox_fill = QColor(255, 100, 100, 40)
            self._bbox_pen_style = Qt.PenStyle.DashLine  # 점선
        else:
            self._bbox_color = QColor(0, 255, 0)       # 녹색
            self._bbox_fill = QColor(0, 255, 0, 40)
            self._bbox_pen_style = Qt.PenStyle.SolidLine  # 실선

    def set_detections(self, detections):
        """
        탐지 데이터를 업데이트합니다 (디바운싱 적용, 사양 C-3).

        마지막 업데이트 이후 충분한 시간이 경과한 경우에만 업데이트합니다.

        매개변수:
            detections: 탐지 데이터 딕셔너리 목록
        """
        now = time.time() * 1000
        if now - self._last_update < self._debounce_ms:
            return  # 디바운스 간격 내 업데이트 무시
        self._last_update = now
        self.detections = detections
        self.update()

    def clear_detections(self):
        """모든 탐지 데이터를 뷰에서 지웁니다."""
        self.detections = []
        self.update()

    def paintEvent(self, event):
        """
        파노라마 뷰를 그리는 페인트 이벤트 핸들러입니다.

        배경, 눈금, 수평선, 탐지 바운딩 박스를 순서대로 그립니다.

        매개변수:
            event: 페인트 이벤트 객체
        """
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w = self.width()
        h = self.height()

        # 배경 그리기
        painter.fillRect(0, 0, w, h, self._bg_color)

        # 스케일 팩터 계산
        scale_x = w / self._pano_w
        scale_y = h / self._pano_h

        # 상단에 방위각 눈금 그리기
        self._draw_scale(painter, w, h)

        # cy 비율 위치에 수평선 그리기
        horizon_y = int(0.50 * h)  # 중앙에 수평선
        painter.setPen(QPen(self._grid_color, 1, Qt.PenStyle.DashLine))
        painter.drawLine(0, horizon_y, w, horizon_y)

        # 각 탐지에 대해 바운딩 박스 그리기
        for det in self.detections:
            self._draw_detection(painter, det, scale_x, scale_y, w, h)

        painter.end()

    def _draw_scale(self, painter, w, h):
        """
        뷰 상단에 방위각 눈금을 그립니다.

        카메라 유형에 따라 다른 범위의 눈금을 표시합니다.

        매개변수:
            painter: QPainter 객체
            w: 뷰 너비
            h: 뷰 높이
        """
        painter.setPen(QPen(self._scale_color, 1))
        font = QFont()
        font.setPointSize(8)
        painter.setFont(font)

        # 카메라 유형에 따른 눈금 범위
        if self.camera_type == "IR":
            bearings = [-55, -40, -20, 0, 20, 40, 55]
        else:
            bearings = [-90, -60, -30, 0, 30, 60, 90]

        for bearing in bearings:
            # 방위각을 x 좌표로 변환
            x_norm = (bearing + self._half_fov) / self._fov_deg
            x = int(x_norm * w)

            # 눈금선 그리기
            painter.drawLine(x, 0, x, 10)

            # 라벨 그리기
            label = f"{bearing}°"
            fm = QFontMetrics(font)
            label_w = fm.horizontalAdvance(label)
            label_x = x - label_w // 2
            # 가시 영역 내로 클램핑
            label_x = max(2, min(w - label_w - 2, label_x))
            painter.drawText(label_x, 22, label)

        # 중심선 (0°) 그리기
        center_x = w // 2
        painter.setPen(QPen(self._grid_color, 1, Qt.PenStyle.DashLine))
        painter.drawLine(center_x, 25, center_x, h)

    def _draw_detection(self, painter, det, scale_x, scale_y, view_w, view_h):
        """
        단일 탐지 바운딩 박스와 라벨을 그립니다.

        매개변수:
            painter: QPainter 객체
            det: 탐지 데이터 딕셔너리
            scale_x: X축 스케일 팩터
            scale_y: Y축 스케일 팩터
            view_w: 뷰 너비
            view_h: 뷰 높이
        """
        # 탐지 좌표를 뷰 좌표로 스케일링
        cx = det['cx_px'] * scale_x
        cy = det['cy_px'] * scale_y
        w = det['w_px'] * scale_x
        h = det['h_px'] * scale_y

        x1 = cx - w / 2
        y1 = cy - h / 2

        # 채워진 사각형 그리기 (EO: 실선, IR: 점선)
        pen = QPen(self._bbox_color, 2, self._bbox_pen_style)
        painter.setPen(pen)
        painter.setBrush(QBrush(self._bbox_fill))
        painter.drawRect(int(x1), int(y1), int(w), int(h))

        # 박스 위에 라벨 그리기
        painter.setPen(self._text_color)
        font = QFont()
        font.setPointSize(9)
        font.setBold(True)
        painter.setFont(font)

        ship_name = det.get('ship_name', f"Ship-{det['ship_idx']}")
        rel_bearing = det['rel_bearing']
        label = f"{ship_name} ({rel_bearing:.1f}°)"

        fm = QFontMetrics(font)
        label_w = fm.horizontalAdvance(label)
        label_x = int(cx - label_w / 2)
        label_y = int(y1 - 5)

        # 라벨을 가시 영역 내로 클램핑
        label_x = max(2, min(view_w - label_w - 2, label_x))
        label_y = max(30, label_y)

        painter.drawText(label_x, label_y, label)


class CombinedPanoramaView(QWidget):
    """
    EO/IR 통합 파노라마 뷰 위젯입니다.

    EO 카메라 범위(±90°)를 기준으로 표시하고,
    EO bbox는 녹색 실선, IR bbox는 빨간색 점선으로 오버레이합니다.
    IR 범위(±55°) 밖에서는 EO bbox만 표시됩니다.
    """

    def __init__(self, parent=None):
        """
        CombinedPanoramaView를 초기화합니다.

        매개변수:
            parent: 부모 위젯
        """
        super().__init__(parent)
        self.eo_detections = []  # EO 카메라 탐지 데이터
        self.ir_detections = []  # IR 카메라 탐지 데이터
        self.setMinimumHeight(100)
        self.setMinimumWidth(400)
        self._last_update = 0
        self._debounce_ms = 50

        # EO 기준 파노라마 설정
        self._pano_w = EO_PANO_W_PX
        self._pano_h = EO_PANO_H_PX
        self._fov_deg = EO_FOV_DEG
        self._half_fov = 90.0

        # 색상 설정
        self._bg_color = QColor(30, 30, 40)
        self._grid_color = QColor(60, 60, 80)
        self._text_color = QColor(255, 255, 255)
        self._scale_color = QColor(150, 150, 170)

        # EO bbox 색상 (녹색 실선)
        self._eo_bbox_color = QColor(0, 255, 0)
        self._eo_bbox_fill = QColor(0, 255, 0, 40)

        # IR bbox 색상 (빨간색 점선)
        self._ir_bbox_color = QColor(255, 100, 100)
        self._ir_bbox_fill = QColor(255, 100, 100, 40)

        # IR 범위 표시 색상
        self._ir_range_color = QColor(255, 100, 100, 30)

    def set_detections(self, detections_dict):
        """
        듀얼 카메라 탐지 데이터를 업데이트합니다.

        매개변수:
            detections_dict: {'eo': [...], 'ir': [...]} 형식의 딕셔너리
        """
        now = time.time() * 1000
        if now - self._last_update < self._debounce_ms:
            return
        self._last_update = now

        if isinstance(detections_dict, dict):
            self.eo_detections = detections_dict.get('eo', [])
            self.ir_detections = detections_dict.get('ir', [])
        elif isinstance(detections_dict, list) and len(detections_dict) > 0:
            if isinstance(detections_dict[0], dict) and 'eo' in detections_dict[0]:
                data = detections_dict[0]
                self.eo_detections = data.get('eo', [])
                self.ir_detections = data.get('ir', [])
            else:
                self.eo_detections = detections_dict
                self.ir_detections = []
        else:
            self.eo_detections = []
            self.ir_detections = []

        self.update()

    def clear_detections(self):
        """모든 탐지 데이터를 지웁니다."""
        self.eo_detections = []
        self.ir_detections = []
        self.update()

    def paintEvent(self, event):
        """파노라마 뷰를 그립니다."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w = self.width()
        h = self.height()

        # 배경 그리기
        painter.fillRect(0, 0, w, h, self._bg_color)

        # IR 범위 영역 표시 (±55°)
        self._draw_ir_range(painter, w, h)

        # 스케일 팩터 계산 (EO 기준)
        scale_x = w / self._pano_w
        scale_y = h / self._pano_h

        # 상단에 방위각 눈금 그리기
        self._draw_scale(painter, w, h)

        # 수평선 그리기
        horizon_y = int(0.50 * h)
        painter.setPen(QPen(self._grid_color, 1, Qt.PenStyle.DashLine))
        painter.drawLine(0, horizon_y, w, horizon_y)

        # EO bbox 그리기 (녹색 실선)
        for det in self.eo_detections:
            self._draw_detection(painter, det, scale_x, scale_y, w, h,
                                 self._eo_bbox_color, self._eo_bbox_fill,
                                 Qt.PenStyle.SolidLine, "EO")

        # IR bbox 그리기 (빨간색 점선) - EO 좌표계로 변환하여 표시
        for det in self.ir_detections:
            self._draw_ir_detection_on_eo(painter, det, w, h)

        painter.end()

    def _draw_ir_range(self, painter, w, h):
        """IR 카메라 범위(±55°) 경계선만 표시합니다."""
        # IR 범위의 시작/끝 x 좌표 계산 (EO 좌표계 기준)
        ir_left_norm = (-55.0 + 90.0) / 180.0
        ir_right_norm = (55.0 + 90.0) / 180.0
        ir_left_x = int(ir_left_norm * w)
        ir_right_x = int(ir_right_norm * w)

        # IR 범위 경계선만 표시 (점선, 배경 색칠 없음)
        painter.setPen(QPen(self._ir_bbox_color, 1, Qt.PenStyle.DotLine))
        painter.drawLine(ir_left_x, 25, ir_left_x, h)
        painter.drawLine(ir_right_x, 25, ir_right_x, h)

    def _draw_scale(self, painter, w, h):
        """뷰 상단에 방위각 눈금을 그립니다."""
        painter.setPen(QPen(self._scale_color, 1))
        font = QFont()
        font.setPointSize(8)
        painter.setFont(font)

        # EO 범위 눈금 (±90°)
        bearings = [-90, -55, -30, 0, 30, 55, 90]

        for bearing in bearings:
            x_norm = (bearing + self._half_fov) / self._fov_deg
            x = int(x_norm * w)

            painter.drawLine(x, 0, x, 10)

            label = f"{bearing}°"
            fm = QFontMetrics(font)
            label_w = fm.horizontalAdvance(label)
            label_x = x - label_w // 2
            label_x = max(2, min(w - label_w - 2, label_x))

            # ±55° 표시는 IR 색상으로
            if bearing in [-55, 55]:
                painter.setPen(QPen(self._ir_bbox_color, 1))
                painter.drawText(label_x, 22, label)
                painter.setPen(QPen(self._scale_color, 1))
            else:
                painter.drawText(label_x, 22, label)

        # 중심선 (0°)
        center_x = w // 2
        painter.setPen(QPen(self._grid_color, 1, Qt.PenStyle.DashLine))
        painter.drawLine(center_x, 25, center_x, h)

    def _draw_detection(self, painter, det, scale_x, scale_y, view_w, view_h,
                        bbox_color, bbox_fill, pen_style, camera_label):
        """탐지 bbox를 그립니다."""
        cx = det['cx_px'] * scale_x
        cy = det['cy_px'] * scale_y
        bw = det['w_px'] * scale_x
        bh = det['h_px'] * scale_y

        x1 = cx - bw / 2
        y1 = cy - bh / 2

        pen = QPen(bbox_color, 2, pen_style)
        painter.setPen(pen)
        painter.setBrush(QBrush(bbox_fill))
        painter.drawRect(int(x1), int(y1), int(bw), int(bh))

        # 라벨 그리기 (상단 중앙)
        painter.setPen(self._text_color)
        font = QFont()
        font.setPointSize(9)
        font.setBold(True)
        painter.setFont(font)

        ship_name = det.get('ship_name', f"Ship-{det['ship_idx']}")
        rel_bearing = det['rel_bearing']
        label = f"{ship_name} ({rel_bearing:.1f}°)"

        fm = QFontMetrics(font)
        label_w = fm.horizontalAdvance(label)
        label_x = int(cx - label_w / 2)
        label_y = int(y1 - 5)

        label_x = max(2, min(view_w - label_w - 2, label_x))
        label_y = max(30, label_y)

        painter.drawText(label_x, label_y, label)

        # 카메라 타입 라벨 (bbox 왼쪽에 표시)
        painter.setPen(bbox_color)
        small_font = QFont()
        small_font.setPointSize(8)
        small_font.setBold(True)
        painter.setFont(small_font)
        painter.drawText(int(x1 - 20), int(y1 + bh / 2 + 4), camera_label)

    def _draw_ir_detection_on_eo(self, painter, det, view_w, view_h):
        """IR 탐지를 EO 좌표계로 변환하여 그립니다."""
        # IR bbox의 원본 좌표 사용 (left, top, right, bottom)
        ir_left = det.get('left', det['cx_px'] - det['w_px'] / 2)
        ir_top = det.get('top', det['cy_px'] - det['h_px'] / 2)
        ir_right = det.get('right', det['cx_px'] + det['w_px'] / 2)
        ir_bottom = det.get('bottom', det['cy_px'] + det['h_px'] / 2)

        # IR 파노라마 좌표를 각도로 변환 (IR: 1536px = 110°)
        # IR 중앙이 0°, 좌측이 -55°, 우측이 +55°
        ir_left_deg = (ir_left / IR_PANO_W_PX - 0.5) * IR_FOV_DEG
        ir_right_deg = (ir_right / IR_PANO_W_PX - 0.5) * IR_FOV_DEG

        # 각도를 EO 파노라마 좌표로 변환 (EO: 3840px = 180°)
        # EO 중앙이 0°, 좌측이 -90°, 우측이 +90°
        eo_left_norm = (ir_left_deg + 90.0) / 180.0
        eo_right_norm = (ir_right_deg + 90.0) / 180.0

        # 뷰 좌표로 변환
        x1 = eo_left_norm * view_w
        x2 = eo_right_norm * view_w
        bw = x2 - x1

        # 수직 좌표는 비율로 변환 (IR과 EO의 종횡비가 다르므로)
        ir_top_ratio = ir_top / IR_PANO_H_PX
        ir_bottom_ratio = ir_bottom / IR_PANO_H_PX

        y1 = ir_top_ratio * view_h
        y2 = ir_bottom_ratio * view_h
        bh = y2 - y1

        # IR bbox 그리기 (빨간색 점선, 일정한 간격)
        pen = QPen(self._ir_bbox_color, 2, Qt.PenStyle.CustomDashLine)
        pen.setDashPattern([4, 4])  # 4픽셀 점, 4픽셀 빈공간 (일정 간격)
        painter.setPen(pen)
        painter.setBrush(QBrush(self._ir_bbox_fill))
        painter.drawRect(int(x1), int(y1), int(bw), int(bh))

        # IR 라벨 (bbox 오른쪽에 표시)
        painter.setPen(self._ir_bbox_color)
        small_font = QFont()
        small_font.setPointSize(8)
        small_font.setBold(True)
        painter.setFont(small_font)
        painter.drawText(int(x2 + 3), int((y1 + y2) / 2 + 4), "IR")


class DualPanoramaView(QWidget):
    """
    EO/IR 듀얼 카메라 파노라마 뷰 위젯입니다.

    CombinedPanoramaView를 사용하여 EO/IR을 하나의 뷰에 함께 표시합니다.
    """

    def __init__(self, parent=None):
        """
        DualPanoramaView를 초기화합니다.

        매개변수:
            parent: 부모 위젯
        """
        super().__init__(parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        # 라벨
        header = QLabel("EO/IR Combined View (Green=EO Solid, Red=IR Dashed)")
        header.setStyleSheet("color: #aaa; font-size: 10px;")
        layout.addWidget(header)

        # 통합 파노라마 뷰
        self._combined_view = CombinedPanoramaView()
        layout.addWidget(self._combined_view)

        # 하위 호환용 (개별 뷰 접근)
        self._eo_view = self._combined_view
        self._ir_view = self._combined_view

        # 디바운싱
        self._last_update = 0
        self._debounce_ms = 50

    def set_detections(self, detections_dict):
        """
        듀얼 카메라 탐지 데이터를 업데이트합니다.

        매개변수:
            detections_dict: {'eo': [...], 'ir': [...]} 형식의 딕셔너리
                또는 레거시 형식의 리스트
        """
        self._combined_view.set_detections(detections_dict)

    def clear_detections(self):
        """모든 탐지 데이터를 지웁니다."""
        self._combined_view.clear_detections()

    def get_eo_view(self):
        """EO 카메라 뷰를 반환합니다 (하위 호환)."""
        return self._combined_view

    def get_ir_view(self):
        """IR 카메라 뷰를 반환합니다 (하위 호환)."""
        return self._combined_view
