"""
파노라마 뷰 위젯 모듈

이 모듈은 카메라 탐지 시각화를 위한 파노라마 뷰 위젯입니다.
-90°에서 +90° 범위의 시야각에서 탐지된 선박을 바운딩 박스로 표시합니다.

클래스:
    PanoramaView: 파노라마 뷰 위젯

사양:
    C-1: 파노라마 뷰 표시
    C-2: 바운딩 박스 그리기
    C-3: 디바운싱을 통한 과도한 다시그리기 예방

상수:
    PANO_W_PX: 파노라마 이미지 너비 (1920 픽셀)
    PANO_H_PX: 파노라마 이미지 높이 (320 픽셀)
"""

import time
from PyQt6.QtWidgets import QWidget
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush, QFont, QFontMetrics


# 파노라마 상수 (simulation_worker.py와 일치)
PANO_W_PX = 1920  # 파노라마 이미지 너비
PANO_H_PX = 320   # 파노라마 이미지 높이


class PanoramaView(QWidget):
    """
    카메라 탐지를 표시하는 파노라마 뷰 위젯입니다.

    -90°에서 +90° 시야각(FOV) 내에서 탐지된 선박을 바운딩 박스로 표시합니다.
    디바운싱을 적용하여 과도한 화면 갱신을 예방합니다.

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

    def __init__(self, parent=None):
        """
        PanoramaView를 초기화합니다.

        매개변수:
            parent: 부모 위젯
        """
        super().__init__(parent)
        self.detections = []  # 탐지 데이터 목록
        self.setMinimumHeight(100)  # 최소 높이
        self.setMinimumWidth(400)   # 최소 너비
        self._last_update = 0       # 마지막 업데이트 시간
        self._debounce_ms = 50      # 디바운스 간격 (사양 C-3: 30-100ms)

        # 색상 설정
        self._bg_color = QColor(30, 30, 40)        # 배경색
        self._grid_color = QColor(60, 60, 80)      # 그리드 색상
        self._bbox_color = QColor(0, 255, 0)       # 바운딩 박스 테두리 (녹색)
        self._bbox_fill = QColor(0, 255, 0, 40)    # 바운딩 박스 채우기 (반투명)
        self._text_color = QColor(255, 255, 255)   # 텍스트 (흰색)
        self._scale_color = QColor(150, 150, 170)  # 눈금 색상

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
        scale_x = w / PANO_W_PX
        scale_y = h / PANO_H_PX

        # 상단에 방위각 눈금 그리기
        self._draw_scale(painter, w, h)

        # cy 비율 위치에 수평선 그리기
        horizon_y = int(0.60 * h)
        painter.setPen(QPen(self._grid_color, 1, Qt.PenStyle.DashLine))
        painter.drawLine(0, horizon_y, w, horizon_y)

        # 각 탐지에 대해 바운딩 박스 그리기
        for det in self.detections:
            self._draw_detection(painter, det, scale_x, scale_y, w, h)

        painter.end()

    def _draw_scale(self, painter, w, h):
        """
        뷰 상단에 방위각 눈금을 그립니다.

        -90°, -60°, -30°, 0°, 30°, 60°, 90° 위치에
        눈금선과 라벨을 표시합니다.

        매개변수:
            painter: QPainter 객체
            w: 뷰 너비
            h: 뷰 높이
        """
        painter.setPen(QPen(self._scale_color, 1))
        font = QFont()
        font.setPointSize(8)
        painter.setFont(font)

        # 주요 방위각에 눈금과 라벨 그리기
        bearings = [-90, -60, -30, 0, 30, 60, 90]
        for bearing in bearings:
            # 방위각을 x 좌표로 변환
            x_norm = (bearing + 90.0) / 180.0
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

        # 채워진 사각형 그리기
        painter.setPen(QPen(self._bbox_color, 2))
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
