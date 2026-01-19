"""
눈금자 오버레이 모듈

이 모듈은 지도 뷰 위에 위도/경도 눈금자를 표시하는 오버레이 위젯을 제공합니다.

클래스:
    RulerOverlay: 눈금자 오버레이 위젯

기능:
    - 상단에 경도 눈금 표시
    - 좌측에 위도 눈금 표시
    - 줌 레벨에 따른 최적 눈금 간격 자동 조정
    - 정규화된 경도와 실제 경도 동시 표시
"""

import math

from PyQt6.QtWidgets import QWidget
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor, QPen, QFont, QPainter

from app.core.models.project import current_project
from app.core.geometry import get_optimal_grid_step, normalize_lon


class RulerOverlay(QWidget):
    """
    지도 뷰 위에 위도/경도 눈금자를 표시하는 오버레이 위젯입니다.

    마우스 이벤트를 투명하게 전달하여 지도 상호작용을 방해하지 않습니다.
    줌 레벨에 따라 눈금 간격을 자동으로 조정합니다.

    속성:
        view: 부모 지도 뷰 위젯
    """

    def __init__(self, parent=None):
        """
        RulerOverlay를 초기화합니다.

        매개변수:
            parent: 부모 위젯 (일반적으로 지도 뷰)
        """
        super().__init__(parent)
        # 마우스 이벤트를 하위 위젯으로 전달
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        # 시스템 배경 비활성화
        self.setAttribute(Qt.WidgetAttribute.WA_NoSystemBackground)
        self.view = parent  # 부모 지도 뷰 참조
        self.setStyleSheet("background-color: transparent; color: black;")

    def paintEvent(self, event):
        """
        눈금자를 그리는 페인트 이벤트 핸들러입니다.

        상단에 경도 눈금, 좌측에 위도 눈금을 그립니다.

        매개변수:
            event: 페인트 이벤트 객체
        """
        painter = QPainter(self)

        # 상단 눈금자 배경 (반투명 흰색)
        painter.fillRect(0, 0, self.width(), 25, QColor(245, 245, 245, 230))
        # 좌측 눈금자 배경 (반투명 흰색)
        painter.fillRect(0, 0, 25, self.height(), QColor(245, 245, 245, 230))

        # 눈금자 경계선 그리기
        painter.setPen(QPen(Qt.GlobalColor.gray, 1))
        painter.drawLine(0, 25, self.width(), 25)   # 상단 경계선
        painter.drawLine(25, 0, 25, self.height())  # 좌측 경계선

        # 맵 정보 확인
        mi = current_project.map_info
        if not self.view or not mi:
            return

        # 현재 뷰포트의 씬 영역 계산
        scene_rect = self.view.mapToScene(self.view.viewport().rect()).boundingRect()

        # 스케일 계산
        scale = mi.pixels_per_degree
        current_zoom = self.view.transform().m11()
        total_scale = scale * current_zoom

        if total_scale <= 0:
            total_scale = 1

        # 줌 레벨에 따른 최적 눈금 간격 계산
        deg_step = get_optimal_grid_step(total_scale)
        pixel_step = deg_step * scale

        width = self.width()

        # 경도 눈금 범위 계산
        start_lon_idx = math.floor((scene_rect.left() / scale) / deg_step)
        end_lon_idx = math.ceil((scene_rect.right() / scale) / deg_step)

        painter.setFont(QFont("Arial", 8))

        # 눈금 간격에 따른 소수점 자릿수 결정
        if deg_step < 0.1:
            decimals = 2
        elif deg_step < 1:
            decimals = 1
        else:
            decimals = 0

        def fmt(val):
            """값을 적절한 소수점 자릿수로 포맷팅합니다."""
            if decimals == 0:
                return f"{int(round(val))}"
            else:
                return f"{val:.{decimals}f}"

        # 경도 눈금 그리기 (상단)
        for i in range(start_lon_idx, end_lon_idx + 1):
            lon_deg = i * deg_step
            sx = lon_deg * scale
            vx = self.view.mapFromScene(sx, scene_rect.top()).x()

            if 25 <= vx <= width:
                painter.setPen(QPen(Qt.GlobalColor.black, 1))
                painter.drawLine(vx, 0, vx, 5)

                # 정규화된 경도 (-180 ~ 180)
                norm_lon = normalize_lon(lon_deg)

                # 정규화된 경도와 실제 경도 함께 표시
                txt = f"{fmt(norm_lon)}({fmt(lon_deg)})"

                # 180° 배수인 경우 굵게 표시
                is_major = (abs(lon_deg % 180) < 1e-9)

                font = painter.font()
                font.setBold(is_major)
                painter.setFont(font)

                painter.drawText(vx + 4, 15, txt)

        # 위도 눈금 범위 계산
        height = self.height()
        start_lat_idx = math.floor((-scene_rect.bottom() / scale) / deg_step)
        end_lat_idx = math.ceil((-scene_rect.top() / scale) / deg_step)

        # 위도 눈금 그리기 (좌측)
        for i in range(start_lat_idx, end_lat_idx + 1):
            lat_deg = i * deg_step
            sy = -lat_deg * scale  # Y축은 반전 (북쪽이 위)
            vy = self.view.mapFromScene(scene_rect.left(), sy).y()

            if 25 <= vy <= height:
                painter.setPen(QPen(Qt.GlobalColor.black, 1))
                painter.drawLine(0, vy, 5, vy)
                painter.save()
                painter.translate(15, vy + 4)

                txt = fmt(lat_deg)

                # 0° (적도)인 경우 굵게 표시
                is_zero = (abs(lat_deg) < 0.001)

                font = painter.font()
                font.setBold(is_zero)
                painter.setFont(font)

                painter.drawText(0, 0, txt)
                painter.restore()
