"""
색상 탭바 위젯 모듈

이 모듈은 테마에 따라 색상이 변경되는 커스텀 탭바를 제공합니다.

클래스:
    ColoredTabBar: 테마 인식 탭바 위젯
"""

from PyQt6.QtWidgets import (
    QApplication, QTabBar, QStyleOptionTab
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor, QPainter
from app.core.models.project import current_project


class ColoredTabBar(QTabBar):
    """
    테마에 따라 색상이 자동으로 변경되는 탭바 위젯입니다.

    프로젝트 설정의 테마 모드(Light/Dark/System)에 따라
    탭의 배경색과 텍스트 색상이 자동으로 조정됩니다.
    """

    def paintEvent(self, event):
        """
        탭바를 그리는 이벤트 핸들러입니다.

        테마 설정에 따라 적절한 배경색과 텍스트 색상으로 탭을 렌더링합니다.

        매개변수:
            event: 페인트 이벤트 객체
        """
        painter = QPainter(self)
        option = QStyleOptionTab()

        # 테마 모드 확인
        s = current_project.settings
        mode = s.theme_mode
        is_dark = False

        if mode == "Dark":
            is_dark = True
        elif mode == "Light":
            is_dark = False
        else:  # System - 시스템 설정 따름
            if QApplication.styleHints().colorScheme() == Qt.ColorScheme.Dark:
                is_dark = True

        # 테마에 따른 색상 설정
        bg_color = QColor(Qt.GlobalColor.black) if is_dark else QColor(Qt.GlobalColor.white)
        text_color = QColor(Qt.GlobalColor.white) if is_dark else QColor(Qt.GlobalColor.black)

        # 각 탭 그리기
        for i in range(self.count()):
            self.initStyleOption(option, i)

            # 배경 그리기
            painter.fillRect(option.rect, bg_color)

            # 텍스트 그리기
            painter.setPen(text_color)
            font = painter.font()
            painter.setFont(font)
            painter.drawText(option.rect, Qt.AlignmentFlag.AlignCenter, self.tabText(i))

            # 아이콘이 있으면 그리기 (왼쪽 정렬)
            if not self.tabIcon(i).isNull():
                self.tabIcon(i).paint(
                    painter,
                    option.rect.adjusted(5, 0, -5, 0),
                    Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter
                )
