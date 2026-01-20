"""
메인 윈도우 모듈

이 모듈은 Maritime Signal Synthesizer 애플리케이션의 메인 윈도우를 제공합니다.
선박 경로 편집, 이벤트 관리, 시나리오 설정, 시뮬레이션 제어 등의 핵심 기능을 포함합니다.

클래스:
    AddShipDialog: 새 선박 추가 다이얼로그
    ColoredTabBar: 테마 인식 탭바 위젯
    MainWindow: 애플리케이션 메인 윈도우

주요 기능:
    - 프로젝트 생성/열기/저장
    - 선박 및 영역 객체 관리
    - 경로 포인트 추가/이동/삭제
    - 지도 뷰에서 시각적 편집
    - Path/Event/Scenario/Simulation 탭 전환
    - 테마 설정 (Light/Dark/System)
"""

import sys
import os
import math
import json
import csv
import random
import datetime
import re
import shutil
import traceback
import socket
import time
import tempfile
import uuid
import numpy as np

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QFormLayout,
    QPushButton, QLabel, QLineEdit, QTextEdit, QComboBox, QCheckBox, QSpinBox, QDoubleSpinBox,
    QSlider, QProgressBar, QListWidget, QListWidgetItem, QTableWidget, QTableWidgetItem,
    QHeaderView, QTreeWidget, QTreeWidgetItem, QTabWidget, QTabBar, QStyleOptionTab,
    QStackedWidget, QGroupBox, QFrame, QSplitter, QScrollArea, QFileDialog,
    QColorDialog, QInputDialog, QDialog, QDialogButtonBox, QMenuBar, QMenu,
    QToolBar, QStatusBar, QGraphicsView, QGraphicsScene, QGraphicsItem, QGraphicsPathItem, QDockWidget,
    QGraphicsPolygonItem, QGraphicsEllipseItem, QGraphicsTextItem, QAbstractItemView,
    QAbstractSpinBox, QTextBrowser, QSizePolicy, QDateTimeEdit, QGraphicsRectItem
)
from PyQt6.QtCore import (
    Qt, pyqtSignal, QObject, QThread, QTimer, QPoint, QPointF, QRect, QRectF, QSize, QSizeF,
    QEvent, QUrl, QDate, QTime, QDateTime
)
from PyQt6.QtGui import (
    QColor, QPalette, QPen, QBrush, QFont, QPainter, QPainterPath, QPolygonF, QIcon, QPixmap,
    QCursor, QAction
)
from app.core.constants import APP_NAME, DEFAULT_WINDOW_SIZE, CSV_HEADER
from app.core.models.project import current_project
from app.core.models.ship import ShipData
from app.core.models.area import Area
from app.core.models.event import EventTrigger
from app.core.utils import sanitize_filename
from app.core.geometry import coords_to_pixel, pixel_to_coords, resample_polyline_numpy, resample_polygon_equidistant, normalize_lon, great_circle_path_pixels, calculate_bearing
from app.ui.map.map_view import MapView
from app.ui.dialogs.new_project_dialog import NewProjectDialog
from app.ui.dialogs.settings_dialog import SettingsDialog
from app.ui.dialogs.help_dialog import HelpDialog
from app.ui.panels.simulation_panel import SimulationPanel
from app.ui.panels.event_panel import EventScriptPanel
from app.ui.panels.scenario_panel import ScenarioPanel
from app.ui.widgets import message_box as msg
import app.core.state as app_state


class AddShipDialog(QDialog):
    """
    새 선박을 추가하기 위한 다이얼로그입니다.

    선박 이름, 클래스(종류), 제원(길이, 폭, 흘수, 공기흘수)을 입력받습니다.
    선박 클래스 선택 시 해당 클래스의 기본 제원이 자동으로 설정됩니다.

    속성:
        SHIP_CLASSES: 지원하는 선박 클래스 목록
        ship_dimensions: 클래스별 기본 제원 딕셔너리
        name_edit: 선박 이름 입력 필드
        class_combo: 선박 클래스 선택 콤보박스
        length_spin: 길이 입력 스핀박스
        beam_spin: 폭 입력 스핀박스
        draft_spin: 흘수 입력 스핀박스
        air_draft_spin: 공기흘수 입력 스핀박스
    """

    SHIP_CLASSES = ["CONTAINER", "TANKER", "CARGO", "PASSENGER", "FISHING", "BUOY", "OTHER"]

    def __init__(self, parent=None, default_name="Ship-0"):
        """
        AddShipDialog를 초기화합니다.

        매개변수:
            parent: 부모 위젯
            default_name: 기본 선박 이름
        """
        super().__init__(parent)
        from app.core.models.ship import SHIP_CLASS_DIMENSIONS

        self.ship_dimensions = SHIP_CLASS_DIMENSIONS
        self.setWindowTitle("Add Ship")
        self.setMinimumWidth(350)

        layout = QVBoxLayout(self)

        # 선박 기본 정보 그룹
        basic_group = QGroupBox("Basic Information")
        basic_layout = QFormLayout(basic_group)

        self.name_edit = QLineEdit(default_name)
        basic_layout.addRow("Ship Name:", self.name_edit)

        self.class_combo = QComboBox()
        self.class_combo.addItems(self.SHIP_CLASSES)
        self.class_combo.setCurrentText("CONTAINER")
        self.class_combo.currentTextChanged.connect(self._on_class_changed)
        basic_layout.addRow("Ship Class:", self.class_combo)

        layout.addWidget(basic_group)

        # 선박 제원 그룹
        dim_group = QGroupBox("Dimensions (meters)")
        dim_layout = QFormLayout(dim_group)

        self.length_spin = QDoubleSpinBox()
        self.length_spin.setRange(1, 500)
        self.length_spin.setValue(300.0)
        self.length_spin.setDecimals(1)
        self.length_spin.setSuffix(" m")
        dim_layout.addRow("Length:", self.length_spin)

        self.beam_spin = QDoubleSpinBox()
        self.beam_spin.setRange(1, 100)
        self.beam_spin.setValue(40.0)
        self.beam_spin.setDecimals(1)
        self.beam_spin.setSuffix(" m")
        dim_layout.addRow("Beam (Width):", self.beam_spin)

        self.draft_spin = QDoubleSpinBox()
        self.draft_spin.setRange(1, 50)
        self.draft_spin.setValue(15.0)
        self.draft_spin.setDecimals(1)
        self.draft_spin.setSuffix(" m")
        dim_layout.addRow("Draft:", self.draft_spin)

        self.air_draft_spin = QDoubleSpinBox()
        self.air_draft_spin.setRange(1, 150)
        self.air_draft_spin.setValue(60.0)
        self.air_draft_spin.setDecimals(1)
        self.air_draft_spin.setSuffix(" m")
        dim_layout.addRow("Air Draft:", self.air_draft_spin)

        layout.addWidget(dim_group)

        # 버튼
        btn_box = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        btn_box.accepted.connect(self.accept)
        btn_box.rejected.connect(self.reject)
        layout.addWidget(btn_box)

    def _on_class_changed(self, ship_class: str):
        """
        선박 클래스가 변경될 때 제원 스핀박스를 업데이트합니다.

        매개변수:
            ship_class: 선택된 선박 클래스
        """
        dims = self.ship_dimensions.get(ship_class, self.ship_dimensions["OTHER"])
        self.length_spin.setValue(dims[0])
        self.beam_spin.setValue(dims[1])
        self.draft_spin.setValue(dims[2])
        self.air_draft_spin.setValue(dims[3])

    def get_data(self):
        """
        사용자가 입력한 선박 데이터를 반환합니다.

        반환값:
            선박 데이터 딕셔너리 (name, ship_class, length_m, beam_m, draft_m, air_draft_m)
        """
        return {
            "name": self.name_edit.text().strip() or "Unnamed",
            "ship_class": self.class_combo.currentText(),
            "length_m": self.length_spin.value(),
            "beam_m": self.beam_spin.value(),
            "draft_m": self.draft_spin.value(),
            "air_draft_m": self.air_draft_spin.value()
        }


class ColoredTabBar(QTabBar):
    """
    테마에 따라 색상이 변경되는 커스텀 탭바입니다.

    프로젝트 설정의 테마 모드(Light/Dark/System)에 따라
    탭의 배경색과 텍스트 색상이 자동으로 조정됩니다.
    빈 탭(스페이서)은 남은 공간을 채우도록 크기가 조정됩니다.
    """

    def tabSizeHint(self, index):
        """
        탭의 크기 힌트를 반환합니다.

        빈 텍스트의 탭은 남은 공간을 모두 차지하도록 크기가 계산됩니다.

        매개변수:
            index: 탭 인덱스

        반환값:
            탭 크기 (QSize)
        """
        size = super().tabSizeHint(index)
        if self.tabText(index) == "":
            # 스페이서 탭: 남은 공간 계산
            total_width = self.width()
            used_width = 0
            for i in range(self.count()):
                if i != index:
                    used_width += super().tabSizeHint(i).width()
            space = total_width - used_width
            if space < 0: space = 0
            return QSize(space, size.height())
        return size

    def minimumTabSizeHint(self, index):
        """Return a flexible minimum size for the spacer tab."""
        size = super().minimumTabSizeHint(index)
        if self.tabText(index) == "":
            return QSize(0, size.height())
        return size

    def resizeEvent(self, event):
        """
        리사이즈 이벤트 핸들러입니다.

        탭바 크기가 변경될 때 지오메트리를 업데이트합니다.

        매개변수:
            event: 리사이즈 이벤트 객체
        """
        super().resizeEvent(event)
        self.updateGeometry()

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
        if mode == "Dark": is_dark = True
        elif mode == "Light": is_dark = False
        else:  # System - 시스템 설정 따름
            if QApplication.styleHints().colorScheme() == Qt.ColorScheme.Dark: is_dark = True  # type: ignore

        # 테마에 따른 색상 설정
        bg_color = QColor(Qt.GlobalColor.black) if is_dark else QColor(Qt.GlobalColor.white)
        text_color = QColor(Qt.GlobalColor.white) if is_dark else QColor(Qt.GlobalColor.black)

        # 각 탭 그리기
        for i in range(self.count()):
            if self.tabText(i) == "": continue  # 스페이서 탭 건너뜀

            self.initStyleOption(option, i)

            # 배경 그리기
            painter.fillRect(option.rect, bg_color)

            # 텍스트 그리기
            painter.setPen(text_color)
            font = painter.font()
            painter.setFont(font)
            painter.drawText(option.rect, Qt.AlignmentFlag.AlignCenter, self.tabText(i))

            # 구분선 그리기
            if i < self.count() - 1 and self.tabText(i+1) != "":
                painter.setPen(QPen(QColor(200, 200, 200), 1))
                painter.drawLine(option.rect.topRight() + QPoint(0, 5), option.rect.bottomRight() - QPoint(0, 5))

            # 아이콘이 있으면 그리기
            if not self.tabIcon(i).isNull():
                self.tabIcon(i).paint(painter, option.rect.adjusted(5, 0, -5, 0), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)


class MainWindow(QMainWindow):
    """
    애플리케이션의 메인 윈도우입니다.

    프로젝트 관리, 선박/영역 편집, 이벤트 설정, 시나리오 구성,
    시뮬레이션 제어 등 모든 핵심 기능을 제공합니다.

    시그널:
        data_changed: 데이터가 변경되었을 때 발생

    속성:
        sim_panel: 시뮬레이션 패널
        event_panel: 이벤트 스크립트 패널
        scenario_panel: 시나리오 패널
        tabs: 메인 탭 위젯
        view: 지도 뷰
        scene: 그래픽스 씬
        obj_combo: 객체 선택 콤보박스
        data_table: 데이터 테이블
    """

    data_changed = pyqtSignal()

    def __init__(self):
        """메인 윈도우를 초기화합니다."""
        super().__init__()
        self.setWindowTitle(APP_NAME)
        self.resize(*DEFAULT_WINDOW_SIZE)
        self.setMinimumSize(800, 600)
        self.setMaximumSize(16777215, 16777215)
        self.apply_theme()
        self.update_stylesheets()

        # 패널 생성
        self.sim_panel = SimulationPanel(self)
        self.event_panel = EventScriptPanel(self)
        self.scenario_panel = ScenarioPanel(self)
        self.scenario_panel.data_changed.connect(self.on_data_changed)

        # UI 초기화
        self.init_ui()
        self.update_ui_state(False)
        self.update_info_strip()

        # 시그널 연결
        self.data_changed.connect(self.on_data_changed)

        # 탭 상태 초기화
        self.update_sim_tab_state("STOP")

    def apply_theme(self):
        """
        현재 테마 설정을 애플리케이션에 적용합니다.

        테마 모드(Light/Dark/System)에 따라 전체 애플리케이션의
        팔레트 색상을 설정합니다.
        """
        mode = current_project.settings.theme_mode
        is_dark = False
        if mode == "Dark": is_dark = True
        elif mode == "Light": is_dark = False
        else:  # System - 시스템 설정 따름
            if QApplication.styleHints().colorScheme() == Qt.ColorScheme.Dark: is_dark = True  # type: ignore

        p = QApplication.palette()

        if is_dark:
            # 다크 테마 팔레트 설정
            p.setColor(QPalette.ColorRole.Window, QColor(53, 53, 53))
            p.setColor(QPalette.ColorRole.WindowText, Qt.GlobalColor.white)
            p.setColor(QPalette.ColorRole.Base, QColor(25, 25, 25))
            p.setColor(QPalette.ColorRole.AlternateBase, QColor(53, 53, 53))
            p.setColor(QPalette.ColorRole.ToolTipBase, Qt.GlobalColor.white)
            p.setColor(QPalette.ColorRole.ToolTipText, Qt.GlobalColor.white)
            p.setColor(QPalette.ColorRole.Text, Qt.GlobalColor.white)
            p.setColor(QPalette.ColorRole.Button, QColor(53, 53, 53))
            p.setColor(QPalette.ColorRole.ButtonText, Qt.GlobalColor.white)
            p.setColor(QPalette.ColorRole.BrightText, Qt.GlobalColor.red)
            p.setColor(QPalette.ColorRole.Link, QColor(42, 130, 218))
            p.setColor(QPalette.ColorRole.Highlight, QColor(42, 130, 218))
            p.setColor(QPalette.ColorRole.HighlightedText, Qt.GlobalColor.black)
        else:
            # 라이트 테마 팔레트 설정
            p.setColor(QPalette.ColorRole.Window, Qt.GlobalColor.white)
            p.setColor(QPalette.ColorRole.WindowText, Qt.GlobalColor.black)
            p.setColor(QPalette.ColorRole.Base, Qt.GlobalColor.white)
            p.setColor(QPalette.ColorRole.AlternateBase, QColor(245, 245, 245))
            p.setColor(QPalette.ColorRole.ToolTipBase, Qt.GlobalColor.white)
            p.setColor(QPalette.ColorRole.ToolTipText, Qt.GlobalColor.black)
            p.setColor(QPalette.ColorRole.Text, Qt.GlobalColor.black)
            p.setColor(QPalette.ColorRole.Button, Qt.GlobalColor.white)
            p.setColor(QPalette.ColorRole.ButtonText, Qt.GlobalColor.black)
            p.setColor(QPalette.ColorRole.BrightText, Qt.GlobalColor.red)
            p.setColor(QPalette.ColorRole.Highlight, QColor(42, 130, 218))
            p.setColor(QPalette.ColorRole.HighlightedText, Qt.GlobalColor.white)

        QApplication.setPalette(p)

    def update_stylesheets(self):
        """
        현재 테마에 맞게 스타일시트를 업데이트합니다.

        위젯별 세부 스타일(배경색, 텍스트 색상, 테두리 등)을 설정합니다.
        """
        s = current_project.settings
        mode = s.theme_mode
        is_dark = False
        if mode == "Dark": is_dark = True
        elif mode == "Light": is_dark = False
        else:  # System
            if QApplication.styleHints().colorScheme() == Qt.ColorScheme.Dark: is_dark = True  # type: ignore

        # 테마에 따른 색상 정의
        bg_color = "#353535" if is_dark else "#ffffff"
        text_color = "#ffffff" if is_dark else "black"
        input_bg = "#252525" if is_dark else "#ffffff"
        btn_bg = "#454545" if is_dark else "#e0e0e0"
        border_color = "#ffffff" if is_dark else "#000000"
        toolbar_bg = "#2d2d2d" if is_dark else "#f0f0f0"

        # 비활성화 상태 색상
        disabled_bg = "#2a2a2a" if is_dark else "#f5f5f5"
        disabled_text = "#555555" if is_dark else "#a0a0a0"
        disabled_border = "#333333" if is_dark else "#d0d0d0"

        # 전체 스타일시트 적용
        self.setStyleSheet(f"""
            QWidget {{ color: {text_color} !important; }}
            QMainWindow, QDialog, QTableWidget, QTextEdit, QListWidget, QGraphicsView {{ background-color: {bg_color}; }}
            QMenuBar {{ background-color: {toolbar_bg}; color: {text_color}; max-height: 25px; }}
            QMenuBar::item {{ background-color: transparent; color: {text_color}; }}
            QMenuBar::item:selected {{ background-color: {btn_bg}; }}
            QMenu {{ background-color: {bg_color}; color: {text_color}; border: 1px solid {border_color}; }}
            QMenu::item:selected {{ background-color: #2a82da; color: white; }}
            QToolBar {{ background-color: {toolbar_bg}; border: 1px solid {border_color}; border-bottom: none; }}
            QToolBar QLabel {{ padding: 0 5px; color: {text_color}; }}
            QLineEdit, QSpinBox, QDoubleSpinBox {{
                background-color: {input_bg};
                color: {text_color};
                border: 1px solid {border_color};
                padding: 2px;
                border-radius: 2px;
            }}
            QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus {{
                border: 2px solid {s.select_color};
                padding: 1px;
            }}

            QPushButton {{
                background-color: {btn_bg};
                color: {text_color};
                border: 1px solid {border_color};
                padding: 4px;
                border-radius: 4px;
            }}
            QPushButton:hover {{
                background-color: {input_bg};
                border: 2px solid {s.select_color};
                padding: 3px;
            }}
            QPushButton:pressed {{ background-color: {s.select_color}; color: white; }}
            QPushButton:disabled {{
                background-color: {disabled_bg};
                color: {disabled_text};
                border: 1px solid {disabled_border};
            }}

            QHeaderView::section {{ background-color: {btn_bg}; color: {text_color}; }}
            QComboBox {{ background-color: {input_bg}; color: {text_color}; border: 1px solid {border_color}; }}
            QComboBox QAbstractItemView {{ background-color: {input_bg}; color: {text_color}; selection-background-color: #2a82da; selection-color: white; }}
            QTableWidget {{ gridline-color: {border_color}; }}
            QTableWidget::item {{ color: {text_color}; }}
            QGroupBox {{
                border: 1px solid {border_color};
                margin-top: 1.1em;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px 0 3px;
            }}
            QPushButton#spdBtn {{
                background-color: {s.btn_speed_color}; color: white; border-radius: 8px;
                max-height: 24px; padding: 2px 10px; }}
            QPushButton#spdBtn:hover {{ opacity: 0.8; }}
            QPushButton#spdBtn:disabled {{ background-color: {disabled_bg}; color: {disabled_text}; border: 1px solid {disabled_border}; }}
            QPushButton#simBtn {{
                background-color: {s.btn_sim_color}; color: white; border-radius: 8px;
                max-height: 24px; padding: 2px 10px; }}
            QPushButton#simBtn:hover {{ opacity: 0.8; }}
            QPushButton#simBtn:disabled {{ background-color: {disabled_bg}; color: {disabled_text}; border: 1px solid {disabled_border}; }}
            QPushButton#rtgBtn {{
                background-color: {s.btn_rtg_color}; color: white; border-radius: 8px;
                max-height: 24px; padding: 2px 10px; }}
            QPushButton#rtgBtn:hover {{ opacity: 0.8; }}
            QPushButton#rtgBtn:disabled {{ background-color: {disabled_bg}; color: {disabled_text}; border: 1px solid {disabled_border}; }}
            QLabel#boldLbl {{ }}

            QTabWidget::pane {{ border: 1px solid {border_color}; }}
            QSplitter::handle {{ background-color: transparent; border: none; }}
            QSplitter::handle:horizontal {{ background-color: transparent; border: none; }}
            QSplitter::handle:vertical {{ background-color: transparent; border: none; }}

            QMessageBox {{ min-height: 30px; }}
            QMessageBox QLabel {{ min-height: 35px; }}
        """)

        # 좌표 라벨 스타일 업데이트
        if hasattr(self, 'lbl_coords'):
            self.lbl_coords.setStyleSheet(f"padding: 2px 5px; color: {text_color};")

    def init_ui(self):
        """
        UI 컴포넌트를 초기화하고 배치합니다.

        상단 툴바, 정보 패널, 메인 탭 위젯, 좌표 표시 라벨 등을 생성합니다.
        """
        central = QWidget()
        self.setCentralWidget(central)
        main_v = QVBoxLayout(central)
        main_v.setContentsMargins(0, 0, 0, 0)
        main_v.setSpacing(0)

        # 상단 툴바 설정
        top_tb = QToolBar()
        top_tb.setStyleSheet("""
            QToolBar {
                background-color: transparent;
                border-bottom: 1px solid #CCC;
                spacing: 3px;
            }
            QToolBar::separator {
                background-color: #808080;
                width: 2px;
                margin-top: 4px;
                margin-bottom: 4px;
            }
        """)
        main_v.addWidget(top_tb)

        # 파일 메뉴 액션
        top_tb.addWidget(QLabel(" File: "))
        top_tb.addAction("New Project", self.new_project)
        top_tb.addAction("Open Project", self.open_project)
        top_tb.addAction("Save Project", self.save_project)
        top_tb.addAction("Clear Cache", self.clear_cache)
        top_tb.addAction("Exit", self.close)
        top_tb.addSeparator()

        # 도구 메뉴 액션
        top_tb.addWidget(QLabel(" Tools: "))
        top_tb.addAction("User Guide", self.open_user_guide)
        top_tb.addAction("Settings", self.open_settings)
        top_tb.addSeparator()

        # 프로젝트 정보 패널
        self.info_panel = QFrame()
        self.info_panel.setFrameShape(QFrame.Shape.StyledPanel)
        self.info_panel.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        h_info = QHBoxLayout(self.info_panel)
        h_info.setContentsMargins(5, 2, 5, 2)

        lbl_t = QLabel("Title:"); lbl_t.setObjectName("boldLbl")
        self.lbl_proj_name = QLabel("-")
        lbl_s = QLabel("Start:"); lbl_s.setObjectName("boldLbl")
        self.lbl_proj_time = QLabel("-")

        h_info.addWidget(lbl_t); h_info.addWidget(self.lbl_proj_name)
        h_info.addWidget(lbl_s); h_info.addWidget(self.lbl_proj_time)
        h_info.addStretch()
        main_v.addWidget(self.info_panel)

        # 탭 위젯 설정
        self.tabs = QTabWidget()
        self.tabs.setTabPosition(QTabWidget.TabPosition.South)
        self.tabs.setTabBar(ColoredTabBar(self.tabs))
        self.tabs.setDocumentMode(True)
        self.tabs.tabBar().setExpanding(False)  # type: ignore

        # 맵 에디터 탭 (Path 탭)
        self.map_editor_widget = QWidget()
        main_h = QHBoxLayout(self.map_editor_widget)
        main_h.setContentsMargins(5, 5, 5, 5)
        main_h.setSpacing(5)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setHandleWidth(10)

        # 좌측 영역 (지도 뷰)
        left_widget = QWidget()
        left_widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        left_v = QVBoxLayout(left_widget)
        left_v.setContentsMargins(0, 0, 0, 0)
        left_v.setSpacing(2)

        # 편집 툴바
        tb = QToolBar()
        tb.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        tb.setStyleSheet("""
            QToolBar {
                background-color: transparent;
                border-bottom: 1px solid #CCC;
                spacing: 3px;
            }
            QToolBar::separator {
                background-color: #808080;
                width: 2px;
                margin-top: 4px;
                margin-bottom: 4px;
            }
        """)
        left_v.addWidget(tb)

        # 초기화/선택 액션
        tb.addWidget(QLabel("Initialize: "))
        a_sel = tb.addAction("Select")
        a_sel.triggered.connect(lambda: self.set_map_mode("SELECT"))  # type: ignore
        tb.addSeparator()

        # 객체 관리 액션
        tb.addWidget(QLabel("Object:"))
        tb.addAction("Add Ship", self.add_ship_dialog)
        tb.addAction("Add Area", self.add_area_dialog)
        tb.addAction("Change Area Color", self.change_area_color)
        tb.addAction("Delete Selected", self.delete_object)
        tb.addSeparator()

        # 포인트 관리 액션
        tb.addWidget(QLabel("Point:"))
        tb.addAction("Add", self.req_add_point)
        tb.addAction("Move", lambda: self.set_map_mode("MOVE_POINT"))
        tb.addAction("Delete", lambda: self.set_map_mode("DELETE_POINT"))

        a_del_chk = tb.addAction("Delete Checked")
        a_del_chk.triggered.connect(self.delete_checked_points)  # type: ignore
        a_del_chk.setToolTip("Delete all points checked in the table")  # type: ignore

        # 그래픽스 씬 및 뷰
        self.scene = QGraphicsScene()
        self.view = MapView(self.scene, self)
        self.view.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        left_v.addWidget(self.view, 1)

        splitter.addWidget(left_widget)

        # 우측 영역 (객체 선택 및 데이터 테이블)
        right_widget = QWidget()
        right_v = QVBoxLayout(right_widget)
        right_v.setContentsMargins(0, 0, 0, 0)

        lbl_obj = QLabel("Select Object"); lbl_obj.setObjectName("boldLbl")
        right_v.addWidget(lbl_obj)
        self.obj_combo = QComboBox()
        self.obj_combo.currentIndexChanged.connect(self.on_obj_selected)
        right_v.addWidget(self.obj_combo)

        self.data_table = QTableWidget()
        self.data_table.itemChanged.connect(self.on_table_changed)
        h = self.data_table.horizontalHeader()
        h.setSectionResizeMode(QHeaderView.ResizeMode.Stretch)  # type: ignore
        right_v.addWidget(self.data_table)

        splitter.addWidget(right_widget)

        splitter.setSizes([int(self.width() * 0.75), int(self.width() * 0.25)])
        main_h.addWidget(splitter)

        # 탭 추가
        self.tabs.addTab(self.map_editor_widget, "Path")
        self.tabs.addTab(self.event_panel, "Event")
        self.tabs.addTab(self.scenario_panel, "Scenario")

        # 스페이서 탭 (빈 공간 채우기용)
        self.spacer_widget = QWidget()
        self.tabs.addTab(self.spacer_widget, "")
        self.tabs.setTabEnabled(3, False)

        # 시뮬레이션 탭
        self.tabs.addTab(self.sim_panel, "Simulation")

        main_v.addWidget(self.tabs)

        # 시그널 연결
        self.tabs.currentChanged.connect(self.on_tab_changed)
        self.sim_panel.state_changed.connect(self.update_sim_tab_state)

        # 좌표 표시 라벨
        self.lbl_coords = QLabel("Lat: 0.00000 Lon: 0.00000")
        self.lbl_coords.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.lbl_coords.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.lbl_coords.setMinimumHeight(25)
        left_v.addWidget(self.lbl_coords, 0)

        self.view.coord_changed.connect(self.update_status)
        self.update_stylesheets()

    def set_map_mode(self, mode):
        """
        지도 편집 모드를 설정합니다.

        매개변수:
            mode: 모드 문자열 ("SELECT", "ADD_POINT", "MOVE_POINT", "DELETE_POINT", "ADD_AREA")
        """
        self.view.mode = mode
        self.view.update_cursor()
        self.redraw_map()

    def update_status(self, px, py):
        """
        마우스 위치의 좌표를 상태 라벨에 업데이트합니다.

        매개변수:
            px: 픽셀 X 좌표
            py: 픽셀 Y 좌표
        """
        mi = current_project.map_info
        _, _, lat, lon = pixel_to_coords(px, py, mi)
        norm_lon = normalize_lon(lon)

        # 헤딩 편차 계산 (자선 기준)
        heading_text = self._calculate_heading_deviation(px, py)

        self.lbl_coords.setText(f"{heading_text}Lat: {lat:.5f} Lon: {norm_lon:.5f}")

    def _calculate_heading_deviation(self, cursor_px, cursor_py):
        """
        자선 출발점 기준 마우스 커서 방향의 헤딩 편차를 계산합니다.

        반환값:
            "상대편차(R)/절대헤딩(T) " 형식 문자열, 또는 계산 불가 시 빈 문자열
        """
        # 자선 가져오기
        own_idx = current_project.settings.own_ship_idx
        own_ship = current_project.get_ship_by_idx(own_idx)

        if not own_ship or len(own_ship.raw_points) < 1:
            return "-/- "

        # 자선 출발점 좌표
        start_px, start_py = own_ship.raw_points[0]
        mi = current_project.map_info
        _, _, start_lat, start_lon = pixel_to_coords(start_px, start_py, mi)
        _, _, cursor_lat, cursor_lon = pixel_to_coords(cursor_px, cursor_py, mi)

        # 자선 출발점과 커서가 동일 위치면 계산 불가
        if abs(cursor_px - start_px) < 1 and abs(cursor_py - start_py) < 1:
            return "-/- "

        # 커서 방향 절대 헤딩 (진북 기준)
        absolute_heading = calculate_bearing(start_lat, start_lon, cursor_lat, cursor_lon)

        # 자선 초기 헤딩 결정 (initial_heading 또는 첫 두 점 사이 방향)
        if own_ship.initial_heading is not None:
            own_heading = own_ship.initial_heading
        elif len(own_ship.raw_points) >= 2:
            p1_px, p1_py = own_ship.raw_points[0]
            p2_px, p2_py = own_ship.raw_points[1]
            _, _, lat1, lon1 = pixel_to_coords(p1_px, p1_py, mi)
            _, _, lat2, lon2 = pixel_to_coords(p2_px, p2_py, mi)
            own_heading = calculate_bearing(lat1, lon1, lat2, lon2)
        else:
            # 경로점이 1개뿐이고 initial_heading도 없으면 계산 불가
            return f"-/{absolute_heading:.1f}(T) "

        # 상대 헤딩 편차 계산 (-180 ~ +180)
        relative_deviation = absolute_heading - own_heading
        if relative_deviation > 180:
            relative_deviation -= 360
        elif relative_deviation < -180:
            relative_deviation += 360

        # 부호 표시 (+/-)
        sign = "+" if relative_deviation >= 0 else ""
        return f"{sign}{relative_deviation:.1f}(R)/{absolute_heading:.1f}(T) "

    def update_info_strip(self):
        """프로젝트 정보 패널을 업데이트합니다."""
        p = current_project
        self.lbl_proj_name.setText(p.project_name)
        self.lbl_proj_time.setText(p.start_time.strftime('%Y-%m-%d %H:%M:%S'))

    def update_obj_combo(self):
        """
        객체 선택 콤보박스를 업데이트합니다.

        선박과 영역 목록을 정렬하여 콤보박스에 추가합니다.
        자선(OwnShip)이 먼저 표시되고, 수동 타겟, 영역 순으로 표시됩니다.
        랜덤 생성 선박(ID >= 1000)은 Path 탭에서 제외됩니다.
        """
        self.obj_combo.blockSignals(True)
        self.obj_combo.clear()

        # 정렬 규칙:
        # 1) 자선 (OwnShip)
        # 2) 수동 타겟 (idx < 1000)
        # 3) 랜덤 타겟 (idx >= 1000) -> Path 탭에서는 제외

        own_idx = current_project.settings.own_ship_idx
        own_ship = None
        manual_targets = []

        for s in current_project.ships:
            if s.idx == own_idx: own_ship = s
            elif s.idx >= 1000: continue  # 랜덤 타겟은 콤보박스에 추가하지 않음
            else: manual_targets.append(s)

        manual_targets.sort(key=lambda x: x.idx)

        # 정렬된 선박 목록 구성
        sorted_ships = ([own_ship] if own_ship else []) + manual_targets

        for s in sorted_ships:
            tag = "OwnShip" if s.idx == own_idx else f"Target{s.idx}"
            self.obj_combo.addItem(f"[Ship] {s.name} (MMSI: {s.mmsi}) ({tag})", s.idx)

        for st in current_project.areas:
            self.obj_combo.addItem(f"[Area] {st.name}", st.id)

        self.obj_combo.blockSignals(False)
        self.on_obj_selected()

    def on_obj_selected(self):
        """
        객체가 선택되었을 때 호출됩니다.

        선택된 객체에 따라 데이터 테이블을 업데이트하고 지도를 다시 그립니다.
        """
        idx = self.obj_combo.currentIndex()
        if idx < 0:
            self.data_table.setRowCount(0)
            return

        txt = self.obj_combo.currentText()
        if "[Ship]" in txt:
            sid = self.obj_combo.currentData()
            self.show_ship_table(sid)
        elif "[Area]" in txt:
            sid = self.obj_combo.currentData()
            self.show_struct_table(sid)

        self.redraw_map()

    def show_ship_table(self, idx):
        """
        선박의 경로 포인트 데이터를 테이블에 표시합니다.

        매개변수:
            idx: 선박 인덱스
        """
        ship = current_project.get_ship_by_idx(idx)
        if not ship: return

        self.data_table.blockSignals(True)
        self.data_table.setColumnCount(6)
        self.data_table.setHorizontalHeaderLabels(["chk", "idx", "Lat", "Lon", "Speed(kn)", "MMSI"])
        self.data_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)  # type: ignore
        self.data_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)  # type: ignore
        self.data_table.setRowCount(len(ship.raw_points))

        mi = current_project.map_info
        for i, (px, py) in enumerate(ship.raw_points):
            _, _, lat, lon = pixel_to_coords(px, py, mi)
            spd = ship.raw_speeds.get(i, 0.0)

            # 체크박스 아이템
            chk_item = QTableWidgetItem()
            chk_item.setFlags(Qt.ItemFlag.ItemIsUserCheckable | Qt.ItemFlag.ItemIsEnabled)
            chk_item.setCheckState(Qt.CheckState.Unchecked)
            self.data_table.setItem(i, 0, chk_item)

            self.data_table.setItem(i, 1, QTableWidgetItem(str(i)))
            self.data_table.setItem(i, 2, QTableWidgetItem(f"{lat:.6f}"))
            self.data_table.setItem(i, 3, QTableWidgetItem(f"{normalize_lon(lon):.6f}"))

            # 속도는 첫 번째 포인트만 편집 가능
            if i == 0:
                self.data_table.setItem(i, 4, QTableWidgetItem(f"{spd:.1f}"))
            else:
                item = QTableWidgetItem("-")
                item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                self.data_table.setItem(i, 4, item)

            self.data_table.setItem(i, 5, QTableWidgetItem(str(ship.mmsi)))
            self.data_table.setRowHeight(i, 60)
        self.data_table.blockSignals(False)

    def show_struct_table(self, sid):
        """
        영역의 꼭짓점 데이터를 테이블에 표시합니다.

        매개변수:
            sid: 영역 ID
        """
        st = current_project.get_area_by_id(sid)
        if not st: return

        self.data_table.blockSignals(True)
        self.data_table.setColumnCount(3)
        self.data_table.setHorizontalHeaderLabels(["Vtx", "Lat", "Lon"])
        self.data_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)  # type: ignore
        self.data_table.setRowCount(len(st.geometry))

        mi = current_project.map_info
        for i, (px, py) in enumerate(st.geometry):
            _, _, lat, lon = pixel_to_coords(px, py, mi)
            self.data_table.setItem(i, 0, QTableWidgetItem(str(i)))
            self.data_table.setItem(i, 1, QTableWidgetItem(f"{lat:.6f}"))
            self.data_table.setItem(i, 2, QTableWidgetItem(f"{normalize_lon(lon):.6f}"))
            self.data_table.setRowHeight(i, 60)
        self.data_table.blockSignals(False)

    def on_table_changed(self, item):
        """
        테이블 셀 값이 변경되었을 때 호출됩니다.

        변경된 값을 해당 객체(선박 또는 영역)에 반영합니다.

        매개변수:
            item: 변경된 테이블 아이템
        """
        row = item.row()
        col = item.column()

        txt = self.obj_combo.currentText()
        sid = self.obj_combo.currentData()
        mi = current_project.map_info

        if "[Ship]" in txt:
            ship = current_project.get_ship_by_idx(sid)
            if not ship or row >= len(ship.raw_points): return

            # 위도/경도 변경
            if col in [2, 3]:
                try:
                    lat = float(self.data_table.item(row, 2).text())
                    lon = float(self.data_table.item(row, 3).text())
                    px, py = coords_to_pixel(lat, lon, mi)
                    ship.raw_points[row] = (px, py)
                    self.handle_path_change(ship, redraw=False)
                except: pass
            # 속도 변경 (첫 번째 포인트만)
            elif col == 4:
                if row == 0:
                    try:
                        spd = float(item.text())
                        ship.raw_speeds[row] = spd
                        self.invalidate_simulation_data(ship)
                    except: pass

            self.redraw_map()
            self.data_changed.emit()

        elif "[Area]" in txt:
             st = current_project.get_area_by_id(sid)
             if not st or row >= len(st.geometry): return
             try:
                 lat = float(self.data_table.item(row, 1).text())
                 lon = float(self.data_table.item(row, 2).text())
                 px, py = coords_to_pixel(lat, lon, mi)
                 st.geometry[row] = (px, py)
                 self.redraw_map()
                 self.data_changed.emit()
             except: pass

    def delete_checked_points(self):
        """체크된 포인트들을 삭제합니다."""
        txt = self.obj_combo.currentText()
        if "[Ship]" not in txt:
            msg.show_warning(self, "Warning", "Select a Ship first.")
            return

        sid = self.obj_combo.currentData()
        ship = current_project.get_ship_by_idx(sid)
        if not ship: return

        # 체크된 행 수집
        rows_to_del = []
        for row in range(self.data_table.rowCount()):
            item = self.data_table.item(row, 0)
            if item and item.checkState() == Qt.CheckState.Checked:
                rows_to_del.append(row)

        if not rows_to_del:
            msg.show_information(self, "Info", "No points selected.")
            return

        if msg.show_question(self, "Delete", f"Delete {len(rows_to_del)} points?") == msg.StandardButton.No:
            return

        # 역순으로 삭제 (인덱스 유지)
        rows_to_del.sort(reverse=True)

        for idx in rows_to_del:
            if idx < len(ship.raw_points):
                ship.raw_points.pop(idx)

                # 속도 인덱스 재조정
                new_speeds = {}
                new_notes = {}
                for k, v in ship.raw_speeds.items():
                    if k < idx: new_speeds[k] = v
                    elif k > idx: new_speeds[k-1] = v
                ship.raw_speeds = new_speeds

                for k, v in ship.raw_notes.items():
                    if k < idx: new_notes[k] = v
                    elif k > idx: new_notes[k-1] = v
                ship.raw_notes = new_notes

        self.handle_path_change(ship)
        self.show_ship_table(sid)
        self.redraw_map()

    def redraw_map(self):
        """
        지도를 다시 그립니다.

        모든 선박의 경로와 영역을 씬에 그립니다.
        선택된 객체는 강조 표시됩니다.
        """
        self.scene.clear()
        self.add_boundary_masks()

        sel_txt = self.obj_combo.currentText()
        sel_id = self.obj_combo.currentData()
        highlight_points = (self.view.mode in ["MOVE_POINT", "DELETE_POINT"])

        settings = current_project.settings

        # 선박 경로 그리기
        for s in current_project.ships:
            # 랜덤 생성 선박(ID >= 1000)은 Path 화면에 표시하지 않음
            if s.idx >= 1000: continue

            is_sel = ("[Ship]" in sel_txt and s.idx == sel_id)
            if not s.raw_points: continue
            pts = s.raw_points

            # 경로선 그리기 (대권항로 곡선)
            if len(pts) >= 2:
                # 대권항로로 보간된 경로 점 생성
                mi = current_project.map_info
                gc_pts = great_circle_path_pixels(pts, mi, points_per_segment=30)

                path = QGraphicsPathItem()
                pp = QPainterPath()

                # 날짜변경선 처리를 위한 threshold
                threshold = 180 * mi.pixels_per_degree

                if gc_pts:
                    pp.moveTo(QPointF(gc_pts[0][0], gc_pts[0][1]))
                    for i in range(1, len(gc_pts)):
                        # 경도 점프가 큰 경우 moveTo로 선 끊기
                        if abs(gc_pts[i][0] - gc_pts[i-1][0]) > threshold:
                            pp.moveTo(QPointF(gc_pts[i][0], gc_pts[i][1]))
                        else:
                            pp.lineTo(QPointF(gc_pts[i][0], gc_pts[i][1]))

                path.setPath(pp)
                w = settings.path_thickness

                # 선박 유형에 따른 색상 설정
                if s.idx == settings.own_ship_idx: c_hex = settings.own_color
                else: c_hex = settings.target_color

                if is_sel:
                    c_hex = settings.select_color
                    w *= settings.select_thickness_mult
                pen = QPen(QColor(c_hex), w)
                pen.setCosmetic(True)
                pen.setStyle(Qt.PenStyle.DashLine)
                path.setPen(pen)
                path.setZValue(1)
                self.scene.addItem(path)

            # 포인트 색상 설정
            if s.idx == settings.own_ship_idx: c_hex = settings.own_color
            else: c_hex = settings.target_color

            if is_sel: c_hex = settings.select_color
            c_dot = QColor(c_hex)

            # 각 포인트 그리기
            for i, (px, py) in enumerate(s.raw_points):
                r = 6
                el = QGraphicsEllipseItem(-r/2, -r/2, r, r)
                el.setBrush(QBrush(c_dot))
                el.setPen(QPen(Qt.PenStyle.NoPen))

                # 포인트 이동/삭제 모드에서 강조 링 표시
                if highlight_points:
                    ring = QGraphicsEllipseItem(-(r/2)-4, -(r/2)-4, r+8, r+8)
                    pen_ring = QPen(QColor("#FF8C00"), 2)
                    pen_ring.setCosmetic(True)
                    ring.setPen(pen_ring)
                    ring.setBrush(QBrush(Qt.BrushStyle.NoBrush))
                    ring.setPos(px, py)
                    ring.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
                    ring.setZValue(1.5)
                    self.scene.addItem(ring)

                el.setPos(px, py)
                el.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
                el.setZValue(2)
                self.scene.addItem(el)

            # Single Point + Heading 모드: 헤딩 각도 라벨 표시
            # 일반 경로 모드: 시작점('i')과 끝점('f') 라벨 표시
            if s.initial_heading is not None and s.raw_points:
                # 헤딩 각도 라벨 표시 (점 우측 상단)
                pt = s.raw_points[0]
                heading_text = f"{s.initial_heading:.1f}°"
                th = QGraphicsTextItem(heading_text)
                th.setDefaultTextColor(Qt.GlobalColor.black)
                th.setPos(pt[0] + 8, pt[1] - 20)
                th.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
                th.setZValue(3)
                self.scene.addItem(th)
            elif s.raw_points:
                start_pt = s.raw_points[0]
                end_pt = s.raw_points[-1]
                ti = QGraphicsTextItem("i")
                ti.setDefaultTextColor(Qt.GlobalColor.black)
                ti.setPos(start_pt[0] + 5, start_pt[1] - 25)
                ti.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
                ti.setZValue(3)
                self.scene.addItem(ti)
                tf = QGraphicsTextItem("f")
                tf.setDefaultTextColor(Qt.GlobalColor.black)
                tf.setPos(end_pt[0] + 5, end_pt[1] - 25)
                tf.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
                tf.setZValue(3)
                self.scene.addItem(tf)

        # 영역 그리기
        for st in current_project.areas:
            is_sel = ("[Area]" in sel_txt and st.id == sel_id)
            poly = QPolygonF([QPointF(x, y) for x, y in st.geometry])
            item = QGraphicsPolygonItem(poly)
            pen = QPen(Qt.GlobalColor.black, 2 if not is_sel else 4)
            pen.setCosmetic(True)
            item.setPen(pen)
            item.setBrush(QBrush(QColor(st.color)))
            item.setZValue(0)
            self.scene.addItem(item)

    def add_boundary_masks(self):
        """
        지도 경계 마스크를 추가합니다.

        유효한 좌표 범위(-180~180 경도, -90~90 위도) 외부를
        마스크 색상으로 덮습니다.
        """
        mi = current_project.map_info
        scale = mi.pixels_per_degree
        if scale <= 0: return

        c = QColor(current_project.settings.mask_color)
        limit = 1e9

        # 경도 -180 왼쪽 마스크
        r1 = QGraphicsRectItem(-limit, -limit, (-180 * scale) + limit, 2 * limit)
        r1.setBrush(QBrush(c))
        r1.setPen(QPen(Qt.PenStyle.NoPen))
        r1.setZValue(-100)
        self.scene.addItem(r1)

        # 경도 180 오른쪽 마스크
        r2 = QGraphicsRectItem(180 * scale, -limit, limit - (180 * scale), 2 * limit)
        r2.setBrush(QBrush(c))
        r2.setPen(QPen(Qt.PenStyle.NoPen))
        r2.setZValue(-100)
        self.scene.addItem(r2)

        # 위도 90 위쪽 마스크 (y < -90*scale)
        r3 = QGraphicsRectItem(-limit, -limit, 2 * limit, (-90 * scale) + limit)
        r3.setBrush(QBrush(c))
        r3.setPen(QPen(Qt.PenStyle.NoPen))
        r3.setZValue(-100)
        self.scene.addItem(r3)

        # 위도 -90 아래쪽 마스크 (y > 90*scale)
        r4 = QGraphicsRectItem(-limit, 90 * scale, 2 * limit, limit - (90 * scale))
        r4.setBrush(QBrush(c))
        r4.setPen(QPen(Qt.PenStyle.NoPen))
        r4.setZValue(-100)
        self.scene.addItem(r4)

    def add_point_click(self, px, py):
        """
        지도 클릭으로 포인트를 추가합니다.

        매개변수:
            px: 픽셀 X 좌표
            py: 픽셀 Y 좌표
        """
        txt = self.obj_combo.currentText()
        if "[Ship]" not in txt:
            msg.show_warning(self, "Warning", "Please select a Ship first.")
            self.set_map_mode("SELECT")
            return

        # 위도 유효성 검사
        mi = current_project.map_info
        _, _, lat, _ = pixel_to_coords(px, py, mi)

        if not (-90 <= lat <= 90):
            msg.show_warning(self, "Invalid Latitude", "Latitude must be between -90 and +90 degrees. This point will not be saved.")
            self.set_map_mode("SELECT")
            return

        sid = self.obj_combo.currentData()
        ship = current_project.get_ship_by_idx(sid)
        if not ship: return

        # 포인트 정보 입력 다이얼로그
        d = QDialog(self)
        d.setWindowTitle("Add Point Info")
        f = QFormLayout(d)
        spin_spd = QDoubleSpinBox()
        spin_spd.setRange(0, 1000)

        is_first_point = (len(ship.raw_points) == 0)
        if is_first_point:
            spin_spd.setValue(10.0)
            f.addRow("Speed (kn):", spin_spd)

        edit_note = QLineEdit()

        f.addRow("Note:", edit_note)
        bb = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        bb.accepted.connect(d.accept)
        bb.rejected.connect(d.reject)
        f.addWidget(bb)

        if d.exec() == QDialog.DialogCode.Accepted:
            new_idx = len(ship.raw_points)
            ship.raw_points.append((px, py))
            if is_first_point:
                ship.raw_speeds[new_idx] = spin_spd.value()
            ship.raw_notes[new_idx] = edit_note.text()
            self.handle_path_change(ship)

    def req_add_point(self):
        """포인트 추가 방법을 선택하는 다이얼로그를 표시합니다."""
        txt = self.obj_combo.currentText()
        if "[Ship]" not in txt:
            msg.show_warning(self, "Warning", "Please select a Ship first.")
            return

        dlg = QDialog(self)
        dlg.setWindowTitle("Add Point Method")
        v = QVBoxLayout(dlg)

        lbl = QLabel("How would you like to add the point?")
        v.addWidget(lbl)

        btn_map = QPushButton("Click on Map")
        btn_manual = QPushButton("Input Lat/Lon manually")
        btn_heading = QPushButton("Single Point + Heading (Great Circle)")

        v.addWidget(btn_map)
        v.addWidget(btn_manual)
        v.addWidget(btn_heading)

        def on_map():
            dlg.accept()
            self.set_map_mode("ADD_POINT")

        def on_manual():
            dlg.reject()
            self.manual_add_point_dialog()

        def on_heading():
            dlg.reject()
            self.single_point_heading_dialog()

        btn_map.clicked.connect(on_map)
        btn_manual.clicked.connect(on_manual)
        btn_heading.clicked.connect(on_heading)

        dlg.exec()

    def manual_add_point_dialog(self):
        """좌표를 직접 입력하여 포인트를 추가하는 다이얼로그를 표시합니다."""
        d = QDialog(self)
        d.setWindowTitle("Input Coordinates")
        f = QFormLayout(d)

        lat_spin = QDoubleSpinBox(); lat_spin.setRange(-90, 90); lat_spin.setDecimals(6)
        lon_spin = QDoubleSpinBox(); lon_spin.setRange(-180, 180); lon_spin.setDecimals(6)

        f.addRow("Latitude:", lat_spin)
        f.addRow("Longitude:", lon_spin)

        bb = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        bb.accepted.connect(d.accept)
        bb.rejected.connect(d.reject)
        f.addWidget(bb)

        if d.exec() == QDialog.DialogCode.Accepted:
            mi = current_project.map_info
            px, py = coords_to_pixel(lat_spin.value(), lon_spin.value(), mi)
            self.add_point_click(px, py)

    def single_point_heading_dialog(self):
        """
        Single Point + Heading 모드로 선박 경로를 설정하는 다이얼로그를 표시합니다.

        점 하나와 초기 헤딩만 입력받아 시뮬레이션 시 그 방향의 대권항로로 이동합니다.
        랜덤 생성 선박과 달리 사용자가 위치와 헤딩을 직접 지정합니다.
        """
        txt = self.obj_combo.currentText()
        if "[Ship]" not in txt:
            msg.show_warning(self, "Warning", "Please select a Ship first.")
            return

        sid = self.obj_combo.currentData()
        ship = current_project.get_ship_by_idx(sid)
        if not ship:
            return

        # 이미 포인트가 있으면 경고
        if ship.raw_points:
            result = msg.show_question(
                self, "Existing Points",
                "This ship already has points. Using Single Point + Heading mode will replace all existing points.\n\nContinue?"
            )
            if result == msg.StandardButton.No:
                return

        d = QDialog(self)
        d.setWindowTitle("Single Point + Heading")
        f = QFormLayout(d)

        # 위치 입력
        lbl_pos = QLabel("<b>Start Position</b>")
        f.addRow(lbl_pos)

        lat_spin = QDoubleSpinBox()
        lat_spin.setRange(-90, 90)
        lat_spin.setDecimals(6)
        lat_spin.setValue(0.0)
        f.addRow("Latitude:", lat_spin)

        lon_spin = QDoubleSpinBox()
        lon_spin.setRange(-180, 180)
        lon_spin.setDecimals(6)
        lon_spin.setValue(0.0)
        f.addRow("Longitude:", lon_spin)

        # 헤딩 입력
        lbl_hdg = QLabel("<b>Initial Heading</b>")
        f.addRow(lbl_hdg)

        hdg_spin = QDoubleSpinBox()
        hdg_spin.setRange(0, 360)
        hdg_spin.setDecimals(1)
        hdg_spin.setValue(0.0)
        hdg_spin.setSuffix("°")
        f.addRow("Heading (0=N, 90=E):", hdg_spin)

        # 속도 입력
        lbl_spd = QLabel("<b>Initial Speed</b>")
        f.addRow(lbl_spd)

        spd_spin = QDoubleSpinBox()
        spd_spin.setRange(0, 1000)
        spd_spin.setDecimals(1)
        spd_spin.setValue(10.0)
        spd_spin.setSuffix(" kn")
        f.addRow("Speed:", spd_spin)

        # 설명 라벨
        info_lbl = QLabel(
            "<i>The ship will move along a great circle route<br>"
            "in the specified heading direction during simulation.</i>"
        )
        info_lbl.setWordWrap(True)
        f.addRow(info_lbl)

        bb = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        bb.accepted.connect(d.accept)
        bb.rejected.connect(d.reject)
        f.addWidget(bb)

        if d.exec() == QDialog.DialogCode.Accepted:
            mi = current_project.map_info
            px, py = coords_to_pixel(lat_spin.value(), lon_spin.value(), mi)

            # 기존 포인트 초기화
            ship.raw_points = [(px, py)]
            ship.raw_speeds = {0: spd_spin.value()}
            ship.raw_notes = {0: f"Heading: {hdg_spin.value():.1f}°"}

            # initial_heading 설정 (이 값이 있으면 Single Point + Heading 모드)
            ship.initial_heading = hdg_spin.value()

            self.handle_path_change(ship)

    def handle_path_change(self, ship, redraw=True):
        """
        경로가 변경되었을 때 처리합니다.

        경로를 리샘플링하고 시뮬레이션 데이터를 무효화합니다.

        매개변수:
            ship: 변경된 선박
            redraw: 지도 다시 그리기 여부
        """
        if len(ship.raw_points) < 2:
            ship.resampled_points = list(ship.raw_points)
        else:
            ship.resampled_points = resample_polyline_numpy(ship.raw_points, 60)

        self.invalidate_simulation_data(ship)

        if redraw:
            self.redraw_map()
            self.on_obj_selected()

        self.data_changed.emit()

    def invalidate_simulation_data(self, ship):
        """
        선박의 시뮬레이션 데이터를 무효화합니다.

        경로나 속도가 변경되었을 때 호출되어 시뮬레이션 데이터를 초기화합니다.

        매개변수:
            ship: 대상 선박
        """
        ship.time_series = []
        ship.packed_data = None
        ship.cumulative_time = []
        ship.total_duration_sec = 0.0
        self.check_sim_ready()

        # 시뮬레이션 실행 중이면 리셋
        if self.sim_panel and self.sim_panel.worker and self.sim_panel.worker.running:
            self.sim_panel.reset_state()

    def check_sim_ready(self):
        """시뮬레이션 준비 상태를 확인합니다."""
        ready = True
        if not current_project.ships: ready = False

        # 모든 선박이 포인트를 가지고 있는지 확인
        for s in current_project.ships:
            if not s.raw_points:
                ready = False
                break

        txt = self.obj_combo.currentText()
        if "[Ship]" in txt:
            sid = self.obj_combo.currentData()
            s = current_project.get_ship_by_idx(sid)

    def delete_points(self, s_idx, p_idx):
        """
        선박의 특정 포인트를 삭제합니다.

        매개변수:
            s_idx: 선박 인덱스
            p_idx: 포인트 인덱스
        """
        if msg.show_question(self, "Delete", "Delete this point and all subsequent points?") == msg.StandardButton.No:
            return

        ship = current_project.get_ship_by_idx(s_idx)
        if ship:
            ship.raw_points.pop(p_idx)

            # 인덱스 재조정
            new_speeds = {}
            new_notes = {}
            for k, v in ship.raw_speeds.items():
                if k < p_idx: new_speeds[k] = v
                elif k > p_idx: new_speeds[k-1] = v
            ship.raw_speeds = new_speeds

            for k, v in ship.raw_notes.items():
                if k < p_idx: new_notes[k] = v
                elif k > p_idx: new_notes[k-1] = v
            ship.raw_notes = new_notes

            self.handle_path_change(ship)
            self.set_map_mode("SELECT")

    def add_area_dialog(self):
        """영역 추가 모드를 시작합니다."""
        self.set_map_mode("ADD_AREA")

    def finish_add_area(self, raw_pts):
        """
        영역 추가를 완료합니다.

        매개변수:
            raw_pts: 드로잉된 원시 포인트 목록
        """
        d = QDialog(self)
        d.setWindowTitle("New Area")
        f = QFormLayout(d)

        name_edit = QLineEdit(f"Area-{len(current_project.areas)}")
        spin = QSpinBox()
        spin.setValue(12)
        spin.setMinimum(3)

        c_btn = QPushButton()
        c_btn.setStyleSheet("background-color: #808080")
        color_val = ["#808080"]
        def pk():
            c = QColorDialog.getColor()
            if c.isValid():
                color_val[0] = c.name()
                c_btn.setStyleSheet(f"background-color: {c.name()}")
        c_btn.clicked.connect(pk)

        f.addRow("Name:", name_edit)
        f.addRow("Vertices:", spin)
        f.addRow("Color:", c_btn)

        bb = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok)
        bb.accepted.connect(d.accept)
        f.addWidget(bb)
        d.exec()

        # 지정된 꼭짓점 수로 리샘플링
        resampled = resample_polygon_equidistant(raw_pts, spin.value())

        sid = len(current_project.areas)
        st = Area(id=sid, name=name_edit.text() or f"Area-{sid}", geometry=resampled, color=color_val[0])
        current_project.areas.append(st)
        self.update_obj_combo()
        self.set_map_mode("SELECT")
        self.data_changed.emit()

    def change_area_color(self):
        """선택된 영역의 색상을 변경합니다."""
        txt = self.obj_combo.currentText()
        if "[Area]" not in txt:
            msg.show_information(self, "Info", "Please select an Area.")
            return
        sid = self.obj_combo.currentData()
        area = current_project.get_area_by_id(sid)
        if area:
            c = QColorDialog.getColor(QColor(area.color))
            if c.isValid():
                area.color = c.name()
                self.redraw_map()
                self.data_changed.emit()

    def translate_path_dialog(self, ship_idx):
        """
        경로 이동 다이얼로그를 표시합니다.

        매개변수:
            ship_idx: 선박 인덱스
        """
        ship = current_project.get_ship_by_idx(ship_idx)
        if not ship: return

        d = QDialog(self)
        d.setWindowTitle("Translate Path")
        f = QFormLayout(d)
        e_spin = QDoubleSpinBox(); e_spin.setRange(-100000, 100000); e_spin.setSuffix(" deg(Lon)")
        n_spin = QDoubleSpinBox(); n_spin.setRange(-100000, 100000); n_spin.setSuffix(" deg(Lat)")
        f.addRow("Delta Longitude:", e_spin)
        f.addRow("Delta Latitude:", n_spin)
        bb = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        bb.accepted.connect(d.accept)
        bb.rejected.connect(d.reject)
        f.addWidget(bb)

        if d.exec() == QDialog.DialogCode.Accepted:
            dlon, dlat = e_spin.value(), n_spin.value()
            mi = current_project.map_info

            scale = mi.pixels_per_degree
            dpx = dlon * scale
            dpy = -dlat * scale

            new_raw = []
            for (px, py) in ship.raw_points:
                new_raw.append((px + dpx, py + dpy))
            ship.raw_points = new_raw
            self.handle_path_change(ship)

    def new_project(self):
        """새 프로젝트를 생성합니다."""
        dlg = NewProjectDialog(self)
        if dlg.exec() == QDialog.DialogCode.Accepted:
            title, path, lat, lon, start_t = dlg.get_data()
            current_project.reset()
            current_project.project_name = title
            current_project.project_path = path
            current_project.start_time = start_t
            current_project.map_info.center_lat = lat
            current_project.map_info.center_lon = lon
            current_project.seed = int(datetime.datetime.now().timestamp())
            current_project.settings.path_thickness = 3
            current_project.settings.traveled_path_thickness = 3

            self.save_project()
            self.update_ui_state(True)
            self.update_info_strip()
            self.update_obj_combo()
            self.redraw_map()
            self.check_sim_ready()

            # 지도 중심 이동
            px, py = coords_to_pixel(lat, lon, current_project.map_info)
            self.view.centerOn(px, py)

            # 초기 줌 설정
            self.view.resetTransform()
            self.view.scale(0.01, 0.01)

    def open_project(self):
        """프로젝트 파일을 열어 로드합니다."""
        fpath, _ = QFileDialog.getOpenFileName(self, "Open Project", "", "JSON Files (*.json)")
        if not fpath: return

        try:
            with open(fpath, encoding='utf-8') as f:
                data = json.load(f)

            p = current_project
            p.reset()
            p.project_path = os.path.dirname(fpath)
            p.project_name = data.get("name", "Untitled")
            p.start_time = datetime.datetime.fromisoformat(data["start_time"])
            p.unit_time = data.get("unit_time", 1.0)
            p.seed = data.get("seed", int(datetime.datetime.now().timestamp()))

            # 맵 정보 로드
            m = data.get("map_info", {})
            p.map_info.center_lat = m.get("center_lat", 35.0)
            p.map_info.center_lon = m.get("center_lon", 129.0)

            # 설정 로드
            s = data.get("settings", {})
            p.settings.own_ship_idx = s.get("own_ship_idx", 0)
            p.settings.include_gp_signal = s.get("include_gp_signal", False)
            p.settings.jitter_enabled = s.get("jitter_enabled", False)
            p.settings.own_color = s.get("own_color", "#008000")
            p.settings.target_color = s.get("target_color", "#FF0000")
            p.settings.select_color = s.get("select_color", "#0000FF")
            p.settings.btn_speed_color = s.get("btn_speed_color", "#FF9800")
            p.settings.btn_sim_color = s.get("btn_sim_color", "#2196F3")
            p.settings.btn_rtg_color = s.get("btn_rtg_color", "#E57373")
            p.settings.random_color = s.get("random_color", "#FFCDD2")
            p.settings.speed_variance = s.get("speed_variance", 1.0)
            p.settings.path_thickness = s.get("path_thickness", 3)
            p.settings.traveled_path_thickness = s.get("traveled_path_thickness", 3)
            p.settings.dropout_probs = s.get("dropout_probs", {})
            p.settings.theme_mode = s.get("theme_mode", "System")
            p.settings.mask_color = s.get("mask_color", "#404040")
            p.settings.ais_fragment_delivery_prob = s.get("ais_fragment_delivery_prob", 0.95)

            # 선박 로드
            for s_data in data.get("ships", []):
                ship = ShipData(s_data["ship_index"], s_data["ship_name"])
                ship.note = s_data.get("note", "")
                ship.mmsi = s_data.get("mmsi", 0)

                pts = []
                spds = {}
                for i, pt in enumerate(s_data.get("control_points", [])):
                    px, py = coords_to_pixel(pt["lat"], pt["lon"], p.map_info)
                    pts.append((px, py))
                    spds[i] = pt.get("speed", 5.0)

                ship.raw_points = pts
                ship.raw_speeds = spds

                # 선박 클래스 및 제원 로드 (이전 버전 호환)
                ship.ship_class = s_data.get("ship_class", "CONTAINER")
                ship.length_m = s_data.get("length_m", 300.0)
                ship.beam_m = s_data.get("beam_m", 40.0)
                ship.draft_m = s_data.get("draft_m", 15.0)
                ship.air_draft_m = s_data.get("air_draft_m", 60.0)
                ship.height_m = s_data.get("height_m", ship.air_draft_m * 0.5)

                # Single Point + Heading 모드 로드
                ship.initial_heading = s_data.get("initial_heading", None)

                ship.total_duration_sec = 0.0
                ship.packed_data = None

                # 경로 리샘플링
                if len(ship.raw_points) < 2:
                    ship.resampled_points = list(ship.raw_points)
                else:
                    ship.resampled_points = resample_polyline_numpy(ship.raw_points, 60)

                p.ships.append(ship)

            # 영역 로드
            for a_data in data.get("areas", []):
                pts = []
                for pt in a_data.get("polygon_points", []):
                    px, py = coords_to_pixel(pt["lat"], pt["lon"], p.map_info)
                    pts.append((px, py))

                area = Area(
                    id=a_data["area_id"],
                    name=a_data["area_name"],
                    geometry=pts,
                    color=a_data.get("area_color", "#808080")
                )
                area.note = a_data.get("note", "")
                p.areas.append(area)

            p.events = []

            # Event 폴더에서 이벤트 로드 (새 표준)
            event_dir = os.path.join(p.project_path, "Event")
            if os.path.exists(event_dir):
                for fname in os.listdir(event_dir):
                    if fname.endswith(".json"):
                        try:
                            with open(os.path.join(event_dir, fname), 'r', encoding='utf-8') as f:
                                e_data = json.load(f)
                                evt = EventTrigger(
                                    id=e_data["id"],
                                    name=e_data["name"],
                                    enabled=e_data.get("enabled", True),
                                    trigger_type=e_data["trigger_type"],
                                    condition_value=e_data["condition_value"],
                                    action_type=e_data["action_type"],
                                    target_ship_idx=e_data["target_ship_idx"],
                                    action_value=e_data["action_value"],
                                    is_relative_to_end=e_data.get("is_relative_to_end", False),
                                    reference_ship_idx=e_data.get("reference_ship_idx", -1)
                                )
                                evt.action_option = e_data.get("action_option", "")  # type: ignore
                                p.events.append(evt)
                        except: pass

            # 폴백: Event 폴더가 비어있거나 없으면 project.json에서 로드 (레거시)
            if not p.events and data.get("events"):
                for e_data in data.get("events", []):
                    evt = EventTrigger(
                        id=e_data["id"],
                        name=e_data["name"],
                        enabled=e_data.get("enabled", True),
                        trigger_type=e_data["trigger_type"],
                        condition_value=e_data["condition_value"],
                        action_type=e_data["action_type"],
                        target_ship_idx=e_data["target_ship_idx"],
                        action_value=e_data["action_value"],
                        is_relative_to_end=e_data.get("is_relative_to_end", False),
                        reference_ship_idx=e_data.get("reference_ship_idx", -1)
                    )
                    evt.action_option = e_data.get("action_option", "")  # type: ignore
                    p.events.append(evt)

            # Scenario 폴더에서 시나리오 로드
            scen_dir = os.path.join(p.project_path, "Scenario")
            app_state.loaded_scenarios = []
            app_state.current_scenario = None
            if os.path.exists(scen_dir):
                for fname in os.listdir(scen_dir):
                    if fname.endswith(".scenario.json") or fname.endswith(".json"):
                        try:
                            from app.core.models.scenario import Scenario
                            scen = Scenario.load_from_file(os.path.join(scen_dir, fname))
                            app_state.loaded_scenarios.append(scen)

                            # 시나리오 이벤트를 레지스트리에 동기화
                            for s_evt in scen.events:
                                existing = next((e for e in p.events if e.id == s_evt.id), None)
                                if existing:
                                    # 시나리오 버전으로 덮어쓰기
                                    existing.name = s_evt.name
                                    existing.enabled = s_evt.enabled
                                    existing.trigger_type = s_evt.trigger_type
                                    existing.condition_value = s_evt.condition_value
                                    existing.action_type = s_evt.action_type
                                    existing.target_ship_idx = s_evt.target_ship_idx
                                    existing.reference_ship_idx = s_evt.reference_ship_idx
                                    existing.action_value = s_evt.action_value
                                    existing.is_relative_to_end = s_evt.is_relative_to_end
                                    existing.action_option = getattr(s_evt, 'action_option', "")  # type: ignore
                        except: pass

            # UI 업데이트
            self.update_ui_state(True)
            self.update_info_strip()
            self.update_obj_combo()
            self.apply_theme()
            self.update_stylesheets()
            self.event_panel.refresh_all()
            self.redraw_map()
            self.check_sim_ready()

        except Exception as e:
            traceback.print_exc()
            msg.show_critical(self, "Error Loading Project", str(e))

    def save_project(self):
        """현재 프로젝트를 파일로 저장합니다."""
        p = current_project
        if not p.project_path: return

        # 폴더 생성
        os.makedirs(os.path.join(p.project_path, "Object"), exist_ok=True)
        os.makedirs(os.path.join(p.project_path, "Scenario"), exist_ok=True)
        os.makedirs(os.path.join(p.project_path, "Event"), exist_ok=True)

        # 선박 저장
        ships_list = []
        for s in p.ships:
            control_points = []
            for i, (px, py) in enumerate(s.raw_points):
                _, _, lat, lon = pixel_to_coords(px, py, p.map_info)
                spd = s.raw_speeds.get(i, 5.0)
                control_points.append({
                    "lat": lat,
                    "lon": lon,
                    "speed": spd
                })

            ship_data = {
                "ship_index": s.idx,
                "ship_name": s.name,
                "mmsi": s.mmsi,
                "note": s.note,
                "control_points": control_points,
                "ship_class": s.ship_class,
                "length_m": s.length_m,
                "beam_m": s.beam_m,
                "draft_m": s.draft_m,
                "air_draft_m": s.air_draft_m,
                "height_m": s.height_m
            }
            # Single Point + Heading 모드 저장
            if s.initial_heading is not None:
                ship_data["initial_heading"] = s.initial_heading
            ships_list.append(ship_data)

            # 개별 선박 파일 저장
            ship_fname = f"Ship_{s.idx}.json"
            with open(os.path.join(p.project_path, "Object", ship_fname), 'w', encoding='utf-8') as f:
                json.dump(ships_list[-1], f, indent=4)

        # 영역 저장
        areas_list = []
        for a in p.areas:
            poly_pts = []
            for (px, py) in a.geometry:
                _, _, lat, lon = pixel_to_coords(px, py, p.map_info)
                poly_pts.append({"lat": lat, "lon": lon})

            areas_list.append({
                "area_id": a.id,
                "area_name": a.name,
                "area_color": a.color,
                "note": a.note,
                "polygon_points": poly_pts
            })

            # 개별 영역 파일 저장
            area_fname = f"Area_{a.id}.json"
            with open(os.path.join(p.project_path, "Object", area_fname), 'w', encoding='utf-8') as f:
                json.dump(areas_list[-1], f, indent=4)

        # 이벤트 저장 (Event 폴더에 개별 파일)
        events_list = []
        for e in p.events:
            e_data = {
                "id": e.id,
                "name": e.name,
                "enabled": e.enabled,
                "trigger_type": e.trigger_type,
                "condition_value": e.condition_value,
                "action_type": e.action_type,
                "target_ship_idx": e.target_ship_idx,
                "action_value": e.action_value,
                "is_relative_to_end": e.is_relative_to_end,
                "reference_ship_idx": e.reference_ship_idx,
                "action_option": getattr(e, 'action_option', "")  # type: ignore
            }

            fname = sanitize_filename(e.name) + ".json"
            with open(os.path.join(p.project_path, "Event", fname), 'w', encoding='utf-8') as f:
                json.dump(e_data, f, indent=4)

        # 시나리오 저장
        if app_state.loaded_scenarios:
            for scen in app_state.loaded_scenarios:
                fname = sanitize_filename(scen.name) + ".scenario.json"
                scen.save_to_file(os.path.join(p.project_path, "Scenario", fname))

        # 메인 프로젝트 파일 저장
        data = {
            "name": p.project_name,
            "project_path": p.project_path,
            "start_time": p.start_time.isoformat(),
            "unit_time": p.unit_time,
            "seed": p.seed,
            "map_info": {
                "center_lat": p.map_info.center_lat,
                "center_lon": p.map_info.center_lon
            },
            "settings": {
                "own_ship_idx": p.settings.own_ship_idx,
                "include_gp_signal": p.settings.include_gp_signal,
                "jitter_enabled": p.settings.jitter_enabled,
                "own_color": p.settings.own_color,
                "target_color": p.settings.target_color,
                "select_color": p.settings.select_color,
                "btn_speed_color": p.settings.btn_speed_color,
                "btn_sim_color": p.settings.btn_sim_color,
                "btn_rtg_color": p.settings.btn_rtg_color,
                "random_color": p.settings.random_color,
                "speed_variance": p.settings.speed_variance,
                "path_thickness": p.settings.path_thickness,
                "traveled_path_thickness": p.settings.traveled_path_thickness,
                "mask_color": p.settings.mask_color,
                "dropout_probs": p.settings.dropout_probs,
                "theme_mode": p.settings.theme_mode,
                "ais_fragment_delivery_prob": p.settings.ais_fragment_delivery_prob
            },
            "ships": ships_list,
            "areas": areas_list,
            "events": []  # 이벤트는 Event 폴더에 저장됨
        }

        fname = f"{sanitize_filename(p.project_name)}.json"
        full_path = os.path.join(p.project_path, fname)

        try:
            with open(full_path, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=4)
            msg.show_information(self, "Saved", f"Project Saved to:\n{fname}")
        except Exception as e:
            msg.show_critical(self, "Error Saving", str(e))

    def clear_cache(self):
        """프로젝트 폴더의 __pycache__ 디렉토리를 삭제합니다."""
        if msg.show_question(self, "Clear Cache", "Delete all __pycache__ folders in the project?\n(They will be regenerated automatically)") != msg.StandardButton.Yes:
            return

        base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        count = 0
        for root, dirs, files in os.walk(base_dir):
            if "__pycache__" in dirs:
                try:
                    shutil.rmtree(os.path.join(root, "__pycache__"))
                    count += 1
                except: pass
        msg.show_information(self, "Done", f"Deleted {count} cache directories.")

    def add_ship_dialog(self):
        """새 선박 추가 다이얼로그를 표시합니다."""
        idx = 0
        existing = [s.idx for s in current_project.ships]
        while idx in existing: idx += 1

        dlg = AddShipDialog(self, default_name=f"Ship-{idx}")
        if dlg.exec() != QDialog.DialogCode.Accepted:
            return

        data = dlg.get_data()
        s = ShipData(idx, data["name"])

        # 선박 클래스 및 제원 설정
        s.ship_class = data["ship_class"]
        s.length_m = data["length_m"]
        s.beam_m = data["beam_m"]
        s.draft_m = data["draft_m"]
        s.air_draft_m = data["air_draft_m"]
        s.height_m = data["air_draft_m"] * 0.5  # 가시 높이 = 공기흘수의 절반

        # 고유 MMSI 생성
        existing_mmsis = {ship.mmsi for ship in current_project.ships}
        while True:
            candidate = random.randint(100000000, 999999999)
            if candidate not in existing_mmsis:
                s.mmsi = candidate
                break

        current_project.ships.append(s)
        self.update_obj_combo()
        self.check_sim_ready()
        self.data_changed.emit()

    def delete_object(self):
        """선택된 객체(선박 또는 영역)를 삭제합니다."""
        txt = self.obj_combo.currentText()
        sid = self.obj_combo.currentData()

        delete_msg = "Are you sure you want to delete this Object?"
        if "[Ship]" in txt:
            delete_msg += "\nPath, Speed, Duration data will be lost."
        if msg.show_question(self, "Delete Object", delete_msg) == msg.StandardButton.No:
            return

        if "[Ship]" in txt:
             current_project.ships = [s for s in current_project.ships if s.idx != sid]
        elif "[Area]" in txt:
             current_project.areas = [s for s in current_project.areas if s.id != sid]

        self.update_obj_combo()
        self.redraw_map()
        self.check_sim_ready()
        self.data_changed.emit()

    def on_tab_changed(self, index):
        """
        탭이 변경되었을 때 호출됩니다.

        각 탭에 맞는 UI 업데이트를 수행합니다.

        매개변수:
            index: 선택된 탭 인덱스
        """
        w = self.tabs.widget(index)

        # 테마 확인
        s = current_project.settings
        mode = s.theme_mode
        is_dark = False
        if mode == "Dark": is_dark = True
        elif mode == "Light": is_dark = False
        else:
            if QApplication.styleHints().colorScheme() == Qt.ColorScheme.Dark: is_dark = True  # type: ignore
        input_bg = "#252525" if is_dark else "#ffffff"
        sel_color = input_bg

        if w == self.map_editor_widget:
            self.redraw_map()
            sel_color = input_bg
        elif w == self.sim_panel:
            self.sim_panel.draw_static_map()
            self.sim_panel.refresh_tables()
        elif w == self.event_panel:
            self.event_panel.refresh_all()
        elif w == self.scenario_panel:
            self.scenario_panel.refresh_ui()

    def update_sim_tab_state(self, state):
        """
        시뮬레이션 탭 상태를 업데이트합니다.

        시뮬레이션 상태에 따라 탭 텍스트와 아이콘을 변경합니다.

        매개변수:
            state: 상태 문자열 ("PLAY", "PAUSE", "STOP")
        """
        idx = self.tabs.indexOf(self.sim_panel)
        if idx == -1: return

        def get_white_icon(shape):
            """흰색 아이콘을 생성합니다."""
            pix = QPixmap(16, 16)
            pix.fill(Qt.GlobalColor.transparent)
            p = QPainter(pix)
            p.setRenderHint(QPainter.RenderHint.Antialiasing)
            p.setBrush(QBrush(Qt.GlobalColor.white))
            p.setPen(Qt.PenStyle.NoPen)

            if shape == 'play':
                path = QPainterPath()
                path.moveTo(3, 2)
                path.lineTo(14, 8)
                path.lineTo(3, 14)
                path.closeSubpath()
                p.drawPath(path)
            elif shape == 'pause':
                p.drawRect(3, 2, 4, 12)
                p.drawRect(9, 2, 4, 12)

            p.end()
            return QIcon(pix)

        if state == "PLAY":
            self.tabs.setTabText(idx, "   Simulation (Running)")
            self.tabs.setTabIcon(idx, get_white_icon('play'))
        elif state == "PAUSE":
            self.tabs.setTabText(idx, "   Simulation (Paused)")
            self.tabs.setTabIcon(idx, get_white_icon('pause'))
        elif state == "STOP":
            self.tabs.setTabText(idx, "   Simulation (Stopped)")
            # 빨간색 정지 아이콘 생성
            pix = QPixmap(16, 16)
            pix.fill(Qt.GlobalColor.transparent)
            p = QPainter(pix)
            p.setBrush(QBrush(Qt.GlobalColor.red))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawRect(2, 2, 12, 12)
            p.end()
            self.tabs.setTabIcon(idx, QIcon(pix))

    def open_settings(self):
        """설정 다이얼로그를 열고 변경사항을 적용합니다."""
        dlg = SettingsDialog(self)
        if dlg.exec():
            dlg.apply_changes()
            self.apply_theme()
            self.update_stylesheets()
            self.update_obj_combo()
            self.redraw_map()
            self.data_changed.emit()

    def open_user_guide(self):
        """사용자 가이드 다이얼로그를 표시합니다."""
        dlg = HelpDialog(self)
        dlg.exec()

    def on_data_changed(self):
        """데이터가 변경되었을 때 관련 패널들을 업데이트합니다."""
        if self.sim_panel and self.sim_panel.isVisible():
            self.sim_panel.draw_static_map()
            self.sim_panel.refresh_tables()
        self.event_panel.refresh_all()
        self.scenario_panel.refresh_ui()

    def update_ui_state(self, enabled):
        """
        UI 상태를 업데이트합니다.

        매개변수:
            enabled: UI 활성화 여부
        """
        self.view.setEnabled(enabled)
        self.obj_combo.setEnabled(enabled)

    def closeEvent(self, event):
        """
        윈도우 닫기 이벤트 핸들러입니다.

        프로젝트 저장 여부를 확인하고 시뮬레이션을 정리합니다.

        매개변수:
            event: 닫기 이벤트 객체
        """
        reply = msg.show_question(
            self,
            "Save Project",
            "Do you want to save the project?",
            msg.StandardButton.Yes | msg.StandardButton.No | msg.StandardButton.Cancel
        )

        if reply == msg.StandardButton.Yes:
            self.sim_panel.cleanup()
            self.save_project()
            event.accept()
        elif reply == msg.StandardButton.No:
            self.sim_panel.cleanup()
            event.accept()
        else:
            event.ignore()
