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
    QStackedWidget, QGroupBox, QFrame, QSplitter, QScrollArea, QMessageBox, QFileDialog,
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
from app.core.geometry import coords_to_pixel, pixel_to_coords, resample_polyline_numpy, resample_polygon_equidistant, normalize_lon
from app.ui.map.map_view import MapView
from app.ui.dialogs.new_project_dialog import NewProjectDialog
from app.ui.dialogs.settings_dialog import SettingsDialog
from app.ui.dialogs.help_dialog import HelpDialog
from app.ui.panels.simulation_panel import SimulationPanel
from app.ui.panels.event_panel import EventScriptPanel
from app.ui.panels.scenario_panel import ScenarioPanel
import app.core.state as app_state

class ColoredTabBar(QTabBar):
    def tabSizeHint(self, index):
        size = super().tabSizeHint(index)
        if self.tabText(index) == "":
            total_width = self.width()
            used_width = 0
            for i in range(self.count()):
                if i != index:
                    used_width += super().tabSizeHint(i).width()
            space = total_width - used_width
            if space < 0: space = 0
            return QSize(space, size.height())
        return size

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.updateGeometry()

    def paintEvent(self, event):
        painter = QPainter(self)
        option = QStyleOptionTab()
        
        s = current_project.settings
        mode = s.theme_mode
        is_dark = False
        if mode == "Dark": is_dark = True
        elif mode == "Light": is_dark = False
        else: # System
            if QApplication.styleHints().colorScheme() == Qt.ColorScheme.Dark: is_dark = True

        bg_color = QColor(Qt.GlobalColor.black) if is_dark else QColor(Qt.GlobalColor.white)
        text_color = QColor(Qt.GlobalColor.white) if is_dark else QColor(Qt.GlobalColor.black)

        for i in range(self.count()):
            if self.tabText(i) == "": continue
            
            self.initStyleOption(option, i)
            
            # Draw background
            painter.fillRect(option.rect, bg_color)
            
            # Draw Text
            painter.setPen(text_color)
            font = painter.font()
            painter.setFont(font)
            painter.drawText(option.rect, Qt.AlignmentFlag.AlignCenter, self.tabText(i))
            
            # Draw Separator
            if i < self.count() - 1 and self.tabText(i+1) != "":
                painter.setPen(QPen(QColor(200, 200, 200), 1))
                painter.drawLine(option.rect.topRight() + QPoint(0, 5), option.rect.bottomRight() - QPoint(0, 5))
            
            # Draw Icon if exists (Simple centered-left drawing)
            if not self.tabIcon(i).isNull():
                self.tabIcon(i).paint(painter, option.rect.adjusted(5, 0, -5, 0), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)

class MainWindow(QMainWindow):
    data_changed = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle(APP_NAME)
        self.resize(*DEFAULT_WINDOW_SIZE)
        self.apply_theme()
        self.update_stylesheets()
        
        self.sim_panel = SimulationPanel(self)
        self.event_panel = EventScriptPanel(self)
        self.scenario_panel = ScenarioPanel(self)
        self.scenario_panel.data_changed.connect(self.on_data_changed)
        
        self.init_ui()
        self.update_ui_state(False)
        self.update_info_strip()
        
        self.data_changed.connect(self.on_data_changed)
        
        # Initialize tab state
        self.update_sim_tab_state("STOP")

    def format_duration(self, seconds):
        d = int(seconds // 86400)
        h = int((seconds % 86400) // 3600)
        m = int((seconds % 3600) // 60)
        s = seconds % 60
        if d > 0: return f"{d}d {h}h {m}m {s:.1f}s"
        if h > 0: return f"{h}h {m}m {s:.1f}s"
        if m > 0: return f"{m}m {s:.1f}s"
        return f"{s:.1f}s"

    def apply_theme(self):
        mode = current_project.settings.theme_mode
        is_dark = False
        if mode == "Dark": is_dark = True
        elif mode == "Light": is_dark = False
        else: # System
            if QApplication.styleHints().colorScheme() == Qt.ColorScheme.Dark: is_dark = True

        p = QApplication.palette()
        
        if is_dark:
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
        s = current_project.settings
        mode = s.theme_mode
        is_dark = False
        if mode == "Dark": is_dark = True
        elif mode == "Light": is_dark = False
        else: # System
            if QApplication.styleHints().colorScheme() == Qt.ColorScheme.Dark: is_dark = True
        
        bg_color = "#353535" if is_dark else "#ffffff"
        text_color = "#ffffff" if is_dark else "black"
        input_bg = "#252525" if is_dark else "#ffffff"
        btn_bg = "#454545" if is_dark else "#e0e0e0"
        border_color = "#ffffff" if is_dark else "#000000"
        toolbar_bg = "#2d2d2d" if is_dark else "#f0f0f0"
        
        # Disabled state colors
        disabled_bg = "#2a2a2a" if is_dark else "#f5f5f5"
        disabled_text = "#555555" if is_dark else "#a0a0a0"
        disabled_border = "#333333" if is_dark else "#d0d0d0"
        
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
        """)
        
        if hasattr(self, 'lbl_coords'):
            self.lbl_coords.setStyleSheet(f"padding: 2px 5px; color: {text_color};")

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_v = QVBoxLayout(central)
        main_v.setContentsMargins(0,0,0,0)
        main_v.setSpacing(0)
        
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

        top_tb.addWidget(QLabel(" File: "))
        top_tb.addAction("New Project", self.new_project)
        top_tb.addAction("Open Project", self.open_project)
        top_tb.addAction("Save Project", self.save_project)
        top_tb.addAction("Clear Cache", self.clear_cache)
        top_tb.addAction("Exit", self.close)
        top_tb.addSeparator()

        top_tb.addWidget(QLabel(" Tools: "))
        
        top_tb.addAction("User Guide", self.open_user_guide)
        top_tb.addAction("Settings", self.open_settings)
        top_tb.addSeparator()
        
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
        
        # --- Tabs Setup ---
        self.tabs = QTabWidget()
        self.tabs.setTabPosition(QTabWidget.TabPosition.South)
        self.tabs.setTabBar(ColoredTabBar(self.tabs))
        self.tabs.setDocumentMode(True)
        self.tabs.tabBar().setExpanding(False)
        
        # Map Editor Tab
        self.map_editor_widget = QWidget()
        main_h = QHBoxLayout(self.map_editor_widget)
        main_h.setContentsMargins(5,5,5,5)
        main_h.setSpacing(15) 
        
        left_widget = QWidget()
        left_v = QVBoxLayout(left_widget)
        left_v.setContentsMargins(0,0,0,0)
        
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
        
        tb.addWidget(QLabel("Initialize: "))
        a_sel = tb.addAction("Select")
        a_sel.triggered.connect(lambda: self.set_map_mode("SELECT"))
        tb.addSeparator()
        
        tb.addWidget(QLabel("Object:"))
        tb.addAction("Add Ship", self.add_ship_dialog)
        tb.addAction("Add Area", self.add_area_dialog)
        tb.addAction("Change Area Color", self.change_area_color)
        tb.addAction("Delete Selected", self.delete_object)
        tb.addSeparator()
        
        tb.addWidget(QLabel("Point:"))
        tb.addAction("Add", self.req_add_point)
        tb.addAction("Move", lambda: self.set_map_mode("MOVE_POINT"))
        tb.addAction("Delete", lambda: self.set_map_mode("DELETE_POINT"))
        
        a_del_chk = tb.addAction("Delete Checked")
        a_del_chk.triggered.connect(self.delete_checked_points)
        a_del_chk.setToolTip("Delete all points checked in the table")
        
        self.scene = QGraphicsScene()
        self.view = MapView(self.scene, self)
        left_v.addWidget(self.view)
        
        main_h.addWidget(left_widget, 3)
        
        right_widget = QWidget()
        right_v = QVBoxLayout(right_widget)
        right_v.setContentsMargins(0,0,0,0)
        
        lbl_obj = QLabel("Select Object"); lbl_obj.setObjectName("boldLbl")
        right_v.addWidget(lbl_obj)
        self.obj_combo = QComboBox()
        self.obj_combo.currentIndexChanged.connect(self.on_obj_selected)
        right_v.addWidget(self.obj_combo)
        
        self.lbl_dur = QLabel("Duration: -")
        self.lbl_dur.setObjectName("boldLbl")
        right_v.addWidget(self.lbl_dur)
        
        self.data_table = QTableWidget()
        self.data_table.itemChanged.connect(self.on_table_changed)
        h = self.data_table.horizontalHeader()
        h.setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        right_v.addWidget(self.data_table)
        
        
        main_h.addWidget(right_widget, 1)
        
        self.tabs.addTab(self.map_editor_widget, "Path")
        self.tabs.addTab(self.event_panel, "Event")
        self.tabs.addTab(self.scenario_panel, "Scenario")
        
        # Spacer
        self.spacer_widget = QWidget()
        self.tabs.addTab(self.spacer_widget, "") 
        self.tabs.setTabEnabled(3, False)
        
        self.tabs.addTab(self.sim_panel, "Simulation")
        
        main_v.addWidget(self.tabs)
        
        self.tabs.currentChanged.connect(self.on_tab_changed)
        self.sim_panel.state_changed.connect(self.update_sim_tab_state)
        
        self.lbl_coords = QLabel("Lat: 0.00000 Lon: 0.00000")
        self.lbl_coords.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        left_v.addWidget(self.lbl_coords)
        
        self.view.coord_changed.connect(self.update_status)
        self.update_stylesheets()

        
    def set_map_mode(self, mode):
        self.view.mode = mode
        self.view.update_cursor()
        # self.status.showMessage(f"Mode: {mode}")
        self.redraw_map()

    def update_status(self, px, py):
        mi = current_project.map_info
        _, _, lat, lon = pixel_to_coords(px, py, mi)
        norm_lon = normalize_lon(lon)
        self.lbl_coords.setText(f"Lat: {lat:.5f} Lon: {norm_lon:.5f}")

    def update_info_strip(self):
        p = current_project
        self.lbl_proj_name.setText(p.project_name)
        self.lbl_proj_time.setText(p.start_time.strftime('%Y-%m-%d %H:%M:%S'))

    def update_obj_combo(self):
        self.obj_combo.blockSignals(True)
        self.obj_combo.clear()
        
        # Sorting Rules:
        # 1) Own Ship (Index 0 or settings.own_ship_idx)
        # 2) Manual Targets (idx < 1000)
        # 3) Random Targets (idx >= 1000)
        
        own_idx = current_project.settings.own_ship_idx
        own_ship = None
        manual_targets = []
        random_targets = []
        
        for s in current_project.ships:
            if s.idx == own_idx: own_ship = s
            elif s.idx >= 1000: random_targets.append(s)
            else: manual_targets.append(s)
            
        manual_targets.sort(key=lambda x: x.idx)
        random_targets.sort(key=lambda x: x.idx)
        
        sorted_ships = ([own_ship] if own_ship else []) + manual_targets + random_targets
        
        for s in sorted_ships:
            tag = "OwnShip" if s.idx == own_idx else f"Target{s.idx}"
            self.obj_combo.addItem(f"[Ship] {s.name} (MMSI: {s.mmsi}) ({tag})", s.idx)
            
        for st in current_project.areas:
            self.obj_combo.addItem(f"[Area] {st.name}", st.id)
            
        self.obj_combo.blockSignals(False)
        self.on_obj_selected()

    def on_obj_selected(self):
        idx = self.obj_combo.currentIndex()
        if idx < 0: 
            self.data_table.setRowCount(0)
            self.lbl_dur.setText("Duration: -")
            return
            
        txt = self.obj_combo.currentText()
        if "[Ship]" in txt:
            sid = self.obj_combo.currentData()
            # self.btn_spd.setEnabled(True)
            self.show_ship_table(sid)
        elif "[Area]" in txt:
            sid = self.obj_combo.currentData()
            # self.btn_spd.setEnabled(False) 
            self.show_struct_table(sid)
            self.lbl_dur.setText("Duration: -")
            
        self.redraw_map()

    def show_ship_table(self, idx):
        ship = current_project.get_ship_by_idx(idx)
        if not ship: return
        dur_str = "-" # Duration is dynamic now
        self.lbl_dur.setText(f"Duration: {dur_str}")
        
        self.data_table.blockSignals(True)
        self.data_table.setColumnCount(6)
        self.data_table.setHorizontalHeaderLabels(["chk", "idx", "Lat", "Lon", "Speed(kn)", "MMSI"])
        self.data_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        self.data_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        self.data_table.setRowCount(len(ship.raw_points))
        
        mi = current_project.map_info
        for i, (px, py) in enumerate(ship.raw_points):
            _, _, lat, lon = pixel_to_coords(px, py, mi)
            spd = ship.raw_speeds.get(i, 0.0)
            
            chk_item = QTableWidgetItem()
            chk_item.setFlags(Qt.ItemFlag.ItemIsUserCheckable | Qt.ItemFlag.ItemIsEnabled)
            chk_item.setCheckState(Qt.CheckState.Unchecked)
            self.data_table.setItem(i, 0, chk_item)

            self.data_table.setItem(i, 1, QTableWidgetItem(str(i)))
            self.data_table.setItem(i, 2, QTableWidgetItem(f"{lat:.6f}"))
            self.data_table.setItem(i, 3, QTableWidgetItem(f"{normalize_lon(lon):.6f}"))
            self.data_table.setItem(i, 4, QTableWidgetItem(f"{spd:.1f}"))
            self.data_table.setItem(i, 5, QTableWidgetItem(str(ship.mmsi)))
        self.data_table.blockSignals(False)

    def show_struct_table(self, sid):
        st = current_project.get_area_by_id(sid)
        if not st: return
        
        self.data_table.blockSignals(True)
        self.data_table.setColumnCount(3)
        self.data_table.setHorizontalHeaderLabels(["Vtx", "Lat", "Lon"])
        self.data_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        self.data_table.setRowCount(len(st.geometry))
        
        mi = current_project.map_info
        for i, (px, py) in enumerate(st.geometry):
            _, _, lat, lon = pixel_to_coords(px, py, mi)
            self.data_table.setItem(i, 0, QTableWidgetItem(str(i)))
            self.data_table.setItem(i, 1, QTableWidgetItem(f"{lat:.6f}"))
            self.data_table.setItem(i, 2, QTableWidgetItem(f"{normalize_lon(lon):.6f}"))
        self.data_table.blockSignals(False)

    def on_table_changed(self, item):
        row = item.row()
        col = item.column()
        
        txt = self.obj_combo.currentText()
        sid = self.obj_combo.currentData()
        mi = current_project.map_info
        
        if "[Ship]" in txt:
            ship = current_project.get_ship_by_idx(sid)
            if not ship or row >= len(ship.raw_points): return
            
            if col in [2, 3]:
                try:
                    lat = float(self.data_table.item(row, 2).text())
                    lon = float(self.data_table.item(row, 3).text())
                    px, py = coords_to_pixel(lat, lon, mi)
                    ship.raw_points[row] = (px, py)
                    self.handle_path_change(ship, redraw=False) 
                except: pass
            elif col == 4: 
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
        txt = self.obj_combo.currentText()
        if "[Ship]" not in txt:
            QMessageBox.warning(self, "Warning", "Select a Ship first.")
            return

        sid = self.obj_combo.currentData()
        ship = current_project.get_ship_by_idx(sid)
        if not ship: return

        rows_to_del = []
        for row in range(self.data_table.rowCount()):
            item = self.data_table.item(row, 0)
            if item and item.checkState() == Qt.CheckState.Checked:
                rows_to_del.append(row)
        
        if not rows_to_del:
            QMessageBox.information(self, "Info", "No points selected.")
            return

        if QMessageBox.question(self, "Delete", f"Delete {len(rows_to_del)} points?", 
                                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No) == QMessageBox.StandardButton.No:
            return

        rows_to_del.sort(reverse=True)
        
        for idx in rows_to_del:
            if idx < len(ship.raw_points):
                ship.raw_points.pop(idx)
                
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
        self.scene.clear()
        self.add_boundary_masks()
        
        sel_txt = self.obj_combo.currentText()
        sel_id = self.obj_combo.currentData()
        highlight_points = (self.view.mode in ["MOVE_POINT", "DELETE_POINT"])
        
        settings = current_project.settings
        
        for s in current_project.ships:
            is_sel = ("[Ship]" in sel_txt and s.idx == sel_id)
            if not s.raw_points: continue
            pts = s.raw_points

            if len(pts) >= 2:
                path = QGraphicsPathItem()
                pp = QPainterPath()
                pp.addPolygon(QPolygonF([QPointF(x,y) for x,y in pts]))
                path.setPath(pp)
                w = settings.path_thickness
                
                if s.idx == settings.own_ship_idx: c_hex = settings.own_color
                elif s.idx >= 1000: c_hex = settings.random_color
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

            if s.idx == settings.own_ship_idx: c_hex = settings.own_color
            elif s.idx >= 1000: c_hex = settings.random_color
            else: c_hex = settings.target_color
            
            if is_sel: c_hex = settings.select_color
            c_dot = QColor(c_hex)
            
            for i, (px, py) in enumerate(s.raw_points):
                r = 6 
                el = QGraphicsEllipseItem(-r/2, -r/2, r, r)
                el.setBrush(QBrush(c_dot))
                el.setPen(QPen(Qt.PenStyle.NoPen))
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
                
                if settings.show_speed_notes and s.is_generated:
                    spd = s.raw_speeds.get(i, 5.0)
                    txt = QGraphicsTextItem(f"{spd:.1f} kn")
                    txt.setDefaultTextColor(Qt.GlobalColor.black)
                    txt.setFont(QFont("Arial", 8))
                    txt.setPos(px + 10, py + 10)
                    txt.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
                    txt.setZValue(3)
                    self.scene.addItem(txt)

            if s.raw_points:
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

        for st in current_project.areas:
            is_sel = ("[Area]" in sel_txt and st.id == sel_id)
            poly = QPolygonF([QPointF(x,y) for x,y in st.geometry])
            item = QGraphicsPolygonItem(poly)
            pen = QPen(Qt.GlobalColor.black, 2 if not is_sel else 4)
            pen.setCosmetic(True)
            item.setPen(pen)
            item.setBrush(QBrush(QColor(st.color)))
            item.setZValue(0)
            self.scene.addItem(item)

    def add_boundary_masks(self):
        mi = current_project.map_info
        scale = mi.pixels_per_degree
        if scale <= 0: return
        
        c = QColor(current_project.settings.mask_color)
        limit = 1e9
        
        # Left of -180
        r1 = QGraphicsRectItem(-limit, -limit, (-180 * scale) + limit, 2 * limit)
        r1.setBrush(QBrush(c))
        r1.setPen(QPen(Qt.PenStyle.NoPen))
        r1.setZValue(-100)
        self.scene.addItem(r1)
        
        # Right of 180
        r2 = QGraphicsRectItem(180 * scale, -limit, limit - (180 * scale), 2 * limit)
        r2.setBrush(QBrush(c))
        r2.setPen(QPen(Qt.PenStyle.NoPen))
        r2.setZValue(-100)
        self.scene.addItem(r2)
        
        # Top (Lat > 90 => y < -90*scale)
        r3 = QGraphicsRectItem(-limit, -limit, 2 * limit, (-90 * scale) + limit)
        r3.setBrush(QBrush(c))
        r3.setPen(QPen(Qt.PenStyle.NoPen))
        r3.setZValue(-100)
        self.scene.addItem(r3)
        
        # Bottom (Lat < -90 => y > 90*scale)
        r4 = QGraphicsRectItem(-limit, 90 * scale, 2 * limit, limit - (90 * scale))
        r4.setBrush(QBrush(c))
        r4.setPen(QPen(Qt.PenStyle.NoPen))
        r4.setZValue(-100)
        self.scene.addItem(r4)

    def add_point_click(self, px, py):
        txt = self.obj_combo.currentText()
        if "[Ship]" not in txt:
            QMessageBox.warning(self, "Warning", "Please select a Ship first.")
            self.set_map_mode("SELECT")
            return
        
        # --- Latitude Validation Start ---
        mi = current_project.map_info
        _, _, lat, _ = pixel_to_coords(px, py, mi) # Only need lat for validation

        if not (-90 <= lat <= 90):
            QMessageBox.warning(self, "Invalid Latitude", "Latitude must be between -90 and +90 degrees. This point will not be saved.")
            self.set_map_mode("SELECT") # Go back to select mode after warning
            return # Exit the function without adding the point
        # --- Latitude Validation End ---

        sid = self.obj_combo.currentData()
        ship = current_project.get_ship_by_idx(sid)
        if not ship: return
        d = QDialog(self)
        d.setWindowTitle("Add Point Info")
        f = QFormLayout(d)
        spin_spd = QDoubleSpinBox()
        spin_spd.setRange(0, 1000)
        prev_spd = 100.0
        if ship.raw_speeds:
             last_idx = len(ship.raw_points) - 1
             if last_idx in ship.raw_speeds:
                 prev_spd = ship.raw_speeds[last_idx]
        spin_spd.setValue(prev_spd)

        edit_note = QLineEdit()
        
        f.addRow("Speed (kn):", spin_spd)
        f.addRow("Note:", edit_note)
        bb = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        bb.accepted.connect(d.accept)
        bb.rejected.connect(d.reject)
        f.addWidget(bb)
        
        if d.exec() == QDialog.DialogCode.Accepted:
            new_idx = len(ship.raw_points)
            ship.raw_points.append((px, py))
            ship.raw_speeds[new_idx] = spin_spd.value()
            ship.raw_notes[new_idx] = edit_note.text()
            self.handle_path_change(ship)

    def req_add_point(self):
        txt = self.obj_combo.currentText()
        if "[Ship]" not in txt:
            QMessageBox.warning(self, "Warning", "Please select a Ship first.")
            return

        dlg = QDialog(self)
        dlg.setWindowTitle("Add Point Method")
        v = QVBoxLayout(dlg)
        
        lbl = QLabel("How would you like to add the point?")
        v.addWidget(lbl)
        
        btn_map = QPushButton("Click on Map")
        btn_manual = QPushButton("Input Lat/Lon manually")
        
        v.addWidget(btn_map)
        v.addWidget(btn_manual)
        
        def on_map():
            dlg.accept()
            self.set_map_mode("ADD_POINT")
            
        def on_manual():
            dlg.reject()
            self.manual_add_point_dialog()
            
        btn_map.clicked.connect(on_map)
        btn_manual.clicked.connect(on_manual)
        
        dlg.exec()

    def manual_add_point_dialog(self):
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

    def handle_path_change(self, ship, redraw=True):
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
        # ship.is_generated = False # No longer used in the same way
        ship.time_series = [] 
        ship.packed_data = None 
        ship.cumulative_time = []
        ship.total_duration_sec = 0.0
        self.check_sim_ready()
        
        if self.sim_panel and self.sim_panel.worker and self.sim_panel.worker.running:
            self.sim_panel.reset_state()

    def check_sim_ready(self):
        ready = True
        if not current_project.ships: ready = False
        # Logic changed: Ready if ships exist and have points. No pre-generation needed.
        for s in current_project.ships:
            if not s.raw_points:
                ready = False
                break
        
        txt = self.obj_combo.currentText()
        if "[Ship]" in txt:
            sid = self.obj_combo.currentData()
            s = current_project.get_ship_by_idx(sid)
            if s:
                dur_str = "-" # Dynamic duration
                self.lbl_dur.setText(f"Duration: {dur_str}")

    def delete_points(self, s_idx, p_idx):
        if QMessageBox.question(self, "Delete", "Delete this point and all subsequent points?", 
                                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No, 
                                QMessageBox.StandardButton.No) == QMessageBox.StandardButton.No:
            return

        ship = current_project.get_ship_by_idx(s_idx)
        if ship:
            ship.raw_points.pop(p_idx)
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
        self.set_map_mode("ADD_AREA")

    def finish_add_area(self, raw_pts):
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
        
        resampled = resample_polygon_equidistant(raw_pts, spin.value())
        
        sid = len(current_project.areas)
        st = Area(id=sid, name=name_edit.text() or f"Area-{sid}", geometry=resampled, color=color_val[0])
        current_project.areas.append(st)
        self.update_obj_combo()
        self.set_map_mode("SELECT")
        self.data_changed.emit()

    def change_area_color(self):
        txt = self.obj_combo.currentText()
        if "[Area]" not in txt:
            QMessageBox.information(self, "Info", "Please select an Area.")
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
            
            px, py = coords_to_pixel(lat, lon, current_project.map_info)
            self.view.centerOn(px, py)
            
            self.view.resetTransform()
            self.view.scale(0.01, 0.01)

    def open_project(self):
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
            
            m = data.get("map_info", {})
            p.map_info.center_lat = m.get("center_lat", 35.0)
            p.map_info.center_lon = m.get("center_lon", 129.0)
            
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
            p.settings.random_target_speed_variance = s.get("random_target_speed_variance", 1.0)
            p.settings.path_thickness = s.get("path_thickness", 3)
            p.settings.traveled_path_thickness = s.get("traveled_path_thickness", 3)
            p.settings.dropout_probs = s.get("dropout_probs", {})
            p.settings.theme_mode = s.get("theme_mode", "System")
            

            p.settings.mask_color = s.get("mask_color", "#404040")

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
                
                ship.total_duration_sec = 0.0
                ship.packed_data = None 
                
                if len(ship.raw_points) < 2:
                    ship.resampled_points = list(ship.raw_points)
                else:
                    ship.resampled_points = resample_polyline_numpy(ship.raw_points, 60)
                
                p.ships.append(ship)

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
            
            # Load from Event folder (New Standard)
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
                                evt.action_option = e_data.get("action_option", "")
                                p.events.append(evt)
                        except: pass
            
            # Fallback: Load from project.json if Event folder was empty or didn't exist (Legacy)
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
                    evt.action_option = e_data.get("action_option", "")
                    p.events.append(evt)

            # Load Scenarios from Project/Scenario folder
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
                            
                            # Sync events to registry
                            for s_evt in scen.events:
                                # Check if exists in project events
                                existing = next((e for e in p.events if e.id == s_evt.id), None)
                                if existing:
                                    # Override with scenario version (as per spec 5.2)
                                    existing.name = s_evt.name
                                    existing.enabled = s_evt.enabled
                                    existing.trigger_type = s_evt.trigger_type
                                    existing.condition_value = s_evt.condition_value
                                    existing.action_type = s_evt.action_type
                                    existing.target_ship_idx = s_evt.target_ship_idx
                                    existing.reference_ship_idx = s_evt.reference_ship_idx
                                    existing.action_value = s_evt.action_value
                                    existing.is_relative_to_end = s_evt.is_relative_to_end
                                    existing.action_option = getattr(s_evt, 'action_option', "")
                        except: pass
            
            self.update_ui_state(True)
            self.update_info_strip()
            self.update_obj_combo()
            self.apply_theme()
            self.update_stylesheets()
            self.event_panel.refresh_all()
            self.redraw_map()
            
            # if self.sim_win:
            #     self.sim_win.close()
            #     self.sim_win = None
            self.check_sim_ready()

        except Exception as e:
            traceback.print_exc()
            QMessageBox.critical(self, "Error Loading Project", str(e))

    def save_project(self):
        p = current_project
        if not p.project_path: return
        
        # Create folders
        os.makedirs(os.path.join(p.project_path, "Object"), exist_ok=True)
        os.makedirs(os.path.join(p.project_path, "Scenario"), exist_ok=True)
        os.makedirs(os.path.join(p.project_path, "Event"), exist_ok=True)
        
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
            
            ships_list.append({
                "ship_index": s.idx,
                "ship_name": s.name,
                "mmsi": s.mmsi,
                "note": s.note,
                "control_points": control_points
            })
            
            # Save individual ship file
            ship_fname = f"Ship_{s.idx}.json"
            with open(os.path.join(p.project_path, "Object", ship_fname), 'w', encoding='utf-8') as f:
                json.dump(ships_list[-1], f, indent=4)

        # Save Areas individually

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
            
            area_fname = f"Area_{a.id}.json"
            with open(os.path.join(p.project_path, "Object", area_fname), 'w', encoding='utf-8') as f:
                json.dump(areas_list[-1], f, indent=4)

        events_list = []
        # Save Events individually to Event folder
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
                "action_option": getattr(e, 'action_option', "")
            }
            
            # Save to Event/{name}.json
            fname = sanitize_filename(e.name) + ".json"
            with open(os.path.join(p.project_path, "Event", fname), 'w', encoding='utf-8') as f:
                json.dump(e_data, f, indent=4)
            
        # Save Scenarios
        if app_state.loaded_scenarios:
            for scen in app_state.loaded_scenarios:
                fname = sanitize_filename(scen.name) + ".scenario.json"
                scen.save_to_file(os.path.join(p.project_path, "Scenario", fname))

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
                "random_target_speed_variance": p.settings.random_target_speed_variance,
                "path_thickness": p.settings.path_thickness,
                "traveled_path_thickness": p.settings.traveled_path_thickness,
                "mask_color": p.settings.mask_color,
                "dropout_probs": p.settings.dropout_probs,
                "theme_mode": p.settings.theme_mode
                "simulation_speed_variance": getattr(p.settings, "simulation_speed_variance", 0.1)
            },
            # Minimal info in project.json, full data in subfolders/files
            "ships": ships_list, # Keeping for backward compat or easy loading
            "areas": areas_list, # Keeping for backward compat
            "events": [] # Events are now in Event folder
        }
        
        fname = f"{sanitize_filename(p.project_name)}.json"
        full_path = os.path.join(p.project_path, fname)
        
        try:
            with open(full_path, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=4)
            QMessageBox.information(self, "Saved", f"Project Saved to:\n{fname}")
        except Exception as e:
            QMessageBox.critical(self, "Error Saving", str(e))

    def clear_cache(self):
        if QMessageBox.question(self, "Clear Cache", "Delete all __pycache__ folders in the project?\n(They will be regenerated automatically)", 
                                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No) != QMessageBox.StandardButton.Yes:
            return
            
        base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        count = 0
        for root, dirs, files in os.walk(base_dir):
            if "__pycache__" in dirs:
                try:
                    shutil.rmtree(os.path.join(root, "__pycache__"))
                    count += 1
                except: pass
        QMessageBox.information(self, "Done", f"Deleted {count} cache directories.")

    def add_ship_dialog(self):
        idx = 0
        existing = [s.idx for s in current_project.ships]
        while idx in existing: idx += 1
        
        name, ok = QInputDialog.getText(self, "Add Ship", "Ship Name:", text=f"Ship-{idx}")
        if not ok: return
        if not name.strip(): name = f"Ship-{idx}"
        
        s = ShipData(idx, name)
        
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
        txt = self.obj_combo.currentText()
        sid = self.obj_combo.currentData()
        
        msg = "Are you sure you want to delete this Object?"
        if "[Ship]" in txt:
            msg += "\nPath, Speed, Duration data will be lost."
        if QMessageBox.question(self, "Delete Object", msg, 
                                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No, 
                                QMessageBox.StandardButton.No) == QMessageBox.StandardButton.No:
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
        w = self.tabs.widget(index)
        
        # Determine is_dark for map redraw logic if needed
        s = current_project.settings
        mode = s.theme_mode
        is_dark = False
        if mode == "Dark": is_dark = True
        elif mode == "Light": is_dark = False
        else:
            if QApplication.styleHints().colorScheme() == Qt.ColorScheme.Dark: is_dark = True
        input_bg = "#252525" if is_dark else "#ffffff" # Used for map bg logic if needed
        sel_color = input_bg # Default
        
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
        idx = self.tabs.indexOf(self.sim_panel)
        if idx == -1: return
        
        def get_white_icon(shape):
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
            # Create Red Stop Icon
            pix = QPixmap(16, 16)
            pix.fill(Qt.GlobalColor.transparent)
            p = QPainter(pix)
            p.setBrush(QBrush(Qt.GlobalColor.red))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawRect(2, 2, 12, 12)
            p.end()
            self.tabs.setTabIcon(idx, QIcon(pix))

    def open_settings(self):
        dlg = SettingsDialog(self)
        if dlg.exec():
            dlg.apply_changes()
            self.apply_theme()
            self.update_stylesheets()
            self.update_obj_combo()
            self.redraw_map()
            self.data_changed.emit()

    def open_user_guide(self):
        dlg = HelpDialog(self)
        dlg.exec()


    # def open_sim_panel(self):
    #     if not self.sim_win:
    #         self.sim_win = SimulationWindow(self)
    #     self.sim_win.show()

    def on_data_changed(self):
        if self.sim_panel and self.sim_panel.isVisible():
            self.sim_panel.draw_static_map()
            self.sim_panel.refresh_tables()
        self.event_panel.refresh_all()
        self.scenario_panel.refresh_ui()

    def update_ui_state(self, enabled):
        self.view.setEnabled(enabled)
        # self.act_speed.setEnabled(enabled)
        self.obj_combo.setEnabled(enabled)

    def closeEvent(self, event):
        reply = QMessageBox.question(
            self, 
            "Save Project", 
            "Do you want to save the project?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No | QMessageBox.StandardButton.Cancel
        )

        if reply == QMessageBox.StandardButton.Yes:
            self.sim_panel.cleanup()
            self.save_project()
            event.accept()
        elif reply == QMessageBox.StandardButton.No:
            self.sim_panel.cleanup()
            event.accept()
        else:
            event.ignore()