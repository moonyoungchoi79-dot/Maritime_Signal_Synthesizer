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
    QToolBar, QStatusBar, QGraphicsView, QGraphicsScene, QGraphicsItem, QGraphicsPathItem,
    QGraphicsPolygonItem, QGraphicsEllipseItem, QGraphicsTextItem, QAbstractItemView,
    QAbstractSpinBox, QTextBrowser, QSizePolicy, QDateTimeEdit
)
from PyQt6.QtCore import (
    Qt, pyqtSignal, QObject, QThread, QTimer, QPoint, QPointF, QRect, QRectF, QSize, QSizeF,
    QEvent, QUrl, QDate, QTime, QDateTime
)
from PyQt6.QtGui import (
    QColor, QPalette, QPen, QBrush, QFont, QPainter, QPainterPath, QPolygonF, QIcon, QPixmap,
    QCursor, QAction
)
from app.ui.map.map_view import MapView
from app.core.geometry import coords_to_pixel, pixel_to_coords
from app.core.models.project import current_project
from app.core.nmea import parse_nmea_fields

class ReplayMapView(MapView):
    def showContextMenu(self, pos):
        pass 
    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            pos_scene = self.mapToScene(event.pos())
            items = self.scene().items(pos_scene)
            if self.main_window:
                for item in items:
                    for sid, ship_item in self.main_window.ship_items.items():
                        if item == ship_item:
                            self.main_window.show_ship_details(sid)
                            return
            self.start_pan(event)
        else:
            super().mousePressEvent(event)

class ReplayWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Log Replay")
        self.resize(1000, 700)
        
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        
        # Top controls
        top = QHBoxLayout()
        self.btn_load = QPushButton("Load CSV")
        self.btn_load.clicked.connect(self.load_csv)
        top.addWidget(self.btn_load)
        
        self.lbl_file = QLabel("No file loaded")
        top.addWidget(self.lbl_file)
        top.addStretch()
        layout.addLayout(top)
        
        # Map
        self.scene = QGraphicsScene()
        self.view = ReplayMapView(self.scene, self)
        self.view.mode = "VIEW"
        layout.addWidget(self.view)
        
        # Bottom controls
        bot = QHBoxLayout()
        
        self.btn_play = QPushButton("Play")
        self.btn_pause = QPushButton("Pause")
        self.btn_stop = QPushButton("Stop")
        self.btn_play.clicked.connect(self.play)
        self.btn_pause.clicked.connect(self.pause)
        self.btn_stop.clicked.connect(self.stop)
        
        bot.addWidget(self.btn_play)
        bot.addWidget(self.btn_pause)
        bot.addWidget(self.btn_stop)
        
        bot.addWidget(QLabel("Speed:"))
        self.spin_speed = QDoubleSpinBox()
        self.spin_speed.setRange(0.1, 100.0)
        self.spin_speed.setValue(1.0)
        bot.addWidget(self.spin_speed)
        
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.sliderPressed.connect(self.slider_pressed)
        self.slider.sliderReleased.connect(self.slider_released)
        self.slider.valueChanged.connect(self.slider_moved)
        bot.addWidget(self.slider)
        
        self.lbl_time = QLabel("00:00:00")
        bot.addWidget(self.lbl_time)
        
        layout.addLayout(bot)
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.tick)
        self.last_tick_time = 0
        
        self.data_by_time = [] # List of (timestamp, {ship_id: (lat, lon, hdg)})
        self.start_ts = 0
        self.end_ts = 0
        self.current_ts = 0
        self.is_playing = False
        self.slider_dragging = False
        
        self.ship_items = {} # id -> QGraphicsPolygonItem
        self.current_ship_states = {}

    def load_csv(self):
        path, _ = QFileDialog.getOpenFileName(self, "Open CSV Log", "", "CSV Files (*.csv)")
        if not path: return
        
        self.lbl_file.setText(path)
        self.data_by_time = []
        self.ship_items = {}
        self.scene.clear()
        
        try:
            with open(path, 'r', encoding='utf-8') as f:
                reader = csv.DictReader(f)
                temp_data = []
                for row in reader:
                    try:
                        ts_str = row.get('rx_time')
                        dt = datetime.datetime.strptime(ts_str, '%Y-%m-%dT%H:%M:%SZ').replace(tzinfo=datetime.timezone.utc)
                        ts = dt.timestamp()
                        
                        raw = row.get('raw', '')
                        fields = parse_nmea_fields(raw)
                        
                        lat = float(row.get('lat_deg', 0))
                        if lat == 0 and 'lat_deg' in fields: lat = fields['lat_deg']
                        
                        lon = float(row.get('lon_deg', 0))
                        if lon == 0 and 'lon_deg' in fields: lon = fields['lon_deg']
                        
                        hdg = float(row.get('heading_true_deg', 0))
                        if hdg == 0 and 'heading_true_deg' in fields: hdg = fields['heading_true_deg']
                        
                        if hdg == 0: hdg = float(row.get('cog_true_deg', 0))
                        if hdg == 0 and 'cog_true_deg' in fields: hdg = fields['cog_true_deg']
                        
                        sog = float(row.get('sog_knots', 0))
                        if sog == 0 and 'sog_knots' in fields: sog = fields['sog_knots']
                        
                        ship_id = int(row.get('receiver_ship_index', 0))
                        
                        # If it's a target report (RATLL/RATTM), use target ID
                        if 'target_no' in fields:
                            ship_id = 1000 + fields['target_no'] # Offset for targets
                        
                        if lat != 0 or lon != 0:
                            temp_data.append((ts, ship_id, lat, lon, hdg, sog))
                    except: continue
            
            if not temp_data:
                QMessageBox.warning(self, "Error", "No valid position data found.")
                return
                
            temp_data.sort(key=lambda x: x[0])
            self.start_ts = temp_data[0][0]
            self.end_ts = temp_data[-1][0]
            self.current_ts = self.start_ts
            
            self.data_by_time = temp_data
            
            self.slider.setRange(0, int(self.end_ts - self.start_ts))
            self.update_view()
            
        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))

    def play(self):
        if not self.data_by_time: return
        self.is_playing = True
        self.last_tick_time = time.time()
        self.timer.start(50)

    def pause(self):
        self.is_playing = False
        self.timer.stop()

    def stop(self):
        self.pause()
        self.current_ts = self.start_ts
        self.slider.setValue(0)
        self.update_view()

    def tick(self):
        if not self.is_playing: return
        now = time.time()
        dt = now - self.last_tick_time
        self.last_tick_time = now
        
        self.current_ts += dt * self.spin_speed.value()
        if self.current_ts > self.end_ts:
            self.current_ts = self.end_ts
            self.pause()
            
        if not self.slider_dragging:
            self.slider.setValue(int(self.current_ts - self.start_ts))
        self.update_view()

    def slider_pressed(self):
        self.slider_dragging = True

    def slider_released(self):
        self.slider_dragging = False
        self.current_ts = self.start_ts + self.slider.value()
        self.update_view()

    def slider_moved(self, val):
        if self.slider_dragging:
            self.current_ts = self.start_ts + val
            self.update_view()

    def update_view(self):
        # Find positions at current_ts
        # Simple approach: find nearest past event for each ship
        # Optimization: Since data is sorted, we can search
        
        mi = current_project.map_info
        
        # Filter data up to current_ts
        # In a real efficient replay, we'd keep an index. For now, simple iteration is okay for small logs.
        # Better: Iterate backwards from binary search point
        
        import bisect
        idx = bisect.bisect_right(self.data_by_time, (self.current_ts, 999999, 0, 0, 0, 0))
        
        # Get latest pos for each ship
        self.current_ship_states = {}
        # Look back a bit (e.g. 1000 records) or scan all previous?
        # Scanning all previous is slow.
        # Let's just scan from 0 to idx, but that's O(N).
        # Optimization: cache state?
        # For this snippet, let's just take the last seen position for each ID in the slice
        
        subset = self.data_by_time[:idx]
        for ts, sid, lat, lon, hdg, sog in subset:
            self.current_ship_states[sid] = {'lat': lat, 'lon': lon, 'hdg': hdg, 'sog': sog, 'ts': ts}
            
        self.lbl_time.setText(datetime.datetime.fromtimestamp(self.current_ts, tz=datetime.timezone.utc).strftime('%H:%M:%S'))
        
        # Draw
        for sid, state in self.current_ship_states.items():
            lat, lon, hdg = state['lat'], state['lon'], state['hdg']
            px, py = coords_to_pixel(lat, lon, mi)
            
            if sid not in self.ship_items:
                poly = QPolygonF([QPointF(0, -10), QPointF(-5, 5), QPointF(5, 5)])
                item = QGraphicsPolygonItem(poly)
                item.setBrush(QBrush(Qt.GlobalColor.blue if sid < 1000 else Qt.GlobalColor.red))
                item.setPen(QPen(Qt.GlobalColor.black))
                self.scene.addItem(item)
                self.ship_items[sid] = item
            
            item = self.ship_items[sid]
            item.setPos(px, py)
            item.setRotation(hdg)
            item.setVisible(True)

    def show_ship_details(self, sid):
        if sid not in self.current_ship_states: return
        state = self.current_ship_states[sid]
        
        info = f"""
        <h3>Ship ID: {sid}</h3>
        <hr>
        <b>Time:</b> {datetime.datetime.fromtimestamp(state['ts'], tz=datetime.timezone.utc).strftime('%H:%M:%S')}<br>
        <b>Lat:</b> {state['lat']:.6f}<br>
        <b>Lon:</b> {state['lon']:.6f}<br>
        <b>Speed:</b> {state['sog']:.1f} kn<br>
        <b>Heading:</b> {state['hdg']:.1f}°
        """
        QMessageBox.information(self, "Ship Details", info)