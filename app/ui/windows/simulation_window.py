import os
import sys
import math
import csv
import random
import datetime
import numpy as np

# Ensure project root is in sys.path when running this file directly
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, "../../../"))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QPushButton, QLabel, QLineEdit, 
    QTextEdit, QComboBox, QCheckBox, QSpinBox, QTableWidget, QHeaderView, QGroupBox, 
    QMessageBox, QFileDialog, QDialog, QGraphicsScene, QGraphicsPathItem, 
    QGraphicsPolygonItem, QGraphicsEllipseItem, QGraphicsTextItem, QAbstractSpinBox,
    QTableWidgetItem, QDoubleSpinBox, QGraphicsItem
)
from PyQt6.QtCore import (
    Qt, pyqtSignal, QThread, QPointF
)
from PyQt6.QtGui import (
    QColor, QPen, QBrush, QPainterPath, QPolygonF, QFont
)

from app.ui.windows.main_window import MainWindow
from app.core.constants import CSV_HEADER
from app.core.models.project import current_project, ShipData
from app.core.utils import sanitize_filename
from app.core.geometry import coords_to_pixel, pixel_to_coords, normalize_lon
from app.core.nmea import parse_nmea_fields
from app.ui.map.sim_map_view import SimMapView
from app.workers.simulation_worker import SimulationWorker
from app.ui.dialogs.rtg_dialog import RTGDialog
from app.ui.widgets.time_input_widget import TimeInputWidget
import app.core.state as app_state

class SimulationWindow(QWidget):
    state_changed = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        # self.setWindowTitle("Simulation Panel")
        self.ship_items = {}
        self.ship_indicators = {}
        self.ship_trails = {} # idx -> list of QPointF
        self.trail_items = {} # idx -> QGraphicsPathItem
        self.highlighted_ship_idx = -1
        self.init_ui()
        self.worker = None
        self.worker_thread = None
        self.is_paused = False
        self.data_ready_flag = False
        self.suppress_close_warning = False

    def set_follow_target(self, idx):
        combo_idx = self.combo_follow.findData(idx)
        if combo_idx >= 0:
            self.combo_follow.setCurrentIndex(combo_idx)

    def action_random_target_generate(self):
        dlg = RTGDialog(self)
        if dlg.exec() == QDialog.DialogCode.Accepted:
            data = dlg.get_data()
            R = data["R"]
            N_AI_only = data["N_AI_only"]
            N_RA_only = data["N_RA_only"]
            N_both = data["N_both"]

            own_ship = current_project.get_ship_by_idx(current_project.settings.own_ship_idx)
            if not own_ship:
                QMessageBox.warning(self, "Warning", "자선(Own Ship)이 설정되어 있지 않습니다. 랜덤 타겟을 생성할 수 없습니다.")
                return
            
            if not own_ship.raw_points:
                QMessageBox.warning(self, "Warning", "자선(Own Ship)의 경로 데이터가 없습니다. 메인 화면에서 경로를 먼저 생성해주세요.")
                return

            self.generate_random_targets_logic(own_ship, R, N_AI_only, N_RA_only, N_both)

    def action_clear_random_targets(self):
        targets = [s for s in current_project.ships if s.idx >= 1000]
        if not targets:
            QMessageBox.information(self, "Info", "삭제할 랜덤 타겟이 없습니다.")
            return

        if QMessageBox.question(self, "Clear Random Targets", f"{len(targets)}개의 랜덤 타겟을 삭제하시겠습니까?", 
                                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No) != QMessageBox.StandardButton.Yes:
            return

        current_project.ships = [s for s in current_project.ships if s.idx < 1000]
        
        # Clear trails for removed ships
        active_idxs = {s.idx for s in current_project.ships}
        to_remove = [idx for idx in self.ship_trails.keys() if idx not in active_idxs]
        for idx in to_remove:
            del self.ship_trails[idx]
            if idx in self.trail_items:
                self.scene.removeItem(self.trail_items[idx])
                del self.trail_items[idx]
        
        self.refresh_tables()
        self.draw_static_map()
        
        if self.window():
            self.window().update_obj_combo()
            self.window().redraw_map()
            
        if self.worker and self.worker.running:
            self.worker.refresh_active_ships()

    def init_ui(self):
        layout = QHBoxLayout(self)
        
        self.scene = QGraphicsScene()
        self.view = SimMapView(self.scene, self)
        self.view.mode = "VIEW"
        
        layout.addWidget(self.view, 3)
        
        right = QWidget()
        r_layout = QVBoxLayout(right)
        
        top_controls_widget = QWidget()
        top_layout = QVBoxLayout(top_controls_widget)
        top_layout.setContentsMargins(0,0,0,0)

        grp = QGroupBox("Controls")
        grid = QGridLayout(grp)
        self.ip_edit = QLineEdit("127.0.0.1")
        self.port_edit = QLineEdit("10110")
        self.speed_spin = QSpinBox()
        self.speed_spin.setRange(1, 86400)
        self.speed_spin.setValue(1)
        self.speed_spin.valueChanged.connect(self.on_speed_change)
        
        self.combo_follow = QComboBox()
        self.combo_follow.addItem("None", -1)
        
        grid.addWidget(QLabel("UDP IP:"), 0, 0)
        grid.addWidget(self.ip_edit, 0, 1)
        grid.addWidget(QLabel("UDP Port:"), 0, 2)
        grid.addWidget(self.port_edit, 0, 3)
        
        grid.addWidget(QLabel("Speed (x):"), 1, 0)
        self.speed_spin.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
        grid.addWidget(self.speed_spin, 1, 1)
        grid.addWidget(QLabel("Follow:"), 1, 2)
        grid.addWidget(self.combo_follow, 1, 3)

        self.perf_label = QLabel("-")
        self.eta_label = QLabel("-")
        
        grid.addWidget(QLabel("Performance:"), 2, 0)
        grid.addWidget(self.perf_label, 2, 1)
        grid.addWidget(QLabel("Time Remaining:"), 2, 2) # Moved to row 2
        grid.addWidget(self.eta_label, 2, 3)
        
        grid.addWidget(QLabel("Total Time (s):"), 3, 0)
        self.duration_input = TimeInputWidget()
        self.duration_input.valueChanged.connect(self.on_duration_changed)
        grid.addWidget(self.duration_input, 3, 1)
        
        self.btn_add_time = QPushButton("+10m")
        self.btn_add_time.setEnabled(True)
        self.btn_add_time.clicked.connect(self.action_add_time)
        grid.addWidget(self.btn_add_time, 3, 2)

        # Layout Ratio 1:2
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(3, 2)

        top_layout.addWidget(grp)
        
        self.on_off_group = QGroupBox("Signal On/Off")
        on_off_layout = QVBoxLayout(self.on_off_group)
        self.on_off_table = self.create_signal_on_off_table()
        on_off_layout.addWidget(self.on_off_table)
        top_layout.addWidget(self.on_off_group)
        
        self.interval_group = QGroupBox("Signal Interval (sec)")
        interval_layout = QVBoxLayout(self.interval_group)
        self.interval_table = self.create_signal_interval_table()
        interval_layout.addWidget(self.interval_table)
        top_layout.addWidget(self.interval_group)

        r_layout.addWidget(top_controls_widget)
        
        self.btn_rtg = QPushButton("Random Target Generate")
        self.btn_rtg.setObjectName("rtgBtn")
        self.btn_rtg.clicked.connect(self.action_random_target_generate)
        r_layout.addWidget(self.btn_rtg)

        self.btn_clear_rtg = QPushButton("Clear Random Targets")
        self.btn_clear_rtg.clicked.connect(self.action_clear_random_targets)
        r_layout.addWidget(self.btn_clear_rtg)
        
        btn_box = QHBoxLayout()
        self.btn_start = QPushButton("Start")
        self.btn_pause = QPushButton("Pause")
        self.btn_stop = QPushButton("Stop")
        
        self.btn_pause.setEnabled(False)
        self.btn_stop.setEnabled(False)
        
        self.btn_start.clicked.connect(self.action_start)
        self.btn_pause.clicked.connect(self.action_pause)
        self.btn_stop.clicked.connect(self.action_stop)
        
        btn_box.addWidget(self.btn_start)
        btn_box.addWidget(self.btn_pause)
        btn_box.addWidget(self.btn_stop)
        r_layout.addLayout(btn_box)
        
        log_group = QGroupBox("Raw NMEA Output")
        log_layout = QVBoxLayout(log_group)
        self.log_list = QTextEdit()
        self.log_list.setReadOnly(True)
        log_layout.addWidget(self.log_list)

        r_layout.addWidget(log_group, 1)
        
        hbox_bot = QHBoxLayout()
        self.btn_export = QPushButton("Export CSV")
        self.btn_export.clicked.connect(self.export_csv)
        self.btn_export.setEnabled(False)
        
        self.btn_save_log = QPushButton("Save Log")
        self.btn_save_log.clicked.connect(self.save_log_to_file)
        
        self.btn_clear = QPushButton("Clear Terminal")
        self.btn_clear.clicked.connect(self.log_list.clear)
        
        hbox_bot.addWidget(self.btn_export)
        hbox_bot.addWidget(self.btn_save_log)
        hbox_bot.addWidget(self.btn_clear)
        log_layout.addLayout(hbox_bot)
        
        layout.addWidget(right, 1)
        
        self.draw_static_map()
        self.update_follow_combo()

    def on_duration_changed(self, val):
        if self.worker:
            self.worker.update_duration(val)

    def action_add_time(self):
        val = self.duration_input.get_seconds()
        self.duration_input.set_seconds(val + 600)

    def generate_random_targets_logic(self, own_ship, R_nm, N_ai, N_ra, N_both):
        # Set deterministic seed for RTG
        seed_val = (current_project.seed + len(current_project.ships)) % (2**32 - 1)
        random.seed(seed_val)

        # 1. Determine Reference Time and State
        t_now = 0.0
        if self.worker and self.worker.running:
            t_now = self.worker.current_time
        
        # Get Own Ship state at t_now
        own_lat, own_lon, own_spd_kn = 0.0, 0.0, 0.0
        
        mi = current_project.map_info
        
        # Use current dynamic state if available, otherwise start pos
        if self.worker and self.worker.running and own_ship.idx in self.worker.dynamic_ships:
            dyn = self.worker.dynamic_ships[own_ship.idx]
            # Convert ECEF to LLA
            from app.core.geometry import ecef_to_lla
            own_lat, own_lon, _ = ecef_to_lla(dyn['x'], dyn['y'], dyn['z'])
            own_spd_kn = dyn['spd']
        elif own_ship.resampled_points:
            px, py = own_ship.resampled_points[0]
            _, _, own_lat, own_lon = pixel_to_coords(px, py, mi)
            own_spd_kn = own_ship.raw_speeds.get(0, 5.0)

        # 2. Generation Loop
        total_targets = N_ai + N_ra + N_both
        mi = current_project.map_info
        
        # R in degrees (Approx)
        # 1 nm = 1/60 degree lat
        R_deg_lat = R_nm / 60.0
        
        for i in range(total_targets):
            # Determine Type
            if i < N_ai: s_type = "AI"
            elif i < N_ai + N_ra: s_type = "RA"
            else: s_type = "BOTH"
            
            # 3.2 Start Position (Uniform in Circle)
            r = R_deg_lat * math.sqrt(random.random())
            theta = random.random() * 2 * math.pi
            
            d_lat = r * math.cos(theta)
            # Adjust lon for latitude (cos factor), guard against div by zero
            cos_lat = math.cos(math.radians(own_lat))
            if abs(cos_lat) < 0.01: cos_lat = 0.01
            d_lon = r * math.sin(theta) / cos_lat
            
            start_lat = own_lat + d_lat
            start_lon = own_lon + d_lon
            
            # Clamp Start Lat
            start_lat = max(-89.9, min(89.9, start_lat))
            
            # 3.5 Speed Model (unified variance for wind/current effects)
            variance = current_project.settings.speed_variance
            sigma = math.sqrt(variance)
            spd = abs(random.gauss(own_spd_kn, sigma)) # Abs as per requirement
            if spd < 0.1: spd = 0.1
            
            # 3.4 Motion Model (Straight Line)
            heading = random.random() * 360.0
            
            # Generate Path from t_now to End of Own Ship Duration
            # Since we don't know duration, we just generate a path long enough (e.g. 24 hours)
            duration_remaining = 24 * 3600.0
            
            # --- Zigzag Logic ---
            use_zigzag = current_project.settings.rtg_zigzag_enabled
            max_turns = current_project.settings.rtg_zigzag_turns
            angle_limit = current_project.settings.rtg_zigzag_angle_limit
            
            segments = []
            if use_zigzag:
                n_turns = random.randint(1, max_turns)
                # Split duration into n_turns + 1 segments
                cut_points = sorted([random.random() for _ in range(n_turns)])
                cut_points = [0.0] + cut_points + [1.0]
                for k in range(len(cut_points)-1):
                    dur = (cut_points[k+1] - cut_points[k]) * duration_remaining
                    segments.append(dur)
            else:
                segments.append(duration_remaining)
            
            # REVERTED OPTIMIZATION: Use dense points for accuracy
            # We generate points every 1 second to ensure the ship follows the path exactly
            # even at high simulation speeds (e.g. 10000x), preventing deviation and heading mismatch.
            
            raw_pixels = []
            curr_lat, curr_lon = start_lat, start_lon
            curr_heading = heading
            
            px_start, py_start = coords_to_pixel(start_lat, start_lon, mi)
            raw_pixels.append((px_start, py_start))
            
            dt = 1.0 # 1 second interval
            
            for k, seg_dur in enumerate(segments):
                steps = int(seg_dur / dt)
                
                # Velocity components (knots -> deg/sec)
                # 1 knot = 1 nm/h = 1/60 deg_lat/h
                dist_per_sec_deg = (spd / 3600.0) / 60.0
                
                d_lat_step = dist_per_sec_deg * math.cos(math.radians(curr_heading)) * dt
                d_lon_step_base = dist_per_sec_deg * math.sin(math.radians(curr_heading)) * dt
                
                for _ in range(steps):
                    curr_lat += d_lat_step
                    if curr_lat > 89.9: curr_lat = 89.9
                    elif curr_lat < -89.9: curr_lat = -89.9
                    
                    cos_lat = math.cos(math.radians(curr_lat))
                    if abs(cos_lat) < 0.01: cos_lat = 0.01
                    
                    d_lon_step = d_lon_step_base / cos_lat
                    curr_lon += d_lon_step
                    
                    px, py = coords_to_pixel(curr_lat, curr_lon, mi)
                    raw_pixels.append((px, py))
                
                # Change heading for next segment
                if k < len(segments) - 1:
                    change = random.uniform(-angle_limit, angle_limit)
                    curr_heading = (curr_heading + change) % 360.0
            
            # Create Ship Object
            # 1. Find unique index >= 1000
            new_idx = 1001
            existing_idxs = {s.idx for s in current_project.ships}
            while new_idx in existing_idxs: new_idx += 1
            
            # 2. Find unique internal number for naming "Random-{N}"
            r_num = 1
            existing_names = {s.name for s in current_project.ships}
            while f"Random-{r_num}" in existing_names: r_num += 1
            
            new_ship = ShipData(new_idx, f"Random-{r_num}")
            new_ship.mmsi = random.randint(100000000, 999999999)
            new_ship.is_generated = True
            
            # 1. Populate Raw Data (Essential for Saving & Editing)
            new_ship.raw_points = raw_pixels
            new_ship.raw_speeds = {0: spd}
            
            # 2. Populate Simulation Data
            new_ship.resampled_points = raw_pixels
            
            # Set Signals
            if s_type == "AI":
                new_ship.signals_enabled['AIVDM'] = True
                new_ship.signals_enabled['RATTM'] = False
                new_ship.signals_enabled['Camera'] = False
            elif s_type == "RA":
                new_ship.signals_enabled['AIVDM'] = False
                new_ship.signals_enabled['RATTM'] = True
                new_ship.signals_enabled['Camera'] = False
            else:
                new_ship.signals_enabled['AIVDM'] = True
                new_ship.signals_enabled['RATTM'] = True
                new_ship.signals_enabled['Camera'] = False
            
            current_project.ships.append(new_ship)
            
        # 4. Real-time Update
        self.refresh_tables()
        self.draw_static_map()
        
        # Fix: Restore positions to current simulation time if active
        if self.worker:
            t_restore = self.worker.current_time
            pos_dict = {}
            mi = current_project.map_info
            for s in current_project.ships:
                if not s.is_generated: continue
                st = self.calculate_ship_state(s, t_restore)
                px, py = coords_to_pixel(st['lat'], st['lon'], mi)
                pos_dict[s.idx] = (px, py, st['hdg'])
            self.update_positions(pos_dict)
        
        # Update Main Window (Combo box and Map)
        if self.window():
            self.window().update_obj_combo()
            self.window().redraw_map()
        
        # If worker is running, we need to ensure it picks up the new ships.
        # Assuming worker reads current_project.ships in its loop or we trigger an update.
        if self.worker and self.worker.running:
            self.worker.refresh_active_ships()

    def create_signal_on_off_table(self):
        table = QTableWidget()
        self.on_off_table_ref = table
        talkers = ["AIVDM", "RATTM", "Camera"]
        ships = [s for s in current_project.ships if s.idx != current_project.settings.own_ship_idx]
        
        table.setRowCount(len(ships) + 1)
        table.setColumnCount(len(talkers) + 1)
        
        table.setHorizontalHeaderLabels(["Ctrl"] + talkers)
        table.setVerticalHeaderLabels(["ALL"] + [s.name for s in ships])
        
        for r in range(table.rowCount()):
            for c in range(table.columnCount()):
                is_checked = True
                if c > 0 and talkers[c-1] == "Camera":
                    is_checked = False

                if r > 0 and c > 0:
                    ship = ships[r-1]
                    talker = talkers[c-1]
                    default_val = False if talker == "Camera" else True
                    is_checked = ship.signals_enabled.get(talker, default_val)

                widget = self._create_on_off_cell(is_checked)
                checkbox = widget.findChild(QCheckBox)
                
                checkbox.setProperty("row", r)
                checkbox.setProperty("col", c)
                checkbox.stateChanged.connect(self.on_signal_on_off_toggled)
                table.setCellWidget(r, c, widget)

        table.verticalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        for i in range(1, table.columnCount()):
            table.horizontalHeader().setSectionResizeMode(i, QHeaderView.ResizeMode.Stretch)
        table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
                
        return table
        
    def _create_on_off_cell(self, is_checked):
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.setContentsMargins(0,0,0,0)
        cb = QCheckBox()
        cb.setChecked(is_checked)
        layout.addWidget(cb)
        return widget

    def on_signal_on_off_toggled(self, state):
        sender_cb = self.sender()
        if not sender_cb: return

        r = sender_cb.property("row")
        c = sender_cb.property("col")
        is_checked = (state == Qt.CheckState.Checked.value)

        table = self.on_off_table_ref
        talkers = ["AIVDM", "RATTM", "Camera"]
        ships = [s for s in current_project.ships if s.idx != current_project.settings.own_ship_idx]

        rows_model = [r] if r > 0 else range(1, len(ships) + 1)
        cols_model = [c] if c > 0 else range(1, len(talkers) + 1)
        if r == 0 and c == 0:
            rows_model = range(1, len(ships) + 1)
            cols_model = range(1, len(talkers) + 1)
        
        for row_idx in rows_model:
            for col_idx in cols_model:
                ship = ships[row_idx - 1]
                talker = talkers[col_idx - 1]
                ship.signals_enabled[talker] = is_checked

        if r == 0 or c == 0:
            rows_ui = range(table.rowCount()) if r == 0 else [r]
            cols_ui = range(table.columnCount()) if c == 0 else [c]
            if r == 0 and c == 0:
                rows_ui = range(table.rowCount())
                cols_ui = range(table.columnCount())

            for row in rows_ui:
                for col in cols_ui:
                    if row == r and col == c: continue
                    self._set_on_off_checkbox(row, col, is_checked)
    
    def _set_on_off_checkbox(self, r, c, checked):
        widget = self.on_off_table_ref.cellWidget(r, c)
        if widget:
            cb = widget.findChild(QCheckBox)
            if cb:
                cb.blockSignals(True)
                cb.setChecked(checked)
                cb.blockSignals(False)

    def create_signal_interval_table(self):
        table = QTableWidget()
        self.interval_table_ref = table
        talkers = ["AIVDM", "RATTM", "Camera"]
        ships = [s for s in current_project.ships if s.idx != current_project.settings.own_ship_idx]

        table.setRowCount(len(ships) + 1)
        table.setColumnCount(len(talkers) + 1)

        table.setHorizontalHeaderLabels(["Ctrl"] + talkers)
        table.setVerticalHeaderLabels(["ALL"] + [s.name for s in ships])

        for r in range(table.rowCount()):
            for c in range(table.columnCount()):
                val = 1.0
                if r > 0 and c > 0:
                    ship = ships[r - 1]
                    talker = talkers[c - 1]
                    val = ship.signal_intervals.get(talker, 1.0)
                
                spin = self._create_interval_cell(val)
                spin.setProperty("row", r)
                spin.setProperty("col", c)
                spin.valueChanged.connect(self.on_signal_interval_changed)
                table.setCellWidget(r, c, spin)

        table.verticalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        table.setColumnWidth(0, 40)
        table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Fixed)
        for i in range(1, table.columnCount()):
            table.horizontalHeader().setSectionResizeMode(i, QHeaderView.ResizeMode.Stretch)
        
        return table

    def _create_interval_cell(self, value):
        spin = QDoubleSpinBox()
        spin.setRange(0.1, 3600.0)
        spin.setSingleStep(0.1)
        spin.setDecimals(1)
        spin.setValue(value)
        spin.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
        spin.lineEdit().setAlignment(Qt.AlignmentFlag.AlignCenter)
        return spin

    def on_signal_interval_changed(self, value):
        sender_spin = self.sender()
        if not sender_spin: return

        r = sender_spin.property("row")
        c = sender_spin.property("col")

        table = self.interval_table_ref
        talkers = ["AIVDM", "RATTM", "Camera"]
        ships = [s for s in current_project.ships if s.idx != current_project.settings.own_ship_idx]

        rows_model = [r] if r > 0 else range(1, len(ships) + 1)
        cols_model = [c] if c > 0 else range(1, len(talkers) + 1)
        if r == 0 and c == 0:
            rows_model = range(1, len(ships) + 1)
            cols_model = range(1, len(talkers) + 1)

        for row_idx in rows_model:
            for col_idx in cols_model:
                ship = ships[row_idx - 1]
                talker = talkers[col_idx - 1]
                ship.signal_intervals[talker] = value

        if r == 0 or c == 0:
            rows_ui = range(table.rowCount()) if r == 0 else [r]
            cols_ui = range(table.columnCount()) if c == 0 else [c]
            if r == 0 and c == 0:
                rows_ui = range(table.rowCount())
                cols_ui = range(table.columnCount())

            for row in rows_ui:
                for col in cols_ui:
                    if row == r and col == c: continue
                    self._set_interval_spinbox(row, col, value)

    def _set_interval_spinbox(self, r, c, value):
        widget = self.interval_table_ref.cellWidget(r, c)
        if widget and isinstance(widget, QDoubleSpinBox):
            widget.blockSignals(True)
            widget.setValue(value)
            widget.blockSignals(False)

    def refresh_tables(self):
        if self.on_off_table:
            self.on_off_group.layout().removeWidget(self.on_off_table)
            self.on_off_table.deleteLater()
        self.on_off_table = self.create_signal_on_off_table()
        self.on_off_group.layout().addWidget(self.on_off_table)

        if self.interval_table:
            self.interval_group.layout().removeWidget(self.interval_table)
            self.interval_table.deleteLater()
        self.interval_table = self.create_signal_interval_table()
        self.interval_group.layout().addWidget(self.interval_table)
        
        self.update_follow_combo()

    def update_follow_combo(self):
        current_idx = self.combo_follow.currentData()
        self.combo_follow.blockSignals(True)
        self.combo_follow.clear()
        self.combo_follow.addItem("None", -1)
        
        # Sort ships: Own, Manual, Random
        own_idx = current_project.settings.own_ship_idx
        own_ship = None
        manual = []
        random_tgts = []
        
        for s in current_project.ships:
            if s.idx == own_idx: own_ship = s
            elif s.idx >= 1000: random_tgts.append(s)
            else: manual.append(s)
            
        manual.sort(key=lambda x: x.idx)
        random_tgts.sort(key=lambda x: x.idx)
        
        sorted_ships = ([own_ship] if own_ship else []) + manual + random_tgts
        
        for s in sorted_ships:
            tag = "Own" if s.idx == own_idx else ("R-Tgt" if s.idx >= 1000 else "Tgt")
            self.combo_follow.addItem(f"[{tag}] {s.name}", s.idx)
            
        idx = self.combo_follow.findData(current_idx)
        if idx >= 0: self.combo_follow.setCurrentIndex(idx)
        else: self.combo_follow.setCurrentIndex(0)
            
        self.combo_follow.blockSignals(False)

    def draw_static_map(self):
        self.scene.clear()
        self.ship_items.clear()
        self.ship_indicators.clear()
        self.trail_items.clear() # Cleared from scene, need to rebuild if trails exist
        
        # Sync Duration Spin (Only initialize if 0 to avoid overwriting user input)
        own_ship = current_project.get_ship_by_idx(current_project.settings.own_ship_idx)
        if own_ship and own_ship.raw_points and self.duration_input.get_seconds() == 0:
            self.duration_input.blockSignals(True)
            self.duration_input.set_seconds(own_ship.total_duration_sec)
            self.duration_input.blockSignals(False)
        
        for ship in current_project.ships:
             if not ship.raw_points: continue 
             
             is_own = (ship.idx == current_project.settings.own_ship_idx)
             
             # Color Logic
             if is_own:
                 c_hex = current_project.settings.own_color
             elif ship.idx >= 1000: # Random Target convention
                 c_hex = current_project.settings.random_color
             else:
                 c_hex = current_project.settings.target_color
                 
             c = QColor(c_hex)
             
             path = QGraphicsPathItem()
             pp = QPainterPath()
             
             points = [QPointF(x,y) for x,y in ship.raw_points]
             if points:
                 pp.moveTo(points[0])
                 mi = current_project.map_info
                 threshold = 180 * mi.pixels_per_degree
                 for i in range(1, len(points)):
                     if abs(points[i].x() - points[i-1].x()) > threshold:
                         pp.moveTo(points[i])
                     else:
                         pp.lineTo(points[i])
             
             path.setPath(pp)
             
             width = current_project.settings.path_thickness
             
             if ship.idx == self.highlighted_ship_idx:
                 c = QColor(current_project.settings.highlight_path_color)
                 width += 1
                 
             pen = QPen(c, width)
             pen.setCosmetic(True); pen.setStyle(Qt.PenStyle.DotLine)
             path.setPen(pen)
             self.scene.addItem(path)
             
             heading = 0.0
             if len(ship.raw_points) > 1:
                 p1 = ship.raw_points[0]
                 p2 = ship.raw_points[1]
                 dx = p2[0] - p1[0]
                 dy = p2[1] - p1[1]
                 heading = math.degrees(math.atan2(dx, -dy))
             
             s = 8.0
             s_half = s / 2.0
             tri_h = s * math.sqrt(3) / 2.0

             ship_poly = QPolygonF([
                 QPointF(s_half, s_half),
                 QPointF(-s_half, s_half),
                 QPointF(-s_half, -s_half),
                 QPointF(0, -s_half - tri_h),
                 QPointF(s_half, -s_half)
             ])

             marker = QGraphicsPolygonItem(ship_poly)
             marker.setBrush(QBrush(c))
             marker.setPen(QPen(Qt.GlobalColor.black, 1))
             marker.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
             marker.setPos(ship.raw_points[0][0], ship.raw_points[0][1])
             marker.setRotation(heading)
             marker.setVisible(True)
             marker.setZValue(10)
             self.scene.addItem(marker)
             self.ship_items[ship.idx] = marker

             if not is_own:
                 ai_s = s * 5
                 ai_h = ai_s * math.sqrt(3) / 2.0
                 ai_poly = QPolygonF([
                     QPointF(0, -2 * ai_h / 3),
                     QPointF(-ai_s / 2, ai_h / 3),
                     QPointF(ai_s / 2, ai_h / 3)
                 ])
                 
                 ind_color = QColor(current_project.settings.random_color) if ship.idx >= 1000 else QColor(current_project.settings.target_color)
                 
                 ai_indicator = QGraphicsPolygonItem(ai_poly)
                 ai_pen = QPen(ind_color, 1.5)
                 ai_pen.setCosmetic(True)
                 ai_indicator.setPen(ai_pen)
                 ai_indicator.setBrush(QBrush(Qt.BrushStyle.NoBrush))
                 ai_indicator.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
                 ai_indicator.setPos(ship.raw_points[0][0], ship.raw_points[0][1])
                 ai_indicator.setRotation(heading)
                 ai_indicator.setVisible(ship.signals_enabled.get('AIVDM', True))
                 ai_indicator.setZValue(9)
                 self.scene.addItem(ai_indicator)

                 ra_d = s * 4.5
                 ra_r = ra_d / 2
                 ra_indicator = QGraphicsEllipseItem(-ra_r, -ra_r, ra_d, ra_d)
                 ra_pen = QPen(ind_color, 1.5)
                 ra_pen.setCosmetic(True)
                 ra_indicator.setPen(ra_pen)
                 ra_indicator.setBrush(QBrush(Qt.BrushStyle.NoBrush))
                 ra_indicator.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
                 ra_indicator.setPos(ship.raw_points[0][0], ship.raw_points[0][1])
                 ra_indicator.setVisible(ship.signals_enabled.get('RATTM', True) or ship.signals_enabled.get('RATLL', True))
                 ra_indicator.setZValue(9)
                 self.scene.addItem(ra_indicator)

                 self.ship_indicators[ship.idx] = {
                     'ai': ai_indicator,
                     'ra': ra_indicator
                 }
             
             if ship.raw_points:
                 start_pt = ship.raw_points[0]
                 end_pt = ship.raw_points[-1]
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
            poly = QPolygonF([QPointF(x,y) for x,y in st.geometry])
            item = QGraphicsPolygonItem(poly)
            item.setPen(QPen(Qt.GlobalColor.black, 2))
            item.setBrush(QBrush(QColor(st.color)))
            self.scene.addItem(item)
            
        self.view.fitInView(self.scene.itemsBoundingRect(), Qt.AspectRatioMode.KeepAspectRatio)
    
    def update_performance_display(self, text):
        import re
        match = re.search(r"SPS: (\d+\.\d+)", text)
        if match:
            sps_value = match.group(1)
            self.perf_label.setText(f"SPS: {sps_value}")
        else:
            self.perf_label.setText("SPS: N/A")

    def sync_view(self):
        main_win = self.window()
        if not hasattr(main_win, 'view'): return
        main_view = main_win.view
        self.view.resetTransform()
        self.view.setTransform(main_view.transform())
        self.view.horizontalScrollBar().setValue(main_view.horizontalScrollBar().value())
        self.view.verticalScrollBar().setValue(main_view.verticalScrollBar().value())

    def set_ui_locked(self, locked):
        self.ip_edit.setEnabled(not locked)
        self.port_edit.setEnabled(not locked)

    def action_start(self):
        if self.worker and self.worker.running and self.worker.paused:
            self.worker.resume()
            self.state_changed.emit("PLAY")
            self.btn_start.setEnabled(False)
            self.btn_pause.setEnabled(True)
            self.btn_stop.setEnabled(True)
            self.btn_export.setEnabled(False)
            return

        # Explicitly stop and clean up any previous worker/thread
        if self.worker_thread:
            try:
                if self.worker_thread.isRunning():
                    self.worker.stop()
                    self.worker_thread.quit()
                    self.worker_thread.wait()
            except RuntimeError: pass # Thread already deleted
            if self.worker:
                self.worker.deleteLater()
            if self.worker_thread:
                self.worker_thread.deleteLater()
            self.worker = None
            self.worker_thread = None
            
        # Clear trails on new start
        self.ship_trails = {}
        self.trail_items = {}
        self.highlighted_ship_idx = -1

        active_ships = [s for s in current_project.ships if s.is_generated]
        if not current_project.ships:
            QMessageBox.warning(self, "Info", "No ships generated.")
            return

        ip = self.ip_edit.text()
        try:
            port = int(self.port_edit.text())
        except:
            QMessageBox.critical(self, "Error", "Invalid Port")
            return
            
        self.worker_thread = QThread()
        self.worker = SimulationWorker(ip, port, self.speed_spin.value(), self.duration_input.get_seconds())
        self.worker.moveToThread(self.worker_thread)
        self.worker_thread.started.connect(self.worker.run)
        self.worker.signal_generated.connect(self.append_log)
        self.worker.log_message.connect(self.append_log)
        self.worker.performance_updated.connect(self.update_performance_display)
        self.worker.positions_updated.connect(self.update_positions)
        self.worker.export_data_ready.connect(self.on_export_data)
        self.worker.finished.connect(self.on_finished)
        
        self.worker_thread.start()
        self.state_changed.emit("PLAY")
        self.btn_start.setEnabled(False)
        self.btn_pause.setEnabled(True)
        self.btn_stop.setEnabled(True)
        self.btn_export.setEnabled(False)
        self.set_ui_locked(True)
        self.duration_input.setEnabled(True) # Allow duration edit during run
        self.btn_add_time.setEnabled(True)
        self.data_ready_flag = False

    def action_pause(self):
        if self.worker:
            self.worker.pause()
            self.state_changed.emit("PAUSE")
            self.btn_start.setEnabled(True)
            self.btn_pause.setEnabled(False)

    def action_stop(self):
        if self.worker:
            self.worker.stop()
            # state_changed("STOP") will be emitted in on_finished

    def on_finished(self):
        if self.worker_thread:
             try:
                 if self.worker_thread.isRunning():
                     self.worker_thread.quit()
                     self.worker_thread.wait()
             except RuntimeError: pass
             self.worker_thread.deleteLater()
             self.worker_thread = None
        
        self.state_changed.emit("STOP")
        if self.worker:
            self.worker.deleteLater()
            self.worker = None
            
        self.btn_start.setEnabled(True)
        self.btn_pause.setEnabled(False)
        self.btn_stop.setEnabled(False)
        self.btn_export.setEnabled(self.data_ready_flag)
        self.set_ui_locked(False)
        self.duration_input.setEnabled(True)
        self.btn_add_time.setEnabled(True)

    def on_speed_change(self, val):
        if self.worker:
            self.worker.update_speed(val)

    def append_log(self, text):
        self.log_list.append(text)
        if len(self.log_list.toPlainText()) > 10000:
            self.log_list.clear()
            
    def update_positions(self, pos_dict):
        # Update Trails
        settings = current_project.settings
        for idx, data in pos_dict.items():
            px, py = data[0], data[1]
            
            if idx not in self.ship_trails:
                self.ship_trails[idx] = []
            
            self.ship_trails[idx].append(QPointF(px, py))
            
            # Create or update trail item
            if idx not in self.trail_items or self.trail_items[idx].scene() != self.scene:
                path_item = QGraphicsPathItem()
                self.scene.addItem(path_item)
                self.trail_items[idx] = path_item
            
            path_item = self.trail_items[idx]
            pp = QPainterPath()
            
            # Handle Date Line Crossing (Wrap-around)
            points = self.ship_trails[idx]
            if points:
                pp.moveTo(points[0])
                mi = current_project.map_info
                threshold = 180 * mi.pixels_per_degree  # Threshold to detect wrap-around
                
                for i in range(1, len(points)):
                    if abs(points[i].x() - points[i-1].x()) > threshold:
                        pp.moveTo(points[i])
                    else:
                        pp.lineTo(points[i])

            path_item.setPath(pp)
            
            if idx == settings.own_ship_idx:
                color = QColor(settings.own_color)
            elif idx >= 1000:
                color = QColor(settings.random_color)
            else:
                color = QColor(settings.target_color)
                
            if idx == self.highlighted_ship_idx:
                color = QColor(settings.highlight_path_color)
            
            pen = QPen(color, settings.traveled_path_thickness)
            pen.setCosmetic(True)
            pen.setStyle(Qt.PenStyle.SolidLine)
            path_item.setPen(pen)
            path_item.setZValue(5) # Below ships (10), above static paths (0)
            
        # Time Remaining Logic (Fallback to Own Ship if no highlight)
        target_idx = self.highlighted_ship_idx
        if target_idx == -1:
             target_idx = current_project.settings.own_ship_idx
        
        # Use Simulation Duration for ETA
        if target_idx != -1:
            if self.worker:
                t_now = 0.0
                if self.worker:
                    t_now = self.worker.current_time
                
                remaining = max(0, ship.total_duration_sec - t_now)
                d = int(remaining // 86400)
                h = int((remaining % 86400) // 3600)
                m = int((remaining % 3600) // 60)
                s = int(remaining % 60)
                
                rem_parts = []
                if d > 0: rem_parts.append(f"{d}d")
                if h > 0: rem_parts.append(f"{h}h")
                if m > 0: rem_parts.append(f"{m}m")
                rem_parts.append(f"{s}s")
                self.eta_label.setText(" ".join(rem_parts))
            else:
                self.eta_label.setText("-")
        else:
            self.eta_label.setText("-")

        for idx, data in pos_dict.items():
            px, py = data[0], data[1]
            heading = data[2]
            
            ship = current_project.get_ship_by_idx(idx)
            if not ship: continue

            if idx in self.ship_items:
                item = self.ship_items[idx]
                item.setPos(px, py)
                item.setRotation(heading)
                item.setVisible(True)

            if idx in self.ship_indicators:
                indicators = self.ship_indicators[idx]
                
                ai_indicator = indicators['ai']
                ai_indicator.setPos(px, py)
                ai_indicator.setRotation(heading)
                ai_indicator.setVisible(ship.signals_enabled.get('AIVDM', True))

                ra_indicator = indicators['ra']
                ra_indicator.setPos(px, py)
                ra_indicator.setVisible(
                    ship.signals_enabled.get('RATTM', True)
                )
        
        follow_idx = self.combo_follow.currentData()
        if follow_idx is not None and follow_idx != -1:
            if follow_idx in pos_dict:
                data = pos_dict[follow_idx]
                px, py = data[0], data[1]
                self.view.centerOn(px, py)

    def on_export_data(self, ready):
        self.data_ready_flag = ready

    def save_log_to_file(self):
        text = self.log_list.toPlainText()
        if not text:
            QMessageBox.information(self, "Info", "저장할 로그 내용이 없습니다.")
            return
            
        path, _ = QFileDialog.getSaveFileName(self, "Save Log", "", "Text Files (*.txt)")
        if not path: return
        
        try:
            with open(path, 'w', encoding='utf-8') as f:
                f.write(text)
            QMessageBox.information(self, "Saved", "로그가 저장되었습니다.")
        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))

    def export_csv(self):
        if not self.data_ready_flag or not self.worker or not self.worker.temp_file_handle:
            QMessageBox.warning(self, "Info", "No data to export.")
            return
        
        own_idx = current_project.settings.own_ship_idx
        own_ship = current_project.get_ship_by_idx(own_idx)
        own_name = own_ship.name if own_ship else "Unknown"
        default_name = f"SignalData_ownship_{own_idx}_{sanitize_filename(own_name)}.csv"
    
        path, _ = QFileDialog.getSaveFileName(self, "Save CSV", 
                    os.path.join(current_project.project_path, default_name), "CSV (*.csv)")
        if not path: return
        
        try:
            self.worker.temp_file_handle.seek(0)
            
            with open(path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(CSV_HEADER)
                
                start_id = 1
                for line in self.worker.temp_file_handle:
                    parts = line.strip().split(',', 1)
                    if len(parts) < 2: continue
                    
                    ts_epoch = float(parts[0])
                    raw = parts[1]
                    ts_str = datetime.datetime.fromtimestamp(ts_epoch, tz=datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
                    
                    fields = parse_nmea_fields(raw)
                    row = [
                        start_id,
                        ts_str,
                        own_idx,
                        own_name,
                        fields.get('talker','') ,
                        fields.get('sentence_type','') ,
                        fields.get('raw','') ,
                        fields.get('lat_deg','') ,
                        fields.get('lon_deg','') ,
                        fields.get('utm_zone','') ,
                        fields.get('utm_hemisphere','') ,
                        fields.get('utm_easting_m','') ,
                        fields.get('utm_northing_m','') ,
                        fields.get('sog_knots','') ,
                        fields.get('cog_true_deg','') ,
                        fields.get('heading_true_deg','')
                    ]
                    writer.writerow(row)
                    start_id += 1
            
            QMessageBox.information(self, "Done", "Exported successfully.")
        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))
        finally:
            pass

    def reset_state(self):
        if self.worker and self.worker.running:
            self.action_stop()
        self.log_list.clear()
        self.log_list.append("Simulation Reset due to path change or project reload.")
        self.draw_static_map()
    
    def cleanup(self):
        if self.worker_thread and self.worker_thread.isRunning():
            self.action_stop() # Request the worker to stop
            self.worker_thread.quit() # Tell the thread's event loop to exit
            self.worker_thread.wait() # Wait for the thread to finish execution
            
            # Clean up the worker and thread objects
            if self.worker:
                self.worker.deleteLater()
            if self.worker_thread:
                self.worker_thread.deleteLater()

    def calculate_ship_state(self, ship, t_current):
        # If worker is running, return dynamic state (convert ECEF to LLA)
        if self.worker and ship.idx in self.worker.dynamic_ships:
            dyn = self.worker.dynamic_ships[ship.idx]
            from app.core.geometry import ecef_to_lla
            lat, lon, _ = ecef_to_lla(dyn['x'], dyn['y'], dyn['z'])
            return {'lat': lat, 'lon': lon, 'spd': dyn['spd'], 'hdg': dyn['hdg']}

        # If not running, return start position
        mi = current_project.map_info
        if ship.resampled_points:
            px, py = ship.resampled_points[0]
            _, _, lat, lon = pixel_to_coords(px, py, mi)
            
            # Initial heading
            hdg = 0.0
            if len(ship.resampled_points) > 1:
                px2, py2 = ship.resampled_points[1]
                _, _, lat2, lon2 = pixel_to_coords(px2, py2, mi)
                d_lat = lat2 - lat
                d_lon = (lon2 - lon) * math.cos(math.radians((lat+lat2)/2))
                hdg = math.degrees(math.atan2(d_lon, d_lat))
                hdg = (hdg + 360) % 360
            
            return {'lat': lat, 'lon': lon, 'spd': ship.raw_speeds.get(0, 0.0), 'hdg': hdg}
            
        return {'lat':0, 'lon':0, 'spd':0, 'hdg':0}

    def show_target_info(self, idx):
        ship = current_project.get_ship_by_idx(idx)
        if not ship: return
        
        t_now = 0.0
        if self.worker and self.worker.running:
            t_now = self.worker.current_time
            
        state = self.calculate_ship_state(ship, t_now)
        sim_limit = self.duration_input.get_seconds()
        
        eta_str = "Unknown"
        if sim_limit > 0:
             remaining = max(0, sim_limit - t_now)
             d = int(remaining // 86400)
             h = int((remaining % 86400) // 3600)
             m = int((remaining % 3600) // 60)
             s = int(remaining % 60)
             
             rem_parts = []
             if d > 0: rem_parts.append(f"{d}d")
             if h > 0: rem_parts.append(f"{h}h")
             if m > 0: rem_parts.append(f"{m}m")
             rem_parts.append(f"{s}s")
             eta_str = " ".join(rem_parts)
        
        # --- Enabled Events & Scenario Info ---
        enabled_events_list = []
        # 1. Project Events
        for e in current_project.events:
            if e.enabled and e.target_ship_idx == idx:
                enabled_events_list.append(f"[Proj] {e.name}")
        
        # 2. Scenario Events
        active_scenarios = []
        if app_state.current_scenario and app_state.current_scenario.enabled:
            scen = app_state.current_scenario
            active_scenarios.append(scen.name)
            
            for e in scen.events:
                if e.enabled:
                    # Scope Check
                    should_apply = False
                    if scen.scope_mode == "ALL_SHIPS": should_apply = True
                    elif scen.scope_mode == "OWN_ONLY":
                        if e.target_ship_idx == current_project.settings.own_ship_idx: should_apply = True
                    elif scen.scope_mode == "TARGET_ONLY":
                        if e.target_ship_idx != current_project.settings.own_ship_idx: should_apply = True
                    elif scen.scope_mode == "SELECTED_SHIPS":
                        if e.target_ship_idx in scen.selected_ships: should_apply = True
                    
                    if should_apply and e.target_ship_idx == idx:
                        enabled_events_list.append(f"[Scen] {e.name}")

        events_str = "<br>".join(enabled_events_list) if enabled_events_list else "None"
        scen_str = ", ".join(active_scenarios) if active_scenarios else "None"
        
        # --- Signal Status ---
        sig_status = []
        for k, v in ship.signals_enabled.items():
            sig_status.append(f"{k}: {'ON' if v else 'OFF'}")
        sig_str = ", ".join(sig_status)

        info = f"""
        <h3>{ship.name} (ID: {ship.idx})</h3>
        <b>MMSI:</b> {ship.mmsi}<br>
        <b>Type:</b> {'Random Target' if ship.idx >= 1000 else 'Manual Target'}<br>
        <hr>
        <b>Sim Time:</b> {t_now:.1f} s<br>
        <b>Lat:</b> {state['lat']:.6f}<br>
        <b>Lon:</b> {state['lon']:.6f}<br>
        <b>Speed:</b> {state['spd']:.1f} kn<br>
        <b>Heading:</b> {state['hdg']:.1f}°<br>
        <b>Remaining:</b> {eta_str}<br>
        <hr>
        <b>Active Scenario:</b> {scen_str}<br>
        <b>Enabled Events:</b><br>
        {events_str}<br>
        <hr>
        <b>Signals:</b> {sig_str}
        """
        
        msg = QMessageBox(self)
        msg.setWindowTitle("Target Detail")
        msg.setText(info)
        msg.setTextFormat(Qt.TextFormat.RichText)
        
        btn_del = msg.addButton("Delete", QMessageBox.ButtonRole.DestructiveRole)
        btn_high = msg.addButton("Highlight Path", QMessageBox.ButtonRole.ActionRole)
        msg.addButton(QMessageBox.StandardButton.Close)
        
        msg.exec()
        
        if msg.clickedButton() == btn_del:
            if QMessageBox.question(self, "Delete Target", f"Delete '{ship.name}'?", 
                                   QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No) == QMessageBox.StandardButton.Yes:
                self.delete_single_target(idx)
        elif msg.clickedButton() == btn_high:
            self.highlighted_ship_idx = idx
            self.draw_static_map() # Redraw to apply highlight style

    def delete_single_target(self, idx):
        current_project.ships = [s for s in current_project.ships if s.idx != idx]
        
        self.refresh_tables()
        self.draw_static_map()
        
        if self.window():
            self.window().update_obj_combo()
            self.window().redraw_map()
            
        if self.worker and self.worker.running:
            self.worker.refresh_active_ships()
