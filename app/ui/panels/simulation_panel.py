import os
import math
import csv
import random
import datetime
import re
import numpy as np

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QLineEdit, 
    QTextEdit, QComboBox, QCheckBox, QSpinBox, QDoubleSpinBox, QTableWidget, 
    QHeaderView, QGroupBox, QMessageBox, QFileDialog, QDialog, QGraphicsScene, QListWidget,
    QGraphicsItem, QGraphicsPathItem, QGraphicsPolygonItem, QGraphicsEllipseItem, 
    QGraphicsTextItem, QAbstractSpinBox, QMenu, QGraphicsRectItem, QFormLayout
)
from PyQt6.QtCore import (
    Qt, pyqtSignal, QThread, QTimer, QPointF
)
from PyQt6.QtGui import (
    QColor, QPen, QBrush, QPainterPath, QPolygonF, QFont
)

from app.core.constants import CSV_HEADER
from app.core.models.project import current_project, ShipData
from app.core.utils import sanitize_filename
from app.core.geometry import coords_to_pixel, pixel_to_coords, normalize_lon
from app.core.nmea import parse_nmea_fields
from app.workers.simulation_workor import SimulationWorker
from app.ui.map.sim_map_view import SimMapView
from app.ui.dialogs.rtg_dialog import RTGDialog
import app.core.state as app_state

class TargetInfoDialog(QDialog):
    def __init__(self, parent, ship_idx, worker):
        super().__init__(parent)
        self.ship_idx = ship_idx
        self.worker = worker
        self.setWindowTitle("Target Detail")
        self.resize(300, 400)
        
        layout = QVBoxLayout(self)
        self.lbl_info = QLabel()
        self.lbl_info.setTextFormat(Qt.TextFormat.RichText)
        layout.addWidget(self.lbl_info)
        
        btn_box = QHBoxLayout()
        self.btn_del = QPushButton("Delete")
        self.btn_high = QPushButton("Highlight Path")
        self.btn_close = QPushButton("Close")
        
        self.btn_del.clicked.connect(self.accept) # Use accept/reject codes to signal action
        self.btn_high.clicked.connect(lambda: self.done(2)) # Custom code 2
        self.btn_close.clicked.connect(self.reject)
        
        btn_box.addWidget(self.btn_del)
        btn_box.addWidget(self.btn_high)
        btn_box.addWidget(self.btn_close)
        layout.addLayout(btn_box)
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_info)
        self.timer.start(500) # Update every 500ms
        self.update_info()

    def update_info(self):
        ship = current_project.get_ship_by_idx(self.ship_idx)
        if not ship:
            self.lbl_info.setText("Ship not found.")
            return
            
        t_now = 0.0
        if self.worker and self.worker.running:
            t_now = self.worker.current_time
            
        # Get dynamic state if available
        state = {'lat':0, 'lon':0, 'spd':0, 'hdg':0}
        if self.worker and self.ship_idx in self.worker.dynamic_ships:
            dyn = self.worker.dynamic_ships[self.ship_idx]
            state = {'lat': dyn['lat'], 'lon': dyn['lon'], 'spd': dyn['spd'], 'hdg': dyn['hdg']}
        elif self.parent().calculate_ship_state:
            state = self.parent().calculate_ship_state(ship, t_now)
            
        eta_str = "-"
        if ship.total_duration_sec > 0:
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
             eta_str = " ".join(rem_parts)
        
        enabled_events_list = []
        for e in current_project.events:
            if e.enabled and e.target_ship_idx == self.ship_idx:
                enabled_events_list.append(f"[Proj] {e.name}")
        
        active_scenarios = []
        if app_state.loaded_scenarios:
            for scen in app_state.loaded_scenarios:
                if scen.enabled:
                    active_scenarios.append(scen.name)
                    for e in scen.events:
                        if e.enabled:
                            should_apply = False
                            if scen.scope_mode == "ALL_SHIPS": should_apply = True
                            elif scen.scope_mode == "OWN_ONLY":
                                if e.target_ship_idx == current_project.settings.own_ship_idx: should_apply = True
                            elif scen.scope_mode == "TARGET_ONLY":
                                if e.target_ship_idx != current_project.settings.own_ship_idx: should_apply = True
                            elif scen.scope_mode == "SELECTED_SHIPS":
                                if e.target_ship_idx in scen.selected_ships: should_apply = True
                            
                            if should_apply and e.target_ship_idx == self.ship_idx:
                                enabled_events_list.append(f"[Scen] {e.name}")

        events_str = "<br>".join(enabled_events_list) if enabled_events_list else "None"
        scen_str = ", ".join(active_scenarios) if active_scenarios else "None"
        
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
        self.lbl_info.setText(info)

class SimulationPanel(QWidget):
    state_changed = pyqtSignal(str)
    simulation_status_updated = pyqtSignal(str, list)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.ship_items = {}
        self.ship_indicators = {}
        self.ship_trails = {} # idx -> list of QPointF
        self.trail_items = {} # idx -> QGraphicsPathItem
        self.planned_paths = {} # idx -> QGraphicsPathItem
        self.highlighted_ship_idx = -1
        self.worker = None
        self.worker_thread = None
        self.is_paused = False
        self.data_ready_flag = False
        self.suppress_close_warning = False
        self.blink_timers = {}
        self.sim_time_limit = 3600
        self.sim_ended = False
        
        # UI elements for simulation time
        self._duration_spinbox = None
        self._ext_duration_le = None
        self._ext_start_btn = None
        
        # Fix for SimMapView right-click crash (MapView expects obj_combo)
        self.obj_combo = QComboBox(self)
        self.obj_combo.setVisible(False)
        
        self.init_ui()

    def set_follow_target(self, idx):
        combo_idx = self.combo_follow.findData(idx)
        if combo_idx >= 0:
            self.combo_follow.setCurrentIndex(combo_idx)

    def action_random_target_generate(self):
        dlg = RTGDialog(self)
        if dlg.exec() == QDialog.DialogCode.Accepted:
            data = dlg.get_data()
            area_id = data.get("area_id", -1)
            R = data["R"]
            N_AI_only = data["N_AI_only"]
            N_RA_only = data["N_RA_only"]
            N_both = data["N_both"]

            own_ship = current_project.get_ship_by_idx(current_project.settings.own_ship_idx)
            if not own_ship:
                QMessageBox.warning(self, "Warning", "Own Ship is not set. Cannot generate random targets.")
                return
            
            if not own_ship.is_generated:
                QMessageBox.warning(self, "Warning", "Own Ship speed/path data is not generated. Please generate speed data in 'Speed Generator' first.")
                return

            self.generate_random_targets_logic(own_ship, R, N_AI_only, N_RA_only, N_both)

    def action_clear_random_targets(self):
        targets = [s for s in current_project.ships if s.idx >= 1000]
        if not targets:
            QMessageBox.information(self, "Info", "No random targets to delete.")
            return

        if QMessageBox.question(self, "Clear Random Targets", f"Delete {len(targets)} random targets?", 
                                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No) != QMessageBox.StandardButton.Yes:
            return

        current_project.ships = [s for s in current_project.ships if s.idx < 1000]
        
        self.refresh_tables()
        self.draw_static_map()
        
        if self.window():
            self.window().update_obj_combo()
            self.window().redraw_map()
            
        if self.worker and self.worker.running:
            self.worker.refresh_active_ships()

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)

        # --- Top Controls (Single Line) ---
        top_controls = QWidget()
        top_layout = QHBoxLayout(top_controls)
        top_layout.setContentsMargins(0, 0, 0, 0)
        
        # IP/Port
        self.ip_edit = QLineEdit("127.0.0.1"); self.ip_edit.setFixedWidth(100)
        self.port_edit = QLineEdit("10110"); self.port_edit.setFixedWidth(60)
        top_layout.addWidget(QLabel("IP:"))
        top_layout.addWidget(self.ip_edit)
        top_layout.addWidget(QLabel("Port:"))
        top_layout.addWidget(self.port_edit)
        
        # Speed
        self.speed_spin = QSpinBox()
        self.speed_spin.setRange(1, 86400)
        self.speed_spin.setValue(1)
        self.speed_spin.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.speed_spin.setFixedWidth(50)
        self.speed_spin.valueChanged.connect(self.on_speed_change)
        top_layout.addWidget(QLabel("Speed(x):"))
        top_layout.addWidget(self.speed_spin)
        
        # Duration
        self.spin_sim_duration = QSpinBox()
        self.spin_sim_duration.setRange(10, 864000)
        self.spin_sim_duration.setValue(3600)
        self.spin_sim_duration.setSuffix(" s")
        self.spin_sim_duration.setToolTip("Simulation Duration")
        top_layout.addWidget(QLabel("Duration:"))
        top_layout.addWidget(self.spin_sim_duration)

        # Extra Time
        self.spin_extra_time = QSpinBox()
        self.spin_extra_time.setRange(10, 36000)
        self.spin_extra_time.setValue(600)
        self.spin_extra_time.setSuffix(" s")
        self.spin_extra_time.setEnabled(False)
        self.btn_add_time = QPushButton("+")
        self.btn_add_time.setFixedWidth(30)
        self.btn_add_time.setEnabled(False)
        self.btn_add_time.clicked.connect(self.action_add_extra_time)
        top_layout.addWidget(QLabel("Extra:"))
        top_layout.addWidget(self.spin_extra_time)
        top_layout.addWidget(self.btn_add_time)

        # Follow
        self.combo_follow = QComboBox()
        self.combo_follow.addItem("None", -1)
        top_layout.addWidget(QLabel("Follow:"))
        top_layout.addWidget(self.combo_follow)
        
        # Show Paths (New)
        self.chk_show_paths = QCheckBox("Show Paths")
        self.chk_show_paths.setChecked(True)
        self.chk_show_paths.toggled.connect(self.draw_static_map)
        top_layout.addWidget(self.chk_show_paths)
        
        # Capture
        self.btn_capture = QPushButton("Capture")
        self.btn_capture.clicked.connect(self.action_capture)
        top_layout.addWidget(self.btn_capture)
        
        # Start/Pause/Stop
        self.btn_start = QPushButton("Start")
        self.btn_pause = QPushButton("Pause")
        self.btn_stop = QPushButton("Stop")
        self.btn_pause.setEnabled(False)
        self.btn_stop.setEnabled(False)
        self.btn_start.clicked.connect(self.action_start)
        self.btn_pause.clicked.connect(self.action_pause)
        self.btn_stop.clicked.connect(self.action_stop)
        
        top_layout.addWidget(self.btn_start)
        top_layout.addWidget(self.btn_pause)
        top_layout.addWidget(self.btn_stop)
        
        # RTG
        self.btn_rtg = QPushButton("Generate Random Target")
        self.btn_rtg.setToolTip("Generate Random Target")
        self.btn_rtg.clicked.connect(self.action_random_target_generate)
        self.btn_clear_rtg = QPushButton("Clear Random Target")
        self.btn_clear_rtg.clicked.connect(self.action_clear_random_targets)
        top_layout.addWidget(self.btn_rtg)
        top_layout.addWidget(self.btn_clear_rtg)
        
        # Info
        self.perf_label = QLabel("SPS: -")
        self.eta_label = QLabel("Rem: -")
        top_layout.addWidget(self.perf_label)
        top_layout.addWidget(self.eta_label)
        
        top_layout.addStretch()
        
        layout.addWidget(top_controls)
        
        # --- Middle Area (Map + Right Panel) ---
        mid_widget = QWidget()
        mid_layout = QHBoxLayout(mid_widget)
        mid_layout.setContentsMargins(0, 0, 0, 0)
        
        # Map
        self.scene = QGraphicsScene()
        self.view = SimMapView(self.scene, self)
        self.view.mode = "VIEW"
        self.view.view_changed.connect(self.update_pen_widths)
        mid_layout.addWidget(self.view, 3)
        
        # Right Panel (Tables + Log)
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)
        
        self.status_group = QGroupBox("Simulation Status")
        status_layout = QVBoxLayout(self.status_group)
        self.lbl_active_scen = QLabel("Active Scenario: None")
        
        self.edit_status_filter = QLineEdit()
        self.edit_status_filter.setPlaceholderText("Filter events...")
        self.edit_status_filter.textChanged.connect(self.emit_simulation_status)
        
        self.list_active_events = QListWidget()
        self.list_active_events.setFixedHeight(80)
        self.list_active_events.itemDoubleClicked.connect(self.on_event_list_double_clicked)
        status_layout.addWidget(self.lbl_active_scen)
        status_layout.addWidget(self.edit_status_filter)
        status_layout.addWidget(self.list_active_events)
        right_layout.addWidget(self.status_group)

        self.on_off_group = QGroupBox("Signal On/Off")
        on_off_layout = QVBoxLayout(self.on_off_group)
        self.on_off_table = self.create_signal_on_off_table()
        on_off_layout.addWidget(self.on_off_table)
        right_layout.addWidget(self.on_off_group)
        
        self.interval_group = QGroupBox("Signal Interval (sec)")
        interval_layout = QVBoxLayout(self.interval_group)
        self.interval_table = self.create_signal_interval_table()
        interval_layout.addWidget(self.interval_table)
        right_layout.addWidget(self.interval_group)
        
        log_group = QGroupBox("Raw NMEA Output")
        log_layout = QVBoxLayout(log_group)
        
        filter_layout = QHBoxLayout()
        filter_layout.addWidget(QLabel("Filter:"))
        self.log_filter_combo = QComboBox()
        self.log_filter_combo.addItems(["ALL", "AIVDM", "RATTM"])
        filter_layout.addWidget(self.log_filter_combo)
        
        self.log_keyword_edit = QLineEdit()
        self.log_keyword_edit.setPlaceholderText("Keyword filter...")
        filter_layout.addWidget(self.log_keyword_edit)
        
        self.chk_auto_scroll = QCheckBox("Auto Scroll")
        self.chk_auto_scroll.setChecked(True)
        filter_layout.addWidget(self.chk_auto_scroll)
        
        filter_layout.addStretch()
        log_layout.addLayout(filter_layout)

        self.log_list = QTextEdit()
        self.log_list.setReadOnly(True)
        self.log_list.document().setMaximumBlockCount(1000)
        self.log_list.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.log_list.customContextMenuRequested.connect(self.show_log_context_menu)
        log_layout.addWidget(self.log_list)

        right_layout.addWidget(log_group, 1)
        
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
        
        mid_layout.addWidget(right_panel, 1)
        layout.addWidget(mid_widget)
        
        self.draw_static_map()
        self.update_follow_combo()

    def generate_random_targets_logic(self, own_ship, R_nm, N_ai, N_ra, N_both, area_id=-1):
        # Set deterministic seed for RTG based on project seed and current ship count
        # This ensures that if we reload and generate, we get the same targets.
        seed_val = (current_project.seed + len(current_project.ships)) % (2**32 - 1)
        random.seed(seed_val)

        # 1. Determine Reference Time and State
        t_now = 0.0
        if self.worker and self.worker.running:
            t_now = self.worker.current_time
        
        # Get Own Ship state at t_now
        own_lat, own_lon, own_spd_kn = 0.0, 0.0, 0.0
        
        # Find index in own_ship data corresponding to t_now
        if own_ship.cumulative_time:
            times = np.array(own_ship.cumulative_time)
            idx = np.searchsorted(times, t_now)
            if idx >= len(times): idx = len(times) - 1
            
            mi = current_project.map_info
            px, py = own_ship.resampled_points[idx] # This is pixel
            _, _, own_lat, own_lon = pixel_to_coords(px, py, mi)
            own_spd_kn = own_ship.node_velocities_kn[idx]

        # 2. Generation Loop
        total_targets = N_ai + N_ra + N_both
        mi = current_project.map_info
        
        target_area_poly = None
        area_bbox = None
        if area_id != -1:
            area = current_project.get_area_by_id(area_id)
            if area:
                poly_pts = [QPointF(px, py) for px, py in area.geometry]
                target_area_poly = QPolygonF(poly_pts)
                area_bbox = target_area_poly.boundingRect()

        # R in degrees (Approx)
        # 1 nm = 1/60 degree lat
        R_deg_lat = R_nm / 60.0
        
        for i in range(total_targets):
            # Determine Type
            if i < N_ai: s_type = "AI"
            elif i < N_ai + N_ra: s_type = "RA"
            else: s_type = "BOTH"
            
            # 3.2 Start Position (Uniform in Circle)
            if target_area_poly:
                found = False
                for _ in range(200):
                    rx = random.uniform(area_bbox.left(), area_bbox.right())
                    ry = random.uniform(area_bbox.top(), area_bbox.bottom())
                    if target_area_poly.containsPoint(QPointF(rx, ry), Qt.FillRule.OddEvenFill):
                        _, _, start_lat, start_lon = pixel_to_coords(rx, ry, mi)
                        found = True
                        break
                if not found:
                    c = area_bbox.center()
                    _, _, start_lat, start_lon = pixel_to_coords(c.x(), c.y(), mi)
            else:
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
            
            # 3.5 Speed Model
            variance = current_project.settings.random_target_speed_variance
            sigma = math.sqrt(variance)
            spd = abs(random.gauss(own_spd_kn, sigma)) # Abs as per requirement
            if spd < 0.1: spd = 0.1
            
            # 3.4 Motion Model (Straight Line)
            heading = random.random() * 360.0
            
            # Generate Path from t_now to End of Own Ship Duration
            duration_remaining = own_ship.total_duration_sec - t_now
            if duration_remaining < 1.0: duration_remaining = 1.0
            
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
            
            raw_points_geo = [] # (lat, lon)
            curr_lat, curr_lon = start_lat, start_lon
            curr_heading = heading
            
            px_start, py_start = coords_to_pixel(start_lat, start_lon, mi)
            raw_pixels = [(px_start, py_start)]
            times = [t_now]
            
            total_actual_dur = 0.0
            current_t = t_now
            
            for k, seg_dur in enumerate(segments):
                dist_nm = spd * (seg_dur / 3600.0)
                dist_deg = dist_nm / 60.0
                
                end_lat_seg = curr_lat + dist_deg * math.cos(math.radians(curr_heading))
                
                # Boundary check
                ratio = 1.0
                hit_boundary = False
                if end_lat_seg > 90:
                    ratio = (90 - curr_lat) / (end_lat_seg - curr_lat) if end_lat_seg != curr_lat else 0
                    end_lat_seg = 90
                    hit_boundary = True
                elif end_lat_seg < -90:
                    ratio = (-90 - curr_lat) / (end_lat_seg - curr_lat) if end_lat_seg != curr_lat else 0
                    end_lat_seg = -90
                    hit_boundary = True
                
                actual_dist_deg = dist_deg * ratio
                mid_lat = (curr_lat + end_lat_seg) / 2.0
                cos_mid = math.cos(math.radians(mid_lat))
                if abs(cos_mid) < 0.01: cos_mid = 0.01
                
                end_lon_seg = curr_lon + (actual_dist_deg * math.sin(math.radians(curr_heading)) / cos_mid)
                
                px, py = coords_to_pixel(end_lat_seg, end_lon_seg, mi)
                raw_pixels.append((px, py))
                
                current_t += (seg_dur * ratio)
                times.append(current_t)

                curr_lat = end_lat_seg
                curr_lon = end_lon_seg
                total_actual_dur += seg_dur * ratio
                
                if hit_boundary: break
                
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
            
            # 2. Populate Simulation Data (Essential for Immediate Visualization)
            # Optimized: Use waypoints directly instead of dense resampling.
            # The simulation worker interpolates linearly between these points.
            new_ship.resampled_points = raw_pixels
            new_ship.cumulative_time = times
            new_ship.node_velocities_kn = [spd] * len(times)
            new_ship.total_duration_sec = times[-1]
            
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
        
        self.update_indicators_visibility()
    
    def _set_on_off_checkbox(self, r, c, checked):
        widget = self.on_off_table_ref.cellWidget(r, c)
        if widget:
            cb = widget.findChild(QCheckBox)
            if cb:
                cb.blockSignals(True)
                cb.setChecked(checked)
                cb.blockSignals(False)

    def update_indicators_visibility(self):
        for idx, indicators in self.ship_indicators.items():
            ship = current_project.get_ship_by_idx(idx)
            if not ship: continue
            
            if 'ai' in indicators:
                indicators['ai'].setVisible(ship.signals_enabled.get('AIVDM', True))
            if 'ra' in indicators:
                indicators['ra'].setVisible(ship.signals_enabled.get('RATTM', True) or ship.signals_enabled.get('RATLL', True))

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
        self.planned_paths.clear()
        self.add_boundary_masks()
        self.stop_all_blinks()
        
        scale = self.view.transform().m11()
        if scale <= 0: scale = 1.0

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
             pp.addPolygon(QPolygonF([QPointF(x,y) for x,y in ship.raw_points]))
             path.setPath(pp)
             
             width = current_project.settings.path_thickness
             
             if ship.idx == self.highlighted_ship_idx:
                 c = QColor(current_project.settings.highlight_path_color)
                 width += 1
                 
             pen = QPen(c, width)
             pen.setCosmetic(False)
             pen.setWidthF(width / scale)
             pen.setStyle(Qt.PenStyle.CustomDashLine)
             pen.setDashPattern([1, 4])
             path.setPen(pen)
             if self.chk_show_paths.isChecked():
                 self.scene.addItem(path)
             self.planned_paths[ship.idx] = path
             
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
                 
                 tf = QGraphicsTextItem("f")
                 tf.setDefaultTextColor(Qt.GlobalColor.black)
                 tf.setPos(end_pt[0] + 5, end_pt[1] - 25)
                 tf.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
                 tf.setZValue(3)

                 if self.chk_show_paths.isChecked():
                     self.scene.addItem(ti)
                     self.scene.addItem(tf)

        for st in current_project.areas:
            poly = QPolygonF([QPointF(x,y) for x,y in st.geometry])
            item = QGraphicsPolygonItem(poly)
            item.setPen(QPen(Qt.GlobalColor.black, 2))
            item.setBrush(QBrush(QColor(st.color)))
            self.scene.addItem(item)
            
        # Restore Dynamic State (Trails and Positions)
        settings = current_project.settings
        current_ship_idxs = {s.idx for s in current_project.ships}
        
        scale = self.view.transform().m11()
        if scale <= 0: scale = 1.0
        
        # 1. Restore Trails
        for idx, points in self.ship_trails.items():
            if idx not in current_ship_idxs: continue
            if not points: continue
            
            path_item = QGraphicsPathItem()
            pp = QPainterPath()
            pp.addPolygon(QPolygonF(points))
            path_item.setPath(pp)
            
            if idx == settings.own_ship_idx: color = QColor(settings.own_color)
            elif idx >= 1000: color = QColor(settings.random_color)
            else: color = QColor(settings.target_color)
            
            if idx == self.highlighted_ship_idx:
                color = QColor(settings.highlight_path_color)
            
            pen = QPen(color, settings.traveled_path_thickness)
            pen.setCosmetic(False)
            pen.setWidthF(settings.traveled_path_thickness / scale)
            pen.setStyle(Qt.PenStyle.SolidLine)
            path_item.setPen(pen)
            path_item.setZValue(5)
            path_item.setVisible(self.chk_show_paths.isChecked())
            self.scene.addItem(path_item)
            self.trail_items[idx] = path_item

        # 2. Restore Positions
        if self.worker:
             t_now = self.worker.current_time
             mi = current_project.map_info
             
             for s in current_project.ships:
                 if not s.is_generated: continue
                 st = self.calculate_ship_state(s, t_now)
                 px, py = coords_to_pixel(st['lat'], st['lon'], mi)
                 heading = st['hdg']
                 
                 if s.idx in self.ship_items:
                     self.ship_items[s.idx].setPos(px, py)
                     self.ship_items[s.idx].setRotation(heading)
                 
                 if s.idx in self.ship_indicators:
                     inds = self.ship_indicators[s.idx]
                     inds['ai'].setPos(px, py)
                     inds['ai'].setRotation(heading)
                     inds['ra'].setPos(px, py)
    
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

    def update_performance_display(self, text):
        import re
        match = re.search(r"SPS: (\d+\.\d+)", text)
        if match:
            sps_value = match.group(1)
            self.perf_label.setText(f"SPS: {sps_value}")
        else:
            self.perf_label.setText("SPS: N/A")

    def update_pen_widths(self):
        scale = self.view.transform().m11()
        if scale <= 0: scale = 1.0
        
        settings = current_project.settings
        
        # Planned Paths
        for idx, item in self.planned_paths.items():
            width = settings.path_thickness
            if idx == self.highlighted_ship_idx: width += 1
            pen = item.pen()
            pen.setWidthF(width / scale)
            item.setPen(pen)
            
        # Trails
        for idx, item in self.trail_items.items():
            width = settings.traveled_path_thickness
            pen = item.pen()
            pen.setWidthF(width / scale)
            item.setPen(pen)

    def set_ui_locked(self, locked):
        self.ip_edit.setEnabled(not locked)
        self.port_edit.setEnabled(not locked)
        self.spin_sim_duration.setEnabled(not locked)
        # Extra time controls are managed separately based on sim state

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
        self.stop_all_blinks()
            
        # Clear trails on new start
        self.ship_trails = {}
        self.trail_items = {}
        self.highlighted_ship_idx = -1
            
        # Reset map to initial state (t=0)
        self.draw_static_map()

        active_ships = [s for s in current_project.ships if s.is_generated]
        if not active_ships:
            QMessageBox.warning(self, "Info", "No ships generated.")
            return
            
        # Set Simulation Limit
        self.sim_time_limit = self.spin_sim_duration.value()
        self.sim_ended = False
        self.spin_extra_time.setEnabled(False)
        self.btn_add_time.setEnabled(False)

        ip = self.ip_edit.text()
        try:
            port = int(self.port_edit.text())
        except:
            QMessageBox.critical(self, "Error", "Invalid Port")
            return
            
        self.worker_thread = QThread()
        self.worker = SimulationWorker(ip, port, self.speed_spin.value(), self.spin_sim_duration.value())
        self.worker.moveToThread(self.worker_thread)
        self.worker_thread.started.connect(self.worker.run)
        self.worker.finished.connect(self.worker_thread.quit)
        self.worker.signal_generated.connect(self.append_log)
        self.worker.log_message.connect(self.append_log)
        self.worker.performance_updated.connect(self.update_performance_display)
        self.worker.positions_updated.connect(self.update_positions)
        self.worker.export_data_ready.connect(self.on_export_data)
        self.worker.finished.connect(self.on_finished)
        self.worker.event_triggered.connect(self.on_event_triggered)
        
        self.worker_thread.start()
        self.state_changed.emit("PLAY")
        self.btn_start.setEnabled(False)
        self.btn_pause.setEnabled(True)
        self.btn_stop.setEnabled(True)
        self.btn_export.setEnabled(False)
        self.set_ui_locked(True)
        self.data_ready_flag = False

    def action_add_extra_time(self):
        extra = self.spin_extra_time.value()
        self.sim_time_limit += extra
        if self.worker:
            self.worker.update_duration(self.sim_time_limit)
        self.sim_ended = False
        self.spin_extra_time.setEnabled(False)
        self.btn_add_time.setEnabled(False)
        # User can now click Start/Resume to continue
        self.btn_start.setEnabled(True)

    def action_capture(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save Map Capture", "", "PNG Image (*.png);;JPEG Image (*.jpg)")
        if not path: return
        
        pixmap = self.view.grab()
        if pixmap.save(path):
            QMessageBox.information(self, "Saved", f"Map captured to {os.path.basename(path)}")
        else:
            QMessageBox.warning(self, "Error", "Failed to save image.")

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
            
        self.btn_start.setEnabled(True)
        self.btn_pause.setEnabled(False)
        self.btn_stop.setEnabled(False)
        self.btn_export.setEnabled(self.data_ready_flag)
        self.set_ui_locked(False)
        self.stop_all_blinks()
        self.spin_extra_time.setEnabled(False)
        self.btn_add_time.setEnabled(False)

    def on_speed_change(self, val):
        if self.worker:
            self.worker.update_speed(val)

    def append_log(self, text):
        filter_val = self.log_filter_combo.currentText()
        if filter_val != "ALL" and filter_val not in text:
            return
            
        keyword = self.log_keyword_edit.text().strip()
        if keyword and keyword.lower() not in text.lower():
            return

        if not self.chk_auto_scroll.isChecked():
            sb = self.log_list.verticalScrollBar()
            old_val = sb.value()
            self.log_list.append(text)
            sb.setValue(old_val)
        else:
            self.log_list.append(text)
            sb = self.log_list.verticalScrollBar()
            sb.setValue(sb.maximum())

    def show_log_context_menu(self, pos):
        menu = QMenu(self)
        
        has_selection = bool(self.log_list.textCursor().hasSelection())
        
        action_copy = menu.addAction("Copy")
        action_copy.setEnabled(has_selection)
        action_copy.triggered.connect(self.log_list.copy)
        
        menu.addSeparator()
        
        action_select_all = menu.addAction("Select All")
        action_select_all.triggered.connect(self.log_list.selectAll)
        
        menu.addSeparator()
        action_clear = menu.addAction("Clear")
        action_clear.triggered.connect(self.log_list.clear)
        
        menu.exec(self.log_list.mapToGlobal(pos))
            
    def update_positions(self, pos_dict):
        # Check Simulation Time Limit
        if self.worker and not self.sim_ended:
            if self.worker.current_time >= self.sim_time_limit:
                self.sim_ended = True
                self.worker.pause()
                self.state_changed.emit("PAUSE")
                self.btn_start.setEnabled(True)
                self.btn_pause.setEnabled(False)
                
                self.spin_extra_time.setEnabled(True)
                self.btn_add_time.setEnabled(True)

        # Update Trails
        settings = current_project.settings
        scale = self.view.transform().m11()
        if scale <= 0: scale = 1.0

        for idx, (px, py, _) in pos_dict.items():
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
            pp.addPolygon(QPolygonF(self.ship_trails[idx]))
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
            pen.setCosmetic(False)
            pen.setWidthF(settings.traveled_path_thickness / scale)
            pen.setStyle(Qt.PenStyle.SolidLine)
            path_item.setPen(pen)
            path_item.setZValue(5) # Below ships (10), above static paths (0)
            
        # Time Remaining Logic (Fallback to Own Ship if no highlight)
        target_idx = self.highlighted_ship_idx
        if target_idx == -1:
             target_idx = current_project.settings.own_ship_idx
             
        if target_idx != -1:
            ship = current_project.get_ship_by_idx(target_idx)
            if ship and ship.total_duration_sec > 0:
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

        for idx, (px, py, heading) in pos_dict.items():
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
                px, py, _ = pos_dict[follow_idx]
                self.view.centerOn(px, py)
        
        self.emit_simulation_status()
        self.scene.update()

    def emit_simulation_status(self):
        active_scen_names = []
        events_list = []
        
        # Project Events
        for e in current_project.events:
            if e.enabled:
                ship = current_project.get_ship_by_idx(e.target_ship_idx)
                s_name = ship.name if ship else "Unknown"
                events_list.append(f"[Proj] {e.name} (Tgt: {s_name})")
        
        # Scenario Events
        if app_state.loaded_scenarios:
            for scen in app_state.loaded_scenarios:
                if scen.enabled:
                    active_scen_names.append(scen.name)
                    for e in scen.events:
                        if e.enabled:
                            should_apply = False
                            if scen.scope_mode == "ALL_SHIPS": should_apply = True
                            elif scen.scope_mode == "OWN_ONLY":
                                if e.target_ship_idx == current_project.settings.own_ship_idx: should_apply = True
                            elif scen.scope_mode == "TARGET_ONLY":
                                if e.target_ship_idx != current_project.settings.own_ship_idx: should_apply = True
                            elif scen.scope_mode == "SELECTED_SHIPS":
                                if e.target_ship_idx in scen.selected_ships: should_apply = True
                            
                            if should_apply:
                                ship = current_project.get_ship_by_idx(e.target_ship_idx)
                                s_name = ship.name if ship else "Unknown"
                                events_list.append(f"[Scen] {e.name} (Tgt: {s_name})")
                        
        scen_text = ", ".join(active_scen_names) if active_scen_names else "None"
        self.lbl_active_scen.setText(f"Active Scenario: {scen_text}")
        
        filter_text = self.edit_status_filter.text().lower()
        filtered_events = [e for e in events_list if filter_text in e.lower()]
        
        self.list_active_events.clear()
        self.list_active_events.addItems(filtered_events)

        self.simulation_status_updated.emit(scen_text, events_list)

    def on_event_list_double_clicked(self, item):
        text = item.text()
        # Extract target ship name from string like "[Proj] EventName (Tgt: ShipName)"
        match = re.search(r"\(Tgt: (.*)\)", text)
        if match:
            ship_name = match.group(1)
            target_ship = next((s for s in current_project.ships if s.name == ship_name), None)
            
            if target_ship:
                # Try to get position from current visual items first (most accurate for display)
                if target_ship.idx in self.ship_items:
                    item = self.ship_items[target_ship.idx]
                    self.view.centerOn(item.pos())
                    return

                # Fallback if not currently visible/simulated
                mi = current_project.map_info
                px, py = coords_to_pixel(target_ship.raw_points[0][0], target_ship.raw_points[0][1], mi) if target_ship.raw_points else (0, 0)
                self.view.centerOn(px, py)

    def on_export_data(self, ready):
        self.data_ready_flag = ready

    def save_log_to_file(self):
        text = self.log_list.toPlainText()
        if not text:
            QMessageBox.information(self, "Info", "No log content to save.")
            return
            
        path, _ = QFileDialog.getSaveFileName(self, "Save Log", "", "Text Files (*.txt)")
        if not path: return
        
        try:
            with open(path, 'w', encoding='utf-8') as f:
                f.write(text)
            QMessageBox.information(self, "Saved", "Log saved.")
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
                        fields.get('raw','')
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
        self.stop_all_blinks()
    
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
        self.stop_all_blinks()

    def calculate_ship_state(self, ship, t_current):
        if not ship.cumulative_time:
            return {'lat':0, 'lon':0, 'spd':0, 'hdg':0}
            
        times = np.array(ship.cumulative_time)
        if t_current >= times[-1]:
            idx = len(times) - 2
            alpha = 1.0
        elif t_current <= times[0]:
            idx = 0
            alpha = 0.0
        else:
            idx = np.searchsorted(times, t_current, side='right') - 1
            t1 = times[idx]
            t2 = times[idx+1]
            alpha = (t_current - t1) / (t2 - t1) if t2 > t1 else 0.0
            
        mi = current_project.map_info
        p1 = ship.resampled_points[idx]
        p2 = ship.resampled_points[idx+1]
        
        px = p1[0] + (p2[0] - p1[0]) * alpha
        py = p1[1] + (p2[1] - p1[1]) * alpha
        
        _, _, lat, lon = pixel_to_coords(px, py, mi)
        
        v1 = ship.node_velocities_kn[idx]
        v2 = ship.node_velocities_kn[idx+1]
        
        if alpha >= 1.0 and v2 == 0:
            spd = v1 # Keep momentum at the end (Technically not 0)
        else:
            spd = v1 + (v2 - v1) * alpha
        
        # Calculate Heading
        _, _, lat1, lon1 = pixel_to_coords(p1[0], p1[1], mi)
        _, _, lat2, lon2 = pixel_to_coords(p2[0], p2[1], mi)
        d_lat = lat2 - lat1
        d_lon = (lon2 - lon1) * math.cos(math.radians((lat1+lat2)/2))
        hdg = math.degrees(math.atan2(d_lon, d_lat))
        hdg = (hdg + 360) % 360
        
        return {'lat': lat, 'lon': normalize_lon(lon), 'spd': spd, 'hdg': hdg}

    def show_target_info(self, idx):
        ship = current_project.get_ship_by_idx(idx)
        if not ship: return

        dlg = TargetInfoDialog(self, idx, self.worker)
        res = dlg.exec()
        
        if res == QDialog.DialogCode.Accepted: # Delete
            if QMessageBox.question(self, "Delete Target", f"Delete '{ship.name}'?", 
                                   QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No) == QMessageBox.StandardButton.Yes:
                self.delete_single_target(idx)
        elif res == 2: # Highlight
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

    def on_event_triggered(self, event_name, ship_idx):
        if ship_idx in self.blink_timers:
            self.blink_timers[ship_idx].stop()
            
        timer = QTimer(self)
        timer.setInterval(250)
        count = 0
        
        def tick():
            nonlocal count
            count += 1
            if ship_idx not in self.ship_items:
                timer.stop()
                if ship_idx in self.blink_timers: del self.blink_timers[ship_idx]
                return

            item = self.ship_items[ship_idx]
            if count > 8: # 4 blinks (On/Off * 4)
                timer.stop()
                self.restore_ship_color(ship_idx)
                if ship_idx in self.blink_timers: del self.blink_timers[ship_idx]
                return
            
            if count % 2 == 1:
                item.setBrush(QBrush(Qt.GlobalColor.red))
            else:
                self.restore_ship_color(ship_idx)
        
        timer.timeout.connect(tick)
        self.blink_timers[ship_idx] = timer
        timer.start()

    def restore_ship_color(self, ship_idx):
        if ship_idx not in self.ship_items: return
        item = self.ship_items[ship_idx]
        is_own = (ship_idx == current_project.settings.own_ship_idx)
        if is_own: c_hex = current_project.settings.own_color
        elif ship_idx >= 1000: c_hex = current_project.settings.random_color
        else: c_hex = current_project.settings.target_color
        item.setBrush(QBrush(QColor(c_hex)))

    def stop_all_blinks(self):
        for t in self.blink_timers.values():
            t.stop()
        self.blink_timers.clear()
