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
    QHeaderView, QGroupBox, QFileDialog, QDialog, QGraphicsScene, QListWidget,
    QGraphicsItem, QGraphicsPathItem, QGraphicsPolygonItem, QGraphicsEllipseItem,
    QGraphicsTextItem, QAbstractSpinBox, QMenu, QGraphicsRectItem, QFormLayout, QInputDialog, QTabWidget,
    QGridLayout, QScrollArea
)
from PyQt6.QtCore import (
    Qt, pyqtSignal, QThread, QTimer, QPointF
)
from PyQt6.QtGui import (
    QColor, QPen, QBrush, QPainterPath, QPolygonF, QFont
)
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.ticker as ticker

from app.core.constants import CSV_HEADER
from app.core.models.project import current_project, ShipData
from app.core.utils import sanitize_filename
from app.core.geometry import coords_to_pixel, pixel_to_coords, normalize_lon
from app.core.nmea import parse_nmea_fields
from app.workers.simulation_worker import SimulationWorker
from app.ui.map.sim_map_view import SimMapView
from app.ui.dialogs.rtg_dialog import RTGDialog
from app.ui.widgets.time_input_widget import TimeInputWidget
from app.ui.widgets.panorama_view import PanoramaView
from app.ui.widgets import message_box as msgbox
import app.core.state as app_state

class TargetInfoDialog(QDialog):
    # Panorama constants (matching simulation_worker.py)
    PANO_W_PX = 1920
    PANO_H_PX = 320
    K_H = 2200
    H_MIN, H_MAX = 6, 220
    W_MIN, W_MAX = 6, 300
    CY_RATIO = 0.60

    def __init__(self, parent, ship_idx, worker):
        super().__init__(parent)
        self.ship_idx = ship_idx
        self.worker = worker
        self.setWindowTitle("Target Detail")
        self.resize(650, 750)  # Increased size for more info

        layout = QVBoxLayout(self)

        # Use scroll area for all content
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)

        # Basic Info Group
        basic_group = QGroupBox("Basic Information")
        basic_layout = QVBoxLayout(basic_group)
        self.lbl_basic_info = QLabel()
        self.lbl_basic_info.setTextFormat(Qt.TextFormat.RichText)
        basic_layout.addWidget(self.lbl_basic_info)
        scroll_layout.addWidget(basic_group)

        # Navigation Info Group
        nav_group = QGroupBox("Navigation Status")
        nav_layout = QVBoxLayout(nav_group)
        self.lbl_nav_info = QLabel()
        self.lbl_nav_info.setTextFormat(Qt.TextFormat.RichText)
        nav_layout.addWidget(self.lbl_nav_info)
        scroll_layout.addWidget(nav_group)

        # Relative Info Group (relative to OwnShip)
        rel_group = QGroupBox("Relative to OwnShip")
        rel_layout = QVBoxLayout(rel_group)
        self.lbl_rel_info = QLabel()
        self.lbl_rel_info.setTextFormat(Qt.TextFormat.RichText)
        rel_layout.addWidget(self.lbl_rel_info)
        scroll_layout.addWidget(rel_group)

        # Ship Dimensions Group
        dim_group = QGroupBox("Ship Dimensions")
        dim_layout = QVBoxLayout(dim_group)
        self.lbl_dim_info = QLabel()
        self.lbl_dim_info.setTextFormat(Qt.TextFormat.RichText)
        dim_layout.addWidget(self.lbl_dim_info)
        scroll_layout.addWidget(dim_group)

        # Signal Status Group
        sig_group = QGroupBox("Signal Status")
        sig_layout = QVBoxLayout(sig_group)
        self.lbl_sig_info = QLabel()
        self.lbl_sig_info.setTextFormat(Qt.TextFormat.RichText)
        sig_layout.addWidget(self.lbl_sig_info)
        scroll_layout.addWidget(sig_group)

        # Event Status Group
        event_group = QGroupBox("Event Status")
        event_layout = QVBoxLayout(event_group)
        self.lbl_event_info = QLabel()
        self.lbl_event_info.setTextFormat(Qt.TextFormat.RichText)
        event_layout.addWidget(self.lbl_event_info)
        scroll_layout.addWidget(event_group)

        # Panorama view for camera detections
        pano_group = QGroupBox("Camera Panorama View (-90° to +90°)")
        pano_layout = QVBoxLayout(pano_group)
        self.panorama_view = PanoramaView()
        self.panorama_view.setMinimumHeight(120)
        pano_layout.addWidget(self.panorama_view)
        scroll_layout.addWidget(pano_group)

        scroll.setWidget(scroll_content)
        layout.addWidget(scroll)

        # Button box at bottom (outside scroll area)
        btn_box = QHBoxLayout()
        self.btn_spd = QPushButton("Change Speed")
        self.btn_hdg = QPushButton("Change Heading")
        self.btn_del = QPushButton("Delete")
        self.btn_high = QPushButton("Highlight Path")
        self.btn_close = QPushButton("Close")

        self.btn_spd.clicked.connect(self.change_speed)
        self.btn_hdg.clicked.connect(self.change_heading)
        self.btn_del.clicked.connect(self.accept)
        self.btn_high.clicked.connect(lambda: self.done(2))
        self.btn_close.clicked.connect(self.reject)

        btn_box.addWidget(self.btn_spd)
        btn_box.addWidget(self.btn_hdg)
        btn_box.addWidget(self.btn_del)
        btn_box.addWidget(self.btn_high)
        btn_box.addWidget(self.btn_close)
        layout.addLayout(btn_box)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_info)
        self.timer.start(200)
        self.update_info()

    def update_info(self):
        from app.core.geometry import ecef_to_lla, ecef_heading, ecef_distance

        ship = current_project.get_ship_by_idx(self.ship_idx)
        if not ship:
            self.lbl_basic_info.setText("Ship not found.")
            return

        t_now = 0.0
        if self.worker and self.worker.running:
            t_now = self.worker.current_time

        # Get current dynamic state
        state = {'lat': 0, 'lon': 0, 'spd': 0, 'hdg': 0, 'x': 0, 'y': 0, 'z': 0}
        dyn = None

        if self.worker and self.ship_idx in self.worker.dynamic_ships:
            dyn = self.worker.dynamic_ships[self.ship_idx]
            lat, lon, _ = ecef_to_lla(dyn['x'], dyn['y'], dyn['z'])
            state = {
                'lat': lat, 'lon': lon,
                'spd': dyn.get('sog', dyn['spd']),
                'hdg': dyn['hdg'],
                'x': dyn['x'], 'y': dyn['y'], 'z': dyn['z'],
                'dist_traveled': dyn.get('dist_traveled', 0.0),
                'following_path': dyn.get('following_path', False),
                'mode': dyn.get('mode', None)
            }
        elif self.parent().calculate_ship_state:
            calc_state = self.parent().calculate_ship_state(ship, t_now)
            state.update(calc_state)

        # Calculate detections from THIS ship's perspective
        self._update_panorama_detections()

        # === Basic Information ===
        ship_type = 'Random Target' if ship.idx >= 1000 else ('OwnShip' if ship.idx == current_project.settings.own_ship_idx else 'Manual Target')
        basic_info = f"""
        <b>Name:</b> {ship.name}<br>
        <b>ID:</b> {ship.idx}<br>
        <b>MMSI:</b> {ship.mmsi}<br>
        <b>Type:</b> {ship_type}<br>
        <b>Ship Class:</b> {getattr(ship, 'ship_class', 'CONTAINER')}
        """
        self.lbl_basic_info.setText(basic_info)

        # === Navigation Status ===
        # Format simulation time
        sim_d = int(t_now // 86400)
        sim_h = int((t_now % 86400) // 3600)
        sim_m = int((t_now % 3600) // 60)
        sim_s = int(t_now % 60)
        sim_time_str = f"{sim_d}d {sim_h:02d}:{sim_m:02d}:{sim_s:02d}" if sim_d > 0 else f"{sim_h:02d}:{sim_m:02d}:{sim_s:02d}"

        # Remaining time
        eta_str = "N/A"
        if self.parent() and hasattr(self.parent(), 'sim_time_limit'):
            remaining = max(0, self.parent().sim_time_limit - t_now)
            rem_d = int(remaining // 86400)
            rem_h = int((remaining % 86400) // 3600)
            rem_m = int((remaining % 3600) // 60)
            rem_s = int(remaining % 60)
            if rem_d > 0:
                eta_str = f"{rem_d}d {rem_h:02d}:{rem_m:02d}:{rem_s:02d}"
            else:
                eta_str = f"{rem_h:02d}:{rem_m:02d}:{rem_s:02d}"

        # Speed in various units
        spd_kn = state['spd']
        spd_kmh = spd_kn * 1.852
        spd_mps = spd_kn * 0.514444

        # Mode info
        mode_str = "Path Following" if state.get('following_path', False) else "Vector Mode"
        if state.get('mode'):
            mode_str = state['mode']

        nav_info = f"""
        <b>Simulation Time:</b> {sim_time_str} ({t_now:.1f}s)<br>
        <b>Remaining:</b> {eta_str}<br>
        <hr>
        <b>Latitude:</b> {state['lat']:.6f}°<br>
        <b>Longitude:</b> {state['lon']:.6f}°<br>
        <b>Heading (COG):</b> {state['hdg']:.1f}°<br>
        <hr>
        <b>Speed (SOG):</b> {spd_kn:.2f} kn<br>
        <b>Speed:</b> {spd_kmh:.2f} km/h | {spd_mps:.2f} m/s<br>
        <hr>
        <b>Mode:</b> {mode_str}<br>
        <b>Distance Traveled:</b> {state.get('dist_traveled', 0.0)/1000:.2f} km
        """
        self.lbl_nav_info.setText(nav_info)

        # === Relative to OwnShip ===
        own_idx = current_project.settings.own_ship_idx
        rel_info = "N/A (This is OwnShip)" if self.ship_idx == own_idx else "N/A"

        if self.ship_idx != own_idx and self.worker:
            own_dyn = self.worker.dynamic_ships.get(own_idx)
            if own_dyn and dyn:
                # Distance
                dist_m = ecef_distance(own_dyn['x'], own_dyn['y'], own_dyn['z'],
                                       dyn['x'], dyn['y'], dyn['z'])
                dist_nm = dist_m / 1852.0
                dist_km = dist_m / 1000.0

                # True bearing from OwnShip to this ship
                bearing_true = ecef_heading(own_dyn['x'], own_dyn['y'], own_dyn['z'],
                                            dyn['x'], dyn['y'], dyn['z'])

                # Relative bearing from OwnShip's heading
                own_hdg = own_dyn['hdg']
                rel_bearing = self._wrap_to_180(bearing_true - own_hdg)

                # CPA/TCPA calculation
                cpa_nm, tcpa_sec = self._calculate_cpa_tcpa(own_dyn, dyn)

                # Format TCPA
                if tcpa_sec is not None and tcpa_sec > 0:
                    tcpa_m = int(tcpa_sec // 60)
                    tcpa_s = int(tcpa_sec % 60)
                    tcpa_str = f"{tcpa_m}m {tcpa_s}s"
                elif tcpa_sec is not None and tcpa_sec <= 0:
                    tcpa_str = "Passed"
                else:
                    tcpa_str = "N/A"

                rel_info = f"""
        <b>Distance:</b> {dist_nm:.2f} nm ({dist_km:.2f} km)<br>
        <b>True Bearing:</b> {bearing_true:.1f}°<br>
        <b>Relative Bearing:</b> {rel_bearing:.1f}°<br>
        <hr>
        <b>CPA:</b> {cpa_nm:.2f} nm<br>
        <b>TCPA:</b> {tcpa_str}
                """
        self.lbl_rel_info.setText(rel_info)

        # === Ship Dimensions ===
        dim_info = f"""
        <b>Length:</b> {getattr(ship, 'length_m', 300.0):.1f} m<br>
        <b>Beam:</b> {getattr(ship, 'beam_m', 40.0):.1f} m<br>
        <b>Draft:</b> {getattr(ship, 'draft_m', 15.0):.1f} m<br>
        <b>Air Draft:</b> {getattr(ship, 'air_draft_m', 60.0):.1f} m<br>
        <b>Visible Height:</b> {getattr(ship, 'height_m', 30.0):.1f} m
        """
        self.lbl_dim_info.setText(dim_info)

        # === Signal Status ===
        sig_lines = []
        for sig_type in ['AIVDM', 'RATTM', 'Camera']:
            enabled = ship.signals_enabled.get(sig_type, True)
            interval = ship.signal_intervals.get(sig_type, 1.0)
            status = '<span style="color:green;">ON</span>' if enabled else '<span style="color:red;">OFF</span>'
            sig_lines.append(f"<b>{sig_type}:</b> {status} (Interval: {interval:.1f}s)")

        sig_info = "<br>".join(sig_lines)
        self.lbl_sig_info.setText(sig_info)

        # === Event Status ===
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
                            if scen.scope_mode == "ALL_SHIPS":
                                should_apply = True
                            elif scen.scope_mode == "OWN_ONLY":
                                if e.target_ship_idx == current_project.settings.own_ship_idx:
                                    should_apply = True
                            elif scen.scope_mode == "TARGET_ONLY":
                                if e.target_ship_idx != current_project.settings.own_ship_idx:
                                    should_apply = True
                            elif scen.scope_mode == "SELECTED_SHIPS":
                                if e.target_ship_idx in scen.selected_ships:
                                    should_apply = True

                            if should_apply and e.target_ship_idx == self.ship_idx:
                                enabled_events_list.append(f"[Scen] {e.name}")

        events_str = "<br>".join(enabled_events_list) if enabled_events_list else "None"
        scen_str = ", ".join(active_scenarios) if active_scenarios else "None"

        event_info = f"""
        <b>Active Scenario:</b> {scen_str}<br>
        <b>Enabled Events for this ship:</b><br>
        {events_str}
        """
        self.lbl_event_info.setText(event_info)

    def _calculate_cpa_tcpa(self, own_dyn, tgt_dyn):
        """Calculate CPA (Closest Point of Approach) and TCPA (Time to CPA)."""
        from app.core.geometry import ecef_to_lla, ecef_distance

        # Current positions
        P_own = np.array([own_dyn['x'], own_dyn['y'], own_dyn['z']])
        P_tgt = np.array([tgt_dyn['x'], tgt_dyn['y'], tgt_dyn['z']])

        # Calculate velocities in ECEF
        def calc_ecef_velocity(dyn):
            lat, lon, _ = ecef_to_lla(dyn['x'], dyn['y'], dyn['z'])
            spd_mps = dyn.get('sog', dyn['spd']) * 0.514444
            hdg_rad = math.radians(dyn['hdg'])
            lat_rad = math.radians(lat)
            lon_rad = math.radians(lon)

            # Local NED to ECEF
            n_x = -math.sin(lat_rad) * math.cos(lon_rad)
            n_y = -math.sin(lat_rad) * math.sin(lon_rad)
            n_z = math.cos(lat_rad)

            e_x = -math.sin(lon_rad)
            e_y = math.cos(lon_rad)
            e_z = 0.0

            v_x = (n_x * math.cos(hdg_rad) + e_x * math.sin(hdg_rad)) * spd_mps
            v_y = (n_y * math.cos(hdg_rad) + e_y * math.sin(hdg_rad)) * spd_mps
            v_z = (n_z * math.cos(hdg_rad) + e_z * math.sin(hdg_rad)) * spd_mps

            return np.array([v_x, v_y, v_z])

        V_own = calc_ecef_velocity(own_dyn)
        V_tgt = calc_ecef_velocity(tgt_dyn)

        # Relative position and velocity
        P_rel = P_tgt - P_own
        V_rel = V_tgt - V_own

        v_rel_sq = np.dot(V_rel, V_rel)

        # Current distance
        dist_m = ecef_distance(own_dyn['x'], own_dyn['y'], own_dyn['z'],
                               tgt_dyn['x'], tgt_dyn['y'], tgt_dyn['z'])
        current_cpa_nm = dist_m / 1852.0

        if v_rel_sq < 1e-9:
            # No relative motion
            return current_cpa_nm, None

        # TCPA = - (P_rel . V_rel) / |V_rel|^2
        tcpa = -np.dot(P_rel, V_rel) / v_rel_sq

        if tcpa <= 0:
            # CPA already passed
            return current_cpa_nm, tcpa

        # Future positions at TCPA
        P_own_future = P_own + V_own * tcpa
        P_tgt_future = P_tgt + V_tgt * tcpa

        cpa_m = np.linalg.norm(P_tgt_future - P_own_future)
        cpa_nm = cpa_m / 1852.0

        return cpa_nm, tcpa

    def _wrap_to_180(self, deg):
        """Wrap angle to [-180, 180] range."""
        return ((deg + 180) % 360) - 180

    def _update_panorama_detections(self):
        """Calculate camera detections from this ship's perspective.

        The ship is modeled as a 3D box (length x beam x height).
        The visible width depends on which faces are visible,
        determined by the angle between observer's view direction and target's heading.
        Ships beyond camera d1 distance are filtered out.
        """
        from app.core.geometry import ecef_heading, ecef_distance

        if not self.worker or not self.worker.running:
            self.panorama_view.clear_detections()
            return

        # Get this ship's dynamic state (observer)
        if self.ship_idx not in self.worker.dynamic_ships:
            self.panorama_view.clear_detections()
            return

        observer_dyn = self.worker.dynamic_ships[self.ship_idx]
        observer_hdg = observer_dyn['hdg']

        # Get camera d1 distance (max visibility range)
        camera_d1_nm = current_project.settings.camera_reception.d1
        camera_d1_m = camera_d1_nm * 1852.0  # Convert NM to meters

        detections = []
        EPS = 1e-3

        # Loop through all other ships
        for other_ship in current_project.ships:
            if other_ship.idx == self.ship_idx:
                continue  # Skip self

            if other_ship.idx not in self.worker.dynamic_ships:
                continue

            other_dyn = self.worker.dynamic_ships[other_ship.idx]

            # Calculate true bearing from observer to target
            bearing_true = ecef_heading(
                observer_dyn['x'], observer_dyn['y'], observer_dyn['z'],
                other_dyn['x'], other_dyn['y'], other_dyn['z']
            )

            # Calculate relative bearing
            rel_bearing = self._wrap_to_180(bearing_true - observer_hdg)

            # Skip if outside FOV [-90, +90]
            if rel_bearing < -90 or rel_bearing > 90:
                continue

            # Calculate distance
            dist_m = ecef_distance(
                observer_dyn['x'], observer_dyn['y'], observer_dyn['z'],
                other_dyn['x'], other_dyn['y'], other_dyn['z']
            )
            range_m = max(dist_m, EPS)

            # Skip if ship is beyond camera d1 distance
            if dist_m >= camera_d1_m:
                continue

            # Calculate panorama coordinates
            x_norm = (rel_bearing + 90.0) / 180.0
            cx_px = x_norm * self.PANO_W_PX
            cy_px = self.CY_RATIO * self.PANO_H_PX

            # Ship dimensions
            length_m = getattr(other_ship, 'length_m', 300.0)
            beam_m = getattr(other_ship, 'beam_m', 40.0)
            height_m = getattr(other_ship, 'height_m', 30.0)

            # Calculate the viewing angle relative to target ship's heading
            tgt_heading = other_dyn['hdg']

            # Angle from target ship's bow to observer
            bearing_from_target = (bearing_true + 180.0) % 360.0
            view_angle = self._wrap_to_180(bearing_from_target - tgt_heading)

            # Calculate projected width based on view angle
            # |sin| gives contribution from side (length), |cos| gives contribution from front/back (beam)
            view_angle_rad = math.radians(view_angle)
            projected_width_m = abs(math.sin(view_angle_rad)) * length_m + abs(math.cos(view_angle_rad)) * beam_m

            # Height is always the same (viewing from sea level)
            projected_height_m = height_m

            # Convert to pixels based on distance
            w_px = max(self.W_MIN, min(self.W_MAX, self.K_H * (projected_width_m / range_m)))
            h_px = max(self.H_MIN, min(self.H_MAX, self.K_H * (projected_height_m / range_m)))

            # Clamp bbox to panorama bounds
            x1 = max(0, cx_px - w_px / 2)
            x2 = min(self.PANO_W_PX, cx_px + w_px / 2)
            y1 = max(0, cy_px - h_px / 2)
            y2 = min(self.PANO_H_PX, cy_px + h_px / 2)
            w_px = x2 - x1
            h_px = y2 - y1
            cx_px = (x1 + x2) / 2
            cy_px = (y1 + y2) / 2

            detection = {
                'ship_idx': other_ship.idx,
                'ship_name': other_ship.name,
                'rel_bearing': rel_bearing,
                'cx_px': cx_px,
                'cy_px': cy_px,
                'w_px': w_px,
                'h_px': h_px
            }
            detections.append(detection)

        self.panorama_view.set_detections(detections)

    def change_speed(self):
        ship = current_project.get_ship_by_idx(self.ship_idx)
        if not ship: return
        
        current_spd = 0.0
        if self.worker and self.ship_idx in self.worker.dynamic_ships:
            current_spd = self.worker.dynamic_ships[self.ship_idx]['spd']
        elif self.parent().calculate_ship_state:
            t_now = self.worker.current_time if self.worker else 0.0
            st = self.parent().calculate_ship_state(ship, t_now)
            current_spd = st.get('spd', 0.0)
            
        val, ok = QInputDialog.getDouble(self, "Change Speed", "New Speed (kn):", value=current_spd, min=0, max=10000, decimals=1)
        if ok:
            if self.worker:
                self.parent().request_ship_speed_change(self.ship_idx, val)

    def change_heading(self):
        ship = current_project.get_ship_by_idx(self.ship_idx)
        if not ship: return
        
        current_hdg = 0.0
        if self.worker and self.ship_idx in self.worker.dynamic_ships:
            current_hdg = self.worker.dynamic_ships[self.ship_idx]['hdg']
        elif self.parent().calculate_ship_state:
            t_now = self.worker.current_time if self.worker else 0.0
            st = self.parent().calculate_ship_state(ship, t_now)
            current_hdg = st.get('hdg', 0.0)
            
        val, ok = QInputDialog.getDouble(self, "Change Heading", "New Heading (deg):", value=current_hdg, min=0, max=360, decimals=1)
        if ok:
            if self.worker:
                self.parent().request_ship_heading_change(self.ship_idx, val)

class ExtraTimeDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Add Extra Time")
        layout = QVBoxLayout(self)
        
        self.time_input = TimeInputWidget()
        self.time_input.set_seconds(600) 
        layout.addWidget(QLabel("Additional Time:"))
        layout.addWidget(self.time_input)
        
        btn_box = QHBoxLayout()
        btn_ok = QPushButton("Add")
        btn_cancel = QPushButton("Cancel")
        btn_ok.clicked.connect(self.accept)
        btn_cancel.clicked.connect(self.reject)
        btn_box.addWidget(btn_ok)
        btn_box.addWidget(btn_cancel)
        layout.addLayout(btn_box)
        
    def get_seconds(self):
        return self.time_input.get_seconds()

class SimulationPanel(QWidget):
    state_changed = pyqtSignal(str)
    simulation_status_updated = pyqtSignal(str, list)
    sig_set_ship_speed = pyqtSignal(int, float)
    sig_set_ship_heading = pyqtSignal(int, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.ship_items = {}
        self.ship_indicators = {}
        self.ship_trails = {} 
        self.trail_items = {} 
        self.planned_paths = {} 
        self.highlighted_ship_idx = -1
        self.worker = None
        self.worker_thread = None
        self.is_paused = False
        self.data_ready_flag = False
        self.suppress_close_warning = False
        self.blink_timers = {}
        self.sim_time_limit = 2592000 # 30 Days default
        self.sim_ended = False
        self.last_pos_dict = {}
        self.speed_history = {} 
        
        self._duration_spinbox = None
        self._ext_duration_le = None
        self._ext_start_btn = None
        
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
            ship_class = data.get("ship_class", "CONTAINER")
            N_AI_only = data["N_AI_only"]
            N_RA_only = data["N_RA_only"]
            N_both = data["N_both"]

            own_ship = current_project.get_ship_by_idx(current_project.settings.own_ship_idx)
            if not own_ship:
                msgbox.show_warning(self, "Warning", "Own Ship is not set. Cannot generate random targets.")
                return

            self.generate_random_targets_logic(own_ship, R, N_AI_only, N_RA_only, N_both, area_id, ship_class)

    def action_clear_random_targets(self):
        targets = [s for s in current_project.ships if s.idx >= 1000]
        if not targets:
            msgbox.show_information(self, "Info", "No random targets to delete.")
            return

        if msgbox.show_question(self, "Clear Random Targets", f"Delete {len(targets)} random targets?") != msgbox.StandardButton.Yes:
            return

        current_project.ships = [s for s in current_project.ships if s.idx < 1000]
        
        active_idxs = {s.idx for s in current_project.ships}
        
        to_remove_pos = [idx for idx in self.last_pos_dict.keys() if idx not in active_idxs]
        for idx in to_remove_pos:
            del self.last_pos_dict[idx]
            
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
            if hasattr(self.window(), 'data_changed'):
                self.window().data_changed.emit()
            
        if self.worker and self.worker.running:
            self.worker.refresh_active_ships()

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)

        # --- Top Controls ---
        top_controls = QWidget()
        top_layout = QHBoxLayout(top_controls)
        top_layout.setContentsMargins(0, 0, 0, 0)
        
        self.ip_edit = QLineEdit("127.0.0.1"); self.ip_edit.setFixedWidth(100)
        self.port_edit = QLineEdit("10110"); self.port_edit.setFixedWidth(60)
        top_layout.addWidget(QLabel("IP:"))
        top_layout.addWidget(self.ip_edit)
        top_layout.addWidget(QLabel("Port:"))
        top_layout.addWidget(self.port_edit)
        
        self.speed_spin = QSpinBox()
        self.speed_spin.setRange(1, 86400)
        self.speed_spin.setValue(1)
        self.speed_spin.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.speed_spin.setFixedWidth(50)
        self.speed_spin.valueChanged.connect(self.on_speed_change)
        top_layout.addWidget(QLabel("Speed(x):"))
        top_layout.addWidget(self.speed_spin)
        
        self.spin_sim_duration = TimeInputWidget()
        self.spin_sim_duration.set_seconds(2592000)
        self.spin_sim_duration.valueChanged.connect(self.on_duration_spin_changed)
        top_layout.addWidget(QLabel("Duration:"))
        top_layout.addWidget(self.spin_sim_duration)

        self.btn_add_time = QPushButton("Add Extra Time")
        self.btn_add_time.setEnabled(True)
        self.btn_add_time.clicked.connect(self.action_add_extra_time)
        top_layout.addWidget(self.btn_add_time)

        self.combo_follow = QComboBox()
        self.combo_follow.addItem("None", -1)
        top_layout.addWidget(QLabel("Follow:"))
        top_layout.addWidget(self.combo_follow)
        
        self.chk_show_paths = QCheckBox("Show Paths")
        self.chk_show_paths.setChecked(True)
        self.chk_show_paths.toggled.connect(self.draw_static_map)
        top_layout.addWidget(self.chk_show_paths)
        
        self.btn_capture = QPushButton("Capture")
        self.btn_capture.clicked.connect(self.action_capture)
        top_layout.addWidget(self.btn_capture)
        
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
        
        self.btn_rtg = QPushButton("Generate Random Target")
        self.btn_rtg.setToolTip("Generate Random Target")
        self.btn_rtg.clicked.connect(self.action_random_target_generate)
        self.btn_clear_rtg = QPushButton("Clear Random Target")
        self.btn_clear_rtg.clicked.connect(self.action_clear_random_targets)
        top_layout.addWidget(self.btn_rtg)
        top_layout.addWidget(self.btn_clear_rtg)
        
        self.perf_label = QLabel("SPS: -")
        self.eta_label = QLabel("Rem: -")
        top_layout.addWidget(self.perf_label)
        top_layout.addWidget(self.eta_label)
        
        top_layout.addStretch()
        
        layout.addWidget(top_controls)
        
        mid_widget = QWidget()
        mid_layout = QHBoxLayout(mid_widget)
        mid_layout.setContentsMargins(0, 0, 0, 0)
        
        self.scene = QGraphicsScene()
        self.view = SimMapView(self.scene, self)
        self.view.mode = "VIEW"
        self.view.view_changed.connect(self.update_pen_widths)
        mid_layout.addWidget(self.view, 3)

        # Right panel with scroll
        right_scroll = QScrollArea()
        right_scroll.setWidgetResizable(True)
        right_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        right_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)

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
        self.list_active_events.setMinimumHeight(60)
        self.list_active_events.setMaximumHeight(200)
        self.list_active_events.itemDoubleClicked.connect(self.on_event_list_double_clicked)
        status_layout.addWidget(self.lbl_active_scen)
        status_layout.addWidget(self.edit_status_filter)
        status_layout.addWidget(self.list_active_events)
        right_layout.addWidget(self.status_group)

        self.control_group = self.create_ship_control_group()
        right_layout.addWidget(self.control_group)

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

        self.info_tabs = QTabWidget()
        
        log_tab = QWidget()
        log_tab_layout = QVBoxLayout(log_tab)
        log_tab_layout.setContentsMargins(0,0,0,0)
        
        log_group = QGroupBox("Raw NMEA Output") 
        log_layout = QVBoxLayout(log_group)
        
        filter_layout = QHBoxLayout()
        filter_layout.addWidget(QLabel("Filter:"))
        self.log_filter_combo = QComboBox()
        self.log_filter_combo.addItems(["ALL", "AIVDM", "RATTM", "CAMERA"])
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

        log_tab_layout.addWidget(log_group)
        
        graph_tab = QWidget()
        graph_layout = QVBoxLayout(graph_tab)
        self.figure = Figure(figsize=(5, 3), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        graph_layout.addWidget(self.canvas)
        
        self.info_tabs.addTab(log_tab, "Log")
        self.info_tabs.addTab(graph_tab, "Speed Graph")
        right_layout.addWidget(self.info_tabs, 1)

        right_scroll.setWidget(right_panel)
        mid_layout.addWidget(right_scroll, 1)
        layout.addWidget(mid_widget)
        
        self.draw_static_map()
        self.update_follow_combo()
        self.update_control_combo()

    def _move_great_circle_step(self, lat, lon, hdg_deg, dist_m):
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        hdg_rad = math.radians(hdg_deg)
        R = 6371000.0 # Earth Radius

        ang_dist = dist_m / R

        new_lat_rad = math.asin(
            math.sin(lat_rad) * math.cos(ang_dist) +
            math.cos(lat_rad) * math.sin(ang_dist) * math.cos(hdg_rad)
        )
        
        new_lon_rad = lon_rad + math.atan2(
            math.sin(hdg_rad) * math.sin(ang_dist) * math.cos(lat_rad),
            math.cos(ang_dist) - math.sin(lat_rad) * math.sin(new_lat_rad)
        )
        
        new_lat = math.degrees(new_lat_rad)
        new_lon = normalize_lon(math.degrees(new_lon_rad))

        y = math.sin(lon_rad - new_lon_rad) * math.cos(lat_rad)
        x = math.cos(new_lat_rad) * math.sin(lat_rad) - \
            math.sin(new_lat_rad) * math.cos(lat_rad) * math.cos(lon_rad - new_lon_rad)
        
        back_bearing_rad = math.atan2(y, x)
        back_bearing_deg = math.degrees(back_bearing_rad)
        
        new_hdg = (back_bearing_deg + 180) % 360
        
        return new_lat, new_lon, new_hdg

    def generate_random_targets_logic(self, own_ship, R_nm, N_ai, N_ra, N_both, area_id=-1, ship_class="CONTAINER"):
        import numpy as np  # NumPy for high-speed computation
        from app.core.models.ship import get_ship_dimensions

        seed_val = (current_project.seed + len(current_project.ships)) % (2**32 - 1)
        random.seed(seed_val)
        np.random.seed(seed_val)

        t_now = 0.0
        if self.worker and self.worker.running:
            t_now = self.worker.current_time
        
        own_lat, own_lon, own_spd_kn = 0.0, 0.0, 0.0
        
        mi = current_project.map_info

        if self.worker and self.worker.running and own_ship.idx in self.worker.dynamic_ships:
            dyn = self.worker.dynamic_ships[own_ship.idx]
            from app.core.geometry import ecef_to_lla
            own_lat, own_lon, _ = ecef_to_lla(dyn['x'], dyn['y'], dyn['z'])
            own_spd_kn = dyn['spd']
        elif own_ship.resampled_points:
            px, py = own_ship.resampled_points[0]
            _, _, own_lat, own_lon = pixel_to_coords(px, py, mi)
            own_spd_kn = own_ship.raw_speeds.get(0, 5.0)

        total_targets = N_ai + N_ra + N_both
        
        target_area_poly = None
        area_bbox = None
        if area_id != -1:
            area = current_project.get_area_by_id(area_id)
            if area:
                poly_pts = [QPointF(px, py) for px, py in area.geometry]
                target_area_poly = QPolygonF(poly_pts)
                area_bbox = target_area_poly.boundingRect()

        R_deg_lat = R_nm / 60.0
        
        # Bind coordinate conversion function to local variable for optimization
        ctp = coords_to_pixel
        
        for i in range(total_targets):
            if i < N_ai: s_type = "AI"
            elif i < N_ai + N_ra: s_type = "RA"
            else: s_type = "BOTH"
            
            # --- 1. Start Position Generation ---
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
                cos_lat = math.cos(math.radians(own_lat))
                if abs(cos_lat) < 0.01: cos_lat = 0.01
                d_lon = r * math.sin(theta) / cos_lat
                
                start_lat = own_lat + d_lat
                start_lon = own_lon + d_lon
            
            start_lat = max(-89.9, min(89.9, start_lat))
            
            # --- 2. Speed and Heading Setup ---
            variance = current_project.settings.speed_variance
            sigma = math.sqrt(variance)
            spd = abs(random.gauss(own_spd_kn, sigma)) 
            if spd < 0.1: spd = 0.1
            
            heading = random.random() * 360.0
            
            # --- 3. [Optimized] Batch Great Circle Route Calculation with NumPy ---
            duration_remaining = 24 * 3600.0
            dt = 60.0 # 60 second interval

            # Generate time and distance arrays (Vectorization)
            times = np.arange(0, duration_remaining, dt)
            spd_mps = spd * 0.514444 # Knots to m/s
            dists_m = times * spd_mps

            # Initial position and heading (Radians)
            lat1_rad = math.radians(start_lat)
            lon1_rad = math.radians(start_lon)
            hdg_rad = math.radians(heading)
            R_earth = 6371000.0

            # Angular distances
            ang_dists = dists_m / R_earth

            # Great Circle Route Formula (Direct Geodesic on Sphere) - for loop removed
            sin_lat1 = math.sin(lat1_rad)
            cos_lat1 = math.cos(lat1_rad)
            cos_hdg = math.cos(hdg_rad)
            sin_hdg = math.sin(hdg_rad)

            sin_ang_dists = np.sin(ang_dists)
            cos_ang_dists = np.cos(ang_dists)

            # Batch latitude calculation
            lat2_rads = np.arcsin(sin_lat1 * cos_ang_dists + cos_lat1 * sin_ang_dists * cos_hdg)

            # Batch longitude calculation
            y = sin_hdg * sin_ang_dists * cos_lat1
            x = cos_ang_dists - sin_lat1 * np.sin(lat2_rads)
            lon2_rads = lon1_rad + np.arctan2(y, x)

            # Radians to degrees conversion
            lats_deg = np.degrees(lat2_rads)
            lons_deg = np.degrees(lon2_rads)

            # Range limiting and normalization
            lats_deg = np.clip(lats_deg, -89.9, 89.9)
            lons_deg = (lons_deg + 180) % 360 - 180

            # Pixel conversion (List Comprehension is faster than regular for loop)
            raw_pixels = [ctp(lat, lon, mi) for lat, lon in zip(lats_deg, lons_deg)]

            # --- 4. Create Ship Object ---
            new_idx = 1001
            existing_idxs = {s.idx for s in current_project.ships}
            while new_idx in existing_idxs: new_idx += 1
            
            r_num = 1
            existing_names = {s.name for s in current_project.ships}
            while f"Random-{r_num}" in existing_names: r_num += 1
            
            new_ship = ShipData(new_idx, f"Random-{r_num}")
            new_ship.mmsi = random.randint(100000000, 999999999)
            new_ship.is_generated = True

            new_ship.raw_points = raw_pixels
            new_ship.raw_speeds = {0: spd}

            new_ship.resampled_points = raw_pixels

            # Set ship class and dimensions
            new_ship.ship_class = ship_class
            dims = get_ship_dimensions(ship_class)
            new_ship.length_m = dims[0]
            new_ship.beam_m = dims[1]
            new_ship.draft_m = dims[2]
            new_ship.air_draft_m = dims[3]
            new_ship.height_m = dims[3] * 0.5  # Visible height = half of air draft

            if s_type == "AI":
                new_ship.signals_enabled['AIVDM'] = True
                new_ship.signals_enabled['RATTM'] = False
                new_ship.signals_enabled['Camera'] = True
            elif s_type == "RA":
                new_ship.signals_enabled['AIVDM'] = False
                new_ship.signals_enabled['RATTM'] = True
                new_ship.signals_enabled['Camera'] = True
            else:
                new_ship.signals_enabled['AIVDM'] = True
                new_ship.signals_enabled['RATTM'] = True
                new_ship.signals_enabled['Camera'] = True

            current_project.ships.append(new_ship)
            
        self.refresh_tables()
        self.draw_static_map()
        
        if self.worker and self.worker.running:
            pos_dict = self.last_pos_dict.copy()
            
            t_restore = self.worker.current_time
            mi = current_project.map_info
            
            for s in current_project.ships:
                if not s.is_generated: continue
                if s.idx in pos_dict and s.idx in self.worker.dynamic_ships: continue 
                
                st = self.calculate_ship_state(s, t_restore)
                px, py = coords_to_pixel(st['lat'], st['lon'], mi)
                pos_dict[s.idx] = (px, py, st['hdg'], st['spd'])
            self.update_positions(pos_dict)
        
        if self.window():
            self.window().update_obj_combo()
            self.window().redraw_map()
            if hasattr(self.window(), 'data_changed'):
                self.window().data_changed.emit()
        
        if self.worker and self.worker.running:
            self.worker.refresh_active_ships()

    def on_duration_spin_changed(self, val):
        self.sim_time_limit = val
        if self.worker:
            self.worker.update_duration(val)

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

                if r > 0 and c > 0:
                    ship = ships[r-1]
                    talker = talkers[c-1]
                    is_checked = ship.signals_enabled.get(talker, True)

                widget = self._create_on_off_cell(is_checked)
                checkbox = widget.findChild(QCheckBox)
                
                checkbox.setProperty("row", r)
                checkbox.setProperty("col", c)
                checkbox.stateChanged.connect(self.on_signal_on_off_toggled)
                table.setCellWidget(r, c, widget)

        # Set row height and calculate total table height
        row_height = 60  # Each row height (2x default)
        for r in range(table.rowCount()):
            table.setRowHeight(r, row_height)

        for i in range(1, table.columnCount()):
            table.horizontalHeader().setSectionResizeMode(i, QHeaderView.ResizeMode.Stretch)
        table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)

        # Calculate total table height (header + all rows + margin)
        header_height = table.horizontalHeader().height()
        total_height = header_height + (row_height * table.rowCount()) + 4
        table.setFixedHeight(total_height)
        table.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

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
        table.setColumnCount(len(talkers))

        table.setHorizontalHeaderLabels(talkers)
        table.setVerticalHeaderLabels(["ALL"] + [s.name for s in ships])

        for r in range(table.rowCount()):
            for c in range(table.columnCount()):
                val = 1.0
                if r > 0:
                    ship = ships[r - 1]
                    talker = talkers[c]
                    val = ship.signal_intervals.get(talker, 1.0)

                spin = self._create_interval_cell(val)
                spin.setProperty("row", r)
                spin.setProperty("col", c)
                spin.valueChanged.connect(self.on_signal_interval_changed)
                table.setCellWidget(r, c, spin)

        # Set row height and calculate total table height
        row_height = 60  # Each row height (2x default)
        for r in range(table.rowCount()):
            table.setRowHeight(r, row_height)

        for i in range(table.columnCount()):
            table.horizontalHeader().setSectionResizeMode(i, QHeaderView.ResizeMode.Stretch)

        # Calculate total table height (header + all rows + margin)
        header_height = table.horizontalHeader().height()
        total_height = header_height + (row_height * table.rowCount()) + 4
        table.setFixedHeight(total_height)
        table.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

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

        # r == 0 is ALL row, apply to all ships
        rows_model = [r] if r > 0 else range(1, len(ships) + 1)

        for row_idx in rows_model:
            ship = ships[row_idx - 1]
            talker = talkers[c]
            ship.signal_intervals[talker] = value

        # If ALL row (r == 0), update all other rows in the same column
        if r == 0:
            for row in range(1, table.rowCount()):
                self._set_interval_spinbox(row, c, value)

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
        self.update_control_combo()

    def update_follow_combo(self):
        current_idx = self.combo_follow.currentData()
        self.combo_follow.blockSignals(True)
        self.combo_follow.clear()
        self.combo_follow.addItem("None", -1)
        
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

    def create_ship_control_group(self):
        group = QGroupBox("Manual Ship Control")
        layout = QGridLayout(group)
        
        layout.addWidget(QLabel("Target:"), 0, 0)
        self.combo_control_ship = QComboBox()
        self.combo_control_ship.currentIndexChanged.connect(self.on_control_ship_changed)
        layout.addWidget(self.combo_control_ship, 0, 1, 1, 2)
        
        self.btn_info = QPushButton("Info")
        self.btn_info.clicked.connect(self.on_info_clicked)
        layout.addWidget(self.btn_info, 0, 3)
        
        layout.addWidget(QLabel("Speed (kn):"), 1, 0)
        self.spin_control_spd = QDoubleSpinBox()
        self.spin_control_spd.setRange(0, 10000) 
        self.spin_control_spd.setSingleStep(0.1)
        layout.addWidget(self.spin_control_spd, 1, 1)
        self.btn_set_spd = QPushButton("Set")
        self.btn_set_spd.clicked.connect(self.on_set_speed_clicked)
        layout.addWidget(self.btn_set_spd, 1, 2)
        
        layout.addWidget(QLabel("Heading (deg):"), 2, 0)
        self.spin_control_hdg = QDoubleSpinBox()
        self.spin_control_hdg.setRange(0, 360)
        self.spin_control_hdg.setSingleStep(1.0)
        self.spin_control_hdg.setWrapping(True)
        layout.addWidget(self.spin_control_hdg, 2, 1)
        self.btn_set_hdg = QPushButton("Set")
        self.btn_set_hdg.clicked.connect(self.on_set_heading_clicked)
        layout.addWidget(self.btn_set_hdg, 2, 2)
        
        return group

    def update_control_combo(self):
        current_idx = self.combo_control_ship.currentData()
        self.combo_control_ship.blockSignals(True)
        self.combo_control_ship.clear()
        
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
            self.combo_control_ship.addItem(f"[{tag}] {s.name}", s.idx)
            
        idx = self.combo_control_ship.findData(current_idx)
        if idx >= 0: self.combo_control_ship.setCurrentIndex(idx)
        elif self.combo_control_ship.count() > 0: self.combo_control_ship.setCurrentIndex(0)
            
        self.combo_control_ship.blockSignals(False)
        self.on_control_ship_changed()

    def set_control_ship(self, idx):
        combo_idx = self.combo_control_ship.findData(idx)
        if combo_idx >= 0:
            self.combo_control_ship.setCurrentIndex(combo_idx)

    def on_info_clicked(self):
        idx = self.combo_control_ship.currentData()
        if idx is not None:
            self.open_target_info_dialog(idx)

    def on_control_ship_changed(self):
        idx = self.combo_control_ship.currentData()
        
        # [Sync logic] Update map highlight when ship is selected from combobox
        # Clear highlight (-1) if no index is selected
        self.highlighted_ship_idx = idx if idx is not None else -1
        # Redraw map to reflect changes
        self.draw_static_map()

        if idx is None: return
        
        ship = current_project.get_ship_by_idx(idx)
        if not ship: return
        
        spd = 0.0
        hdg = 0.0
        
        if self.worker and self.worker.running and idx in self.worker.dynamic_ships:
            dyn = self.worker.dynamic_ships[idx]
            spd = dyn['spd']
            hdg = dyn['hdg']
        elif self.calculate_ship_state:
             t_now = 0.0
             st = self.calculate_ship_state(ship, t_now)
             spd = st['spd']
             hdg = st['hdg']
             
        self.spin_control_spd.setValue(spd)
        self.spin_control_hdg.setValue(hdg)

    def on_set_speed_clicked(self):
        idx = self.combo_control_ship.currentData()
        if idx is None: return
        val = self.spin_control_spd.value()
        self.request_ship_speed_change(idx, val)

    def on_set_heading_clicked(self):
        idx = self.combo_control_ship.currentData()
        if idx is None: return
        val = self.spin_control_hdg.value()
        self.request_ship_heading_change(idx, val)

    def build_trail_path(self, points):
        pp = QPainterPath()
        if not points: return pp
        pp.moveTo(points[0])
        
        mi = current_project.map_info
        threshold = 100 * mi.pixels_per_degree
        if threshold <= 0: threshold = 10000
        
        for i in range(1, len(points)):
            p1 = points[i-1]
            p2 = points[i]
            if abs(p1.x() - p2.x()) > threshold:
                pp.moveTo(p2)
            else:
                pp.lineTo(p2)
        return pp

    def draw_static_map(self):
        self.scene.clear()
        self.ship_items.clear()
        self.ship_indicators.clear()
        self.trail_items.clear() 
        self.planned_paths.clear()
        self.add_boundary_masks()
        self.stop_all_blinks()
        
        scale = self.view.transform().m11()
        if scale <= 0: scale = 1.0

        for ship in current_project.ships:
             if not ship.raw_points: continue 
             
             is_own = (ship.idx == current_project.settings.own_ship_idx)
             is_random = (ship.idx >= 1000) # Check if random target
             
             if is_own:
                 c_hex = current_project.settings.own_color
             elif is_random:
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
             
             # [Modified] Show dotted path (Planned Path) only for non-random targets
             if self.chk_show_paths.isChecked() and not is_random:
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
                 
                 ind_color = QColor(current_project.settings.random_color) if is_random else QColor(current_project.settings.target_color)
                 
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

                 # [Modified] Show i, f markers only for non-random targets
                 if self.chk_show_paths.isChecked() and not is_random:
                     self.scene.addItem(ti)
                     self.scene.addItem(tf)

        for st in current_project.areas:
            poly = QPolygonF([QPointF(x,y) for x,y in st.geometry])
            item = QGraphicsPolygonItem(poly)
            item.setPen(QPen(Qt.GlobalColor.black, 2))
            item.setBrush(QBrush(QColor(st.color)))
            self.scene.addItem(item)
            
        settings = current_project.settings
        current_ship_idxs = {s.idx for s in current_project.ships}
        
        scale = self.view.transform().m11()
        if scale <= 0: scale = 1.0
        
        for idx, points in self.ship_trails.items():
            if idx not in current_ship_idxs: continue
            if not points: continue
            
            path_item = QGraphicsPathItem()
            pp = self.build_trail_path(points)
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

        if self.worker:
             t_now = self.worker.current_time
             mi = current_project.map_info
             
             for s in current_project.ships:
                 if s.idx in self.last_pos_dict:
                     data = self.last_pos_dict[s.idx]
                     px, py = data[0], data[1]
                     heading = data[2]
                 else:
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
        
        r1 = QGraphicsRectItem(-limit, -limit, (-180 * scale) + limit, 2 * limit)
        r1.setBrush(QBrush(c))
        r1.setPen(QPen(Qt.PenStyle.NoPen))
        r1.setZValue(-100)
        self.scene.addItem(r1)
        
        r2 = QGraphicsRectItem(180 * scale, -limit, limit - (180 * scale), 2 * limit)
        r2.setBrush(QBrush(c))
        r2.setPen(QPen(Qt.PenStyle.NoPen))
        r2.setZValue(-100)
        self.scene.addItem(r2)
        
        r3 = QGraphicsRectItem(-limit, -limit, 2 * limit, (-90 * scale) + limit)
        r3.setBrush(QBrush(c))
        r3.setPen(QPen(Qt.PenStyle.NoPen))
        r3.setZValue(-100)
        self.scene.addItem(r3)
        
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
        
        for idx, item in self.planned_paths.items():
            width = settings.path_thickness
            if idx == self.highlighted_ship_idx: width += 1
            pen = item.pen()
            pen.setWidthF(width / scale)
            item.setPen(pen)
            
        for idx, item in self.trail_items.items():
            width = settings.traveled_path_thickness
            pen = item.pen()
            pen.setWidthF(width / scale)
            item.setPen(pen)

    def set_ui_locked(self, locked):
        self.ip_edit.setEnabled(not locked)
        self.port_edit.setEnabled(not locked)
        self.spin_sim_duration.setEnabled(not locked)

    def action_start(self):
        if self.worker and self.worker.running and self.worker.paused:
            self.worker.resume()
            self.state_changed.emit("PLAY")
            self.btn_start.setEnabled(False)
            self.btn_pause.setEnabled(True)
            self.btn_stop.setEnabled(True)
            self.btn_export.setEnabled(False)
            return

        if self.worker_thread:
            try:
                if self.worker_thread.isRunning():
                    self.worker.stop()
                    self.worker_thread.quit()
                    self.worker_thread.wait()
            except RuntimeError: pass 
            if self.worker:
                self.worker.deleteLater()
            if self.worker_thread:
                self.worker_thread.deleteLater()
            self.worker = None
            self.worker_thread = None
        self.stop_all_blinks()
            
        self.ship_trails = {}
        self.trail_items = {}
        self.highlighted_ship_idx = -1
        self.last_pos_dict = {}
        self.speed_history = {}
            
        self.draw_static_map()

        active_ships = [s for s in current_project.ships if s.is_generated]
        
        self.sim_time_limit = self.spin_sim_duration.get_seconds()
        self.sim_ended = False

        ip = self.ip_edit.text()
        try:
            port = int(self.port_edit.text())
        except:
            msgbox.show_critical(self, "Error", "Invalid Port")
            return
            
        self.worker_thread = QThread()
        self.worker = SimulationWorker(ip, port, self.speed_spin.value(), self.spin_sim_duration.get_seconds())
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
        self.sig_set_ship_speed.connect(self.worker.set_ship_speed)
        self.sig_set_ship_heading.connect(self.worker.set_ship_heading)
        
        self.worker_thread.start()
        self.state_changed.emit("PLAY")
        self.btn_start.setEnabled(False)
        self.btn_pause.setEnabled(True)
        self.btn_stop.setEnabled(True)
        self.btn_export.setEnabled(False)
        self.set_ui_locked(True)
        self.data_ready_flag = False

    def request_ship_speed_change(self, idx, speed):
        self.sig_set_ship_speed.emit(idx, speed)

    def request_ship_heading_change(self, idx, heading):
        self.sig_set_ship_heading.emit(idx, heading)

    def action_add_extra_time(self):
        dlg = ExtraTimeDialog(self)
        if dlg.exec() == QDialog.DialogCode.Accepted:
            extra = dlg.get_seconds()
            if extra <= 0: return

            self.sim_time_limit += extra
            
            if self.worker:
                self.worker.update_duration(self.sim_time_limit)
            
            if self.sim_ended:
                self.sim_ended = False
                self.btn_start.setEnabled(True)
                self.btn_pause.setEnabled(False)
            

    def action_capture(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save Map Capture", "", "PNG Image (*.png);;JPEG Image (*.jpg)")
        if not path: return

        pixmap = self.view.grab()
        if pixmap.save(path):
            msgbox.show_information(self, "Saved", f"Map captured to {os.path.basename(path)}")
        else:
            msgbox.show_warning(self, "Error", "Failed to save image.")

    def action_pause(self):
        if self.worker:
            self.worker.pause()
            self.state_changed.emit("PAUSE")
            self.btn_start.setEnabled(True)
            self.btn_pause.setEnabled(False)

    def action_stop(self):
        if self.worker:
            self.worker.stop()

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
        self.btn_add_time.setEnabled(True)

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
        if self.worker and not self.sim_ended:
            if self.worker.current_time >= self.sim_time_limit:
                self.sim_ended = True
                self.worker.pause()
                self.state_changed.emit("PAUSE")
                self.btn_start.setEnabled(True)
                self.btn_pause.setEnabled(False)

        settings = current_project.settings
        scale = self.view.transform().m11()
        if scale <= 0: scale = 1.0
        
        active_idxs = {s.idx for s in current_project.ships}
        self.last_pos_dict.update(pos_dict)

        t_now = 0.0
        if self.worker:
            t_now = self.worker.current_time

        for idx, data in pos_dict.items():
            if len(data) > 3:
                spd = data[3]
                if idx not in self.speed_history:
                    self.speed_history[idx] = {'t': [], 'v': []}
                
                self.speed_history[idx]['t'].append(t_now)
                self.speed_history[idx]['v'].append(spd)
                
                if len(self.speed_history[idx]['t']) > 1000:
                    self.speed_history[idx]['t'].pop(0)
                    self.speed_history[idx]['v'].pop(0)

        for idx, data in pos_dict.items():
            px, py = data[0], data[1]
            
            if idx not in active_idxs: continue
            if idx not in self.ship_trails:
                self.ship_trails[idx] = []
            
            self.ship_trails[idx].append(QPointF(px, py))
            
            if idx not in self.trail_items or self.trail_items[idx].scene() != self.scene:
                path_item = QGraphicsPathItem()
                self.scene.addItem(path_item)
                self.trail_items[idx] = path_item
            
            path_item = self.trail_items[idx]
            pp = self.build_trail_path(self.ship_trails[idx])
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
            path_item.setZValue(5) 
            
        self.update_manual_control_ui() 

        remaining = max(0, self.sim_time_limit - t_now)
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
        
        self.update_speed_graph()
        self.emit_simulation_status()
        self.scene.update()

    def update_manual_control_ui(self):
        idx = self.combo_control_ship.currentData()
        if idx is None: return
        
        if self.worker and self.worker.running and idx in self.worker.dynamic_ships:
            dyn = self.worker.dynamic_ships[idx]
            current_spd = dyn.get('spd', 0.0)
            current_hdg = dyn.get('hdg', 0.0)
            
            self.spin_control_spd.blockSignals(True)
            self.spin_control_hdg.blockSignals(True)
            
            if abs(self.spin_control_spd.value() - current_spd) > 0.01:
                self.spin_control_spd.setValue(current_spd)
            
            if abs(self.spin_control_hdg.value() - current_hdg) > 0.01:
                self.spin_control_hdg.setValue(current_hdg)
            
            self.spin_control_spd.blockSignals(False)
            self.spin_control_hdg.blockSignals(False)

    def update_speed_graph(self):
        if self.info_tabs.currentIndex() != 1: return 
        
        target_idx = self.combo_control_ship.currentData()
        if target_idx is None: return
        
        self.ax.clear()
        self.ax.grid(True)
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Speed (kn)")
        
        self.figure.patch.set_facecolor('#ffffff')
        self.ax.set_facecolor('#ffffff')
        
        if target_idx in self.speed_history:
            data = self.speed_history[target_idx]
            self.ax.plot(data['t'], data['v'], 'b-')
            
        self.canvas.draw()

    def emit_simulation_status(self):
        active_scen_names = []
        events_list = []
        
        for e in current_project.events:
            if e.enabled:
                ship = current_project.get_ship_by_idx(e.target_ship_idx)
                s_name = ship.name if ship else "Unknown"
                events_list.append(f"[Proj] {e.name} (Tgt: {s_name})")
        
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
        match = re.search(r"\(Tgt: (.*)\)", text)
        if match:
            ship_name = match.group(1)
            target_ship = next((s for s in current_project.ships if s.name == ship_name), None)
            
            if target_ship:
                if target_ship.idx in self.ship_items:
                    item = self.ship_items[target_ship.idx]
                    self.view.centerOn(item.pos())
                    return

                mi = current_project.map_info
                px, py = coords_to_pixel(target_ship.raw_points[0][0], target_ship.raw_points[0][1], mi) if target_ship.raw_points else (0, 0)
                self.view.centerOn(px, py)

    def on_export_data(self, ready):
        self.data_ready_flag = ready

    def save_log_to_file(self):
        text = self.log_list.toPlainText()
        if not text:
            msgbox.show_information(self, "Info", "No log content to save.")
            return

        path, _ = QFileDialog.getSaveFileName(self, "Save Log", "", "Text Files (*.txt)")
        if not path: return

        try:
            with open(path, 'w', encoding='utf-8') as f:
                f.write(text)
            msgbox.show_information(self, "Saved", "Log saved.")
        except Exception as e:
            msgbox.show_critical(self, "Error", str(e))

    def export_csv(self):
        if not self.data_ready_flag or not self.worker or not self.worker.temp_file_handle:
            msgbox.show_warning(self, "Info", "No data to export.")
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
            
            msgbox.show_information(self, "Done", "Exported successfully.")
        except Exception as e:
            msgbox.show_critical(self, "Error", str(e))
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
            self.action_stop() 
            self.worker_thread.quit() 
            self.worker_thread.wait() 
            
            if self.worker:
                self.worker.deleteLater()
            if self.worker_thread:
                self.worker_thread.deleteLater()
        self.stop_all_blinks()

    def calculate_ship_state(self, ship, t_current):
        if self.worker and ship.idx in self.worker.dynamic_ships:
            dyn = self.worker.dynamic_ships[ship.idx]
            from app.core.geometry import ecef_to_lla
            lat, lon, _ = ecef_to_lla(dyn['x'], dyn['y'], dyn['z'])
            return {'lat': lat, 'lon': lon, 'spd': dyn.get('sog', dyn['spd']), 'hdg': dyn['hdg']}

        mi = current_project.map_info
        if ship.resampled_points:
            px, py = ship.resampled_points[0]
            _, _, lat, lon = pixel_to_coords(px, py, mi)
            
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
        self.set_control_ship(idx)
        self.highlighted_ship_idx = idx
        self.draw_static_map()

    def open_target_info_dialog(self, idx):
        ship = current_project.get_ship_by_idx(idx)
        if not ship: return

        dlg = TargetInfoDialog(self, idx, self.worker)
        res = dlg.exec()
        
        if res == QDialog.DialogCode.Accepted:
            if msgbox.show_question(self, "Delete Target", f"Delete '{ship.name}'?") == msgbox.StandardButton.Yes:
                self.delete_single_target(idx)
        elif res == 2: 
            self.highlighted_ship_idx = idx
            self.draw_static_map() 

    def delete_single_target(self, idx):
        current_project.ships = [s for s in current_project.ships if s.idx != idx]
        
        self.refresh_tables()
        self.draw_static_map()
        
        if self.window():
            self.window().update_obj_combo()
            self.window().redraw_map()
            if hasattr(self.window(), 'data_changed'):
                self.window().data_changed.emit()
            
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
            if count > 8: 
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