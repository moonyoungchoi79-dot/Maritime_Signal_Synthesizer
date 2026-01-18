import math
import random
import datetime
import traceback
import socket
import time
import numpy as np
import tempfile
import bisect

from PyQt6.QtCore import QObject, pyqtSignal, Qt, QPointF, QCoreApplication
from PyQt6.QtGui import QPolygonF

from app.core.models.project import current_project
from app.core.geometry import (
    vincenty_distance,
    pixel_to_coords, coords_to_pixel, normalize_lon,
    lla_to_ecef, ecef_to_lla, ecef_distance, ecef_move, ecef_heading,
    ecef_interpolate, path_points_to_ecef
)
from app.core.nmea import calculate_checksum, encode_ais_payload

# Panorama constants (per specification B-3)
PANO_W_PX = 1920
PANO_H_PX = 320
K_H = 2200
H_MIN, H_MAX = 6, 220
W_MIN, W_MAX = 6, 300
EPS = 1e-3
CY_RATIO = 0.60  # cy_px = 0.60 * pano_h_px


def wrap_to_180(deg):
    """Wrap angle to [-180, 180] range for relative bearing calculation."""
    return ((deg + 180) % 360) - 180


class SimulationWorker(QObject):
    signal_generated = pyqtSignal(str)
    log_message = pyqtSignal(str)
    performance_updated = pyqtSignal(str)
    positions_updated = pyqtSignal(dict)
    export_data_ready = pyqtSignal(bool)
    finished = pyqtSignal()
    random_targets_generated = pyqtSignal()
    event_triggered = pyqtSignal(str, int)
    camera_detections_updated = pyqtSignal(list)  # List of detection dicts for panorama view

    def __init__(self, ip, port, speed_mult, duration_sec):
        super().__init__()
        self.ip = ip
        self.port = port
        self.speed_mult = speed_mult
        self.duration_sec = duration_sec
        self.running = True
        self.paused = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.proj = current_project
        self.temp_file_handle = None 
        self.last_gui_update_time = 0.0
        self.last_log_update_time = 0.0
        self.last_emission_times = {}
        self.last_sps_update_time = 0.0
        self.last_sps_update_m = 0
        self.m = 0 
        self.current_time = 0.0
        self.active_ships = []
        self.triggered_events = set()
        self.dynamic_ships = {}
        self.events = []
        self.max_steps = 0
        # Camera burst loss state tracking per ship (B-5)
        self.camera_burst_states = {}  # {ship_idx: {'in_burst': bool, 'end_time': float}}
        self.camera_detections = []  # Current frame's camera detections for panorama

    def reset_triggered_event(self, event_id: str):
        """Remove ID from triggered_events to allow re-evaluation when event is modified"""
        self.triggered_events.discard(event_id)

    def _densify_path(self, raw_points, mi, max_segment_m=100.0):
        """
        Connect Control Points with Great Circle paths to generate route.
        """
        if not raw_points or len(raw_points) < 2:
            ecef_pts = path_points_to_ecef(raw_points, mi)
            return ecef_pts, [0.0] * len(ecef_pts), 0.0

        dense_ecef = []

        # 1. Pixel -> LLA conversion (Control Points)
        lla_points = []
        for px, py in raw_points:
            _, _, lat, lon = pixel_to_coords(px, py, mi)
            lla_points.append((lat, lon))

        # 2. Great Circle Interpolation between control points
        for i in range(len(lla_points) - 1):
            lat1, lon1 = lla_points[i]
            lat2, lon2 = lla_points[i+1]

            x1, y1, z1 = lla_to_ecef(lat1, lon1, 0.0)
            dense_ecef.append((x1, y1, z1))
            dist_m = vincenty_distance(lat1, lon1, lat2, lon2)
                    # ...
            # [Fix] Add destination point ECEF conversion (was missing)
            x2, y2, z2 = lla_to_ecef(lat2, lon2, 0.0)

            # Great circle distance calculation
            dist_m = vincenty_distance(lat1, lon1, lat2, lon2)

            # Split into smaller segments along great circle if segment is long
            if dist_m > max_segment_m:
                num_steps = int(math.ceil(dist_m / max_segment_m))
                for s in range(1, num_steps):
                    frac = s / num_steps

                    # ECEF Slerp (chord interpolation through Earth center)
                    ix, iy, iz = ecef_interpolate(x1, y1, z1, x2, y2, z2, frac)

                    # Project to surface (altitude 0m correction) -> becomes point on great circle
                    t_lat, t_lon, _ = ecef_to_lla(ix, iy, iz)
                    dx, dy, dz = lla_to_ecef(t_lat, t_lon, 0.0)

                    dense_ecef.append((dx, dy, dz))

        # Add last control point
        last_lat, last_lon = lla_points[-1]
        lx, ly, lz = lla_to_ecef(last_lat, last_lon, 0.0)
        dense_ecef.append((lx, ly, lz))

        # 3. Cumulative distance calculation (required for Rail Logic)
        cumulative_dists = [0.0]
        total_len = 0.0
        for i in range(len(dense_ecef) - 1):
            x1, y1, z1 = dense_ecef[i]
            x2, y2, z2 = dense_ecef[i+1]
            seg_len = ecef_distance(x1, y1, z1, x2, y2, z2)
            total_len += seg_len
            cumulative_dists.append(total_len)

        return dense_ecef, cumulative_dists, total_len

    def run(self):
        try:
            self.log_message.emit(f"Starting Simulation UDP->{self.ip}:{self.port} Speed={self.speed_mult}x")
            self.temp_file_handle = tempfile.TemporaryFile(mode='w+', encoding='utf-8')
            
            seed_val = self.proj.seed % (2**32 - 1)
            random.seed(seed_val)
            np.random.seed(seed_val)
            
            self.active_ships = [s for s in self.proj.ships if s.raw_points] 
            if not self.active_ships:
                self.log_message.emit("No active ships with paths.")
                self.finished.emit()
                return

            self.dynamic_ships = {}
            mi = self.proj.map_info

            for s in self.active_ships:
                s_seed = (self.proj.seed + s.idx) % (2**32 - 1)
                s_rng = random.Random(s_seed)

                # Initial position
                start_px, start_py = s.raw_points[0]
                _, _, start_lat, start_lon = pixel_to_coords(start_px, start_py, mi)
                x, y, z = lla_to_ecef(start_lat, start_lon, 0.0)

                # [Optimization] Random ships (idx >= 1000) use Vector Mode without path generation
                if s.idx >= 1000:
                    # Random heading (0 ~ 360 degrees)
                    init_hdg = s_rng.uniform(0.0, 360.0)
                    self.dynamic_ships[s.idx] = {
                        'x': x, 'y': y, 'z': z,
                        'spd': s.raw_speeds.get(0, 5.0),
                        'sog': s.raw_speeds.get(0, 5.0),
                        'hdg': init_hdg,
                        'dist_traveled': 0.0,
                        'path_segment_idx': 0,
                        'ecef_path': [],
                        'cum_dists': [],
                        'total_path_len': 0.0,
                        'following_path': False,  # Vector Mode
                        'manual_speed': False,
                        'manual_heading': False,
                        'mode': None,
                        'rng': s_rng,
                        'target_recovery_idx': -1,
                        'target_dest_ecef': None
                    }
                else:
                    # Normal ships: generate great circle route from control points
                    points = s.raw_points
                    ecef_path, cum_dists, total_len = self._densify_path(points, mi)

                    # Initial Heading setup
                    init_hdg = 0.0
                    if len(ecef_path) > 1:
                        tx, ty, tz = ecef_path[1]
                        init_hdg = ecef_heading(x, y, z, tx, ty, tz)

                    self.dynamic_ships[s.idx] = {
                        'x': x, 'y': y, 'z': z,
                        'spd': s.raw_speeds.get(0, 5.0),
                        'sog': s.raw_speeds.get(0, 5.0),
                        'hdg': init_hdg,
                        'dist_traveled': 0.0,
                        'path_segment_idx': 0,
                        'ecef_path': ecef_path,
                        'cum_dists': cum_dists,
                        'total_path_len': total_len,
                        'following_path': True,
                        'manual_speed': False,
                        'manual_heading': False,
                        'mode': None,
                        'rng': s_rng,
                        'target_recovery_idx': -1,
                        'target_dest_ecef': None
                    }
            
            self.events = [e for e in self.proj.events if e.enabled]
            self.triggered_events = set()

            own_idx = self.proj.settings.own_ship_idx
            own_ship = self.proj.get_ship_by_idx(own_idx)
            
            dT = self.proj.unit_time
            self.max_steps = math.ceil(self.duration_sec / dT)
            
            self.last_sps_update_time = time.time()
            self.last_sps_update_m = 0            

            mi = self.proj.map_info

            m = 0
            
            while self.running:
                self.m = m
                QCoreApplication.processEvents()
                
                if self.paused:
                      time.sleep(0.1)
                      continue

                # Refresh events logic
                try:
                    active_events = [e for e in self.proj.events if e.enabled]
                    from app.core.state import loaded_scenarios
                    for scen in loaded_scenarios:
                        if not scen.enabled: continue
                        for evt in scen.events:
                            if not evt.enabled: continue
                            should_apply = False
                            if scen.scope_mode == "ALL_SHIPS": should_apply = True
                            elif scen.scope_mode == "OWN_ONLY":
                                if evt.target_ship_idx == self.proj.settings.own_ship_idx: should_apply = True
                            elif scen.scope_mode == "TARGET_ONLY":
                                if evt.target_ship_idx != self.proj.settings.own_ship_idx: should_apply = True
                            elif scen.scope_mode == "SELECTED_SHIPS":
                                if evt.target_ship_idx in scen.selected_ships: should_apply = True
                            if should_apply:
                                if not any(e.id == evt.id for e in active_events): active_events.append(evt)
                    self.events = active_events
                except Exception as e:
                    self.log_message.emit(f"[Warning] Error refreshing events: {e}")

                loop_start = time.time()
                t_m = m * dT
                self.current_time = t_m
                ts_val = self.proj.start_time + datetime.timedelta(seconds=t_m)
                
                self.check_events(t_m)

                # Clear camera detections for this frame
                self.camera_detections = []

                current_positions = {}
                
                own_state = None
                if own_ship and own_ship.raw_points:
                      own_state = self.update_and_get_state(own_ship, dT)
                      px, py = coords_to_pixel(own_state['lat_deg'], own_state['lon_deg'], mi)
                      current_positions[own_idx] = (px, py, own_state['heading_true_deg'], own_state['sog_knots'])

                for tgt in self.active_ships:
                    if tgt.idx == own_idx: continue 
                    tgt_state = self.update_and_get_state(tgt, dT)
                    px, py = coords_to_pixel(tgt_state['lat_deg'], tgt_state['lon_deg'], mi)
                    current_positions[tgt.idx] = (px, py, tgt_state['heading_true_deg'], tgt_state['sog_knots'])
                    if own_ship and own_state:
                        self.generate_ais_radar_group(own_ship, tgt, tgt_state, ts_val)
                
                current_real_time = time.time()
                if current_real_time - self.last_gui_update_time > 0.05:
                    self.positions_updated.emit(current_positions)
                    # Emit camera detections for panorama view (C-3: throttled with GUI update)
                    if self.camera_detections:
                        self.camera_detections_updated.emit(list(self.camera_detections))
                    self.last_gui_update_time = current_real_time

                m += 1

                step_delay = dT / self.speed_mult
                current_loop_elapsed = time.time() - loop_start
                sleep_time = step_delay - current_loop_elapsed

                sps_update_interval = 10 if m <= 200 else 100
                if (m - self.last_sps_update_m) >= sps_update_interval:
                    current_time = time.time()
                    time_since = current_time - self.last_sps_update_time
                    steps_since = m - self.last_sps_update_m
                    sps = steps_since / time_since if time_since > 0 else 0.0
                    log_msg = f"SPS: {sps:.1f} (Elapsed: {current_loop_elapsed:.4f}s)"
                    self.performance_updated.emit(log_msg)
                    self.last_sps_update_time = current_time
                    self.last_sps_update_m = m

                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    time.sleep(0.001)

            self.log_message.emit("Simulation Stopped.")
            self.export_data_ready.emit(True)
            self.finished.emit()
              
        except Exception as e:
            traceback.print_exc()
            self.log_message.emit(f"Sim Error: {e}")
        finally:
            self.sock.close()
            
    def refresh_active_ships(self):
        current_indices = {s.idx for s in self.active_ships}
        new_ships = [s for s in self.proj.ships if s.raw_points and s.idx not in current_indices]
        
        mi = self.proj.map_info
        for s in new_ships:
            s_seed = (self.proj.seed + s.idx) % (2**32 - 1)
            s_rng = random.Random(s_seed)
            points = s.raw_points # [Fix] Use raw_points
            if not points: continue

            # [Fix] Random ship optimization: remove densify_path call, set random heading/Vector Mode
            # Extract start point only and convert to ECEF
            start_px, start_py = points[0]
            _, _, lat, lon = pixel_to_coords(start_px, start_py, mi)
            x, y, z = lla_to_ecef(lat, lon, 0.0)

            # Generate random heading (0 ~ 360)
            init_hdg = s_rng.uniform(0.0, 360.0)

            self.dynamic_ships[s.idx] = {
                'x': x, 'y': y, 'z': z,
                'spd': s.raw_speeds.get(0, 5.0),
                'sog': 5.0,
                'hdg': init_hdg,
                'dist_traveled': 0.0,
                'path_segment_idx': 0,
                'ecef_path': [],        # Leave path empty
                'cum_dists': [],
                'total_path_len': 0.0,
                'following_path': False, # Disable path following (Vector Mode active)
                'manual_speed': False,
                'manual_heading': False,
                'mode': None,
                'rng': s_rng,
                'target_recovery_idx': -1,
                'target_dest_ecef': None
            }
            self.active_ships.append(s)

    def send_nmea(self, raw, ts_obj):
        if self.temp_file_handle:
            try:
                line = f"{ts_obj.timestamp()},{raw}\n"
                self.temp_file_handle.write(line)
            except: pass
        try:
            self.sock.sendto((raw + "\r\n").encode('ascii'), (self.ip, self.port))
        except: pass
        
        cur_t = time.time()
        if cur_t - self.last_log_update_time > 0.2:
            self.signal_generated.emit(raw)
            self.last_log_update_time = cur_t

    def stop(self):
        self.running = False

    def pause(self):
        self.paused = True

    def resume(self):
        self.paused = False

    def update_speed(self, mult):
        self.speed_mult = mult

    def update_duration(self, new_duration):
        self.max_steps = math.ceil(new_duration / self.proj.unit_time)

    def set_ship_speed(self, ship_idx, speed_kn):
        if ship_idx in self.dynamic_ships:
            self.dynamic_ships[ship_idx]['manual_speed'] = True
            self.dynamic_ships[ship_idx]['spd'] = speed_kn
            ship = self.proj.get_ship_by_idx(ship_idx)
            self.log_message.emit(f"[Manual] Speed changed for {ship.name if ship else ship_idx} to {speed_kn} kn")

    def set_ship_heading(self, ship_idx, heading_deg):
        if ship_idx in self.dynamic_ships:
            self.dynamic_ships[ship_idx]['hdg'] = heading_deg
            self.dynamic_ships[ship_idx]['manual_heading'] = True
            self.dynamic_ships[ship_idx]['following_path'] = False
            ship = self.proj.get_ship_by_idx(ship_idx)
            self.log_message.emit(f"[Manual] Heading changed for {ship.name if ship else ship_idx} to {heading_deg} deg")

    def _calculate_ecef_velocity(self, lat, lon, spd_kn, hdg_deg):
        spd_mps = spd_kn * 0.514444
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        hdg_rad = math.radians(hdg_deg)
        
        # Local NED unit vectors in ECEF
        # North
        n_x = -math.sin(lat_rad) * math.cos(lon_rad)
        n_y = -math.sin(lat_rad) * math.sin(lon_rad)
        n_z = math.cos(lat_rad)
        
        # East
        e_x = -math.sin(lon_rad)
        e_y = math.cos(lon_rad)
        e_z = 0.0
        
        # Heading vector composition
        v_x = (n_x * math.cos(hdg_rad) + e_x * math.sin(hdg_rad)) * spd_mps
        v_y = (n_y * math.cos(hdg_rad) + e_y * math.sin(hdg_rad)) * spd_mps
        v_z = (n_z * math.cos(hdg_rad) + e_z * math.sin(hdg_rad)) * spd_mps
        
        return v_x, v_y, v_z

    def _ecef_dist_to_geodesic_nm(self, d_ecef):
        # ECEF Chord length -> Geodesic Arc length conversion
        R = 6371000.0
        if d_ecef >= 2 * R: return math.pi * R / 1852.0
        
        val = d_ecef / (2.0 * R)
        if val > 1.0: val = 1.0
        theta = 2.0 * math.asin(val)
        
        dist_m = R * theta
        return dist_m / 1852.0

    def _check_prerequisites(self, evt) -> bool:
        """Check prerequisite event conditions"""
        prereqs = getattr(evt, 'prerequisite_events', [])
        if not prereqs:
            return True  # Pass if no conditions

        logic = getattr(evt, 'prerequisite_logic', 'AND')
        results = []

        for cond in prereqs:
            # cond can be EventCondition object or dict
            if isinstance(cond, dict):
                event_id = cond.get('event_id', '')
                mode = cond.get('mode', 'TRIGGERED')
            else:
                event_id = cond.event_id
                mode = cond.mode

            is_triggered = event_id in self.triggered_events

            if mode == "TRIGGERED":
                results.append(is_triggered)
            else:  # NOT_TRIGGERED
                results.append(not is_triggered)

        if logic == "AND":
            return all(results)
        else:  # OR
            return any(results)

    def check_events(self, t_current):
        for evt in self.events:
            if evt.id in self.triggered_events: continue

            # Check prerequisite event conditions
            if not self._check_prerequisites(evt):
                continue

            triggered = False

            if evt.trigger_type == "TIME":
                if t_current >= evt.condition_value: triggered = True
            elif evt.trigger_type == "AREA_ENTER":
                ship = self.proj.get_ship_by_idx(evt.target_ship_idx)
                area = self.proj.get_area_by_id(int(evt.condition_value))
                if ship and area and ship.idx in self.dynamic_ships:
                    dyn = self.dynamic_ships[ship.idx]
                    lat, lon, _ = ecef_to_lla(dyn['x'], dyn['y'], dyn['z'])
                    px, py = coords_to_pixel(lat, lon, self.proj.map_info)
                    poly = QPolygonF([QPointF(x,y) for x,y in area.geometry])
                    if poly.containsPoint(QPointF(px, py), Qt.FillRule.OddEvenFill): triggered = True
            elif evt.trigger_type == "AREA_LEAVE":
                ship = self.proj.get_ship_by_idx(evt.target_ship_idx)
                area = self.proj.get_area_by_id(int(evt.condition_value))
                if ship and area and ship.idx in self.dynamic_ships:
                    dyn = self.dynamic_ships[ship.idx]
                    lat, lon, _ = ecef_to_lla(dyn['x'], dyn['y'], dyn['z'])
                    px, py = coords_to_pixel(lat, lon, self.proj.map_info)
                    poly = QPolygonF([QPointF(x,y) for x,y in area.geometry])
                    if not poly.containsPoint(QPointF(px, py), Qt.FillRule.OddEvenFill): triggered = True
            
            elif evt.trigger_type in ["CPA_UNDER", "CPA_OVER", "DIST_UNDER", "DIST_OVER"]:
                tgt_ship = self.proj.get_ship_by_idx(evt.target_ship_idx)
                ref_idx = evt.reference_ship_idx
                if ref_idx == -1: ref_idx = self.proj.settings.own_ship_idx
                ref_ship = self.proj.get_ship_by_idx(ref_idx)
                
                if tgt_ship and ref_ship and tgt_ship.idx in self.dynamic_ships and ref_ship.idx in self.dynamic_ships:
                    td = self.dynamic_ships[tgt_ship.idx]
                    rd = self.dynamic_ships[ref_ship.idx]
                    
                    # 1. Current Distance (Chord -> Arc)
                    dist_chord_m = ecef_distance(td['x'], td['y'], td['z'], rd['x'], rd['y'], rd['z'])
                    dist_nm = self._ecef_dist_to_geodesic_nm(dist_chord_m)
                    
                    if evt.trigger_type == "DIST_UNDER" and dist_nm <= evt.condition_value: triggered = True
                    elif evt.trigger_type == "DIST_OVER" and dist_nm >= evt.condition_value: triggered = True
                    elif evt.trigger_type in ["CPA_UNDER", "CPA_OVER"]:
                        # [Fix] Calculate CPA with ECEF vector operations without LLA conversion
                        # Current position vectors
                        P_t = np.array([td['x'], td['y'], td['z']])
                        P_r = np.array([rd['x'], rd['y'], rd['z']])

                        # Velocity vectors (could be calculated without Lat/Lon from current state,
                        # but keeping existing _calculate_ecef_velocity causes 1 LLA conversion internally.
                        # No major performance issue, but for pure geometric optimization,
                        # could compose vectors in ECEF local frame using ship heading/spd)
                        t_lat, t_lon, _ = ecef_to_lla(td['x'], td['y'], td['z'])
                        r_lat, r_lon, _ = ecef_to_lla(rd['x'], rd['y'], rd['z'])

                        tvx, tvy, tvz = self._calculate_ecef_velocity(t_lat, t_lon, td.get('sog', td['spd']), td['hdg'])
                        rvx, rvy, rvz = self._calculate_ecef_velocity(r_lat, r_lon, rd.get('sog', rd['spd']), rd['hdg'])

                        V_t = np.array([tvx, tvy, tvz])
                        V_r = np.array([rvx, rvy, rvz])

                        # Relative position and relative velocity
                        P_rel = P_t - P_r
                        V_rel = V_t - V_r

                        v_rel_sq = np.dot(V_rel, V_rel)
                        cpa_dist_nm = dist_nm # Default to current

                        if v_rel_sq > 1e-9:
                            # t_cpa (seconds) = - (P_rel . V_rel) / |V_rel|^2
                            t_cpa = -np.dot(P_rel, V_rel) / v_rel_sq

                            if t_cpa > 0:
                                # [Fix] Future position prediction: assume linear constant velocity motion (Linear Projection in ECEF)
                                # Removed LLA-based _move_great_circle_step
                                P_t_future = P_t + V_t * t_cpa
                                P_r_future = P_r + V_r * t_cpa
                                
                                future_dist_chord = np.linalg.norm(P_t_future - P_r_future)
                                cpa_dist_nm = self._ecef_dist_to_geodesic_nm(future_dist_chord)
                        
                        if evt.trigger_type == "CPA_UNDER" and cpa_dist_nm <= evt.condition_value: triggered = True
                        elif evt.trigger_type == "CPA_OVER" and cpa_dist_nm >= evt.condition_value: triggered = True

            if triggered:
                self.log_message.emit(f"[EVENT] {evt.name} triggered.")
                self.event_triggered.emit(evt.name, evt.target_ship_idx)
                self.triggered_events.add(evt.id)
                self.apply_event_action(evt, t_current)

    def apply_event_action(self, evt, t_current):
        sid = evt.target_ship_idx
        if sid not in self.dynamic_ships: return
        dyn = self.dynamic_ships[sid]
        
        if evt.action_type == "STOP":
            dyn['spd'] = 0.0
            dyn['manual_speed'] = True
        elif evt.action_type == "CHANGE_SPEED":
            dyn['spd'] = evt.action_value
            dyn['manual_speed'] = True
        elif evt.action_type == "CHANGE_HEADING":
            dyn['hdg'] = evt.action_value
            dyn['manual_heading'] = True
            dyn['following_path'] = False
        elif evt.action_type == "CHANGE_DESTINATION_LOC":
            try:
                lat, lon = map(float, evt.action_option.split(","))
                tx, ty, tz = lla_to_ecef(lat, lon, 0.0)
                dyn['target_dest_ecef'] = (tx, ty, tz)
                
                dyn['following_path'] = False
                dyn['mode'] = 'GO_TO_COORD'
                dyn['hdg'] = ecef_heading(dyn['x'], dyn['y'], dyn['z'], tx, ty, tz)
            except:
                self.log_message.emit(f"[Error] Invalid coords for event {evt.name}")

        elif evt.action_type == "MANEUVER":
            opt = getattr(evt, 'action_option', "")
            if opt == "ReturnToOriginalPath_ShortestDistance":
                ecef_path = dyn.get('ecef_path', [])
                if ecef_path:
                    cx, cy, cz = dyn['x'], dyn['y'], dyn['z']
                    best_idx = 0
                    best_dist = float('inf')
                    # One-time search at trigger
                    for i in range(len(ecef_path)):
                        px, py, pz = ecef_path[i]
                        d = ecef_distance(cx, cy, cz, px, py, pz)
                        if d < best_dist:
                            best_dist = d
                            best_idx = i
                    
                    dyn['target_recovery_idx'] = best_idx
                    dyn['mode'] = 'RETURN_TO_PATH'
                    
            elif opt == "ChangeDestination_ToOriginalFinal":
                ecef_path = dyn.get('ecef_path', [])
                if ecef_path:
                    tx, ty, tz = ecef_path[-1]
                    dyn['hdg'] = ecef_heading(dyn['x'], dyn['y'], dyn['z'], tx, ty, tz)
                    dyn['following_path'] = False 
                    dyn['manual_heading'] = True 

    def update_and_get_state(self, ship, dT):
        # [Fix] Strengthen dynamic state initialization logic: never reinitialize if already exists
        # This prevents dist_traveled from being reset to 0 by events or other logic
        if ship.idx not in self.dynamic_ships:
            mi = self.proj.map_info
            if ship.raw_points: # Use raw_points
                px, py = ship.raw_points[0]
                _, _, lat, lon = pixel_to_coords(px, py, mi)
                x, y, z = lla_to_ecef(lat, lon, 0.0)
                # Generate great circle route from raw_points
                ecef_path, cum_dists, total_len = self._densify_path(ship.raw_points, mi)
            else:
                x, y, z = lla_to_ecef(0, 0, 0.0)
                ecef_path, cum_dists, total_len = [], [], 0.0
                
            self.dynamic_ships[ship.idx] = {
                'x': x, 'y': y, 'z': z,
                'spd': 5.0, 'sog': 5.0, 'hdg': 0, 
                'dist_traveled': 0.0, 'path_segment_idx': 0,
                'ecef_path': ecef_path, 'cum_dists': cum_dists, 'total_path_len': total_len,
                'following_path': True, 'manual_speed': False, 'manual_heading': False,
                'mode': None, 'rng': random.Random(),
                'target_recovery_idx': -1,
                'target_dest_ecef': None
            }

        dyn = self.dynamic_ships[ship.idx]
        ecef_path = dyn.get('ecef_path', [])
        cum_dists = dyn.get('cum_dists', [])
        total_len = dyn.get('total_path_len', 0.0)

        # 1. Speed calculation
        variance = getattr(self.proj.settings, "speed_variance", 1.0)
        sigma = math.sqrt(variance)
        noise = dyn['rng'].gauss(0, sigma)
        
        target_spd = dyn.get('spd', 0.0)
        if target_spd > 0.01:
            current_speed_kn = max(0.0, target_spd + noise)
        else:
            current_speed_kn = 0.0
        
        dyn['sog'] = current_speed_kn 
            
        move_dist_m = current_speed_kn * 1852.0 * (dT / 3600.0)
        
        # GO_TO_COORD Mode
        if dyn.get('mode') == 'GO_TO_COORD' and dyn.get('target_dest_ecef'):
            tx, ty, tz = dyn['target_dest_ecef']
            curr_dist = ecef_distance(dyn['x'], dyn['y'], dyn['z'], tx, ty, tz)
            
            if curr_dist <= move_dist_m:
                dyn['x'], dyn['y'], dyn['z'] = tx, ty, tz
                dyn['spd'] = 0.0
                dyn['sog'] = 0.0
                dyn['mode'] = None
            else:
                target_hdg = ecef_heading(dyn['x'], dyn['y'], dyn['z'], tx, ty, tz)
                lat, lon, _ = ecef_to_lla(dyn['x'], dyn['y'], dyn['z'])
                nlat, nlon, nhdg = self._move_great_circle_step(lat, lon, target_hdg, move_dist_m)
                
                dyn['x'], dyn['y'], dyn['z'] = lla_to_ecef(nlat, nlon, 0.0)
                dyn['hdg'] = nhdg
                
            return self._make_state_result(0,0,0,0) if dyn['spd']==0 else \
                   self._make_state_result(*ecef_to_lla(dyn['x'], dyn['y'], dyn['z'])[:2], current_speed_kn, dyn['hdg'])

        # RETURN_TO_PATH Mode
        if dyn.get('mode') == 'RETURN_TO_PATH' and ecef_path:
            best_idx = dyn.get('target_recovery_idx', 0)
            if best_idx < 0 or best_idx >= len(ecef_path): best_idx = 0 
            
            px, py, pz = ecef_path[best_idx]
            curr_dist = ecef_distance(dyn['x'], dyn['y'], dyn['z'], px, py, pz)
            
            if curr_dist <= move_dist_m * 1.5: 
                dyn['following_path'] = True
                dyn['mode'] = None
                dyn['dist_traveled'] = cum_dists[best_idx]
                dyn['path_segment_idx'] = best_idx
                dyn['x'], dyn['y'], dyn['z'] = ecef_path[best_idx]
            else:
                tx, ty, tz = ecef_path[best_idx]
                target_hdg = ecef_heading(dyn['x'], dyn['y'], dyn['z'], tx, ty, tz)
                
                lat, lon, _ = ecef_to_lla(dyn['x'], dyn['y'], dyn['z'])
                nlat, nlon, nhdg = self._move_great_circle_step(lat, lon, target_hdg, move_dist_m)
                
                dyn['x'], dyn['y'], dyn['z'] = lla_to_ecef(nlat, nlon, 0.0)
                dyn['hdg'] = nhdg
                
                return self._make_state_result(nlat, nlon, current_speed_kn, nhdg)

        # 3. Movement logic: Rail vs Vector
        active_rail = False
        if dyn.get('following_path') and ecef_path:
            if dyn['dist_traveled'] + move_dist_m <= total_len:
                active_rail = True
            else:
                dyn['following_path'] = False 
                active_rail = False

        if active_rail:
            # --- RAIL MODE (Great Circle Following) ---
            dyn['dist_traveled'] += move_dist_m
            curr_d = dyn['dist_traveled']

            idx = bisect.bisect_right(cum_dists, curr_d) - 1
            idx = max(0, min(idx, len(ecef_path) - 2))
            dyn['path_segment_idx'] = idx

            d_start = cum_dists[idx]
            d_end = cum_dists[idx+1]
            segment_len = d_end - d_start

            frac = (curr_d - d_start) / segment_len if segment_len > 1e-6 else 0.0
            x1, y1, z1 = ecef_path[idx]
            x2, y2, z2 = ecef_path[idx+1]

            # [Position Update] Great circle interpolated position
            dyn['x'], dyn['y'], dyn['z'] = ecef_interpolate(x1, y1, z1, x2, y2, z2, frac)

            # [Key Fix: Heading Update] "Must always change at every moment"
            # Calculate great circle bearing from current position(dyn) to next path point(x2, y2, z2)
            dyn['hdg'] = ecef_heading(dyn['x'], dyn['y'], dyn['z'], x2, y2, z2)

        else:
            # --- VECTOR MODE (Great Circle Free Navigation) ---
            if move_dist_m > 0:
                lat, lon, _ = ecef_to_lla(dyn['x'], dyn['y'], dyn['z'])
                # Great circle movement with current heading -> heading auto-updates with position change
                nlat, nlon, nhdg = self._move_great_circle_step(lat, lon, dyn['hdg'], move_dist_m)

                dyn['x'], dyn['y'], dyn['z'] = lla_to_ecef(nlat, nlon, 0.0)
                dyn['hdg'] = nhdg

        # 4. Return result
        lat, lon, _ = ecef_to_lla(dyn['x'], dyn['y'], dyn['z'])
        
        if lat > 89.9: lat = 89.9
        elif lat < -89.9: lat = -89.9
        lon = normalize_lon(lon)
        
        return self._make_state_result(lat, lon, current_speed_kn, dyn['hdg'])

    def _move_great_circle_step(self, lat, lon, hdg_deg, dist_m):
        """
        Move from current position via Great Circle Sailing and
        return new position and bearing at that point.
        """
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        hdg_rad = math.radians(hdg_deg)
        R = 6371000.0 # Earth Radius
        ang_dist = dist_m / R

        # New Position
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

        # New Heading (Final Bearing + 180 strategy)
        y = math.sin(lon_rad - new_lon_rad) * math.cos(lat_rad)
        x = math.cos(new_lat_rad) * math.sin(lat_rad) - \
            math.sin(new_lat_rad) * math.cos(lat_rad) * math.cos(lon_rad - new_lon_rad)
        
        back_bearing_rad = math.atan2(y, x)
        back_bearing_deg = math.degrees(back_bearing_rad)
        
        new_hdg = (back_bearing_deg + 180) % 360
        
        return new_lat, new_lon, new_hdg

    def _make_state_result(self, lat, lon, sog, cog):
        return {
            "lat_deg": lat,
            "lon_deg": lon,
            "sog_knots": sog,
            "sog_kmh": sog * 1.852,
            "sog_mps": sog * 0.51444444,
            "cog_true_deg": cog,
            "heading_true_deg": cog,
            "elapsed_sec": 0
        }

    def get_current_state(self, ship):
        if ship.idx in self.dynamic_ships:
            dyn = self.dynamic_ships[ship.idx]
            lat, lon, _ = ecef_to_lla(dyn['x'], dyn['y'], dyn['z'])
            sog = dyn.get('sog', dyn['spd'])
            return self._make_state_result(lat, lon, sog, dyn['hdg'])
        return self._make_state_result(0,0,0,0)

    def check_dropout(self, stype):
        prob = self.proj.settings.dropout_probs.get(stype, 0.1)
        return random.random() < prob

    def calculate_dropout_probability(self, distance_nm: float, config) -> float:
        """Calculate distance-based dropout probability"""
        if not config.enabled:
            return 0.0

        d, d0, d1, p0, p1 = distance_nm, config.d0, config.d1, config.p0, config.p1

        if d <= d0:
            return p0  # Stable zone
        elif d >= d1:
            return 1.0 if config.full_block_at_d1 else p1  # Full block or p1
        else:
            # d0 ~ d1 zone: curve interpolation
            t = (d - d0) / (d1 - d0)  # 0 ~ 1 normalization

            if config.curve_type == "linear":
                factor = t
            elif config.curve_type == "logistic":
                factor = 1 / (1 + math.exp(-10 * (t - 0.5)))
            else:  # smoothstep (default)
                factor = t * t * (3 - 2 * t)

            return p0 + (p1 - p0) * factor

    def check_dropout_distance_based(self, stype: str, distance_nm: float) -> bool:
        """Distance-based dropout determination"""
        if not self.proj.settings.reception_model_enabled:
            # Fallback to existing fixed probability method
            return self.check_dropout(stype)

        if stype == "AIVDM":
            config = self.proj.settings.ais_reception
        elif stype == "RATTM":
            config = self.proj.settings.radar_detect
        elif stype == "ARPA":
            config = self.proj.settings.arpa_track
        elif stype == "Camera":
            config = self.proj.settings.camera_reception
        else:
            return False

        prob = self.calculate_dropout_probability(distance_nm, config)
        return random.random() < prob

    def get_jittered_time(self, base_ts):
        if self.proj.settings.jitter_enabled:
            offset = random.choice([-2, -1, 0, 1, 2])
            return base_ts + datetime.timedelta(seconds=offset)
        return base_ts

    def generate_ais_radar_group(self, own, tgt, state, ts_val):
        jitter_ts = self.get_jittered_time(ts_val)
        base = self.make_base_row(own, tgt, state, jitter_ts)

        # Calculate distance between two ships (for distance-based reception model)
        own_dyn = self.dynamic_ships.get(own.idx)
        tgt_dyn = self.dynamic_ships.get(tgt.idx)
        if own_dyn and tgt_dyn:
            dist_m = ecef_distance(own_dyn['x'], own_dyn['y'], own_dyn['z'],
                                   tgt_dyn['x'], tgt_dyn['y'], tgt_dyn['z'])
            dist_nm = self._ecef_dist_to_geodesic_nm(dist_m)
        else:
            dist_nm = 0.0

        stype = "AIVDM"
        ship = self.proj.get_ship_by_idx(tgt.idx)
        if ship:
            if ship.signals_enabled.get(stype, True):
                interval = ship.signal_intervals.get(stype, 1.0)
                current_sim_time = (jitter_ts - self.proj.start_time).total_seconds()
                last_time = self.last_emission_times.get((ship.idx, stype), -float('inf'))
                if (current_sim_time + 1e-9) >= (last_time + interval):
                    if not self.check_dropout_distance_based(stype, dist_nm):
                        ais_msgs = self.gen_ais_multi(base, state, "VDM", tgt.mmsi)
                        for msg in ais_msgs: self.send_nmea(msg, jitter_ts)
                        self.last_emission_times[(ship.idx, stype)] = current_sim_time
        self.try_emit_distance_based(self.gen_ttm(base, state), "RATTM", jitter_ts, tgt.idx, dist_nm)

        # Generate Camera signal (B-1)
        if ship and own_dyn and tgt_dyn:
            if ship.signals_enabled.get("Camera", True):
                cam_raw, detection = self.gen_cam_signal(own_dyn, tgt, tgt_dyn, dist_nm)
                if cam_raw and detection:
                    current_sim_time = (jitter_ts - self.proj.start_time).total_seconds()
                    # Check interval
                    interval = ship.signal_intervals.get("Camera", 1.0)
                    last_time = self.last_emission_times.get((ship.idx, "Camera"), -float('inf'))
                    if (current_sim_time + 1e-9) >= (last_time + interval):
                        # Check dropout with burst support (B-4, B-5)
                        if not self.check_camera_dropout_with_burst(tgt.idx, dist_nm, current_sim_time):
                            self.send_nmea(cam_raw, jitter_ts)
                            self.last_emission_times[(ship.idx, "Camera")] = current_sim_time
                            # Add to current frame's detections for panorama view
                            self.camera_detections.append(detection)

    def try_emit(self, raw, stype, ts_obj, ship_idx):
        ship = self.proj.get_ship_by_idx(ship_idx)
        if not ship: return
        if not ship.signals_enabled.get(stype, True): return
        interval = ship.signal_intervals.get(stype, 1.0)
        current_sim_time = (ts_obj - self.proj.start_time).total_seconds()
        last_time = self.last_emission_times.get((ship_idx, stype), -float('inf'))
        if (current_sim_time + 1e-9) < (last_time + interval): return
        if not raw: return
        self.last_emission_times[(ship_idx, stype)] = current_sim_time
        if self.check_dropout(stype): return
        self.send_nmea(raw, ts_obj)

    def try_emit_distance_based(self, raw, stype, ts_obj, ship_idx, dist_nm):
        """Signal emission function with distance-based dropout"""
        ship = self.proj.get_ship_by_idx(ship_idx)
        if not ship: return
        if not ship.signals_enabled.get(stype, True): return
        interval = ship.signal_intervals.get(stype, 1.0)
        current_sim_time = (ts_obj - self.proj.start_time).total_seconds()
        last_time = self.last_emission_times.get((ship_idx, stype), -float('inf'))
        if (current_sim_time + 1e-9) < (last_time + interval): return
        if not raw: return
        self.last_emission_times[(ship_idx, stype)] = current_sim_time
        if self.check_dropout_distance_based(stype, dist_nm): return
        self.send_nmea(raw, ts_obj)

    def make_base_row(self, rcv, tgt, state, ts_val):
        return { "tgt_ship_index": tgt.idx, "tgt_ship_name": tgt.name, "rx_time": ts_val }

    def gen_ais_multi(self, base, state, stype, mmsi):
        probs = current_project.settings.ais_fragment_probs
        total_prob = sum(probs)
        if total_prob == 0: total_fragments = 1 
        else:
            normalized_probs = [p / total_prob for p in probs]
            total_fragments = random.choices(range(1, 6), weights=normalized_probs, k=1)[0]
        payload = encode_ais_payload(mmsi) 
        fragment_min_len = 1 
        fragment_length = max(fragment_min_len, math.ceil(len(payload) / total_fragments))
        ais_messages = []
        seq_id = random.randint(0, 9) 
        for i in range(total_fragments):
            start_idx = i * fragment_length
            end_idx = min(len(payload), (i + 1) * fragment_length) 
            fragment = payload[start_idx : end_idx]
            raw = f"!AI{stype},{total_fragments},{i+1},{seq_id},A,{fragment},0"
            msg = f"{raw}*{calculate_checksum(raw[1:])}"
            ais_messages.append(msg)
        return ais_messages

    def gen_ttm(self, base, state):
        tgt_num = base['tgt_ship_index']
        dt = base['rx_time']
        utc_time = dt.strftime("%H%M%S.%f")[:9]
        raw = f"$RATTM,{tgt_num:02d},1.5,45.0,T,{state['sog_knots']:.1f},{state['cog_true_deg']:.1f},T,0.5,10.0,N,b,T,,{utc_time},A"
        return f"{raw}*{calculate_checksum(raw[1:])}"

    def gen_cam_signal(self, own_dyn, tgt_ship, tgt_dyn, dist_nm):
        """
        Generate $CAMERA NMEA sentence for camera detection.
        Returns tuple: (raw_nmea, detection_dict) or (None, None) if out of FOV.

        The ship is modeled as a 3D box (length x beam x height).
        The visible width in panorama depends on which faces are visible,
        determined by the angle between observer's view direction and target's heading.
        """
        # Calculate true bearing from own ship to target
        bearing_true = ecef_heading(own_dyn['x'], own_dyn['y'], own_dyn['z'],
                                    tgt_dyn['x'], tgt_dyn['y'], tgt_dyn['z'])
        own_heading = own_dyn['hdg']

        # Calculate relative bearing (A-2)
        rel_bearing = wrap_to_180(bearing_true - own_heading)

        # Check if in FOV [-90, +90] (A-2)
        if rel_bearing < -90 or rel_bearing > 90:
            return None, None

        # Calculate panorama x coordinate
        x_norm = (rel_bearing + 90.0) / 180.0
        cx_px = x_norm * PANO_W_PX
        cy_px = CY_RATIO * PANO_H_PX

        # Ship dimensions
        range_m = dist_nm * 1852.0
        length_m = getattr(tgt_ship, 'length_m', 300.0)
        beam_m = getattr(tgt_ship, 'beam_m', 40.0)
        height_m = getattr(tgt_ship, 'height_m', 30.0)  # Ship height above waterline

        # Calculate the viewing angle relative to target ship's heading
        # This determines which faces of the ship are visible
        tgt_heading = tgt_dyn['hdg']

        # Angle from target ship's bow to observer (reverse of bearing_true)
        # bearing_true: direction from observer to target
        # We need: direction from target to observer, relative to target's heading
        bearing_from_target = (bearing_true + 180.0) % 360.0
        view_angle = wrap_to_180(bearing_from_target - tgt_heading)

        # view_angle interpretation:
        # 0° = viewing from bow (front face: beam x height)
        # 90° = viewing from starboard (side face: length x height)
        # 180° or -180° = viewing from stern (back face: beam x height)
        # -90° = viewing from port (side face: length x height)

        # Calculate projected width based on view angle
        # The visible width is a combination of length and beam faces
        view_angle_rad = math.radians(view_angle)
        # |sin| gives contribution from side (length), |cos| gives contribution from front/back (beam)
        projected_width_m = abs(math.sin(view_angle_rad)) * length_m + abs(math.cos(view_angle_rad)) * beam_m

        # Height is always the same (viewing from sea level, seeing only the side)
        projected_height_m = height_m

        # Convert to pixels based on distance
        w_px = max(W_MIN, min(W_MAX, K_H * (projected_width_m / max(range_m, EPS))))
        h_px = max(H_MIN, min(H_MAX, K_H * (projected_height_m / max(range_m, EPS))))

        # Clamp bbox to panorama bounds
        x1 = max(0, cx_px - w_px / 2)
        x2 = min(PANO_W_PX, cx_px + w_px / 2)
        y1 = max(0, cy_px - h_px / 2)
        y2 = min(PANO_H_PX, cy_px + h_px / 2)
        w_px = x2 - x1
        h_px = y2 - y1
        cx_px = (x1 + x2) / 2
        cy_px = (y1 + y2) / 2

        # Build NMEA sentence
        ship_class = getattr(tgt_ship, 'ship_class', 'CONTAINER')
        raw = f"$CAMERA,{tgt_ship.idx},{ship_class},{rel_bearing:.2f},{PANO_W_PX},{PANO_H_PX},{cx_px:.2f},{cy_px:.2f},{w_px:.2f},{h_px:.2f}"
        checksum = calculate_checksum(raw[1:])
        nmea_str = f"{raw}*{checksum}"

        # Detection dict for panorama view
        detection = {
            'ship_idx': tgt_ship.idx,
            'ship_name': tgt_ship.name,
            'rel_bearing': rel_bearing,
            'cx_px': cx_px,
            'cy_px': cy_px,
            'w_px': w_px,
            'h_px': h_px
        }

        return nmea_str, detection

    def check_camera_dropout_with_burst(self, ship_idx, dist_nm, current_time):
        """
        Check camera dropout with burst loss support (B-4, B-5).
        Returns True if signal should be dropped.
        """
        config = self.proj.settings.camera_reception

        # Check if currently in burst (B-5)
        burst_state = self.camera_burst_states.get(ship_idx, {'in_burst': False, 'end_time': 0})
        if burst_state['in_burst']:
            if current_time < burst_state['end_time']:
                return True  # In burst, always dropout
            else:
                # Burst ended
                burst_state['in_burst'] = False
                self.camera_burst_states[ship_idx] = burst_state

        # Calculate base dropout probability (B-4)
        prob = self.calculate_dropout_probability(dist_nm, config)

        # Random dropout check
        if random.random() < prob:
            # Check for burst start if enabled (B-5)
            if config.burst_enabled:
                burst_prob = min(1.0, prob * config.burst_trigger_mult)
                if random.random() < burst_prob:
                    duration = random.uniform(config.burst_min_sec, config.burst_max_sec)
                    self.camera_burst_states[ship_idx] = {
                        'in_burst': True,
                        'end_time': current_time + duration
                    }
            return True
        return False