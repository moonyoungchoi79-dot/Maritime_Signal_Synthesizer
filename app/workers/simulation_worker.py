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
    haversine_distance, pixel_to_coords, coords_to_pixel, normalize_lon,
    lla_to_ecef, ecef_to_lla, ecef_distance, ecef_move, ecef_heading,
    ecef_interpolate, path_points_to_ecef
)
from app.core.nmea import calculate_checksum, encode_ais_payload

class SimulationWorker(QObject):
    signal_generated = pyqtSignal(str) 
    log_message = pyqtSignal(str)
    performance_updated = pyqtSignal(str)
    positions_updated = pyqtSignal(dict) 
    export_data_ready = pyqtSignal(bool) 
    finished = pyqtSignal()
    random_targets_generated = pyqtSignal()
    event_triggered = pyqtSignal(str, int)
    
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

    def _densify_path(self, raw_points, mi, max_segment_m=100.0):
        """
        제어점(Control Points) 사이를 대권(Great Circle)으로 연결하여 경로를 생성합니다.
        """
        if not raw_points or len(raw_points) < 2:
            ecef_pts = path_points_to_ecef(raw_points, mi)
            return ecef_pts, [0.0] * len(ecef_pts), 0.0

        dense_ecef = []
        
        # 1. Pixel -> LLA 변환 (Control Points)
        lla_points = []
        for px, py in raw_points:
            _, _, lat, lon = pixel_to_coords(px, py, mi)
            lla_points.append((lat, lon))

        # 2. 제어점 간 대권 보간 (Great Circle Interpolation)
        for i in range(len(lla_points) - 1):
            lat1, lon1 = lla_points[i]
            lat2, lon2 = lla_points[i+1]
            
            x1, y1, z1 = lla_to_ecef(lat1, lon1, 0.0)
            dense_ecef.append((x1, y1, z1))
            
            # [수정] 도착점 ECEF 변환 코드 추가 (누락되었던 부분)
            x2, y2, z2 = lla_to_ecef(lat2, lon2, 0.0)
            
            # 대권 거리 계산
            dist_m = haversine_distance(lat1, lon1, lat2, lon2)
            
            # 구간이 길면 잘게 쪼개어 대권상에 점을 배치
            if dist_m > max_segment_m:
                num_steps = int(math.ceil(dist_m / max_segment_m))
                for s in range(1, num_steps):
                    frac = s / num_steps
                    
                    # ECEF Slerp (지구 중심을 통과하는 현의 보간)
                    ix, iy, iz = ecef_interpolate(x1, y1, z1, x2, y2, z2, frac)
                    
                    # 지표면 투영 (고도 0m 보정) -> 대권상의 점이 됨
                    t_lat, t_lon, _ = ecef_to_lla(ix, iy, iz)
                    dx, dy, dz = lla_to_ecef(t_lat, t_lon, 0.0)
                    
                    dense_ecef.append((dx, dy, dz))
        
        # 마지막 제어점 추가
        last_lat, last_lon = lla_points[-1]
        lx, ly, lz = lla_to_ecef(last_lat, last_lon, 0.0)
        dense_ecef.append((lx, ly, lz))
        
        # 3. 누적 거리 계산 (Rail Logic 필수)
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

                # 초기 위치
                start_px, start_py = s.raw_points[0] # [수정] resampled가 아닌 raw_points 사용
                _, _, start_lat, start_lon = pixel_to_coords(start_px, start_py, mi)
                x, y, z = lla_to_ecef(start_lat, start_lon, 0.0)

                # [수정] 제어점(raw_points) 기반으로 경로 생성 -> 대권 항로 보장
                points = s.raw_points 
                ecef_path, cum_dists, total_len = self._densify_path(points, mi)

                # 초기 Heading 설정
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
            points = s.raw_points # [수정] raw_points 사용
            if not points: continue
            
            ecef_path, cum_dists, total_len = self._densify_path(points, mi)
            
            x, y, z = ecef_path[0]
            init_hdg = 0.0
            if len(ecef_path) > 1:
                tx, ty, tz = ecef_path[1]
                init_hdg = ecef_heading(x, y, z, tx, ty, tz)

            self.dynamic_ships[s.idx] = {
                'x': x, 'y': y, 'z': z,
                'spd': s.raw_speeds.get(0, 5.0),
                'sog': 5.0,
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

    def check_events(self, t_current):
        for evt in self.events:
            if evt.id in self.triggered_events: continue
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
                        # CPA Logic with Geodesic consideration
                        # Step A: Estimate T_CPA using vector algebra (valid for finding time)
                        t_lat, t_lon, _ = ecef_to_lla(td['x'], td['y'], td['z'])
                        r_lat, r_lon, _ = ecef_to_lla(rd['x'], rd['y'], rd['z'])
                        
                        tvx, tvy, tvz = self._calculate_ecef_velocity(t_lat, t_lon, td.get('sog', td['spd']), td['hdg'])
                        rvx, rvy, rvz = self._calculate_ecef_velocity(r_lat, r_lon, rd.get('sog', rd['spd']), rd['hdg'])
                        
                        px, py, pz = td['x'] - rd['x'], td['y'] - rd['y'], td['z'] - rd['z']
                        vx, vy, vz = tvx - rvx, tvy - rvy, tvz - rvz
                        
                        v_sq = vx*vx + vy*vy + vz*vz
                        cpa_dist_nm = dist_nm # Default to current
                        
                        if v_sq > 1e-9:
                            t_cpa = -(px*vx + py*vy + pz*vz) / v_sq
                            if t_cpa > 0:
                                # Step B: Simulate position at T_CPA using Great Circle Move (Correcting for Earth curve)
                                t_sog = td.get('sog', td['spd'])
                                r_sog = rd.get('sog', rd['spd'])
                                
                                # Move Target
                                t_move_m = t_sog * 1852.0 * (t_cpa / 3600.0)
                                t_new_lat, t_new_lon, _ = self._move_great_circle_step(t_lat, t_lon, td['hdg'], t_move_m)
                                tx_new, ty_new, tz_new = lla_to_ecef(t_new_lat, t_new_lon, 0.0)
                                
                                # Move Ref
                                r_move_m = r_sog * 1852.0 * (t_cpa / 3600.0)
                                r_new_lat, r_new_lon, _ = self._move_great_circle_step(r_lat, r_lon, rd['hdg'], r_move_m)
                                rx_new, ry_new, rz_new = lla_to_ecef(r_new_lat, r_new_lon, 0.0)
                                
                                # Step C: Measure Distance at predicted positions (Chord -> Arc)
                                future_dist_chord = ecef_distance(tx_new, ty_new, tz_new, rx_new, ry_new, rz_new)
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
        # [수정] 동적 상태 초기화 로직 강화: 이미 존재하면 절대 초기화하지 않음
        # 이로 인해 이벤트나 다른 로직으로 인해 dist_traveled가 0으로 리셋되는 문제 방지
        if ship.idx not in self.dynamic_ships:
            mi = self.proj.map_info
            if ship.raw_points: # raw_points 사용
                px, py = ship.raw_points[0]
                _, _, lat, lon = pixel_to_coords(px, py, mi)
                x, y, z = lla_to_ecef(lat, lon, 0.0)
                # raw_points 기반 대권 경로 생성
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

        # 1. 속도 계산
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

        # 3. 이동 로직: Rail vs Vector
        active_rail = False
        if dyn.get('following_path') and ecef_path:
            if dyn['dist_traveled'] + move_dist_m <= total_len:
                active_rail = True
            else:
                dyn['following_path'] = False 
                active_rail = False

        if active_rail:
            # --- RAIL MODE (대권 추종) ---
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
            
            # [위치 업데이트] 대권 보간 위치
            dyn['x'], dyn['y'], dyn['z'] = ecef_interpolate(x1, y1, z1, x2, y2, z2, frac)
            
            # [핵심 수정: 헤딩 업데이트] "항상 언제나 매 순간 바뀌어야 함"
            # 현재 위치(dyn)에서 다음 미세 경로점(x2, y2, z2)을 향하는 대권 방위각 계산
            dyn['hdg'] = ecef_heading(dyn['x'], dyn['y'], dyn['z'], x2, y2, z2)
            
        else:
            # --- VECTOR MODE (대권 자유 항해) ---
            if move_dist_m > 0:
                lat, lon, _ = ecef_to_lla(dyn['x'], dyn['y'], dyn['z'])
                # 현재 헤딩으로 대권 이동 -> 위치 이동에 따라 헤딩도 자동 변경됨
                nlat, nlon, nhdg = self._move_great_circle_step(lat, lon, dyn['hdg'], move_dist_m)
                
                dyn['x'], dyn['y'], dyn['z'] = lla_to_ecef(nlat, nlon, 0.0)
                dyn['hdg'] = nhdg

        # 4. 결과 반환
        lat, lon, _ = ecef_to_lla(dyn['x'], dyn['y'], dyn['z'])
        
        if lat > 89.9: lat = 89.9
        elif lat < -89.9: lat = -89.9
        lon = normalize_lon(lon)
        
        return self._make_state_result(lat, lon, current_speed_kn, dyn['hdg'])

    def _move_great_circle_step(self, lat, lon, hdg_deg, dist_m):
        """
        현재 위치에서 대권 항해(Great Circle Sailing)로 이동 후 
        새 위치와 해당 지점에서의 방위각(Bearing)을 반환합니다.
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

    def get_jittered_time(self, base_ts):
        if self.proj.settings.jitter_enabled:
            offset = random.choice([-2, -1, 0, 1, 2])
            return base_ts + datetime.timedelta(seconds=offset)
        return base_ts

    def generate_ais_radar_group(self, own, tgt, state, ts_val):
        jitter_ts = self.get_jittered_time(ts_val)
        base = self.make_base_row(own, tgt, state, jitter_ts)
        stype = "AIVDM"
        ship = self.proj.get_ship_by_idx(tgt.idx)
        if ship:
            if ship.signals_enabled.get(stype, True): 
                interval = ship.signal_intervals.get(stype, 1.0)
                current_sim_time = (jitter_ts - self.proj.start_time).total_seconds()
                last_time = self.last_emission_times.get((ship.idx, stype), -float('inf'))
                if (current_sim_time + 1e-9) >= (last_time + interval): 
                    if not self.check_dropout(stype): 
                        ais_msgs = self.gen_ais_multi(base, state, "VDM", tgt.mmsi)
                        for msg in ais_msgs: self.send_nmea(msg, jitter_ts) 
                        self.last_emission_times[(ship.idx, stype)] = current_sim_time 
        self.try_emit(self.gen_ttm(base, state), "RATTM", jitter_ts, tgt.idx)

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