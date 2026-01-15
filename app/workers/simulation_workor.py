import math
import random
import datetime
import traceback
import socket
import time
import numpy as np
import tempfile

from PyQt6.QtCore import QObject, pyqtSignal, Qt, QPointF
from PyQt6.QtGui import QPolygonF

from app.core.models.project import current_project
from app.core.geometry import haversine_distance, pixel_to_coords, coords_to_pixel, normalize_lon, lla_to_ecef, ecef_to_lla
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
        self.m = 0 # New attribute to store current step
        self.current_time = 0.0
        self.cached_times = {} # Cache for numpy arrays of ship times
        self.active_ships = []
        self.triggered_events = set()
        self.dynamic_ships = {} # {ship_idx: {'lat':, 'lon':, 'spd':, 'hdg':, 'last_update_time':}}
        self.events = []
        self.max_steps = 0

    def run(self):
        try:
            self.log_message.emit(f"Starting Simulation UDP->{self.ip}:{self.port} Speed={self.speed_mult}x")
            self.temp_file_handle = tempfile.TemporaryFile(mode='w+', encoding='utf-8')
            
            # Set deterministic seed for simulation run
            seed_val = self.proj.seed % (2**32 - 1)
            random.seed(seed_val)
            np.random.seed(seed_val)
            
            self.active_ships = [s for s in self.proj.ships if s.is_generated and s.cumulative_time]
            if not self.active_ships:
                self.log_message.emit("No active ships generated.")
                self.finished.emit()
                return

            # Pre-cache time arrays as numpy arrays for performance
            self.cached_times = {}
            for s in self.active_ships:
                self.cached_times[s.idx] = np.array(s.cumulative_time)

            self.events = [e for e in self.proj.events if e.enabled]
            self.triggered_events = set()
            self.dynamic_ships = {}

            own_idx = self.proj.settings.own_ship_idx
            own_ship = self.proj.get_ship_by_idx(own_idx)
            
            dT = self.proj.unit_time
            self.max_steps = math.ceil(self.duration_sec / dT)
            
            self.last_sps_update_time = time.time() # Initialize here for accurate first calculation
            self.last_sps_update_m = 0             # Initialize here for accurate first calculation

            mi = self.proj.map_info

            m = 0
            while m <= self.max_steps:
                if not self.running: break
                self.m = m
                
                if self.paused:
                      time.sleep(0.1)
                      continue

                # Refresh events list to allow dynamic updates during simulation
                try:
                    # 1. Project Events
                    active_events = [e for e in self.proj.events if e.enabled]
                    
                    # 2. Scenario Events
                    from app.core.state import loaded_scenarios
                    for scen in loaded_scenarios:
                        if not scen.enabled: continue
                        
                        for evt in scen.events:
                            if not evt.enabled: continue
                            
                            # Scope Check
                            should_apply = False
                            if scen.scope_mode == "ALL_SHIPS":
                                should_apply = True
                            elif scen.scope_mode == "OWN_ONLY":
                                if evt.target_ship_idx == self.proj.settings.own_ship_idx:
                                    should_apply = True
                            elif scen.scope_mode == "TARGET_ONLY":
                                if evt.target_ship_idx != self.proj.settings.own_ship_idx:
                                    should_apply = True
                            elif scen.scope_mode == "SELECTED_SHIPS":
                                if evt.target_ship_idx in scen.selected_ships:
                                    should_apply = True
                                    
                            if should_apply:
                                # Dedup: if event with same ID already in active_events, skip
                                if not any(e.id == evt.id for e in active_events):
                                    active_events.append(evt)
                                    
                    self.events = active_events
                except:
                    pass # Handle potential list modification during iteration

                loop_start = time.time()
                t_m = m * dT
                self.current_time = t_m
                ts_val = self.proj.start_time + datetime.timedelta(seconds=t_m)
                
                self.check_events(t_m)
                  
                current_positions = {}
                
                own_state = None
                if own_ship and own_ship.is_generated:
                      own_state = self.get_state_at_step(own_ship, m)
                      px, py = coords_to_pixel(own_state['lat_deg'], own_state['lon_deg'], mi)
                      current_positions[own_idx] = (px, py, own_state['heading_true_deg'])

                for tgt in self.active_ships:
                    if tgt.idx == own_idx: continue 
                    tgt_state = self.get_state_at_step(tgt, m)
                    px, py = coords_to_pixel(tgt_state['lat_deg'], tgt_state['lon_deg'], mi)
                    current_positions[tgt.idx] = (px, py, tgt_state['heading_true_deg'])
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
                
                # --- Modified SPS Calculation and Emission Logic ---
                # Emit more frequently at the beginning, then less frequently
                sps_update_interval = 10 if m <= 200 else 100 # Emit every 10 steps for first 200, then every 100

                if (m - self.last_sps_update_m) >= sps_update_interval:
                    current_time = time.time()
                    time_since_last_sps_update = current_time - self.last_sps_update_time
                    steps_since_last_sps_update = m - self.last_sps_update_m
                    
                    if time_since_last_sps_update > 0:
                        sps = steps_since_last_sps_update / time_since_last_sps_update
                    else:
                        sps = 0.0 # Avoid division by zero
                        
                    log_msg = f"SPS: {sps:.1f} (Elapsed: {current_loop_elapsed:.4f}s, Sleep: {max(0, sleep_time):.4f}s)"
                    self.performance_updated.emit(log_msg)
                    
                    self.last_sps_update_time = current_time
                    self.last_sps_update_m = m
                # --- End Modified SPS Logic ---

                if sleep_time > 0:
                    time.sleep(sleep_time)

            self.log_message.emit("Simulation Finished.")
            self.export_data_ready.emit(True)
              
        except Exception as e:
            traceback.print_exc()
            self.log_message.emit(f"Sim Error: {e}")
        finally:
            self.sock.close()
            self.finished.emit()
            
    def refresh_active_ships(self):
        self.active_ships = [s for s in self.proj.ships if s.is_generated and s.cumulative_time]
        for s in self.active_ships:
            if s.idx not in self.cached_times:
                self.cached_times[s.idx] = np.array(s.cumulative_time)

    def send_nmea(self, raw, ts_obj):

        if self.temp_file_handle:
            try:
                line = f"{ts_obj.timestamp()},{raw}\n"
                self.temp_file_handle.write(line)
            except: pass
            

        try:
            self.sock.sendto((raw + "\r\n").encode('ascii'), (self.ip, self.port))
        except:
            pass


        current_real_time = time.time()
        if current_real_time - self.last_log_update_time > 0.2:
            self.signal_generated.emit(raw)
            self.last_log_update_time = current_real_time

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

    def check_events(self, t_current):
        for evt in self.events:
            if evt.id in self.triggered_events: continue
            
            triggered = False
            log_detail = ""
            if evt.trigger_type == "TIME":
                if t_current >= evt.condition_value:
                    triggered = True
            elif evt.trigger_type == "AREA_ENTER":
                # Check if target ship is in area
                ship = self.proj.get_ship_by_idx(evt.target_ship_idx)
                area = self.proj.get_area_by_id(int(evt.condition_value))
                if ship and area and ship.is_generated:
                    # Get current position (either dynamic or static)
                    state = self.get_state_at_step(ship, self.m) # This handles dynamic check internally
                    px, py = coords_to_pixel(state['lat_deg'], state['lon_deg'], self.proj.map_info)
                    poly = QPolygonF([QPointF(x,y) for x,y in area.geometry])
                    if poly.containsPoint(QPointF(px, py), Qt.FillRule.OddEvenFill):
                        triggered = True
            elif evt.trigger_type == "AREA_LEAVE":
                # Check if target ship is OUTSIDE area
                ship = self.proj.get_ship_by_idx(evt.target_ship_idx)
                area = self.proj.get_area_by_id(int(evt.condition_value))
                if ship and area and ship.is_generated:
                    state = self.get_state_at_step(ship, self.m)
                    px, py = coords_to_pixel(state['lat_deg'], state['lon_deg'], self.proj.map_info)
                    poly = QPolygonF([QPointF(x,y) for x,y in area.geometry])
                    if not poly.containsPoint(QPointF(px, py), Qt.FillRule.OddEvenFill):
                        triggered = True
            elif evt.trigger_type in ["CPA_UNDER", "CPA_OVER", "DIST_UNDER", "DIST_OVER"]:
                # Check distance between target ship and reference ship (Instantaneous Range)
                target_ship = self.proj.get_ship_by_idx(evt.target_ship_idx)
                
                ref_idx = evt.reference_ship_idx
                if ref_idx == -1: # Fallback for CPA if not set
                    ref_idx = self.proj.settings.own_ship_idx
                
                ref_ship = self.proj.get_ship_by_idx(ref_idx)
                
                if target_ship and ref_ship and target_ship.is_generated and ref_ship.is_generated:
                    tgt_state = self.get_state_at_step(target_ship, self.m)
                    ref_state = self.get_state_at_step(ref_ship, self.m)
                    
                    dist_m = haversine_distance(tgt_state['lat_deg'], tgt_state['lon_deg'], ref_state['lat_deg'], ref_state['lon_deg'])
                    
                    # CPA types use NM, DIST types use Nautical Miles (nm)
                    if evt.trigger_type in ["CPA_UNDER", "CPA_OVER"]:
                        val_compare = dist_m / 1852.0
                        unit_str = "NM"
                    else:
                        val_compare = dist_m / 1852.0
                        unit_str = "nm"
                    
                    if evt.trigger_type in ["CPA_UNDER", "DIST_UNDER"] and val_compare <= evt.condition_value:
                        triggered = True
                    elif evt.trigger_type in ["CPA_OVER", "DIST_OVER"] and val_compare >= evt.condition_value:
                        triggered = True
                        
                    if triggered:
                        ts_str = (self.proj.start_time + datetime.timedelta(seconds=t_current)).strftime("%Y-%m-%dT%H:%M:%SZ")
                        op_str = "<=" if "UNDER" in evt.trigger_type else ">="
                        log_detail = f"ref={ref_ship.name}, tgt={target_ship.name}, dist={val_compare:.1f}{unit_str} {op_str} {evt.condition_value}{unit_str} @ {ts_str}"
            
            if triggered:
                if log_detail:
                    self.log_message.emit(f"[EVENT] {evt.name} fired: {log_detail}")
                else:
                    self.log_message.emit(f"Event Triggered: {evt.name} ({evt.action_type})")
                    
                self.event_triggered.emit(evt.name, evt.target_ship_idx)
                self.triggered_events.add(evt.id)
                self.apply_event_action(evt, t_current)

    def apply_event_action(self, evt, t_current):
        sid = evt.target_ship_idx
        ship = self.proj.get_ship_by_idx(sid)
        if not ship: return
        
        # Initialize dynamic state if not already
        if sid not in self.dynamic_ships:
            # Get current state from static path
            state = self.get_state_at_step(ship, self.m)
            
            # Calculate current path index for continuity
            idx = 0
            times = self.cached_times.get(ship.idx)
            if times is not None:
                 idx = np.searchsorted(times, t_current, side='right') - 1
                 if idx < 0: idx = 0

            self.dynamic_ships[sid] = {
                'lat': state['lat_deg'],
                'lon': state['lon_deg'],
                'spd': state['sog_knots'],
                'hdg': state['heading_true_deg'],
                'last_update_time': t_current,
                'path_idx': idx,
                'following_path': True # Default to following path
            }
            
        dyn = self.dynamic_ships[sid]
        
        if evt.action_type == "STOP":
            dyn['spd'] = 0.0
        elif evt.action_type == "CHANGE_SPEED":
            dyn['spd'] = evt.action_value
            # Re-enable path following if we weren't manually steering
            if not dyn.get('manual_heading', False):
                dyn['following_path'] = True
        elif evt.action_type == "CHANGE_HEADING":
            dyn['hdg'] = evt.action_value
            dyn['manual_heading'] = True
            dyn['following_path'] = False
        elif evt.action_type == "MANEUVER":
            opt = getattr(evt, 'action_option', "")
            if opt == "ReturnToOriginalPath_ShortestDistance":
                dyn['mode'] = 'RETURN_TO_PATH'
            elif opt == "ChangeDestination_ToOriginalFinal":
                if ship.resampled_points:
                    mi = self.proj.map_info
                    # Last point
                    end_px, end_py = ship.resampled_points[-1]
                    _, _, target_lat, target_lon = pixel_to_coords(end_px, end_py, mi)
                    
                    # Calculate heading to target
                    d_lat = target_lat - dyn['lat']
                    mid_lat = (dyn['lat'] + target_lat) / 2.0
                    d_lon = (target_lon - dyn['lon']) * math.cos(math.radians(mid_lat))
                    
                    if abs(d_lat) > 1e-7 or abs(d_lon) > 1e-7:
                        hdg = math.degrees(math.atan2(d_lon, d_lat))
                        dyn['hdg'] = (hdg + 360) % 360


    def get_state_at_step(self, ship, m):
        t_current = m * self.proj.unit_time
        mi = self.proj.map_info
        
        # Check if ship is in dynamic mode (overridden by event)
        if ship.idx in self.dynamic_ships:
            dyn = self.dynamic_ships[ship.idx]
            dt = t_current - dyn['last_update_time']
            
            # --- Guidance Logic for Maneuvers ---
            if dyn.get('mode') == 'RETURN_TO_PATH':
                c_lat, c_lon = dyn['lat'], dyn['lon']
                c_px, c_py = coords_to_pixel(c_lat, c_lon, mi)
                
                best_idx = -1
                best_dist_sq = float('inf')
                
                for i, (px, py) in enumerate(ship.resampled_points):
                    d2 = (px - c_px)**2 + (py - c_py)**2
                    if d2 < best_dist_sq:
                        best_dist_sq = d2
                        best_idx = i
                
                if best_idx != -1:
                    t_px, t_py = ship.resampled_points[best_idx]
                    _, _, t_lat, t_lon = pixel_to_coords(t_px, t_py, mi)
                    dist_m = haversine_distance(c_lat, c_lon, t_lat, t_lon)
                    
                    if dist_m < 30: # 30m threshold to snap to path following
                        dyn['mode'] = 'FOLLOW_PATH_GEOMETRY'
                        dyn['current_path_idx'] = best_idx
                    else:
                        # Steer to closest point (shortest distance)
                        target_idx = min(len(ship.resampled_points)-1, best_idx + 1)
                        t_px, t_py = ship.resampled_points[target_idx]
                        _, _, target_lat, target_lon = pixel_to_coords(t_px, t_py, mi)
                        
                        d_lat = target_lat - c_lat
                        mid_lat = (c_lat + target_lat) / 2.0
                        d_lon = (target_lon - c_lon) * math.cos(math.radians(mid_lat))
                        
                        if abs(d_lat) > 1e-9 or abs(d_lon) > 1e-9:
                            req_hdg = math.degrees(math.atan2(d_lon, d_lat))
                            dyn['hdg'] = (req_hdg + 360) % 360

            elif dyn.get('mode') == 'FOLLOW_PATH_GEOMETRY':
                idx = dyn.get('current_path_idx', 0)
                c_lat, c_lon = dyn['lat'], dyn['lon']
                c_px, c_py = coords_to_pixel(c_lat, c_lon, mi)
                
                best_idx = idx
                min_d = float('inf')
                search_limit = min(len(ship.resampled_points), idx + 200)
                
                for i in range(idx, search_limit):
                    px, py = ship.resampled_points[i]
                    d = (px - c_px)**2 + (py - c_py)**2
                    if d < min_d:
                        min_d = d
                        best_idx = i
                
                dyn['current_path_idx'] = best_idx
                target_idx = min(len(ship.resampled_points)-1, best_idx + 5)
                
                if target_idx > best_idx:
                    t_px, t_py = ship.resampled_points[target_idx]
                    _, _, target_lat, target_lon = pixel_to_coords(t_px, t_py, mi)
                    
                    d_lat = target_lat - c_lat
                    mid_lat = (c_lat + target_lat) / 2.0
                    d_lon = (target_lon - c_lon) * math.cos(math.radians(mid_lat))
                    
                    if abs(d_lat) > 1e-9 or abs(d_lon) > 1e-9:
                         req_hdg = math.degrees(math.atan2(d_lon, d_lat))
                         dyn['hdg'] = (req_hdg + 360) % 360
                elif best_idx == len(ship.resampled_points)-1:
                    dyn['spd'] = 0.0

            # --- Movement Logic ---
            if dyn.get('following_path') and ship.resampled_points:
                # Move along the pre-defined path at the new speed
                dist_remain_m = dyn['spd'] * (dt / 3600.0) * 1852.0
                
                current_idx = dyn.get('path_idx', 0)
                if current_idx >= len(ship.resampled_points) - 1:
                     current_idx = len(ship.resampled_points) - 2
                
                while dist_remain_m > 0:
                    if current_idx >= len(ship.resampled_points) - 1:
                        break
                    
                    p_next = ship.resampled_points[current_idx + 1]
                    _, _, lat_next, lon_next = pixel_to_coords(p_next[0], p_next[1], mi)
                    
                    d_seg = haversine_distance(dyn['lat'], dyn['lon'], lat_next, lon_next)
                    
                    if d_seg > dist_remain_m:
                        # Move intermediate
                        frac = dist_remain_m / d_seg
                        dyn['lat'] += (lat_next - dyn['lat']) * frac
                        
                        d_lon = lon_next - dyn['lon']
                        if d_lon > 180: d_lon -= 360
                        elif d_lon < -180: d_lon += 360
                        
                        dyn['lon'] = normalize_lon(dyn['lon'] + d_lon * frac)
                        
                        # Update heading to face next point
                        d_lat_d = lat_next - dyn['lat']
                        mid_lat = (dyn['lat'] + lat_next) / 2.0
                        d_lon_d = (lon_next - dyn['lon'])
                        if d_lon_d > 180: d_lon_d -= 360
                        elif d_lon_d < -180: d_lon_d += 360
                        d_lon_d = d_lon_d * math.cos(math.radians(mid_lat))
                        
                        if abs(d_lat_d) > 1e-9 or abs(d_lon_d) > 1e-9:
                            dyn['hdg'] = (math.degrees(math.atan2(d_lon_d, d_lat_d)) + 360) % 360
                        
                        dist_remain_m = 0
                    else:
                        # Reached waypoint
                        dyn['lat'] = lat_next
                        dyn['lon'] = lon_next
                        dist_remain_m -= d_seg
                        current_idx += 1
                        dyn['path_idx'] = current_idx
                
                dyn['last_update_time'] = t_current

            elif dt > 0:
                # Update position based on current speed/heading
                # 1 knot = 1 nm/h = 1.852 km/h = 0.5144 m/s
                # approx 1 nm = 1/60 deg lat
                dist_nm = dyn['spd'] * (dt / 3600.0)
                dist_deg = dist_nm / 60.0
                
                d_lat = dist_deg * math.cos(math.radians(dyn['hdg']))
                
                mid_lat = dyn['lat'] + d_lat/2
                cos_mid = math.cos(math.radians(mid_lat))
                if abs(cos_mid) < 0.01: cos_mid = 0.01
                d_lon = dist_deg * math.sin(math.radians(dyn['hdg'])) / cos_mid
                
                dyn['lat'] += d_lat
                dyn['lon'] += d_lon
                dyn['last_update_time'] = t_current
                
                # Clamp lat
                if dyn['lat'] > 90: dyn['lat'] = 90
                elif dyn['lat'] < -90: dyn['lat'] = -90
                
                dyn['lon'] = normalize_lon(dyn['lon'])

            sog = dyn['spd']
            cog = dyn['hdg']
            sog_kmh = sog * 1.852
            sog_mps = sog * 0.51444444
            
            return {
                "lat_deg": dyn['lat'],
                "lon_deg": dyn['lon'],
                "sog_knots": sog,
                "sog_kmh": sog_kmh,
                "sog_mps": sog_mps,
                "cog_true_deg": cog,
                "heading_true_deg": cog,
                "elapsed_sec": t_current
            }

        # Use cached numpy array if available, otherwise fallback (and cache)
        times = self.cached_times.get(ship.idx)
        if times is None:
             if not ship.cumulative_time:
                 return {
                    "lat_deg": 0.0, "lon_deg": 0.0, "sog_knots": 0.0, "sog_kmh": 0.0, "sog_mps": 0.0,
                    "cog_true_deg": 0.0, "heading_true_deg": 0.0, "elapsed_sec": t_current
                 }
             times = np.array(ship.cumulative_time)
             self.cached_times[ship.idx] = times
             
        # Clamp t_current for interpolation to the actual path duration
        # This ensures that if simulation runs longer than path, ship stays at last point
        t_interp = min(t_current, times[-1])
        
        idx = np.searchsorted(times, t_interp, side='right') - 1
        if idx < 0: idx = 0
        if idx >= len(times) - 1: idx = len(times) - 2
        
        t_start = times[idx]
        t_end = times[idx+1]
        
        if t_current < t_start:
            alpha = 0.0
        elif t_end > t_start:
            alpha = (t_interp - t_start) / (t_end - t_start)
        else:
            alpha = 0.0
            
        p1 = ship.resampled_points[idx]
        p2 = ship.resampled_points[idx+1]
        
        _, _, lat1, lon1 = pixel_to_coords(p1[0], p1[1], mi)
        _, _, lat2, lon2 = pixel_to_coords(p2[0], p2[1], mi)
        
        lat = lat1 + (lat2 - lat1) * alpha
        d_lon = lon2 - lon1
        if d_lon > 180: d_lon -= 360
        elif d_lon < -180: d_lon += 360
        lon = normalize_lon(lon1 + d_lon * alpha)
        
        v1 = ship.node_velocities_kn[idx]
        v2 = ship.node_velocities_kn[idx+1]
        sog = v1 + (v2 - v1) * alpha
        
        if t_current < t_start or t_current > times[-1]:
            sog = 0.0
        
        sog_kmh = sog * 1.852
        sog_mps = sog * 0.51444444
        
        dy = lat2 - lat1
        dx = (lon2 - lon1) * math.cos(math.radians((lat1+lat2)/2))
        
        if abs(dx) > 1e-9 or abs(dy) > 1e-9:
             cog = math.degrees(math.atan2(dx, dy))
             cog = (cog + 360) % 360
        else:
             cog = 0.0

        return {
            "lat_deg": lat,
            "lon_deg": lon,
            "sog_knots": sog,
            "sog_kmh": sog_kmh,
            "sog_mps": sog_mps,
            "cog_true_deg": cog,
            "heading_true_deg": cog, 
            "elapsed_sec": t_current
        }

    def send_nmea(self, raw, ts_obj):
        self.signal_generated.emit(raw)
        if self.temp_file_handle:
            try:
                line = f"{ts_obj.timestamp()},{raw}\n"
                self.temp_file_handle.write(line)
            except: pass
            
        try:
            self.sock.sendto((raw + "\r\n").encode('ascii'), (self.ip, self.port))
        except:
            pass

    def check_dropout(self, stype):
        prob = self.proj.settings.dropout_probs.get(stype, 0.1)
        return random.random() < prob

    def get_jittered_time(self, base_ts):
        if self.proj.settings.jitter_enabled:
            offset = random.choice([-2, -1, 0, 1, 2])
            return base_ts + datetime.timedelta(seconds=offset)
        return base_ts

    def add_position_noise(self, lat, lon, ship_idx, talker):
        ship = self.proj.get_ship_by_idx(ship_idx)
        if not ship:
            return lat, lon

        variance = ship.variances.get(talker, 0.0)
        if variance <= 0:
            return lat, lon

        std_dev = np.sqrt(variance)
        
        x, y, z = lla_to_ecef(lat, lon, 0)
        
        noise_x = np.random.normal(0, std_dev)
        noise_y = np.random.normal(0, std_dev)
        noise_z = np.random.normal(0, std_dev)
        
        noisy_x = x + noise_x
        noisy_y = y + noise_y
        noisy_z = z + noise_z
        
        noisy_lat, noisy_lon, _ = ecef_to_lla(noisy_x, noisy_y, noisy_z)
        
        return noisy_lat, noisy_lon

    def generate_ais_radar_group(self, own, tgt, state, ts_val):
        jitter_ts = self.get_jittered_time(ts_val)
        base = self.make_base_row(own, tgt, state, jitter_ts)
        
        # --- AIS Message Group Handling ---
        stype = "AIVDM"
        ship = self.proj.get_ship_by_idx(tgt.idx)
        if ship:
            if ship.signals_enabled.get(stype, True): # Check if AIVDM is enabled
                interval = ship.signal_intervals.get(stype, 1.0)
                current_sim_time = (jitter_ts - self.proj.start_time).total_seconds()
                last_time = self.last_emission_times.get((ship.idx, stype), -float('inf'))

                if (current_sim_time + 1e-9) >= (last_time + interval): # It's time to send
                    if not self.check_dropout(stype): # Check dropout for the whole group
                        ais_msgs = self.gen_ais_multi(base, state, "VDM", tgt.mmsi)
                        for msg in ais_msgs:
                            self.send_nmea(msg, jitter_ts) # Send each fragment directly
                        self.last_emission_times[(ship.idx, stype)] = current_sim_time # Update time AFTER all fragments sent
        # --- End AIS Message Group Handling ---

        self.try_emit(self.gen_ttm(base, state), "RATTM", jitter_ts, tgt.idx)

    def try_emit(self, raw, stype, ts_obj, ship_idx):
        ship = self.proj.get_ship_by_idx(ship_idx)
        if not ship: return

        if not ship.signals_enabled.get(stype, True):
            return

        interval = ship.signal_intervals.get(stype, 1.0)
        current_sim_time = (ts_obj - self.proj.start_time).total_seconds()
        
        last_time = self.last_emission_times.get((ship_idx, stype), -float('inf'))
        
        # Add a small epsilon to handle floating point inaccuracies
        if (current_sim_time + 1e-9) < (last_time + interval):
            return

        if not raw: return
        
        self.last_emission_times[(ship_idx, stype)] = current_sim_time
        
        if self.check_dropout(stype):
            return 
            
        self.send_nmea(raw, ts_obj)

    def make_base_row(self, rcv, tgt, state, ts_val):
        return {
            "tgt_ship_index": tgt.idx,
            "tgt_ship_name": tgt.name,
            "rx_time": ts_val
        }

    def gen_ais_multi(self, base, state, stype, mmsi):
        # Determine number of fragments based on probabilities
        probs = current_project.settings.ais_fragment_probs
        # Normalize probabilities if they don't sum to 1
        total_prob = sum(probs)
        # Avoid division by zero if all probabilities are zero
        if total_prob == 0:
            total_fragments = 1 # Default to 1 fragment if no probabilities are set
        else:
            normalized_probs = [p / total_prob for p in probs]
            # Choose total_fragments (1-5) based on normalized probabilities
            # The `choices` function returns a list, so we take the first element
            total_fragments = random.choices(range(1, 6), weights=normalized_probs, k=1)[0]
        
        payload = encode_ais_payload(mmsi) # Pass MMSI to the encoder
        
        # Calculate fragment length. Ensure it's at least 1 to avoid empty fragments if payload is short.
        # AIS messages have a maximum payload size. For simplicity, we'll split evenly.
        fragment_min_len = 1 # Minimum fragment length
        fragment_length = max(fragment_min_len, math.ceil(len(payload) / total_fragments))
        
        ais_messages = []
        seq_id = random.randint(0, 9) # Sequence ID for multi-part messages
        
        for i in range(total_fragments):
            start_idx = i * fragment_length
            end_idx = min(len(payload), (i + 1) * fragment_length) # Ensure we don't go out of bounds
            
            fragment = payload[start_idx : end_idx]
            
            # Construct the AIS message part
            # !AI<stype>,<total_fragments>,<fragment_number>,<sequence_id>,<channel>,<fragment_payload>,<fill_bits>
            # channel 'A' or 'B' (hardcoded for now to 'A')
            # fill_bits '0' (hardcoded for now)
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
