import math
import random
import datetime
import traceback
import socket
import time
import numpy as np
import tempfile

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
        self.m = 0 # New attribute to store current step
        self.current_time = 0.0
        # self.cached_times = {} # Removed: No pre-calculated times
        self.active_ships = []
        self.triggered_events = set()
        self.dynamic_ships = {} # {ship_idx: {'lat':, 'lon':, 'spd':, 'hdg':, 'last_update_time':}}
        self.events = []
        self.max_steps = 0

    def _densify_path(self, raw_points, mi, max_segment_m=100.0):
        """
        Resamples the path into dense ECEF points to approximate Great Circle movement.
        max_segment_m: Maximum distance between points in meters (default 100m)
        """
        if not raw_points or len(raw_points) < 2:
            return path_points_to_ecef(raw_points, mi)

        dense_ecef = []
        
        # Convert raw pixels to LLA first
        lla_points = []
        for px, py in raw_points:
            _, _, lat, lon = pixel_to_coords(px, py, mi)
            lla_points.append((lat, lon))

        # Iterate segments and densify
        for i in range(len(lla_points) - 1):
            lat1, lon1 = lla_points[i]
            lat2, lon2 = lla_points[i+1]
            
            # Start point ECEF
            x1, y1, z1 = lla_to_ecef(lat1, lon1, 0.0)
            dense_ecef.append((x1, y1, z1))
            
            # Calculate distance
            dist_m = haversine_distance(lat1, lon1, lat2, lon2)
            
            if dist_m > max_segment_m:
                num_steps = int(math.ceil(dist_m / max_segment_m))
                
                for s in range(1, num_steps):
                    frac = s / num_steps
                    # Simple linear interpolation of lat/lon avoids the "underground chord" issue
                    # of ECEF interpolation over long distances.
                    curr_lat = lat1 + (lat2 - lat1) * frac
                    curr_lon = lon1 + (lon2 - lon1) * frac
                    
                    dx, dy, dz = lla_to_ecef(curr_lat, curr_lon, 0.0)
                    dense_ecef.append((dx, dy, dz))
        
        # Add final point
        last_lat, last_lon = lla_points[-1]
        lx, ly, lz = lla_to_ecef(last_lat, last_lon, 0.0)
        dense_ecef.append((lx, ly, lz))
        
        return dense_ecef

    def run(self):
        try:
            self.log_message.emit(f"Starting Simulation UDP->{self.ip}:{self.port} Speed={self.speed_mult}x")
            self.temp_file_handle = tempfile.TemporaryFile(mode='w+', encoding='utf-8')
            
            # Set deterministic seed for simulation run
            seed_val = self.proj.seed % (2**32 - 1)
            random.seed(seed_val)
            np.random.seed(seed_val)
            
            self.active_ships = [s for s in self.proj.ships if s.raw_points] # All ships with paths
            if not self.active_ships:
                self.log_message.emit("No active ships with paths.")
                self.finished.emit()
                return

            # Initialize dynamic state for ALL ships using ECEF coordinates internally
            self.dynamic_ships = {}
            mi = self.proj.map_info

            for s in self.active_ships:
                # Initialize RNG for this ship
                s_seed = (self.proj.seed + s.idx) % (2**32 - 1)
                s_rng = random.Random(s_seed)

                # Use resampled points for geometry
                start_px, start_py = s.resampled_points[0] if s.resampled_points else s.raw_points[0]
                _, _, start_lat, start_lon = pixel_to_coords(start_px, start_py, mi)

                # Convert to ECEF for internal storage
                x, y, z = lla_to_ecef(start_lat, start_lon, 0.0)

                # Pre-compute ECEF path for this ship
                points = s.resampled_points if s.resampled_points else s.raw_points
                # Use densified path to prevent "underground" movement and improve high-speed accuracy
                ecef_path = self._densify_path(points, mi)

                self.dynamic_ships[s.idx] = {
                    'x': x, 'y': y, 'z': z,  # ECEF coordinates (meters)
                    'spd': s.raw_speeds.get(0, 5.0),  # Base target speed (knots)
                    'sog': s.raw_speeds.get(0, 5.0),  # Actual noisy speed (knots)
                    'hdg': 0.0,  # Heading (degrees)
                    'path_idx': 0,
                    'dist_from_last_point': 0.0,
                    'total_dist_traveled': 0.0,
                    'following_path': True,
                    'manual_speed': False,
                    'manual_heading': False,
                    'rng': s_rng,
                    'ecef_path': ecef_path  # Pre-computed ECEF path
                }
            
            self.events = [e for e in self.proj.events if e.enabled]
            self.triggered_events = set()
            # Note: dynamic_ships was already initialized above, do not reset here

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
                
                QCoreApplication.processEvents()
                
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
                except Exception as e:
                    # Handle potential list modification during iteration
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

                # --- Time Control (consistent pacing) ---
                # Calculate required delay based on speed multiplier
                step_delay = dT / self.speed_mult
                current_loop_elapsed = time.time() - loop_start
                sleep_time = step_delay - current_loop_elapsed

                # --- SPS Calculation ---
                sps_update_interval = 10 if m <= 200 else 100

                if (m - self.last_sps_update_m) >= sps_update_interval:
                    current_time = time.time()
                    time_since_last_sps_update = current_time - self.last_sps_update_time
                    steps_since_last_sps_update = m - self.last_sps_update_m

                    if time_since_last_sps_update > 0:
                        sps = steps_since_last_sps_update / time_since_last_sps_update
                    else:
                        sps = 0.0

                    log_msg = f"SPS: {sps:.1f} (Elapsed: {current_loop_elapsed:.4f}s, Sleep: {max(0, sleep_time):.4f}s)"
                    self.performance_updated.emit(log_msg)

                    self.last_sps_update_time = current_time
                    self.last_sps_update_m = m

                # Always respect timing - use minimum sleep to prevent runaway speed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    # Even if behind schedule, give minimal time for Qt events and prevent 100% CPU
                    time.sleep(0.001)

            # Force final update to ensure end state is visible
            current_positions = {}
            if own_ship and own_ship.raw_points and own_idx in self.dynamic_ships:
                  dyn = self.dynamic_ships[own_idx]
                  lat, lon, _ = ecef_to_lla(dyn['x'], dyn['y'], dyn['z'])
                  px, py = coords_to_pixel(lat, lon, mi)
                  current_positions[own_idx] = (px, py, dyn['hdg'], dyn.get('sog', dyn['spd']))

            for tgt in self.active_ships:
                if tgt.idx == own_idx: continue
                if tgt.idx in self.dynamic_ships:
                    dyn = self.dynamic_ships[tgt.idx]
                    lat, lon, _ = ecef_to_lla(dyn['x'], dyn['y'], dyn['z'])
                    px, py = coords_to_pixel(lat, lon, mi)
                    current_positions[tgt.idx] = (px, py, dyn['hdg'], dyn.get('sog', dyn['spd']))
            self.positions_updated.emit(current_positions)

            self.log_message.emit("Simulation Finished.")
            self.export_data_ready.emit(True)
              
        except Exception as e:
            traceback.print_exc()
            self.log_message.emit(f"Sim Error: {e}")
        finally:
            self.sock.close()
            self.finished.emit()
            
    def refresh_active_ships(self):
        self.active_ships = [s for s in self.proj.ships if s.raw_points] # Refresh list

    def send_nmea(self, raw, ts_obj):
        if self.temp_file_handle:
            try:
                line = f"{ts_obj.timestamp()},{raw}\n"
                self.temp_file_handle.write(line)
            except Exception as e:
                self.log_message.emit(f"[Warning] Failed to write to temp file: {e}")

        try:
            self.sock.sendto((raw + "\r\n").encode('ascii'), (self.ip, self.port))
        except socket.error as e:
            self.log_message.emit(f"[Warning] UDP send failed ({self.ip}:{self.port}): {e}")
        except Exception as e:
            self.log_message.emit(f"[Warning] Unexpected error in send_nmea: {e}")

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

    def set_ship_speed(self, ship_idx, speed_kn):
        ship = self.proj.get_ship_by_idx(ship_idx)
        if not ship: return

        # Ensure ship is in dynamic_ships (should be initialized in run)
        if ship_idx not in self.dynamic_ships:
            # Should not happen if refresh_active_ships is correct, but safe fallback?
            return
        self.dynamic_ships[ship_idx]['manual_speed'] = True
        self.dynamic_ships[ship_idx]['spd'] = speed_kn
        self.log_message.emit(f"[Manual] Speed changed for {ship.name} to {speed_kn} kn")

    def set_ship_heading(self, ship_idx, heading_deg):
        ship = self.proj.get_ship_by_idx(ship_idx)
        if not ship: return

        if ship_idx not in self.dynamic_ships:
            return
        
        self.dynamic_ships[ship_idx]['hdg'] = heading_deg
        self.dynamic_ships[ship_idx]['following_path'] = False
        self.log_message.emit(f"[Manual] Heading changed for {ship.name} to {heading_deg} deg")

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
                    state = self.get_current_state(ship)
                    px, py = coords_to_pixel(state['lat_deg'], state['lon_deg'], self.proj.map_info)
                    poly = QPolygonF([QPointF(x,y) for x,y in area.geometry])
                    if poly.containsPoint(QPointF(px, py), Qt.FillRule.OddEvenFill):
                        triggered = True
            elif evt.trigger_type == "AREA_LEAVE":
                # Check if target ship is OUTSIDE area
                ship = self.proj.get_ship_by_idx(evt.target_ship_idx)
                area = self.proj.get_area_by_id(int(evt.condition_value))
                if ship and area:
                    state = self.get_current_state(ship)
                    px, py = coords_to_pixel(state['lat_deg'], state['lon_deg'], self.proj.map_info)
                    poly = QPolygonF([QPointF(x,y) for x,y in area.geometry])
                    if not poly.containsPoint(QPointF(px, py), Qt.FillRule.OddEvenFill):
                        triggered = True
            elif evt.trigger_type in ["CPA_UNDER", "CPA_OVER", "DIST_UNDER", "DIST_OVER"]:
                # Check distance between target ship and reference ship using ECEF
                target_ship = self.proj.get_ship_by_idx(evt.target_ship_idx)

                ref_idx = evt.reference_ship_idx
                if ref_idx == -1:
                    ref_idx = self.proj.settings.own_ship_idx

                ref_ship = self.proj.get_ship_by_idx(ref_idx)

                if target_ship and ref_ship:
                    # Use ECEF distance calculation directly
                    tgt_dyn = self.dynamic_ships.get(target_ship.idx)
                    ref_dyn = self.dynamic_ships.get(ref_ship.idx)

                    if tgt_dyn and ref_dyn:
                        dist_m = ecef_distance(
                            tgt_dyn['x'], tgt_dyn['y'], tgt_dyn['z'],
                            ref_dyn['x'], ref_dyn['y'], ref_dyn['z']
                        )

                        # Convert to nautical miles
                        val_compare = dist_m / 1852.0
                        unit_str = "NM" if evt.trigger_type in ["CPA_UNDER", "CPA_OVER"] else "nm"

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
        
        if sid not in self.dynamic_ships:
            return
            
        dyn = self.dynamic_ships[sid]
        
        if evt.action_type == "STOP":
            dyn['spd'] = 0.0
            dyn['manual_speed'] = True
        elif evt.action_type == "CHANGE_SPEED":
            dyn['spd'] = evt.action_value
            # Re-enable path following if we weren't manually steering
            if not dyn.get('manual_heading', False):
                dyn['following_path'] = True
            dyn['manual_speed'] = True
        elif evt.action_type == "CHANGE_HEADING":
            dyn['hdg'] = evt.action_value
            dyn['manual_heading'] = True
            dyn['following_path'] = False
        elif evt.action_type == "MANEUVER":
            opt = getattr(evt, 'action_option', "")
            if opt == "ReturnToOriginalPath_ShortestDistance":
                dyn['mode'] = 'RETURN_TO_PATH'
            elif opt == "ChangeDestination_ToOriginalFinal":
                ecef_path = dyn.get('ecef_path', [])
                if ecef_path:
                    # Get last point in ECEF path
                    tx, ty, tz = ecef_path[-1]
                    # Calculate heading using ECEF
                    dyn['hdg'] = ecef_heading(dyn['x'], dyn['y'], dyn['z'], tx, ty, tz)

    def get_current_state(self, ship):
        """Helper to return current state dict for event checking (converts ECEF to LLA)."""
        if ship.idx in self.dynamic_ships:
            dyn = self.dynamic_ships[ship.idx]
            # Convert ECEF to LLA for external use
            lat, lon, _ = ecef_to_lla(dyn['x'], dyn['y'], dyn['z'])
            sog = dyn.get('sog', dyn['spd'])
            return {
                "lat_deg": lat,
                "lon_deg": normalize_lon(lon),
                "sog_knots": sog,
                "sog_kmh": sog * 1.852,
                "sog_mps": sog * 0.51444444,
                "heading_true_deg": dyn['hdg'],
                "cog_true_deg": dyn['hdg']
            }
        return {"lat_deg":0, "lon_deg":0, "sog_knots":0, "heading_true_deg":0}

    def update_and_get_state(self, ship, dT):
        """
        Update ship state for one time step using ECEF coordinates internally.
        Returns state with LLA coordinates for external use.
        """
        mi = self.proj.map_info

        # Ensure dynamic state exists
        if ship.idx not in self.dynamic_ships:
            s_seed = (self.proj.seed + ship.idx) % (2**32 - 1)
            s_rng = random.Random(s_seed)

            if ship.resampled_points:
                px, py = ship.resampled_points[0]
                _, _, lat, lon = pixel_to_coords(px, py, mi)
                x, y, z = lla_to_ecef(lat, lon, 0.0)
            else:
                x, y, z = lla_to_ecef(0, 0, 0.0)

            points = ship.resampled_points if ship.resampled_points else ship.raw_points
            # Use densified path to prevent "underground" movement and improve high-speed accuracy
            ecef_path = self._densify_path(points, mi) if points else []

            self.dynamic_ships[ship.idx] = {
                'x': x, 'y': y, 'z': z,
                'spd': 5.0, 'sog': 5.0, 'hdg': 0, 'path_idx': 0,
                'dist_from_last_point': 0.0, 'total_dist_traveled': 0.0,
                'following_path': True, 'manual_speed': False, 'manual_heading': False,
                'rng': s_rng, 'ecef_path': ecef_path
            }

        dyn = self.dynamic_ships[ship.idx]
        ecef_path = dyn.get('ecef_path', [])

        # --- Guidance Logic for Maneuvers (using ECEF) ---
        if dyn.get('mode') == 'RETURN_TO_PATH' and ecef_path:
            cx, cy, cz = dyn['x'], dyn['y'], dyn['z']

            # Find closest point on path
            best_idx = -1
            best_dist = float('inf')

            for i, (px, py, pz) in enumerate(ecef_path):
                d = ecef_distance(cx, cy, cz, px, py, pz)
                if d < best_dist:
                    best_dist = d
                    best_idx = i

            if best_idx != -1:
                # Calculate distance ship will travel this step
                speed_kn = dyn.get('spd', 5.0)
                dist_per_step_m = speed_kn * 1852.0 * (dT / 3600.0)

                # KEY FIX: If ship will reach or overshoot the path this step,
                # snap directly to path to prevent overshoot at high speeds
                if best_dist <= dist_per_step_m * 1.5:
                    # Snap to path position
                    px, py, pz = ecef_path[best_idx]
                    dyn['x'], dyn['y'], dyn['z'] = px, py, pz

                    # Switch to normal path following
                    dyn['path_idx'] = best_idx
                    dyn['dist_from_last_point'] = 0.0
                    dyn['following_path'] = True
                    dyn['mode'] = None

                    # Set heading toward next point
                    if best_idx < len(ecef_path) - 1:
                        tx, ty, tz = ecef_path[best_idx + 1]
                        dyn['hdg'] = ecef_heading(px, py, pz, tx, ty, tz)
                    elif len(ecef_path) >= 2:
                        x1, y1, z1 = ecef_path[-2]
                        x2, y2, z2 = ecef_path[-1]
                        dyn['hdg'] = ecef_heading(x1, y1, z1, x2, y2, z2)
                        dyn['following_path'] = False
                else:
                    # Still approaching - set heading toward closest path point
                    px, py, pz = ecef_path[best_idx]
                    dyn['hdg'] = ecef_heading(cx, cy, cz, px, py, pz)

        elif dyn.get('mode') == 'FOLLOW_PATH_GEOMETRY' and ecef_path:
            # FOLLOW_PATH_GEOMETRY: Snap ship back to path and follow it
            # This mode is activated after RETURN_TO_PATH reaches the path
            idx = dyn.get('current_path_idx', 0)
            cx, cy, cz = dyn['x'], dyn['y'], dyn['z']

            # Find the closest point on path (search forward only to prevent backward jumps)
            best_idx = idx
            min_d = float('inf')
            search_limit = min(len(ecef_path), idx + 200)

            for i in range(idx, search_limit):
                px, py, pz = ecef_path[i]
                d = ecef_distance(cx, cy, cz, px, py, pz)
                if d < min_d:
                    min_d = d
                    best_idx = i

            # CRITICAL FIX: Snap position to closest path point to prevent drift
            # This ensures ship stays ON the path, not just heading toward it
            if best_idx < len(ecef_path):
                px, py, pz = ecef_path[best_idx]
                dyn['x'], dyn['y'], dyn['z'] = px, py, pz

            dyn['current_path_idx'] = best_idx

            # Switch to normal path following mode (uses path_idx based movement)
            dyn['path_idx'] = best_idx
            dyn['dist_from_last_point'] = 0.0
            dyn['following_path'] = True
            dyn['mode'] = None  # Clear FOLLOW_PATH_GEOMETRY mode, now using normal path following

            # Set heading toward next point
            if best_idx < len(ecef_path) - 1:
                tx, ty, tz = ecef_path[best_idx + 1]
                dyn['hdg'] = ecef_heading(px, py, pz, tx, ty, tz)
            elif best_idx >= len(ecef_path) - 1:
                # Reached end of path - calculate final heading from last segment
                if len(ecef_path) >= 2:
                    x1, y1, z1 = ecef_path[-2]
                    x2, y2, z2 = ecef_path[-1]
                    dyn['hdg'] = ecef_heading(x1, y1, z1, x2, y2, z2)
                dyn['following_path'] = False

        # --- Movement Logic (ECEF-based) ---

        # Apply speed noise
        variance = getattr(self.proj.settings, "speed_variance", 1.0)
        sigma = math.sqrt(variance)
        noise = dyn['rng'].gauss(0, sigma)

        if dyn['spd'] > 0.01:
            current_speed = max(0.0, dyn['spd'] + noise)
        else:
            current_speed = 0.0

        # Distance to travel this step (meters)
        dist_step_m = current_speed * 1852.0 * (dT / 3600.0)

        if dyn.get('following_path') and ecef_path:
            dist_remain_m = dist_step_m
            current_idx = dyn.get('path_idx', 0)
            dist_from_last = dyn.get('dist_from_last_point', 0.0)

            if current_idx >= len(ecef_path) - 1:
                # Set heading to path extension direction before switching to vector mode
                if len(ecef_path) >= 2:
                    x1, y1, z1 = ecef_path[-2]
                    x2, y2, z2 = ecef_path[-1]
                    dyn['hdg'] = ecef_heading(x1, y1, z1, x2, y2, z2)
                dyn['following_path'] = False

            while dist_remain_m > 0 and dyn.get('following_path'):
                if current_idx >= len(ecef_path) - 1:
                    # Set heading to path extension direction
                    if len(ecef_path) >= 2:
                        x1, y1, z1 = ecef_path[-2]
                        x2, y2, z2 = ecef_path[-1]
                        dyn['hdg'] = ecef_heading(x1, y1, z1, x2, y2, z2)
                    dyn['following_path'] = False
                    break

                # Current and next waypoints in ECEF
                x_curr, y_curr, z_curr = ecef_path[current_idx]
                x_next, y_next, z_next = ecef_path[current_idx + 1]

                # Segment distance
                d_seg_total = ecef_distance(x_curr, y_curr, z_curr, x_next, y_next, z_next)
                dist_to_next = d_seg_total - dist_from_last

                if dist_to_next > dist_remain_m:
                    # Move intermediate along segment
                    dist_from_last += dist_remain_m
                    dyn['total_dist_traveled'] += dist_remain_m
                    frac = dist_from_last / d_seg_total if d_seg_total > 0 else 0

                    # Interpolate ECEF position
                    dyn['x'], dyn['y'], dyn['z'] = ecef_interpolate(
                        x_curr, y_curr, z_curr, x_next, y_next, z_next, frac
                    )

                    # Update heading
                    dyn['hdg'] = ecef_heading(x_curr, y_curr, z_curr, x_next, y_next, z_next)

                    dyn['dist_from_last_point'] = dist_from_last
                    dist_remain_m = 0
                else:
                    # Reached waypoint
                    dyn['x'], dyn['y'], dyn['z'] = x_next, y_next, z_next
                    dist_remain_m -= dist_to_next
                    dyn['total_dist_traveled'] += dist_to_next
                    current_idx += 1
                    dyn['path_idx'] = current_idx
                    dyn['dist_from_last_point'] = 0.0

            # Vector movement after path end
            if not dyn.get('following_path') and dist_remain_m > 0:
                new_x, new_y, new_z = ecef_move(
                    dyn['x'], dyn['y'], dyn['z'], dyn['hdg'], dist_remain_m
                )
                dyn['x'], dyn['y'], dyn['z'] = new_x, new_y, new_z
                dyn['total_dist_traveled'] += dist_remain_m

        else:
            # Vector movement (not following path)
            if dist_step_m > 0:
                new_x, new_y, new_z = ecef_move(
                    dyn['x'], dyn['y'], dyn['z'], dyn['hdg'], dist_step_m
                )
                dyn['x'], dyn['y'], dyn['z'] = new_x, new_y, new_z
                dyn['total_dist_traveled'] += dist_step_m

        # Convert ECEF to LLA for output
        lat, lon, _ = ecef_to_lla(dyn['x'], dyn['y'], dyn['z'])

        # Clamp latitude and stop if at pole
        if lat > 89.9:
            lat = 89.9
            dyn['spd'] = 0
        elif lat < -89.9:
            lat = -89.9
            dyn['spd'] = 0

        lon = normalize_lon(lon)

        sog = current_speed
        cog = dyn['hdg']

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
        
        # Approximate conversion: meters to degrees
        # 1 deg lat ~= 111,111 meters
        # 1 deg lon ~= 111,111 * cos(lat) meters
        meters_per_deg_lat = 111111.0
        meters_per_deg_lon = 111111.0 * math.cos(math.radians(lat))
        
        noise_n_m = random.gauss(0, std_dev) # North offset in meters
        noise_e_m = random.gauss(0, std_dev) # East offset in meters
        
        d_lat = noise_n_m / meters_per_deg_lat
        d_lon = noise_e_m / meters_per_deg_lon if abs(meters_per_deg_lon) > 1e-6 else 0.0
        
        return lat + d_lat, lon + d_lon

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
