import math
import traceback
import random
import numpy as np
from scipy.interpolate import interp1d, PchipInterpolator

from PyQt6.QtCore import QObject, pyqtSignal

from app.core.models.project import current_project
from app.core.geometry import haversine_distance, resample_polyline_numpy, pixel_to_coords

class SpeedGeneratorWorker(QObject):
    finished = pyqtSignal(int)
    error = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.target_ship_indices = []
        self.variance = 1.0

    def run(self):
        try:
            if not self.target_ship_indices: return
            proj = current_project
            mi = proj.map_info
            
            for target_idx in self.target_ship_indices:
                ship = proj.get_ship_by_idx(target_idx)
                
                # Set deterministic seed for this ship based on project seed and ship index
                seed_val = (proj.seed + target_idx) % (2**32 - 1)
                np.random.seed(seed_val)
                random.seed(seed_val)

                if not ship: continue
                if not ship.raw_points or len(ship.raw_points) < 2:
                    continue
                
                N_calc = max(5000, len(ship.raw_points) * 50)
                
                ship.resampled_points = resample_polyline_numpy(ship.raw_points, N_calc)

                lats = []
                lons = []
                for px, py in ship.resampled_points:
                    _, _, lat, lon = pixel_to_coords(px, py, mi)
                    lats.append(lat)
                    lons.append(lon)
                
                dists_m = [0.0]
                current_dist = 0.0
                for i in range(1, len(lats)):
                    d = haversine_distance(lats[i-1], lons[i-1], lats[i], lons[i])
                    current_dist += d
                    dists_m.append(current_dist)
                
                r_cum = np.array(dists_m)
                
                indices = np.arange(N_calc)
                kf_idx_map = []
                kf_val_map = []
                
                for i, dist_ratio in enumerate(np.linspace(0, 1, len(ship.raw_points))):
                    idx_mesh = int(round(dist_ratio * (N_calc - 1)))
                    kf_idx_map.append(idx_mesh)
                    spd = ship.raw_speeds.get(i, 5.0)
                    kf_val_map.append(spd)
                
                if len(kf_idx_map) < 2:
                    kf_idx_map = [0, N_calc-1]
                    kf_val_map = [5.0, 5.0]

                unique_kf = {}
                for k, v in zip(kf_idx_map, kf_val_map):
                    unique_kf[k] = v
                sorted_kf = sorted(unique_kf.items())
                kf_x = [x[0] for x in sorted_kf]
                kf_y = [x[1] for x in sorted_kf]
                
                if PchipInterpolator is not None and len(kf_x) > 2:
                    interpolator = PchipInterpolator(kf_x, kf_y)
                else:
                    interpolator = interp1d(kf_x, kf_y, kind='linear', fill_value="extrapolate")
                
                v_base_kn = interpolator(indices)
                v_base_kn = np.maximum(0, v_base_kn)
                ship.base_velocities_kn = v_base_kn.tolist()
                
                sigma = math.sqrt(max(0.0, self.variance))
                epsilon = np.random.normal(0, sigma, N_calc)
                v_out_kn = np.maximum(0.0, v_base_kn + epsilon)
                ship.node_velocities_kn = v_out_kn.tolist()
                
                v_out_mps = v_out_kn * 0.51444444
                v_seg_mps = 0.5 * (v_out_mps[:-1] + v_out_mps[1:])
                v_seg_mps = np.maximum(1e-6, v_seg_mps)
                
                seg_dists = np.diff(r_cum)
                
                dt_seg = seg_dists / v_seg_mps
                cum_time = np.concatenate(([0], np.cumsum(dt_seg)))
                ship.total_duration_sec = cum_time[-1]
                
                ship.cumulative_time = cum_time.tolist()
                ship.packed_data = None 
                ship.is_generated = True
                
                self.finished.emit(ship.idx)
              
        except Exception as e:
            traceback.print_exc()
            self.error.emit(str(e))
