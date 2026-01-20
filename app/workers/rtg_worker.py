"""
Random Target Generation (RTG) worker.

Runs RTG in a background thread so the UI stays responsive.
"""

import math
import random
from typing import List, Optional, Sequence, Tuple

from PyQt6.QtCore import QObject, pyqtSignal

from app.core.models.ship import ShipData, get_ship_dimensions
from app.core.geometry import coords_to_pixel


class RTGWorker(QObject):
    """
    Generate random targets off the UI thread.

    Signals:
        finished: List[ShipData]
        progress: (current, total)
        error: error message
    """

    finished = pyqtSignal(list)
    progress = pyqtSignal(int, int)
    error = pyqtSignal(str)

    def __init__(
        self,
        own_lat: float,
        own_lon: float,
        own_spd_kn: float,
        R_nm: float,
        N_ai: int,
        N_ra: int,
        N_both: int,
        ship_class: str,
        map_info,
        seed_val: int,
        speed_variance: float,
        existing_idxs: set,
        existing_names: set,
        area_points: Optional[Sequence[Tuple[float, float]]] = None,
        parent=None,
    ):
        super().__init__(parent)
        self.own_lat = own_lat
        self.own_lon = own_lon
        self.own_spd_kn = own_spd_kn
        self.R_nm = R_nm
        self.N_ai = N_ai
        self.N_ra = N_ra
        self.N_both = N_both
        self.ship_class = ship_class
        self.map_info = map_info
        self.seed_val = seed_val
        self.speed_variance = speed_variance
        self.existing_idxs = set(existing_idxs)
        self.existing_names = set(existing_names)
        self.area_points = list(area_points) if area_points else None
        self._cancelled = False

    def cancel(self):
        self._cancelled = True

    def run(self):
        try:
            ships = self._generate_targets()
            if not self._cancelled:
                self.finished.emit(ships)
        except Exception as exc:
            import traceback
            self.error.emit(f"{exc}\n{traceback.format_exc()}")

    def _generate_targets(self) -> List[ShipData]:
        random.seed(self.seed_val)

        total_targets = self.N_ai + self.N_ra + self.N_both
        ships: List[ShipData] = []

        area_points = self._valid_area_points(self.area_points)
        if area_points:
            bounds = self._bbox(area_points)
        else:
            bounds = None
            R_deg_lat = self.R_nm / 60.0

        for i in range(total_targets):
            if self._cancelled:
                break

            self.progress.emit(i + 1, total_targets)

            if i < self.N_ai:
                s_type = "AI"
            elif i < self.N_ai + self.N_ra:
                s_type = "RA"
            else:
                s_type = "BOTH"

            if area_points and bounds:
                start_px, start_py = self._random_point_in_polygon(area_points, bounds)
            else:
                r = R_deg_lat * math.sqrt(random.random())
                theta = random.random() * 2 * math.pi

                d_lat = r * math.cos(theta)
                cos_lat = math.cos(math.radians(self.own_lat))
                if abs(cos_lat) < 0.01:
                    cos_lat = 0.01
                d_lon = r * math.sin(theta) / cos_lat

                start_lat = self.own_lat + d_lat
                start_lon = self.own_lon + d_lon
                start_lat = max(-89.9, min(89.9, start_lat))

                start_px, start_py = coords_to_pixel(start_lat, start_lon, self.map_info)

            sigma = math.sqrt(self.speed_variance)
            spd = abs(random.gauss(self.own_spd_kn, sigma))
            if spd < 0.1:
                spd = 0.1

            heading = random.random() * 360.0

            new_idx = 1001
            while new_idx in self.existing_idxs:
                new_idx += 1
            self.existing_idxs.add(new_idx)

            r_num = 1
            while f"Random-{r_num}" in self.existing_names:
                r_num += 1
            new_name = f"Random-{r_num}"
            self.existing_names.add(new_name)

            new_ship = ShipData(new_idx, new_name)
            new_ship.mmsi = random.randint(100000000, 999999999)
            new_ship.is_generated = True
            new_ship.is_random_target = True

            new_ship.raw_points = [(start_px, start_py)]
            new_ship.raw_speeds = {0: spd}
            new_ship.resampled_points = [(start_px, start_py)]
            new_ship.initial_heading = heading

            new_ship.ship_class = self.ship_class
            dims = get_ship_dimensions(self.ship_class)
            new_ship.length_m = dims[0]
            new_ship.beam_m = dims[1]
            new_ship.draft_m = dims[2]
            new_ship.air_draft_m = dims[3]
            new_ship.height_m = dims[3] * 0.5

            if s_type == "AI":
                new_ship.signals_enabled["AIVDM"] = True
                new_ship.signals_enabled["RATTM"] = False
                new_ship.signals_enabled["Camera"] = True
            elif s_type == "RA":
                new_ship.signals_enabled["AIVDM"] = False
                new_ship.signals_enabled["RATTM"] = True
                new_ship.signals_enabled["Camera"] = True
            else:
                new_ship.signals_enabled["AIVDM"] = True
                new_ship.signals_enabled["RATTM"] = True
                new_ship.signals_enabled["Camera"] = True

            ships.append(new_ship)

        return ships

    @staticmethod
    def _valid_area_points(points: Optional[Sequence[Tuple[float, float]]]):
        if not points or len(points) < 3:
            return None
        return list(points)

    @staticmethod
    def _bbox(points: Sequence[Tuple[float, float]]) -> Tuple[float, float, float, float]:
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        return min(xs), max(xs), min(ys), max(ys)

    def _random_point_in_polygon(
        self,
        points: Sequence[Tuple[float, float]],
        bounds: Tuple[float, float, float, float],
    ) -> Tuple[float, float]:
        min_x, max_x, min_y, max_y = bounds
        for _ in range(200):
            x = random.uniform(min_x, max_x)
            y = random.uniform(min_y, max_y)
            if self._point_in_polygon(x, y, points):
                return x, y
        return (min_x + max_x) / 2.0, (min_y + max_y) / 2.0

    @staticmethod
    def _point_in_polygon(x: float, y: float, poly: Sequence[Tuple[float, float]]) -> bool:
        inside = False
        j = len(poly) - 1
        for i in range(len(poly)):
            xi, yi = poly[i]
            xj, yj = poly[j]
            intersects = ((yi > y) != (yj > y)) and (
                x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi
            )
            if intersects:
                inside = not inside
            j = i
        return inside
