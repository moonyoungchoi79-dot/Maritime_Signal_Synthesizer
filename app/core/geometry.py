import math
import numpy as np
from typing import List, Tuple, TYPE_CHECKING

if TYPE_CHECKING:
    from app.core.models.map_info import MapInfo

def vincenty_distance(lat1, lon1, lat2, lon2):
    
    a = 6378137.0         
    f = 1 / 298.257223563 
    b = (1 - f) * a       

    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    U1 = math.atan((1 - f) * math.tan(phi1))
    U2 = math.atan((1 - f) * math.tan(phi2))
    L = math.radians(lon2 - lon1)

    sinU1, cosU1 = math.sin(U1), math.cos(U1)
    sinU2, cosU2 = math.sin(U2), math.cos(U2)

    lamb = L
    iter_limit = 100
    
    for _ in range(iter_limit):
        sin_lamb, cos_lamb = math.sin(lamb), math.cos(lamb)
        sin_sigma = math.sqrt((cosU2 * sin_lamb) ** 2 +
                              (cosU1 * sinU2 - sinU1 * cosU2 * cos_lamb) ** 2)
        
        if sin_sigma == 0:
            return 0.0  

        cos_sigma = sinU1 * sinU2 + cosU1 * cosU2 * cos_lamb
        sigma = math.atan2(sin_sigma, cos_sigma)
        
        sin_alpha = cosU1 * cosU2 * sin_lamb / sin_sigma
        cos_sq_alpha = 1 - sin_alpha ** 2
        
        try:
            cos2_sigma_m = cos_sigma - 2 * sinU1 * sinU2 / cos_sq_alpha
        except ZeroDivisionError:
            cos2_sigma_m = 0
            
        C = f / 16 * cos_sq_alpha * (4 + f * (4 - 3 * cos_sq_alpha))
        lamb_prev = lamb
        lamb = L + (1 - C) * f * sin_alpha * (sigma + C * sin_sigma * (
            cos2_sigma_m + C * cos_sigma * (-1 + 2 * cos2_sigma_m ** 2)))
            
        if abs(lamb - lamb_prev) < 1e-12:
            break
    else:
        return _simple_haversine_fallback(lat1, lon1, lat2, lon2)

    u_sq = cos_sq_alpha * (a ** 2 - b ** 2) / (b ** 2)
    A = 1 + u_sq / 16384 * (4096 + u_sq * (-768 + u_sq * (320 - 175 * u_sq)))
    B = u_sq / 1024 * (256 + u_sq * (-128 + u_sq * (74 - 47 * u_sq)))
    delta_sigma = B * sin_sigma * (cos2_sigma_m + B / 4 * (cos_sigma * (-1 + 2 * cos2_sigma_m ** 2) -
        B / 6 * cos2_sigma_m * (-3 + 4 * sin_sigma ** 2) * (-3 + 4 * cos2_sigma_m ** 2)))
        
    return b * A * (sigma - delta_sigma)

def _simple_haversine_fallback(lat1, lon1, lat2, lon2):
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def normalize_lon(lon):
    return (lon + 180) % 360 - 180


def resample_polyline_numpy(points: List[Tuple[float, float]], n_out: int):
    if not points: return []
    if len(points) < 2: return points * n_out 
    try:
        pts = np.array(points)
        d = np.diff(pts, axis=0)
        seg_lens = np.sqrt((d ** 2).sum(axis=1))
        seg_lens = np.maximum(seg_lens, 1e-6) 
        cum_dist = np.concatenate(([0], np.cumsum(seg_lens)))
        total_len = cum_dist[-1]
        if total_len <= 1e-6: return [tuple(pts[0])] * n_out
        target_dists = np.linspace(0, total_len, n_out)
        new_x = np.interp(target_dists, cum_dist, pts[:, 0])
        new_y = np.interp(target_dists, cum_dist, pts[:, 1])
        return list(zip(new_x, new_y))
    except: return points

def resample_polygon_equidistant(points: List[Tuple[float, float]], n_out: int):
    if len(points) < 2: return points
    pts = np.array(points + [points[0]]) 
    d = np.diff(pts, axis=0)
    seg_lens = np.sqrt((d ** 2).sum(axis=1))
    cum_dist = np.concatenate(([0], np.cumsum(seg_lens)))
    total_len = cum_dist[-1]
    target_dists = np.linspace(0, total_len, n_out + 1)[:-1] 
    new_x = np.interp(target_dists, cum_dist, pts[:, 0])
    new_y = np.interp(target_dists, cum_dist, pts[:, 1])
    return list(zip(new_x, new_y))

def pixel_to_coords(px, py, mi: "MapInfo"):
    scale = mi.pixels_per_degree
    if scale <= 0: scale = 1.0
    lon = px / scale
    lat = -py / scale
    return 0.0, 0.0, lat, lon

def coords_to_pixel(lat, lon, mi: "MapInfo"):
    scale = mi.pixels_per_degree
    px = lon * scale
    py = -lat * scale
    return px, py

def get_optimal_grid_step(scale):
    if scale <= 0: return 90
    target_pixel_spacing = 100
    deg_step_candidates = [0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 30, 45, 90]

    for cand in deg_step_candidates:
        if cand * scale >= target_pixel_spacing:
            return cand

    return 90

WGS84_A = 6378137.0  # Semi-major axis
WGS84_F = 1 / 298.257223563  # Flattening
WGS84_E_SQ = WGS84_F * (2 - WGS84_F)  # Eccentricity squared

def lla_to_ecef(lat, lon, alt):
    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)
    
    n = WGS84_A / np.sqrt(1 - WGS84_E_SQ * np.sin(lat_rad)**2)
    
    x = (n + alt) * np.cos(lat_rad) * np.cos(lon_rad)
    y = (n + alt) * np.cos(lat_rad) * np.sin(lon_rad)
    z = ((1 - WGS84_E_SQ) * n + alt) * np.sin(lat_rad)
    
    return x, y, z

def ecef_to_lla(x, y, z):
    p = np.sqrt(x**2 + y**2)
    lon_rad = np.arctan2(y, x)

    # Bowring's formula for latitude
    beta = np.arctan2(z, (1 - WGS84_F) * p)
    lat_rad = np.arctan2(z + (WGS84_A * WGS84_E_SQ / (1 - WGS84_F)) * np.sin(beta)**3,
                        p - (WGS84_A * WGS84_E_SQ) * np.cos(beta)**3)

    n = WGS84_A / np.sqrt(1 - WGS84_E_SQ * np.sin(lat_rad)**2)
    alt = p / np.cos(lat_rad) - n

    return np.degrees(lat_rad), np.degrees(lon_rad), alt


# =============================================================================
# ECEF-based calculation functions for internal simulation
# =============================================================================

def ecef_distance(x1, y1, z1, x2, y2, z2):
    """Calculate Euclidean distance between two ECEF points in meters."""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)


def ecef_move(x, y, z, heading_deg, distance_m):
    """
    Move from ECEF position by distance along heading.
    Heading is true north bearing (0=N, 90=E, 180=S, 270=W).
    Returns new ECEF coordinates.
    """
    # Convert current ECEF to LLA
    lat, lon, alt = ecef_to_lla(x, y, z)

    # Calculate new LLA position using spherical approximation
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    heading_rad = math.radians(heading_deg)

    # Angular distance
    R = 6371000.0  # Earth radius in meters
    angular_dist = distance_m / R

    # New latitude
    new_lat_rad = math.asin(
        math.sin(lat_rad) * math.cos(angular_dist) +
        math.cos(lat_rad) * math.sin(angular_dist) * math.cos(heading_rad)
    )

    # New longitude
    new_lon_rad = lon_rad + math.atan2(
        math.sin(heading_rad) * math.sin(angular_dist) * math.cos(lat_rad),
        math.cos(angular_dist) - math.sin(lat_rad) * math.sin(new_lat_rad)
    )

    new_lat = math.degrees(new_lat_rad)
    new_lon = math.degrees(new_lon_rad)

    # Normalize longitude
    new_lon = normalize_lon(new_lon)

    # Convert back to ECEF
    return lla_to_ecef(new_lat, new_lon, alt)


def ecef_heading(x1, y1, z1, x2, y2, z2):
    """
    Calculate heading (bearing) from point 1 to point 2.
    Returns heading in degrees (0-360, 0=N, 90=E).
    """
    lat1, lon1, _ = ecef_to_lla(x1, y1, z1)
    lat2, lon2, _ = ecef_to_lla(x2, y2, z2)

    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    d_lon = math.radians(lon2 - lon1)

    x = math.sin(d_lon) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - \
        math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(d_lon)

    heading = math.degrees(math.atan2(x, y))
    return (heading + 360) % 360


def ecef_interpolate(x1, y1, z1, x2, y2, z2, fraction):
    """
    Linear interpolation between two ECEF points.
    fraction: 0.0 = point1, 1.0 = point2
    """
    x = x1 + (x2 - x1) * fraction
    y = y1 + (y2 - y1) * fraction
    z = z1 + (z2 - z1) * fraction
    return x, y, z


def path_points_to_ecef(points, mi):
    """
    Convert list of pixel points to ECEF coordinates.
    Returns list of (x, y, z) tuples.
    """
    ecef_points = []
    for px, py in points:
        _, _, lat, lon = pixel_to_coords(px, py, mi)
        x, y, z = lla_to_ecef(lat, lon, 0.0)
        ecef_points.append((x, y, z))
    return ecef_points