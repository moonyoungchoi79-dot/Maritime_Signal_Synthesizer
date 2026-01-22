"""
기하학 연산 모듈

이 모듈은 좌표 변환, 거리 계산, 경로 보간 등의 기하학적 연산을 제공합니다.

주요 기능:
- Vincenty 공식을 사용한 정밀 거리 계산
- WGS84 좌표계 기반 LLA(위도/경도/고도) ↔ ECEF 좌표 변환
- 픽셀 좌표 ↔ 지리 좌표 변환
- 경로 리샘플링 및 보간
- ECEF 기반 이동, 방위각 계산

좌표계 설명:
- LLA: Latitude, Longitude, Altitude (위도, 경도, 고도)
- ECEF: Earth-Centered, Earth-Fixed (지구 중심 고정 좌표계)
- WGS84: World Geodetic System 1984 (GPS에서 사용하는 표준 좌표계)
"""

import math
import numpy as np
from typing import List, Tuple, TYPE_CHECKING

if TYPE_CHECKING:
    from app.models.map_info import MapInfo


def vincenty_distance(lat1, lon1, lat2, lon2):
    """
    Vincenty 공식을 사용하여 두 지점 간 정밀 거리를 계산합니다.

    Vincenty 공식은 지구의 타원체 형태를 고려하여
    Haversine보다 더 정확한 거리를 계산합니다.

    매개변수:
        lat1, lon1: 첫 번째 지점의 위도, 경도 (도 단위)
        lat2, lon2: 두 번째 지점의 위도, 경도 (도 단위)

    반환값:
        두 지점 간 거리 (미터)
    """
    # WGS84 타원체 파라미터
    a = 6378137.0         # 장반경 (적도 반경) - 미터
    f = 1 / 298.257223563 # 편평률 (flattening)
    b = (1 - f) * a       # 단반경 (극 반경) - 미터

    # 위도를 라디안으로 변환
    phi1, phi2 = math.radians(lat1), math.radians(lat2)

    # 환산 위도 계산 (타원체 보정)
    U1 = math.atan((1 - f) * math.tan(phi1))
    U2 = math.atan((1 - f) * math.tan(phi2))

    # 경도 차이 (라디안)
    L = math.radians(lon2 - lon1)

    # 삼각함수 값 미리 계산
    sinU1, cosU1 = math.sin(U1), math.cos(U1)
    sinU2, cosU2 = math.sin(U2), math.cos(U2)

    # 반복 계산을 위한 초기값
    lamb = L
    iter_limit = 100  # 최대 반복 횟수

    # 반복 계산으로 수렴
    for _ in range(iter_limit):
        sin_lamb, cos_lamb = math.sin(lamb), math.cos(lamb)

        # σ의 사인값 계산
        sin_sigma = math.sqrt((cosU2 * sin_lamb) ** 2 +
                              (cosU1 * sinU2 - sinU1 * cosU2 * cos_lamb) ** 2)

        # 두 점이 동일한 경우
        if sin_sigma == 0:
            return 0.0

        # σ의 코사인값 및 σ 계산
        cos_sigma = sinU1 * sinU2 + cosU1 * cosU2 * cos_lamb
        sigma = math.atan2(sin_sigma, cos_sigma)

        # 방위각 관련 계산
        sin_alpha = cosU1 * cosU2 * sin_lamb / sin_sigma
        cos_sq_alpha = 1 - sin_alpha ** 2

        try:
            cos2_sigma_m = cos_sigma - 2 * sinU1 * sinU2 / cos_sq_alpha
        except ZeroDivisionError:
            cos2_sigma_m = 0

        # 보정 계수 C
        C = f / 16 * cos_sq_alpha * (4 + f * (4 - 3 * cos_sq_alpha))
        lamb_prev = lamb

        # 새로운 λ 계산
        lamb = L + (1 - C) * f * sin_alpha * (sigma + C * sin_sigma * (
            cos2_sigma_m + C * cos_sigma * (-1 + 2 * cos2_sigma_m ** 2)))

        # 수렴 확인 (허용 오차: 1e-12 라디안)
        if abs(lamb - lamb_prev) < 1e-12:
            break
    else:
        # 수렴 실패 시 Haversine 공식으로 대체
        return _simple_haversine_fallback(lat1, lon1, lat2, lon2)

    # 최종 거리 계산
    u_sq = cos_sq_alpha * (a ** 2 - b ** 2) / (b ** 2)
    A = 1 + u_sq / 16384 * (4096 + u_sq * (-768 + u_sq * (320 - 175 * u_sq)))
    B = u_sq / 1024 * (256 + u_sq * (-128 + u_sq * (74 - 47 * u_sq)))
    delta_sigma = B * sin_sigma * (cos2_sigma_m + B / 4 * (cos_sigma * (-1 + 2 * cos2_sigma_m ** 2) -
        B / 6 * cos2_sigma_m * (-3 + 4 * sin_sigma ** 2) * (-3 + 4 * cos2_sigma_m ** 2)))

    return b * A * (sigma - delta_sigma)


def _simple_haversine_fallback(lat1, lon1, lat2, lon2):
    """
    Haversine 공식을 사용한 간단한 거리 계산 (대체용).

    Vincenty 공식이 수렴하지 않을 때 사용됩니다.
    구면 지구 모델을 사용하므로 Vincenty보다 정확도가 낮습니다.

    매개변수:
        lat1, lon1: 첫 번째 지점의 위도, 경도 (도 단위)
        lat2, lon2: 두 번째 지점의 위도, 경도 (도 단위)

    반환값:
        두 지점 간 거리 (미터)
    """
    R = 6371000.0  # 지구 평균 반경 (미터)
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    return R * c


def normalize_lon(lon):
    """
    경도를 -180 ~ +180 범위로 정규화합니다.

    매개변수:
        lon: 정규화할 경도 값 (도 단위)

    반환값:
        정규화된 경도 (-180 ~ +180)
    """
    return (lon + 180) % 360 - 180


def resample_polyline_numpy(points: List[Tuple[float, float]], n_out: int):
    """
    폴리라인(경로)을 등간격으로 리샘플링합니다.

    주어진 경로를 n_out 개의 점으로 재구성하여
    각 점 간 거리가 균일하게 만듭니다.

    매개변수:
        points: 원본 경로 점 목록 [(x, y), ...]
        n_out: 출력할 점의 개수

    반환값:
        리샘플링된 점 목록
    """
    if not points:
        return []
    if len(points) < 2:
        return points * n_out

    try:
        pts = np.array(points)
        d = np.diff(pts, axis=0)

        # 각 세그먼트 길이 계산
        seg_lens = np.sqrt((d ** 2).sum(axis=1))
        seg_lens = np.maximum(seg_lens, 1e-6)  # 0 길이 방지

        # 누적 거리 계산
        cum_dist = np.concatenate(([0], np.cumsum(seg_lens)))
        total_len = cum_dist[-1]

        if total_len <= 1e-6:
            return [tuple(pts[0])] * n_out

        # 등간격 거리에서 보간
        target_dists = np.linspace(0, total_len, n_out)
        new_x = np.interp(target_dists, cum_dist, pts[:, 0])
        new_y = np.interp(target_dists, cum_dist, pts[:, 1])

        return list(zip(new_x, new_y))
    except:
        return points


def resample_polygon_equidistant(points: List[Tuple[float, float]], n_out: int):
    """
    다각형을 등간격으로 리샘플링합니다.

    폐곡선(다각형)을 처리하기 위해 시작점과 끝점을 연결합니다.

    매개변수:
        points: 다각형 꼭짓점 목록 [(x, y), ...]
        n_out: 출력할 점의 개수

    반환값:
        리샘플링된 점 목록
    """
    if len(points) < 2:
        return points

    # 폐곡선을 위해 첫 점을 끝에 추가
    pts = np.array(points + [points[0]])
    d = np.diff(pts, axis=0)

    # 각 세그먼트 길이 및 누적 거리 계산
    seg_lens = np.sqrt((d ** 2).sum(axis=1))
    cum_dist = np.concatenate(([0], np.cumsum(seg_lens)))
    total_len = cum_dist[-1]

    # 마지막 점 제외 (첫 점과 동일)
    target_dists = np.linspace(0, total_len, n_out + 1)[:-1]
    new_x = np.interp(target_dists, cum_dist, pts[:, 0])
    new_y = np.interp(target_dists, cum_dist, pts[:, 1])

    return list(zip(new_x, new_y))


def pixel_to_coords(px, py, mi: "MapInfo"):
    """
    픽셀 좌표를 지리 좌표로 변환합니다.

    매개변수:
        px, py: 픽셀 좌표
        mi: 지도 정보 객체 (MapInfo)

    반환값:
        (0.0, 0.0, lat, lon) - 처음 두 값은 사용되지 않음
    """
    scale = mi.pixels_per_degree
    if scale <= 0:
        scale = 1.0

    lon = px / scale
    lat = -py / scale

    return 0.0, 0.0, lat, lon


def coords_to_pixel(lat, lon, mi: "MapInfo"):
    """
    지리 좌표를 픽셀 좌표로 변환합니다.

    매개변수:
        lat, lon: 위도, 경도 (도 단위)
        mi: 지도 정보 객체 (MapInfo)

    반환값:
        (px, py) 픽셀 좌표
    """
    scale = mi.pixels_per_degree
    px = lon * scale
    py = -lat * scale

    return px, py


def get_optimal_grid_step(scale):
    """
    지도 축척에 따른 최적의 격자 간격을 계산합니다.

    지도에 그려지는 위도/경도 격자선의 간격을 결정합니다.
    화면에서 약 100픽셀 간격으로 격자가 표시되도록 합니다.

    매개변수:
        scale: 픽셀/도 (pixels per degree)

    반환값:
        격자 간격 (도 단위)
    """
    if scale <= 0:
        return 90

    target_pixel_spacing = 100  # 목표 픽셀 간격

    # 후보 격자 간격 (도 단위) - 작은 것부터 큰 것 순서
    deg_step_candidates = [0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 30, 45, 90]

    for cand in deg_step_candidates:
        if cand * scale >= target_pixel_spacing:
            return cand

    return 90


# =============================================================================
# WGS84 타원체 상수
# =============================================================================
WGS84_A = 6378137.0  # 장반경 (적도 반경) - 미터
WGS84_F = 1 / 298.257223563  # 편평률
WGS84_E_SQ = WGS84_F * (2 - WGS84_F)  # 이심률의 제곱


def lla_to_ecef(lat, lon, alt):
    """
    LLA 좌표를 ECEF 좌표로 변환합니다.

    LLA (Latitude, Longitude, Altitude)는 GPS에서 사용하는 좌표계이고,
    ECEF (Earth-Centered, Earth-Fixed)는 지구 중심 직교 좌표계입니다.

    매개변수:
        lat: 위도 (도 단위)
        lon: 경도 (도 단위)
        alt: 고도 (미터)

    반환값:
        (x, y, z) ECEF 좌표 (미터)
    """
    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)

    # 곡률 반경 계산
    n = WGS84_A / np.sqrt(1 - WGS84_E_SQ * np.sin(lat_rad)**2)

    # ECEF 좌표 계산
    x = (n + alt) * np.cos(lat_rad) * np.cos(lon_rad)
    y = (n + alt) * np.cos(lat_rad) * np.sin(lon_rad)
    z = ((1 - WGS84_E_SQ) * n + alt) * np.sin(lat_rad)

    return x, y, z


def ecef_to_lla(x, y, z):
    """
    ECEF 좌표를 LLA 좌표로 변환합니다.

    Bowring 공식을 사용하여 반복 계산 없이 빠르게 변환합니다.

    매개변수:
        x, y, z: ECEF 좌표 (미터)

    반환값:
        (lat, lon, alt) - 위도(도), 경도(도), 고도(미터)
    """
    p = np.sqrt(x**2 + y**2)
    lon_rad = np.arctan2(y, x)

    # Bowring 공식을 사용한 위도 계산
    beta = np.arctan2(z, (1 - WGS84_F) * p)
    lat_rad = np.arctan2(z + (WGS84_A * WGS84_E_SQ / (1 - WGS84_F)) * np.sin(beta)**3,
                        p - (WGS84_A * WGS84_E_SQ) * np.cos(beta)**3)

    # 고도 계산
    n = WGS84_A / np.sqrt(1 - WGS84_E_SQ * np.sin(lat_rad)**2)
    alt = p / np.cos(lat_rad) - n

    return np.degrees(lat_rad), np.degrees(lon_rad), alt


# =============================================================================
# ECEF 기반 시뮬레이션 연산 함수
# =============================================================================

def ecef_distance(x1, y1, z1, x2, y2, z2):
    """
    두 ECEF 좌표 간 유클리드 거리를 계산합니다.

    매개변수:
        x1, y1, z1: 첫 번째 점의 ECEF 좌표
        x2, y2, z2: 두 번째 점의 ECEF 좌표

    반환값:
        거리 (미터)
    """
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)


def ecef_move(x, y, z, heading_deg, distance_m):
    """
    ECEF 좌표에서 주어진 방향과 거리만큼 이동한 새 좌표를 계산합니다.

    구면 지구 모델을 사용하여 이동 후 위치를 계산합니다.

    매개변수:
        x, y, z: 시작점 ECEF 좌표
        heading_deg: 방위각 (도 단위, 0=북, 90=동, 180=남, 270=서)
        distance_m: 이동 거리 (미터)

    반환값:
        (x, y, z) 새로운 ECEF 좌표
    """
    # 현재 ECEF를 LLA로 변환
    lat, lon, alt = ecef_to_lla(x, y, z)

    # 구면 삼각법을 사용한 새 위치 계산
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    heading_rad = math.radians(heading_deg)

    # 각거리 계산
    R = 6371000.0  # 지구 반경 (미터)
    angular_dist = distance_m / R

    # 새 위도 계산
    new_lat_rad = math.asin(
        math.sin(lat_rad) * math.cos(angular_dist) +
        math.cos(lat_rad) * math.sin(angular_dist) * math.cos(heading_rad)
    )

    # 새 경도 계산
    new_lon_rad = lon_rad + math.atan2(
        math.sin(heading_rad) * math.sin(angular_dist) * math.cos(lat_rad),
        math.cos(angular_dist) - math.sin(lat_rad) * math.sin(new_lat_rad)
    )

    new_lat = math.degrees(new_lat_rad)
    new_lon = math.degrees(new_lon_rad)

    # 경도 정규화
    new_lon = normalize_lon(new_lon)

    # ECEF로 변환하여 반환
    return lla_to_ecef(new_lat, new_lon, alt)


def ecef_heading(x1, y1, z1, x2, y2, z2):
    """
    두 ECEF 좌표 간 방위각(진방위)을 계산합니다.

    매개변수:
        x1, y1, z1: 시작점 ECEF 좌표
        x2, y2, z2: 목표점 ECEF 좌표

    반환값:
        방위각 (0~360도, 0=북, 90=동)
    """
    # ECEF를 LLA로 변환
    lat1, lon1, _ = ecef_to_lla(x1, y1, z1)
    lat2, lon2, _ = ecef_to_lla(x2, y2, z2)

    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    d_lon = math.radians(lon2 - lon1)

    # 방위각 계산 (구면 삼각법)
    x = math.sin(d_lon) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - \
        math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(d_lon)

    heading = math.degrees(math.atan2(x, y))
    return (heading + 360) % 360


def ecef_interpolate(x1, y1, z1, x2, y2, z2, fraction):
    """
    두 ECEF 좌표 간 선형 보간을 수행합니다.

    매개변수:
        x1, y1, z1: 시작점 ECEF 좌표
        x2, y2, z2: 끝점 ECEF 좌표
        fraction: 보간 비율 (0.0 = 시작점, 1.0 = 끝점)

    반환값:
        (x, y, z) 보간된 ECEF 좌표
    """
    x = x1 + (x2 - x1) * fraction
    y = y1 + (y2 - y1) * fraction
    z = z1 + (z2 - z1) * fraction
    return x, y, z


def path_points_to_ecef(points, mi):
    """
    픽셀 좌표 경로를 ECEF 좌표로 변환합니다.

    매개변수:
        points: 픽셀 좌표 점 목록 [(px, py), ...]
        mi: 지도 정보 객체 (MapInfo)

    반환값:
        ECEF 좌표 목록 [(x, y, z), ...]
    """
    ecef_points = []
    for px, py in points:
        _, _, lat, lon = pixel_to_coords(px, py, mi)
        x, y, z = lla_to_ecef(lat, lon, 0.0)
        ecef_points.append((x, y, z))
    return ecef_points


def great_circle_interpolate(lat1: float, lon1: float, lat2: float, lon2: float,
                             n_points: int = 50) -> List[Tuple[float, float]]:
    """
    두 지점 사이의 대권항로(Great Circle Route)를 따라 보간된 점들을 반환합니다.

    ECEF 좌표계에서 구면선형보간(Slerp)을 사용하여 대권항로를 계산합니다.

    매개변수:
        lat1, lon1: 시작점 위도, 경도 (도 단위)
        lat2, lon2: 끝점 위도, 경도 (도 단위)
        n_points: 보간할 점의 개수 (시작점, 끝점 포함)

    반환값:
        보간된 점들의 목록 [(lat, lon), ...]
    """
    if n_points < 2:
        return [(lat1, lon1), (lat2, lon2)]

    # 시작점과 끝점을 ECEF로 변환
    x1, y1, z1 = lla_to_ecef(lat1, lon1, 0.0)
    x2, y2, z2 = lla_to_ecef(lat2, lon2, 0.0)

    # ECEF 벡터의 크기 (지구 반경에 해당)
    r1 = math.sqrt(x1**2 + y1**2 + z1**2)
    r2 = math.sqrt(x2**2 + y2**2 + z2**2)

    # 단위 벡터로 정규화
    ux1, uy1, uz1 = x1/r1, y1/r1, z1/r1
    ux2, uy2, uz2 = x2/r2, y2/r2, z2/r2

    # 두 벡터 사이의 각도 (구면 거리)
    dot = ux1*ux2 + uy1*uy2 + uz1*uz2
    dot = max(-1.0, min(1.0, dot))  # 부동소수점 오류 방지
    omega = math.acos(dot)

    # 각도가 매우 작으면 직선 보간 사용
    if abs(omega) < 1e-10:
        return [(lat1, lon1), (lat2, lon2)]

    sin_omega = math.sin(omega)

    result = []
    for i in range(n_points):
        t = i / (n_points - 1)

        # Slerp (구면선형보간)
        a = math.sin((1 - t) * omega) / sin_omega
        b = math.sin(t * omega) / sin_omega

        # 보간된 ECEF 좌표 (단위 벡터)
        ux = a * ux1 + b * ux2
        uy = a * uy1 + b * uy2
        uz = a * uz1 + b * uz2

        # 지구 표면으로 스케일 (평균 반경 사용)
        r = (r1 + r2) / 2
        x, y, z = ux * r, uy * r, uz * r

        # LLA로 변환
        lat, lon, _ = ecef_to_lla(x, y, z)
        result.append((lat, lon))

    return result


def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    두 지점 간 방위각(진방위)을 계산합니다.

    매개변수:
        lat1, lon1: 시작점 위도, 경도 (도 단위)
        lat2, lon2: 목표점 위도, 경도 (도 단위)

    반환값:
        방위각 (0~360도, 0=북, 90=동)
    """
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    d_lon = math.radians(lon2 - lon1)

    x = math.sin(d_lon) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - \
        math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(d_lon)

    bearing = math.degrees(math.atan2(x, y))
    return (bearing + 360) % 360


def great_circle_path_pixels(raw_points: List[Tuple[float, float]],
                             mi: "MapInfo",
                             points_per_segment: int = 30) -> List[Tuple[float, float]]:
    """
    제어점들 사이를 대권항로로 연결한 픽셀 좌표 경로를 반환합니다.

    매개변수:
        raw_points: 원본 제어점 목록 [(px, py), ...] (픽셀 좌표)
        mi: 지도 정보 객체 (MapInfo)
        points_per_segment: 각 세그먼트당 보간할 점의 개수

    반환값:
        대권항로를 따라 보간된 픽셀 좌표 목록 [(px, py), ...]
    """
    if not raw_points or len(raw_points) < 2:
        return list(raw_points) if raw_points else []

    result = []

    for i in range(len(raw_points) - 1):
        px1, py1 = raw_points[i]
        px2, py2 = raw_points[i + 1]

        # 픽셀 좌표를 위경도로 변환
        _, _, lat1, lon1 = pixel_to_coords(px1, py1, mi)
        _, _, lat2, lon2 = pixel_to_coords(px2, py2, mi)

        # 대권항로 보간
        gc_points = great_circle_interpolate(lat1, lon1, lat2, lon2, points_per_segment)

        # 위경도를 픽셀 좌표로 변환
        for j, (lat, lon) in enumerate(gc_points):
            # 마지막 세그먼트가 아니면 끝점 제외 (다음 세그먼트 시작점과 중복 방지)
            if i < len(raw_points) - 2 and j == len(gc_points) - 1:
                continue
            px, py = coords_to_pixel(lat, lon, mi)
            result.append((px, py))

    return result
