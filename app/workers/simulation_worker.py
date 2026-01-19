"""
시뮬레이션 워커 모듈

이 모듈은 해상 신호 시뮬레이션의 핵심 엔진을 제공합니다.
선박의 위치를 실시간으로 업데이트하고, AIS/레이더/카메라 신호를 생성하여
UDP로 전송합니다.

클래스:
    SimulationWorker: 시뮬레이션 메인 워커

주요 기능:
    - 선박 위치 업데이트 (대권항법/벡터 모드)
    - NMEA 신호 생성 (AIS, RATTM, Camera)
    - 이벤트 트리거 처리 (시간, 영역, CPA, 거리 조건)
    - 거리 기반 수신 모델 (드롭아웃 확률 계산)
    - 파노라마 뷰용 카메라 탐지 데이터 생성

상수:
    PANO_W_PX: 파노라마 뷰 너비 (픽셀)
    PANO_H_PX: 파노라마 뷰 높이 (픽셀)
    K_H: 시각적 크기 스케일링 상수
    H_MIN, H_MAX: 바운딩 박스 높이 제한
    W_MIN, W_MAX: 바운딩 박스 너비 제한
"""

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

# 파노라마 뷰 상수 (사양 B-3 기준)
PANO_W_PX = 1920  # 파노라마 이미지 너비 (픽셀)
PANO_H_PX = 320   # 파노라마 이미지 높이 (픽셀)
K_H = 2200        # 시각적 크기 스케일링 상수
H_MIN, H_MAX = 6, 220   # 바운딩 박스 높이 최소/최대값 (픽셀)
W_MIN, W_MAX = 6, 300   # 바운딩 박스 너비 최소/최대값 (픽셀)
EPS = 1e-3        # 0 나눗셈 방지용 작은 값
CY_RATIO = 0.60   # 수평선 위치 비율 (cy_px = 0.60 * pano_h_px)


def wrap_to_180(deg):
    """
    각도를 [-180, 180] 범위로 정규화합니다.

    상대 방위각 계산에 사용됩니다.

    매개변수:
        deg: 정규화할 각도 (도)

    반환값:
        -180 ~ 180 범위의 각도
    """
    return ((deg + 180) % 360) - 180


class SimulationWorker(QObject):
    """
    해상 신호 시뮬레이션을 수행하는 메인 워커 클래스입니다.

    대권항법(Great Circle)과 벡터 모드를 지원하여 선박의 위치를 실시간으로
    업데이트하고, AIS/레이더/카메라 신호를 생성합니다. 이벤트 시스템을 통해
    조건부 시나리오를 처리합니다.

    시그널:
        signal_generated(str): NMEA 메시지 생성 시 전달
        log_message(str): 로그 메시지 전달
        performance_updated(str): 성능 정보 (SPS) 전달
        positions_updated(dict): 선박 위치 업데이트 전달
        export_data_ready(bool): 내보내기 데이터 준비 완료
        finished(): 시뮬레이션 종료
        random_targets_generated(): 랜덤 타겟 생성 완료
        event_triggered(str, int): 이벤트 트리거 시 (이벤트명, 선박idx)
        camera_detections_updated(list): 카메라 탐지 데이터 업데이트

    이동 모드:
        Rail Mode: 경로를 따라 대권항법으로 이동 (following_path=True)
        Vector Mode: 현재 방향과 속도로 자유 항해 (following_path=False)
    """

    # NMEA 메시지 생성 시 발생하는 시그널
    signal_generated = pyqtSignal(str)

    # 로그 메시지 시그널
    log_message = pyqtSignal(str)

    # 성능 정보 (SPS: Steps Per Second) 시그널
    performance_updated = pyqtSignal(str)

    # 선박 위치 업데이트 시그널 (dict: {ship_idx: (px, py, heading, sog)})
    positions_updated = pyqtSignal(dict)

    # 내보내기 데이터 준비 완료 시그널
    export_data_ready = pyqtSignal(bool)

    # 시뮬레이션 종료 시그널
    finished = pyqtSignal()

    # 랜덤 타겟 생성 완료 시그널
    random_targets_generated = pyqtSignal()

    # 이벤트 트리거 시그널 (이벤트명, 대상 선박 idx)
    event_triggered = pyqtSignal(str, int)

    # 카메라 탐지 데이터 업데이트 시그널 (파노라마 뷰용)
    camera_detections_updated = pyqtSignal(list)

    def __init__(self, ip, port, speed_mult, duration_sec):
        """
        SimulationWorker를 초기화합니다.

        매개변수:
            ip: UDP 전송 대상 IP 주소
            port: UDP 전송 대상 포트
            speed_mult: 시뮬레이션 속도 배율
            duration_sec: 시뮬레이션 총 시간 (초)
        """
        super().__init__()
        self.ip = ip                      # UDP 전송 IP
        self.port = port                  # UDP 전송 포트
        self.speed_mult = speed_mult      # 속도 배율
        self.duration_sec = duration_sec  # 총 시뮬레이션 시간
        self.running = True               # 실행 상태 플래그
        self.paused = False               # 일시정지 상태 플래그
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP 소켓
        self.proj = current_project       # 현재 프로젝트 참조
        self.temp_file_handle = None      # 임시 파일 핸들 (신호 로깅용)
        self.last_gui_update_time = 0.0   # 마지막 GUI 업데이트 시간
        self.last_log_update_time = 0.0   # 마지막 로그 업데이트 시간
        self.last_emission_times = {}     # 선박별 신호 타입별 마지막 발신 시간
        self.last_sps_update_time = 0.0   # 마지막 SPS 계산 시간
        self.last_sps_update_m = 0        # 마지막 SPS 계산 시 스텝 번호
        self.m = 0                        # 현재 스텝 번호
        self.current_time = 0.0           # 현재 시뮬레이션 시간
        self.active_ships = []            # 활성 선박 목록
        self.triggered_events = set()     # 트리거된 이벤트 ID 집합
        self.dynamic_ships = {}           # 동적 선박 상태 딕셔너리
        self.events = []                  # 활성 이벤트 목록
        self.max_steps = 0                # 최대 스텝 수

        # 카메라 버스트 손실 상태 추적 (사양 B-5)
        # {ship_idx: {'in_burst': bool, 'end_time': float}}
        self.camera_burst_states = {}

        # 현재 프레임의 카메라 탐지 데이터 (파노라마 뷰용)
        self.camera_detections = []

    def reset_triggered_event(self, event_id: str):
        """
        트리거된 이벤트를 초기화하여 재평가를 허용합니다.

        이벤트가 수정되었을 때 다시 평가할 수 있도록 합니다.

        매개변수:
            event_id: 초기화할 이벤트 ID
        """
        self.triggered_events.discard(event_id)

    def _densify_path(self, raw_points, mi, max_segment_m=100.0):
        """
        제어점을 대권 경로로 연결하여 경로를 생성합니다.

        짧은 세그먼트로 분할하여 대권항법 보간을 수행합니다.

        매개변수:
            raw_points: 원본 제어점 목록 [(px, py), ...]
            mi: 맵 정보 객체
            max_segment_m: 최대 세그먼트 길이 (미터)

        반환값:
            (dense_ecef, cumulative_dists, total_len) 튜플
            - dense_ecef: ECEF 좌표 목록
            - cumulative_dists: 누적 거리 목록
            - total_len: 총 경로 길이
        """
        if not raw_points or len(raw_points) < 2:
            ecef_pts = path_points_to_ecef(raw_points, mi)
            return ecef_pts, [0.0] * len(ecef_pts), 0.0

        dense_ecef = []

        # 1. 픽셀 -> 위경도 변환 (제어점)
        lla_points = []
        for px, py in raw_points:
            _, _, lat, lon = pixel_to_coords(px, py, mi)
            lla_points.append((lat, lon))

        # 2. 제어점 간 대권 보간
        for i in range(len(lla_points) - 1):
            lat1, lon1 = lla_points[i]
            lat2, lon2 = lla_points[i+1]

            x1, y1, z1 = lla_to_ecef(lat1, lon1, 0.0)
            dense_ecef.append((x1, y1, z1))
            dist_m = vincenty_distance(lat1, lon1, lat2, lon2)

            # 도착점 ECEF 변환
            x2, y2, z2 = lla_to_ecef(lat2, lon2, 0.0)

            # 대권 거리 계산
            dist_m = vincenty_distance(lat1, lon1, lat2, lon2)

            # 세그먼트가 길면 대권을 따라 더 작은 세그먼트로 분할
            if dist_m > max_segment_m:
                num_steps = int(math.ceil(dist_m / max_segment_m))
                for s in range(1, num_steps):
                    frac = s / num_steps

                    # ECEF Slerp (지구 중심을 통한 현 보간)
                    ix, iy, iz = ecef_interpolate(x1, y1, z1, x2, y2, z2, frac)

                    # 표면으로 투영 (고도 0m 보정) -> 대권 상의 점이 됨
                    t_lat, t_lon, _ = ecef_to_lla(ix, iy, iz)
                    dx, dy, dz = lla_to_ecef(t_lat, t_lon, 0.0)

                    dense_ecef.append((dx, dy, dz))

        # 마지막 제어점 추가
        last_lat, last_lon = lla_points[-1]
        lx, ly, lz = lla_to_ecef(last_lat, last_lon, 0.0)
        dense_ecef.append((lx, ly, lz))

        # 3. 누적 거리 계산 (Rail Logic에 필요)
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
        """
        시뮬레이션 메인 루프를 실행합니다.

        각 시간 스텝마다:
        1. 이벤트 체크 및 처리
        2. 선박 위치 업데이트
        3. NMEA 신호 생성 및 전송
        4. GUI 업데이트
        """
        try:
            self.log_message.emit(f"Starting Simulation UDP->{self.ip}:{self.port} Speed={self.speed_mult}x")
            self.temp_file_handle = tempfile.TemporaryFile(mode='w+', encoding='utf-8')

            # 시드 설정 (재현 가능한 시뮬레이션)
            seed_val = self.proj.seed % (2**32 - 1)
            random.seed(seed_val)
            np.random.seed(seed_val)

            # 경로가 있는 활성 선박 필터링
            self.active_ships = [s for s in self.proj.ships if s.raw_points]
            if not self.active_ships:
                self.log_message.emit("No active ships with paths.")
                self.finished.emit()
                return

            self.dynamic_ships = {}
            mi = self.proj.map_info

            # 각 선박의 동적 상태 초기화
            for s in self.active_ships:
                s_seed = (self.proj.seed + s.idx) % (2**32 - 1)
                s_rng = random.Random(s_seed)

                # 시작 위치 설정
                start_px, start_py = s.raw_points[0]
                _, _, start_lat, start_lon = pixel_to_coords(start_px, start_py, mi)
                x, y, z = lla_to_ecef(start_lat, start_lon, 0.0)

                # 랜덤 선박 (idx >= 1000)은 경로 생성 없이 벡터 모드 사용 (최적화)
                if s.idx >= 1000:
                    # 랜덤 초기 방향 (0 ~ 360도)
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
                        'following_path': False,  # 벡터 모드
                        'manual_speed': False,
                        'manual_heading': False,
                        'mode': None,
                        'rng': s_rng,
                        'target_recovery_idx': -1,
                        'target_dest_ecef': None
                    }
                else:
                    # 일반 선박: 제어점에서 대권 경로 생성
                    points = s.raw_points
                    ecef_path, cum_dists, total_len = self._densify_path(points, mi)

                    # 초기 방향 설정
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

            # 활성 이벤트 로드
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

            # 메인 시뮬레이션 루프
            while self.running:
                self.m = m
                QCoreApplication.processEvents()

                # 일시정지 처리
                if self.paused:
                    time.sleep(0.1)
                    continue

                # 이벤트 목록 새로고침 (시나리오 포함)
                try:
                    active_events = [e for e in self.proj.events if e.enabled]
                    from app.core.state import loaded_scenarios
                    for scen in loaded_scenarios:
                        if not scen.enabled:
                            continue
                        for evt in scen.events:
                            if not evt.enabled:
                                continue
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
                                if not any(e.id == evt.id for e in active_events):
                                    active_events.append(evt)
                    self.events = active_events
                except Exception as e:
                    self.log_message.emit(f"[Warning] Error refreshing events: {e}")

                loop_start = time.time()
                t_m = m * dT
                self.current_time = t_m
                ts_val = self.proj.start_time + datetime.timedelta(seconds=t_m)

                # 이벤트 체크
                self.check_events(t_m)

                # 이번 프레임의 카메라 탐지 데이터 초기화
                self.camera_detections = []

                current_positions = {}

                # OwnShip 상태 업데이트
                own_state = None
                if own_ship and own_ship.raw_points:
                    own_state = self.update_and_get_state(own_ship, dT)
                    px, py = coords_to_pixel(own_state['lat_deg'], own_state['lon_deg'], mi)
                    current_positions[own_idx] = (px, py, own_state['heading_true_deg'], own_state['sog_knots'])

                # 타겟 선박 상태 업데이트
                for tgt in self.active_ships:
                    if tgt.idx == own_idx:
                        continue
                    tgt_state = self.update_and_get_state(tgt, dT)
                    px, py = coords_to_pixel(tgt_state['lat_deg'], tgt_state['lon_deg'], mi)
                    current_positions[tgt.idx] = (px, py, tgt_state['heading_true_deg'], tgt_state['sog_knots'])
                    if own_ship and own_state:
                        self.generate_ais_radar_group(own_ship, tgt, tgt_state, ts_val)

                # GUI 업데이트 (스로틀링: 50ms 간격)
                current_real_time = time.time()
                if current_real_time - self.last_gui_update_time > 0.05:
                    self.positions_updated.emit(current_positions)
                    # 카메라 탐지 데이터 전송 (C-3: GUI 업데이트와 함께 스로틀링)
                    if self.camera_detections:
                        self.camera_detections_updated.emit(list(self.camera_detections))
                    self.last_gui_update_time = current_real_time

                m += 1

                # 시뮬레이션 속도 제어
                step_delay = dT / self.speed_mult
                current_loop_elapsed = time.time() - loop_start
                sleep_time = step_delay - current_loop_elapsed

                # SPS (Steps Per Second) 계산 및 표시
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

                # 대기 시간 조절
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
        """
        활성 선박 목록을 새로고침합니다.

        새로 추가된 선박(주로 랜덤 타겟)을 동적 선박 목록에 추가합니다.
        """
        current_indices = {s.idx for s in self.active_ships}
        new_ships = [s for s in self.proj.ships if s.raw_points and s.idx not in current_indices]

        mi = self.proj.map_info
        for s in new_ships:
            s_seed = (self.proj.seed + s.idx) % (2**32 - 1)
            s_rng = random.Random(s_seed)
            points = s.raw_points
            if not points:
                continue

            # 랜덤 선박 최적화: densify_path 호출 제거, 벡터 모드로 설정
            # 시작점만 추출하여 ECEF로 변환
            start_px, start_py = points[0]
            _, _, lat, lon = pixel_to_coords(start_px, start_py, mi)
            x, y, z = lla_to_ecef(lat, lon, 0.0)

            # 랜덤 방향 생성 (0 ~ 360도)
            init_hdg = s_rng.uniform(0.0, 360.0)

            self.dynamic_ships[s.idx] = {
                'x': x, 'y': y, 'z': z,
                'spd': s.raw_speeds.get(0, 5.0),
                'sog': 5.0,
                'hdg': init_hdg,
                'dist_traveled': 0.0,
                'path_segment_idx': 0,
                'ecef_path': [],        # 경로 비움
                'cum_dists': [],
                'total_path_len': 0.0,
                'following_path': False,  # 경로 추종 비활성화 (벡터 모드)
                'manual_speed': False,
                'manual_heading': False,
                'mode': None,
                'rng': s_rng,
                'target_recovery_idx': -1,
                'target_dest_ecef': None
            }
            self.active_ships.append(s)

    def send_nmea(self, raw, ts_obj):
        """
        NMEA 메시지를 UDP로 전송하고 로그에 기록합니다.

        매개변수:
            raw: NMEA 메시지 문자열
            ts_obj: 타임스탬프 datetime 객체
        """
        # 임시 파일에 기록
        if self.temp_file_handle:
            try:
                line = f"{ts_obj.timestamp()},{raw}\n"
                self.temp_file_handle.write(line)
            except:
                pass

        # UDP 전송
        try:
            self.sock.sendto((raw + "\r\n").encode('ascii'), (self.ip, self.port))
        except:
            pass

        # 로그 업데이트 (스로틀링: 200ms 간격)
        cur_t = time.time()
        if cur_t - self.last_log_update_time > 0.2:
            self.signal_generated.emit(raw)
            self.last_log_update_time = cur_t

    def stop(self):
        """시뮬레이션을 중지합니다."""
        self.running = False

    def pause(self):
        """시뮬레이션을 일시정지합니다."""
        self.paused = True

    def resume(self):
        """시뮬레이션을 재개합니다."""
        self.paused = False

    def update_speed(self, mult):
        """
        시뮬레이션 속도 배율을 업데이트합니다.

        매개변수:
            mult: 새 속도 배율
        """
        self.speed_mult = mult

    def update_duration(self, new_duration):
        """
        시뮬레이션 총 시간을 업데이트합니다.

        매개변수:
            new_duration: 새 시뮬레이션 시간 (초)
        """
        self.max_steps = math.ceil(new_duration / self.proj.unit_time)

    def set_ship_speed(self, ship_idx, speed_kn):
        """
        특정 선박의 속도를 수동으로 설정합니다.

        매개변수:
            ship_idx: 선박 인덱스
            speed_kn: 새 속도 (노트)
        """
        if ship_idx in self.dynamic_ships:
            self.dynamic_ships[ship_idx]['manual_speed'] = True
            self.dynamic_ships[ship_idx]['spd'] = speed_kn
            ship = self.proj.get_ship_by_idx(ship_idx)
            self.log_message.emit(f"[Manual] Speed changed for {ship.name if ship else ship_idx} to {speed_kn} kn")

    def set_ship_heading(self, ship_idx, heading_deg):
        """
        특정 선박의 방향을 수동으로 설정합니다.

        경로 추종을 비활성화하고 벡터 모드로 전환합니다.

        매개변수:
            ship_idx: 선박 인덱스
            heading_deg: 새 방향 (도)
        """
        if ship_idx in self.dynamic_ships:
            self.dynamic_ships[ship_idx]['hdg'] = heading_deg
            self.dynamic_ships[ship_idx]['manual_heading'] = True
            self.dynamic_ships[ship_idx]['following_path'] = False
            ship = self.proj.get_ship_by_idx(ship_idx)
            self.log_message.emit(f"[Manual] Heading changed for {ship.name if ship else ship_idx} to {heading_deg} deg")

    def _calculate_ecef_velocity(self, lat, lon, spd_kn, hdg_deg):
        """
        ECEF 좌표계에서 속도 벡터를 계산합니다.

        매개변수:
            lat: 위도 (도)
            lon: 경도 (도)
            spd_kn: 속도 (노트)
            hdg_deg: 방향 (도)

        반환값:
            (v_x, v_y, v_z) ECEF 속도 벡터 (m/s)
        """
        spd_mps = spd_kn * 0.514444
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        hdg_rad = math.radians(hdg_deg)

        # ECEF에서 로컬 NED 단위 벡터
        # North 방향
        n_x = -math.sin(lat_rad) * math.cos(lon_rad)
        n_y = -math.sin(lat_rad) * math.sin(lon_rad)
        n_z = math.cos(lat_rad)

        # East 방향
        e_x = -math.sin(lon_rad)
        e_y = math.cos(lon_rad)
        e_z = 0.0

        # 방향 벡터 합성
        v_x = (n_x * math.cos(hdg_rad) + e_x * math.sin(hdg_rad)) * spd_mps
        v_y = (n_y * math.cos(hdg_rad) + e_y * math.sin(hdg_rad)) * spd_mps
        v_z = (n_z * math.cos(hdg_rad) + e_z * math.sin(hdg_rad)) * spd_mps

        return v_x, v_y, v_z

    def _ecef_dist_to_geodesic_nm(self, d_ecef):
        """
        ECEF 현(chord) 거리를 측지 호(arc) 거리로 변환합니다.

        매개변수:
            d_ecef: ECEF 좌표계 거리 (미터)

        반환값:
            측지 거리 (해리, nautical miles)
        """
        R = 6371000.0  # 지구 반지름 (미터)
        if d_ecef >= 2 * R:
            return math.pi * R / 1852.0

        val = d_ecef / (2.0 * R)
        if val > 1.0:
            val = 1.0
        theta = 2.0 * math.asin(val)

        dist_m = R * theta
        return dist_m / 1852.0

    def _check_prerequisites(self, evt) -> bool:
        """
        이벤트의 선행 조건을 체크합니다.

        매개변수:
            evt: 이벤트 객체

        반환값:
            모든 선행 조건이 충족되면 True
        """
        prereqs = getattr(evt, 'prerequisite_events', [])
        if not prereqs:
            return True  # 조건이 없으면 통과

        logic = getattr(evt, 'prerequisite_logic', 'AND')
        results = []

        for cond in prereqs:
            # cond는 EventCondition 객체 또는 dict일 수 있음
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
        """
        현재 시간에 대해 모든 이벤트를 체크하고 트리거합니다.

        지원하는 트리거 타입:
        - TIME: 특정 시간 도달
        - AREA_ENTER: 영역 진입
        - AREA_LEAVE: 영역 이탈
        - DIST_UNDER/OVER: 거리 조건
        - CPA_UNDER/OVER: 최근접점 거리 조건

        매개변수:
            t_current: 현재 시뮬레이션 시간 (초)
        """
        for evt in self.events:
            if evt.id in self.triggered_events:
                continue

            # 선행 이벤트 조건 체크
            if not self._check_prerequisites(evt):
                continue

            triggered = False

            if evt.trigger_type == "TIME":
                if t_current >= evt.condition_value:
                    triggered = True

            elif evt.trigger_type == "AREA_ENTER":
                ship = self.proj.get_ship_by_idx(evt.target_ship_idx)
                area = self.proj.get_area_by_id(int(evt.condition_value))
                if ship and area and ship.idx in self.dynamic_ships:
                    dyn = self.dynamic_ships[ship.idx]
                    lat, lon, _ = ecef_to_lla(dyn['x'], dyn['y'], dyn['z'])
                    px, py = coords_to_pixel(lat, lon, self.proj.map_info)
                    poly = QPolygonF([QPointF(x, y) for x, y in area.geometry])
                    if poly.containsPoint(QPointF(px, py), Qt.FillRule.OddEvenFill):
                        triggered = True

            elif evt.trigger_type == "AREA_LEAVE":
                ship = self.proj.get_ship_by_idx(evt.target_ship_idx)
                area = self.proj.get_area_by_id(int(evt.condition_value))
                if ship and area and ship.idx in self.dynamic_ships:
                    dyn = self.dynamic_ships[ship.idx]
                    lat, lon, _ = ecef_to_lla(dyn['x'], dyn['y'], dyn['z'])
                    px, py = coords_to_pixel(lat, lon, self.proj.map_info)
                    poly = QPolygonF([QPointF(x, y) for x, y in area.geometry])
                    if not poly.containsPoint(QPointF(px, py), Qt.FillRule.OddEvenFill):
                        triggered = True

            elif evt.trigger_type in ["CPA_UNDER", "CPA_OVER", "DIST_UNDER", "DIST_OVER"]:
                tgt_ship = self.proj.get_ship_by_idx(evt.target_ship_idx)
                ref_idx = evt.reference_ship_idx
                if ref_idx == -1:
                    ref_idx = self.proj.settings.own_ship_idx
                ref_ship = self.proj.get_ship_by_idx(ref_idx)

                if tgt_ship and ref_ship and tgt_ship.idx in self.dynamic_ships and ref_ship.idx in self.dynamic_ships:
                    td = self.dynamic_ships[tgt_ship.idx]
                    rd = self.dynamic_ships[ref_ship.idx]

                    # 1. 현재 거리 (Chord -> Arc 변환)
                    dist_chord_m = ecef_distance(td['x'], td['y'], td['z'], rd['x'], rd['y'], rd['z'])
                    dist_nm = self._ecef_dist_to_geodesic_nm(dist_chord_m)

                    if evt.trigger_type == "DIST_UNDER" and dist_nm <= evt.condition_value:
                        triggered = True
                    elif evt.trigger_type == "DIST_OVER" and dist_nm >= evt.condition_value:
                        triggered = True
                    elif evt.trigger_type in ["CPA_UNDER", "CPA_OVER"]:
                        # ECEF 벡터 연산으로 CPA 계산 (LLA 변환 없이)
                        # 현재 위치 벡터
                        P_t = np.array([td['x'], td['y'], td['z']])
                        P_r = np.array([rd['x'], rd['y'], rd['z']])

                        # 속도 벡터 계산
                        t_lat, t_lon, _ = ecef_to_lla(td['x'], td['y'], td['z'])
                        r_lat, r_lon, _ = ecef_to_lla(rd['x'], rd['y'], rd['z'])

                        tvx, tvy, tvz = self._calculate_ecef_velocity(t_lat, t_lon, td.get('sog', td['spd']), td['hdg'])
                        rvx, rvy, rvz = self._calculate_ecef_velocity(r_lat, r_lon, rd.get('sog', rd['spd']), rd['hdg'])

                        V_t = np.array([tvx, tvy, tvz])
                        V_r = np.array([rvx, rvy, rvz])

                        # 상대 위치와 상대 속도
                        P_rel = P_t - P_r
                        V_rel = V_t - V_r

                        v_rel_sq = np.dot(V_rel, V_rel)
                        cpa_dist_nm = dist_nm  # 기본값: 현재 거리

                        if v_rel_sq > 1e-9:
                            # t_cpa (초) = - (P_rel . V_rel) / |V_rel|^2
                            t_cpa = -np.dot(P_rel, V_rel) / v_rel_sq

                            if t_cpa > 0:
                                # 미래 위치 예측: 선형 등속 운동 가정 (ECEF 선형 투영)
                                P_t_future = P_t + V_t * t_cpa
                                P_r_future = P_r + V_r * t_cpa

                                future_dist_chord = np.linalg.norm(P_t_future - P_r_future)
                                cpa_dist_nm = self._ecef_dist_to_geodesic_nm(future_dist_chord)

                        if evt.trigger_type == "CPA_UNDER" and cpa_dist_nm <= evt.condition_value:
                            triggered = True
                        elif evt.trigger_type == "CPA_OVER" and cpa_dist_nm >= evt.condition_value:
                            triggered = True

            if triggered:
                self.log_message.emit(f"[EVENT] {evt.name} triggered.")
                self.event_triggered.emit(evt.name, evt.target_ship_idx)
                self.triggered_events.add(evt.id)
                self.apply_event_action(evt, t_current)

    def apply_event_action(self, evt, t_current):
        """
        트리거된 이벤트의 액션을 적용합니다.

        지원하는 액션 타입:
        - STOP: 선박 정지
        - CHANGE_SPEED: 속도 변경
        - CHANGE_HEADING: 방향 변경
        - CHANGE_DESTINATION_LOC: 목적지 변경
        - MANEUVER: 특수 기동 (경로 복귀 등)

        매개변수:
            evt: 이벤트 객체
            t_current: 현재 시뮬레이션 시간
        """
        sid = evt.target_ship_idx
        if sid not in self.dynamic_ships:
            return
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
                # 최단 거리로 원래 경로 복귀
                ecef_path = dyn.get('ecef_path', [])
                if ecef_path:
                    cx, cy, cz = dyn['x'], dyn['y'], dyn['z']
                    best_idx = 0
                    best_dist = float('inf')
                    # 트리거 시점에 일회성 검색
                    for i in range(len(ecef_path)):
                        px, py, pz = ecef_path[i]
                        d = ecef_distance(cx, cy, cz, px, py, pz)
                        if d < best_dist:
                            best_dist = d
                            best_idx = i

                    dyn['target_recovery_idx'] = best_idx
                    dyn['mode'] = 'RETURN_TO_PATH'

            elif opt == "ChangeDestination_ToOriginalFinal":
                # 원래 경로의 최종 목적지로 직행
                ecef_path = dyn.get('ecef_path', [])
                if ecef_path:
                    tx, ty, tz = ecef_path[-1]
                    dyn['hdg'] = ecef_heading(dyn['x'], dyn['y'], dyn['z'], tx, ty, tz)
                    dyn['following_path'] = False
                    dyn['manual_heading'] = True

    def update_and_get_state(self, ship, dT):
        """
        선박의 상태를 업데이트하고 현재 상태를 반환합니다.

        Rail Mode와 Vector Mode를 지원합니다.

        매개변수:
            ship: 선박 객체
            dT: 시간 간격 (초)

        반환값:
            상태 딕셔너리 (lat_deg, lon_deg, sog_knots, heading_true_deg 등)
        """
        # 동적 상태 초기화 (아직 없는 경우에만)
        if ship.idx not in self.dynamic_ships:
            mi = self.proj.map_info
            if ship.raw_points:
                px, py = ship.raw_points[0]
                _, _, lat, lon = pixel_to_coords(px, py, mi)
                x, y, z = lla_to_ecef(lat, lon, 0.0)
                # raw_points에서 대권 경로 생성
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

        # 1. 속도 계산 (노이즈 추가)
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

        # GO_TO_COORD 모드 처리
        if dyn.get('mode') == 'GO_TO_COORD' and dyn.get('target_dest_ecef'):
            tx, ty, tz = dyn['target_dest_ecef']
            curr_dist = ecef_distance(dyn['x'], dyn['y'], dyn['z'], tx, ty, tz)

            if curr_dist <= move_dist_m:
                # 목적지 도착
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

            return self._make_state_result(0, 0, 0, 0) if dyn['spd'] == 0 else \
                   self._make_state_result(*ecef_to_lla(dyn['x'], dyn['y'], dyn['z'])[:2], current_speed_kn, dyn['hdg'])

        # RETURN_TO_PATH 모드 처리
        if dyn.get('mode') == 'RETURN_TO_PATH' and ecef_path:
            best_idx = dyn.get('target_recovery_idx', 0)
            if best_idx < 0 or best_idx >= len(ecef_path):
                best_idx = 0

            px, py, pz = ecef_path[best_idx]
            curr_dist = ecef_distance(dyn['x'], dyn['y'], dyn['z'], px, py, pz)

            if curr_dist <= move_dist_m * 1.5:
                # 경로 복귀 완료
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
            # --- RAIL MODE (대권항법 경로 추종) ---
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

            # 위치 업데이트: 대권 보간 위치
            dyn['x'], dyn['y'], dyn['z'] = ecef_interpolate(x1, y1, z1, x2, y2, z2, frac)

            # 방향 업데이트: 현재 위치에서 다음 경로점까지의 대권 방위각
            dyn['hdg'] = ecef_heading(dyn['x'], dyn['y'], dyn['z'], x2, y2, z2)

        else:
            # --- VECTOR MODE (대권항법 자유 항해) ---
            if move_dist_m > 0:
                lat, lon, _ = ecef_to_lla(dyn['x'], dyn['y'], dyn['z'])
                # 현재 방향으로 대권 이동 -> 위치 변화에 따라 방향 자동 업데이트
                nlat, nlon, nhdg = self._move_great_circle_step(lat, lon, dyn['hdg'], move_dist_m)

                dyn['x'], dyn['y'], dyn['z'] = lla_to_ecef(nlat, nlon, 0.0)
                dyn['hdg'] = nhdg

        # 4. 결과 반환
        lat, lon, _ = ecef_to_lla(dyn['x'], dyn['y'], dyn['z'])

        # 극지방 클리핑
        if lat > 89.9:
            lat = 89.9
        elif lat < -89.9:
            lat = -89.9
        lon = normalize_lon(lon)

        return self._make_state_result(lat, lon, current_speed_kn, dyn['hdg'])

    def _move_great_circle_step(self, lat, lon, hdg_deg, dist_m):
        """
        현재 위치에서 대권항법으로 이동하고 새 위치와 방위각을 반환합니다.

        매개변수:
            lat: 현재 위도 (도)
            lon: 현재 경도 (도)
            hdg_deg: 현재 방향 (도)
            dist_m: 이동 거리 (미터)

        반환값:
            (new_lat, new_lon, new_hdg) 튜플
        """
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        hdg_rad = math.radians(hdg_deg)
        R = 6371000.0  # 지구 반지름
        ang_dist = dist_m / R

        # 새 위치 계산
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

        # 새 방향 계산 (Final Bearing + 180 전략)
        y = math.sin(lon_rad - new_lon_rad) * math.cos(lat_rad)
        x = math.cos(new_lat_rad) * math.sin(lat_rad) - \
            math.sin(new_lat_rad) * math.cos(lat_rad) * math.cos(lon_rad - new_lon_rad)

        back_bearing_rad = math.atan2(y, x)
        back_bearing_deg = math.degrees(back_bearing_rad)

        new_hdg = (back_bearing_deg + 180) % 360

        return new_lat, new_lon, new_hdg

    def _make_state_result(self, lat, lon, sog, cog):
        """
        선박 상태 결과 딕셔너리를 생성합니다.

        매개변수:
            lat: 위도 (도)
            lon: 경도 (도)
            sog: 속력 (노트)
            cog: 침로 (도)

        반환값:
            상태 딕셔너리
        """
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
        """
        특정 선박의 현재 상태를 반환합니다.

        매개변수:
            ship: 선박 객체

        반환값:
            상태 딕셔너리
        """
        if ship.idx in self.dynamic_ships:
            dyn = self.dynamic_ships[ship.idx]
            lat, lon, _ = ecef_to_lla(dyn['x'], dyn['y'], dyn['z'])
            sog = dyn.get('sog', dyn['spd'])
            return self._make_state_result(lat, lon, sog, dyn['hdg'])
        return self._make_state_result(0, 0, 0, 0)

    def check_dropout(self, stype):
        """
        고정 확률 기반 드롭아웃 체크를 수행합니다.

        매개변수:
            stype: 신호 타입 ('AIVDM', 'RATTM', 'Camera')

        반환값:
            드롭아웃이면 True
        """
        prob = self.proj.settings.dropout_probs.get(stype, 0.1)
        return random.random() < prob

    def calculate_dropout_probability(self, distance_nm: float, config) -> float:
        """
        거리 기반 드롭아웃 확률을 계산합니다.

        수신 모델 설정에 따라 거리별 드롭아웃 확률을 계산합니다.

        매개변수:
            distance_nm: 거리 (해리)
            config: 수신 모델 설정 객체

        반환값:
            드롭아웃 확률 (0.0 ~ 1.0)
        """
        if not config.enabled:
            return 0.0

        d, d0, d1, p0, p1 = distance_nm, config.d0, config.d1, config.p0, config.p1

        if d <= d0:
            return p0  # 안정 구간
        elif d >= d1:
            return 1.0 if config.full_block_at_d1 else p1  # 완전 차단 또는 p1
        else:
            # d0 ~ d1 구간: 커브 보간
            t = (d - d0) / (d1 - d0)  # 0 ~ 1 정규화

            if config.curve_type == "linear":
                factor = t
            elif config.curve_type == "logistic":
                factor = 1 / (1 + math.exp(-10 * (t - 0.5)))
            else:  # smoothstep (기본값)
                factor = t * t * (3 - 2 * t)

            return p0 + (p1 - p0) * factor

    def check_dropout_distance_based(self, stype: str, distance_nm: float) -> bool:
        """
        거리 기반 드롭아웃 판정을 수행합니다.

        매개변수:
            stype: 신호 타입
            distance_nm: 거리 (해리)

        반환값:
            드롭아웃이면 True
        """
        if not self.proj.settings.reception_model_enabled:
            # 기존 고정 확률 방식으로 폴백
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
        """
        타임스탬프에 지터를 추가합니다.

        매개변수:
            base_ts: 기본 타임스탬프

        반환값:
            지터가 적용된 타임스탬프
        """
        if self.proj.settings.jitter_enabled:
            offset = random.choice([-2, -1, 0, 1, 2])
            return base_ts + datetime.timedelta(seconds=offset)
        return base_ts

    def generate_ais_radar_group(self, own, tgt, state, ts_val):
        """
        AIS/레이더/카메라 신호 그룹을 생성합니다.

        매개변수:
            own: OwnShip 객체
            tgt: 타겟 선박 객체
            state: 타겟 선박 상태 딕셔너리
            ts_val: 타임스탬프
        """
        jitter_ts = self.get_jittered_time(ts_val)
        base = self.make_base_row(own, tgt, state, jitter_ts)

        # 두 선박 간 거리 계산 (거리 기반 수신 모델용)
        own_dyn = self.dynamic_ships.get(own.idx)
        tgt_dyn = self.dynamic_ships.get(tgt.idx)
        if own_dyn and tgt_dyn:
            dist_m = ecef_distance(own_dyn['x'], own_dyn['y'], own_dyn['z'],
                                   tgt_dyn['x'], tgt_dyn['y'], tgt_dyn['z'])
            dist_nm = self._ecef_dist_to_geodesic_nm(dist_m)
        else:
            dist_nm = 0.0

        # AIS 신호 생성
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
                        for msg in ais_msgs:
                            self.send_nmea(msg, jitter_ts)
                        self.last_emission_times[(ship.idx, stype)] = current_sim_time

        # 레이더 TTM 신호 생성
        self.try_emit_distance_based(self.gen_ttm(base, state), "RATTM", jitter_ts, tgt.idx, dist_nm)

        # 카메라 신호 생성 (사양 B-1)
        if ship and own_dyn and tgt_dyn:
            if ship.signals_enabled.get("Camera", True):
                cam_raw, detection = self.gen_cam_signal(own_dyn, tgt, tgt_dyn, dist_nm)
                if cam_raw and detection:
                    current_sim_time = (jitter_ts - self.proj.start_time).total_seconds()
                    # 인터벌 체크
                    interval = ship.signal_intervals.get("Camera", 1.0)
                    last_time = self.last_emission_times.get((ship.idx, "Camera"), -float('inf'))
                    if (current_sim_time + 1e-9) >= (last_time + interval):
                        # 버스트 지원 드롭아웃 체크 (B-4, B-5)
                        if not self.check_camera_dropout_with_burst(tgt.idx, dist_nm, current_sim_time):
                            self.send_nmea(cam_raw, jitter_ts)
                            self.last_emission_times[(ship.idx, "Camera")] = current_sim_time
                            # 파노라마 뷰용 탐지 데이터 추가
                            self.camera_detections.append(detection)

    def try_emit(self, raw, stype, ts_obj, ship_idx):
        """
        NMEA 신호 발신을 시도합니다 (고정 확률 드롭아웃).

        매개변수:
            raw: NMEA 메시지 문자열
            stype: 신호 타입
            ts_obj: 타임스탬프
            ship_idx: 선박 인덱스
        """
        ship = self.proj.get_ship_by_idx(ship_idx)
        if not ship:
            return
        if not ship.signals_enabled.get(stype, True):
            return
        interval = ship.signal_intervals.get(stype, 1.0)
        current_sim_time = (ts_obj - self.proj.start_time).total_seconds()
        last_time = self.last_emission_times.get((ship_idx, stype), -float('inf'))
        if (current_sim_time + 1e-9) < (last_time + interval):
            return
        if not raw:
            return
        self.last_emission_times[(ship_idx, stype)] = current_sim_time
        if self.check_dropout(stype):
            return
        self.send_nmea(raw, ts_obj)

    def try_emit_distance_based(self, raw, stype, ts_obj, ship_idx, dist_nm):
        """
        거리 기반 드롭아웃이 적용된 NMEA 신호 발신을 시도합니다.

        매개변수:
            raw: NMEA 메시지 문자열
            stype: 신호 타입
            ts_obj: 타임스탬프
            ship_idx: 선박 인덱스
            dist_nm: 거리 (해리)
        """
        ship = self.proj.get_ship_by_idx(ship_idx)
        if not ship:
            return
        if not ship.signals_enabled.get(stype, True):
            return
        interval = ship.signal_intervals.get(stype, 1.0)
        current_sim_time = (ts_obj - self.proj.start_time).total_seconds()
        last_time = self.last_emission_times.get((ship_idx, stype), -float('inf'))
        if (current_sim_time + 1e-9) < (last_time + interval):
            return
        if not raw:
            return
        self.last_emission_times[(ship_idx, stype)] = current_sim_time
        if self.check_dropout_distance_based(stype, dist_nm):
            return
        self.send_nmea(raw, ts_obj)

    def make_base_row(self, rcv, tgt, state, ts_val):
        """
        기본 행 데이터를 생성합니다.

        매개변수:
            rcv: 수신 선박 객체
            tgt: 타겟 선박 객체
            state: 상태 딕셔너리
            ts_val: 타임스탬프

        반환값:
            기본 행 딕셔너리
        """
        return {"tgt_ship_index": tgt.idx, "tgt_ship_name": tgt.name, "rx_time": ts_val}

    def gen_ais_multi(self, base, state, stype, mmsi):
        """
        AIS 메시지를 생성합니다 (멀티 프래그먼트 지원).

        매개변수:
            base: 기본 행 데이터
            state: 상태 딕셔너리
            stype: AIS 타입 (VDM 등)
            mmsi: MMSI 번호

        반환값:
            AIS 메시지 목록
        """
        probs = current_project.settings.ais_fragment_probs
        total_prob = sum(probs)
        if total_prob == 0:
            total_fragments = 1
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
            fragment = payload[start_idx:end_idx]
            raw = f"!AI{stype},{total_fragments},{i+1},{seq_id},A,{fragment},0"
            msg = f"{raw}*{calculate_checksum(raw[1:])}"
            ais_messages.append(msg)

        return ais_messages

    def gen_ttm(self, base, state):
        """
        TTM (Tracked Target Message) NMEA 메시지를 생성합니다.

        매개변수:
            base: 기본 행 데이터
            state: 상태 딕셔너리

        반환값:
            TTM NMEA 메시지 문자열
        """
        tgt_num = base['tgt_ship_index']
        dt = base['rx_time']
        utc_time = dt.strftime("%H%M%S.%f")[:9]
        raw = f"$RATTM,{tgt_num:02d},1.5,45.0,T,{state['sog_knots']:.1f},{state['cog_true_deg']:.1f},T,0.5,10.0,N,b,T,,{utc_time},A"
        return f"{raw}*{calculate_checksum(raw[1:])}"

    def gen_cam_signal(self, own_dyn, tgt_ship, tgt_dyn, dist_nm):
        """
        카메라 탐지 NMEA 메시지를 생성합니다.

        선박을 3D 박스(length x beam x height)로 모델링합니다.
        파노라마에서의 가시 너비는 관찰자의 시선 방향과 타겟의 방향 사이의
        각도에 따라 어떤 면이 보이는지에 의해 결정됩니다.

        매개변수:
            own_dyn: OwnShip 동적 상태
            tgt_ship: 타겟 선박 객체
            tgt_dyn: 타겟 선박 동적 상태
            dist_nm: 거리 (해리)

        반환값:
            (raw_nmea, detection_dict) 튜플 또는 FOV 밖이면 (None, None)
        """
        # OwnShip에서 타겟까지의 진방위 계산
        bearing_true = ecef_heading(own_dyn['x'], own_dyn['y'], own_dyn['z'],
                                    tgt_dyn['x'], tgt_dyn['y'], tgt_dyn['z'])
        own_heading = own_dyn['hdg']

        # 상대 방위각 계산 (사양 A-2)
        rel_bearing = wrap_to_180(bearing_true - own_heading)

        # FOV 체크 [-90, +90] (사양 A-2)
        if rel_bearing < -90 or rel_bearing > 90:
            return None, None

        # 파노라마 x 좌표 계산
        x_norm = (rel_bearing + 90.0) / 180.0
        cx_px = x_norm * PANO_W_PX
        cy_px = CY_RATIO * PANO_H_PX

        # 선박 치수
        range_m = dist_nm * 1852.0
        length_m = getattr(tgt_ship, 'length_m', 300.0)
        beam_m = getattr(tgt_ship, 'beam_m', 40.0)
        height_m = getattr(tgt_ship, 'height_m', 30.0)  # 수면 위 선박 높이

        # 타겟 선박의 방향에 대한 시야각 계산
        # 어떤 면이 보이는지 결정
        tgt_heading = tgt_dyn['hdg']

        # 타겟 선박의 선수에서 관찰자까지의 각도 (bearing_true의 역방향)
        # bearing_true: 관찰자에서 타겟으로의 방향
        # 필요한 것: 타겟에서 관찰자로의 방향, 타겟의 방향에 대한 상대값
        bearing_from_target = (bearing_true + 180.0) % 360.0
        view_angle = wrap_to_180(bearing_from_target - tgt_heading)

        # view_angle 해석:
        # 0° = 선수에서 보는 중 (전면: beam x height)
        # 90° = 우현에서 보는 중 (측면: length x height)
        # 180° 또는 -180° = 선미에서 보는 중 (후면: beam x height)
        # -90° = 좌현에서 보는 중 (측면: length x height)

        # 시야각에 따른 투영 너비 계산
        # 가시 너비는 length와 beam 면의 조합
        view_angle_rad = math.radians(view_angle)
        # |sin|은 측면(length) 기여, |cos|는 전후면(beam) 기여
        projected_width_m = abs(math.sin(view_angle_rad)) * length_m + abs(math.cos(view_angle_rad)) * beam_m

        # 높이는 항상 동일 (해수면에서 보기, 측면만 보임)
        projected_height_m = height_m

        # 거리에 따라 픽셀로 변환
        w_px = max(W_MIN, min(W_MAX, K_H * (projected_width_m / max(range_m, EPS))))
        h_px = max(H_MIN, min(H_MAX, K_H * (projected_height_m / max(range_m, EPS))))

        # 바운딩 박스를 파노라마 경계로 클램핑
        x1 = max(0, cx_px - w_px / 2)
        x2 = min(PANO_W_PX, cx_px + w_px / 2)
        y1 = max(0, cy_px - h_px / 2)
        y2 = min(PANO_H_PX, cy_px + h_px / 2)
        w_px = x2 - x1
        h_px = y2 - y1
        cx_px = (x1 + x2) / 2
        cy_px = (y1 + y2) / 2

        # NMEA 메시지 구성
        ship_class = getattr(tgt_ship, 'ship_class', 'CONTAINER')
        raw = f"$CAMERA,{tgt_ship.idx},{ship_class},{rel_bearing:.2f},{PANO_W_PX},{PANO_H_PX},{cx_px:.2f},{cy_px:.2f},{w_px:.2f},{h_px:.2f}"
        checksum = calculate_checksum(raw[1:])
        nmea_str = f"{raw}*{checksum}"

        # 파노라마 뷰용 탐지 딕셔너리
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
        버스트 손실 지원이 포함된 카메라 드롭아웃을 체크합니다 (사양 B-4, B-5).

        매개변수:
            ship_idx: 선박 인덱스
            dist_nm: 거리 (해리)
            current_time: 현재 시뮬레이션 시간

        반환값:
            신호가 드롭되어야 하면 True
        """
        config = self.proj.settings.camera_reception

        # 현재 버스트 상태 체크 (B-5)
        burst_state = self.camera_burst_states.get(ship_idx, {'in_burst': False, 'end_time': 0})
        if burst_state['in_burst']:
            if current_time < burst_state['end_time']:
                return True  # 버스트 중, 항상 드롭아웃
            else:
                # 버스트 종료
                burst_state['in_burst'] = False
                self.camera_burst_states[ship_idx] = burst_state

        # 기본 드롭아웃 확률 계산 (B-4)
        prob = self.calculate_dropout_probability(dist_nm, config)

        # 랜덤 드롭아웃 체크
        if random.random() < prob:
            # 버스트 시작 체크 (활성화된 경우, B-5)
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
