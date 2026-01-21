"""
카메라 투영 모듈

이 모듈은 동차좌표계 기반의 카메라 투영 시스템을 제공합니다.
World 좌표 → Pinhole 이미지 → 원통(Cylindrical) 파노라마로의 변환을 수행합니다.

클래스:
    CameraIntrinsic: 카메라 내부 파라미터 (초점거리, 주점)
    CameraExtrinsic: 카메라 외부 파라미터 (회전, 위치)
    PanoramaSpec: 파노라마 이미지 사양
    CameraHomography: 동차좌표 변환 행렬
    PanoramicFrameTransform: World → Pinhole → Cylinder 변환
    DualCameraBboxGenerator: EO/IR 듀얼 카메라 bbox 생성기

좌표계 정의:
    World Frame (선박 상대 좌표):
        - X축: 전방 (front/bow)
        - Y축: 우현 (right/starboard)
        - Z축: 하방 (down)

    Camera Frame:
        - X축: 이미지 우측
        - Y축: 이미지 하방
        - Z축: 광축 (전방)

    Image Frame:
        - 원점: 좌상단 (left-top)
        - U축: 우측 (열)
        - V축: 하방 (행)
"""

import math
import numpy as np
from dataclasses import dataclass
from typing import Optional, List, Tuple


@dataclass
class CameraIntrinsic:
    """
    카메라 내부 파라미터 (Intrinsic Parameters)

    속성:
        fu: 수평 초점거리 (픽셀)
        fv: 수직 초점거리 (픽셀)
        cu: 주점 U 좌표 (픽셀)
        cv: 주점 V 좌표 (픽셀)
    """
    fu: float
    fv: float
    cu: float
    cv: float


@dataclass
class CameraExtrinsic:
    """
    카메라 외부 파라미터 (Extrinsic Parameters)

    속성:
        roll_deg: 롤 각도 (도)
        pitch_deg: 피치 각도 (도)
        yaw_deg: 요 각도 (도)
        tx: X축 오프셋 (전방)
        ty: Y축 오프셋 (우현)
        tz: Z축 오프셋 (카메라 높이, 수면 기준 양수)
    """
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    tx: float = 0.0
    ty: float = 0.0
    tz: float = 15.0  # 기본 카메라 높이 15m


@dataclass
class PanoramaSpec:
    """
    파노라마 이미지 사양

    속성:
        dim_U: 파노라마 가로 해상도 (픽셀)
        dim_V: 파노라마 세로 해상도 (픽셀)
        fov_deg: 수평 시야각 (도)
    """
    dim_U: int
    dim_V: int
    fov_deg: float


# EO 카메라 기본 사양 (3840x1080, ±90°)
EO_PANO_SPEC = PanoramaSpec(dim_U=3840, dim_V=1080, fov_deg=180.0)

# IR 카메라 기본 사양 (1536x512, ±55°)
IR_PANO_SPEC = PanoramaSpec(dim_U=1536, dim_V=512, fov_deg=110.0)


def compute_intrinsic_from_panorama(pano_spec: PanoramaSpec) -> CameraIntrinsic:
    """
    파노라마 사양으로부터 등가 핀홀 카메라 내부 파라미터를 계산합니다.

    원통 투영에서 초점거리는 f = dim_U / fov_rad로 계산됩니다.

    매개변수:
        pano_spec: 파노라마 사양

    반환값:
        계산된 카메라 내부 파라미터
    """
    fov_rad = math.radians(pano_spec.fov_deg)
    fu = pano_spec.dim_U / fov_rad
    fv = fu * (pano_spec.dim_V / pano_spec.dim_U)
    cu = pano_spec.dim_U / 2.0
    cv = pano_spec.dim_V / 2.0
    return CameraIntrinsic(fu=fu, fv=fv, cu=cu, cv=cv)


class CameraHomography:
    """
    동차좌표 기반 카메라 변환 행렬

    4x4 동차 변환 행렬을 사용하여 World 좌표를 이미지 좌표로 변환합니다.
    Roll-Pitch-Yaw 회전과 이동 변환을 조합합니다.

    속성:
        fu, fv, cu, cv: 내부 파라미터
        roll_deg, pitch_deg, yaw_deg: 회전 각도
        tx, ty, tz: 이동 오프셋
    """

    def __init__(self, intrinsic: CameraIntrinsic, extrinsic: CameraExtrinsic):
        self.fu = intrinsic.fu
        self.fv = intrinsic.fv
        self.cu = intrinsic.cu
        self.cv = intrinsic.cv

        self.roll_deg = extrinsic.roll_deg
        self.pitch_deg = extrinsic.pitch_deg
        self.yaw_deg = extrinsic.yaw_deg
        self.tx = extrinsic.tx
        self.ty = extrinsic.ty
        self.tz = extrinsic.tz

        self._compute_rotation_elements()
        self._homography = self._make_homography()
        self._homography_inv = np.linalg.inv(self._homography)

    def _compute_rotation_elements(self):
        """Roll-Pitch-Yaw 회전 행렬 요소를 계산합니다."""
        d_to_r = math.pi / 180.0
        cos_roll = math.cos(self.roll_deg * d_to_r)
        sin_roll = math.sin(self.roll_deg * d_to_r)
        cos_pitch = math.cos(self.pitch_deg * d_to_r)
        sin_pitch = math.sin(self.pitch_deg * d_to_r)
        cos_yaw = math.cos(self.yaw_deg * d_to_r)
        sin_yaw = math.sin(self.yaw_deg * d_to_r)

        # 회전 행렬 요소 (ZYX 순서: Yaw-Pitch-Roll)
        self.r0 = (cos_roll * cos_yaw) + (sin_roll * sin_pitch * sin_yaw)
        self.r1 = -(sin_roll * cos_yaw) + (cos_roll * sin_pitch * sin_yaw)
        self.r2 = cos_pitch * sin_yaw
        self.r3 = sin_roll * cos_pitch
        self.r4 = cos_roll * cos_pitch
        self.r5 = -sin_pitch
        self.r6 = -(cos_roll * sin_yaw) + (sin_roll * sin_pitch * cos_yaw)
        self.r7 = (sin_roll * sin_yaw) + (cos_roll * sin_pitch * cos_yaw)
        self.r8 = cos_pitch * cos_yaw

    def _make_homography(self) -> np.ndarray:
        """
        4x4 동차 변환 행렬을 생성합니다.

        반환값:
            4x4 numpy 배열
        """
        ret_mat = np.zeros((4, 4))

        ret_mat[0, 0] = (self.fu * self.r6) + (self.cu * self.r8)
        ret_mat[0, 1] = (self.fu * self.r0) + (self.cu * self.r2)
        ret_mat[0, 2] = (self.fu * self.r3) + (self.cu * self.r5)
        ret_mat[0, 3] = -(self.fu * self.ty) - (self.cu * self.tx)

        ret_mat[1, 0] = (self.fv * self.r7) + (self.cv * self.r8)
        ret_mat[1, 1] = (self.fv * self.r1) + (self.cv * self.r2)
        ret_mat[1, 2] = (self.fv * self.r4) + (self.cv * self.r5)
        ret_mat[1, 3] = -(self.fv * self.tz) - (self.cv * self.tx)

        ret_mat[2, 0] = self.r8
        ret_mat[2, 1] = self.r2
        ret_mat[2, 2] = self.r5
        ret_mat[2, 3] = -self.tx

        ret_mat[3, 3] = 1.0

        return ret_mat

    def update_cam_height(self, height: float):
        """
        카메라 높이를 동적으로 변경합니다.

        선박 상부 점 투영 시 사용됩니다.

        매개변수:
            height: 새 카메라 높이 (미터)
        """
        self.tz = height
        self._homography[1, 3] = -(self.fv * height) - (self.cv * self.tx)
        self._homography_inv = np.linalg.inv(self._homography)

    def get_homography(self) -> np.ndarray:
        """동차 변환 행렬을 반환합니다."""
        return self._homography

    def get_inv_homography(self) -> np.ndarray:
        """역 동차 변환 행렬을 반환합니다."""
        return self._homography_inv


class PanoramicFrameTransform:
    """
    World → Pinhole → Cylinder 투영 변환 체인

    상대 World 좌표를 원통 파노라마 이미지 좌표로 변환합니다.

    속성:
        _homography: CameraHomography 인스턴스
        _pano_spec: PanoramaSpec 인스턴스
        _fov_rad: 수평 FOV (라디안)
    """

    def __init__(self, homography: CameraHomography, pano_spec: PanoramaSpec):
        self._homography = homography
        self._pano_spec = pano_spec
        self._fov_rad = math.radians(pano_spec.fov_deg)
        self._eps = 1e-9

    def get_homography(self) -> CameraHomography:
        """CameraHomography 객체를 반환합니다."""
        return self._homography

    def get_pano_spec(self) -> PanoramaSpec:
        """PanoramaSpec 객체를 반환합니다."""
        return self._pano_spec

    def convert_relXY_2_undistUV(self, rel_x: float, rel_y: float, rel_z: float = 0.0) -> Tuple[float, float]:
        """
        상대 World 좌표를 핀홀 이미지 좌표로 변환합니다.

        매개변수:
            rel_x: 우현 방향 거리 (미터)
            rel_y: 전방 방향 거리 (미터)
            rel_z: 높이 (미터, 위쪽이 음수)

        반환값:
            (undist_u, undist_v) 핀홀 이미지 좌표
        """
        H = self._homography.get_homography()

        # 동차좌표 변환: H @ [rel_y, rel_x, rel_z, 1]^T
        # World: X=front(rel_y), Y=right(rel_x), Z=down(rel_z)
        weight = H[2, 0] * rel_y + H[2, 1] * rel_x + H[2, 2] * rel_z + H[2, 3]

        if weight < self._eps:
            return -64.0, -64.0  # 카메라 뒤쪽 (무효)

        inv_weight = 1.0 / weight
        undist_u = inv_weight * (H[0, 0] * rel_y + H[0, 1] * rel_x + H[0, 2] * rel_z + H[0, 3])
        undist_v = inv_weight * (H[1, 0] * rel_y + H[1, 1] * rel_x + H[1, 2] * rel_z + H[1, 3])

        return undist_u, undist_v

    def convert_undistUV_2_cylinder(self, undist_u: float, undist_v: float) -> Tuple[float, float]:
        """
        핀홀 이미지 좌표를 원통 파노라마 좌표로 변환합니다.

        매개변수:
            undist_u: 핀홀 이미지 U 좌표
            undist_v: 핀홀 이미지 V 좌표

        반환값:
            (cylinder_u, cylinder_v) 원통 파노라마 좌표
        """
        if undist_v <= 0:
            return -64.0, -64.0

        cu = self._homography.cu
        fu = self._homography.fu
        cv = self._homography.cv
        fv = self._homography.fv

        dim_U = self._pano_spec.dim_U
        dim_V = self._pano_spec.dim_V

        # 수평 방위각 계산
        theta = math.atan2(undist_u - cu, fu)

        # FOV 범위 체크
        half_fov = self._fov_rad / 2.0
        if theta < -half_fov or theta > half_fov:
            return -64.0, -64.0

        # 원통 투영 focal length
        cylinder_focal = dim_U / self._fov_rad

        # 수평 좌표 매핑
        cylinder_u = (cylinder_focal * theta) + (dim_U / 2.0)

        # 수직 좌표 매핑 (원통 투영)
        # 핀홀에서 수직 오프셋을 각도로 변환 후 원통에서 재계산
        # 단순 비율 변환 사용 (수직 왜곡 보정 없음 - 해양 시나리오에서는 수평선 근처이므로)
        cylinder_v = ((undist_v - cv) / fv) * (dim_V / 2.0) + (dim_V / 2.0)

        return cylinder_u, cylinder_v

    def convert_relXY_2_cylinder(self, rel_x: float, rel_y: float, rel_z: float = 0.0) -> Tuple[float, float]:
        """
        상대 World 좌표를 원통 파노라마 좌표로 변환합니다 (체인 호출).

        매개변수:
            rel_x: 우현 방향 거리 (미터)
            rel_y: 전방 방향 거리 (미터)
            rel_z: 높이 (미터, 위쪽이 음수)

        반환값:
            (cylinder_u, cylinder_v) 원통 파노라마 좌표
        """
        undist_u, undist_v = self.convert_relXY_2_undistUV(rel_x, rel_y, rel_z)
        if undist_u < 0:
            return -64.0, -64.0
        return self.convert_undistUV_2_cylinder(undist_u, undist_v)

    def convert_relXYZ_batch_2_cylinder(self, rel_xs: np.ndarray, rel_ys: np.ndarray, rel_zs: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        여러 점의 상대 World 좌표를 원통 파노라마 좌표로 일괄 변환합니다 (벡터화).

        매개변수:
            rel_xs: 우현 방향 거리 배열 (미터)
            rel_ys: 전방 방향 거리 배열 (미터)
            rel_zs: 높이 배열 (미터, 위쪽이 음수)

        반환값:
            (cylinder_us, cylinder_vs) 원통 파노라마 좌표 배열
        """
        H = self._homography.get_homography()
        cu = self._homography.cu
        fu = self._homography.fu
        cv = self._homography.cv
        fv = self._homography.fv
        dim_U = self._pano_spec.dim_U
        dim_V = self._pano_spec.dim_V
        half_fov = self._fov_rad / 2.0

        # 동차좌표 변환 (벡터화) - Z 좌표 포함
        weights = H[2, 0] * rel_ys + H[2, 1] * rel_xs + H[2, 2] * rel_zs + H[2, 3]

        # 유효성 마스크
        valid = weights > self._eps

        # 초기화 (무효 값)
        cylinder_us = np.full_like(rel_xs, -64.0)
        cylinder_vs = np.full_like(rel_ys, -64.0)

        if not np.any(valid):
            return cylinder_us, cylinder_vs

        # 유효한 점만 계산 - Z 좌표 포함
        inv_weights = 1.0 / weights[valid]
        undist_us = inv_weights * (H[0, 0] * rel_ys[valid] + H[0, 1] * rel_xs[valid] + H[0, 2] * rel_zs[valid] + H[0, 3])
        undist_vs = inv_weights * (H[1, 0] * rel_ys[valid] + H[1, 1] * rel_xs[valid] + H[1, 2] * rel_zs[valid] + H[1, 3])

        # undist_v > 0 체크
        valid2 = undist_vs > 0
        if not np.any(valid2):
            return cylinder_us, cylinder_vs

        # 방위각 계산
        thetas = np.arctan2(undist_us[valid2] - cu, fu)

        # FOV 범위 체크
        valid3 = (thetas >= -half_fov) & (thetas <= half_fov)
        if not np.any(valid3):
            return cylinder_us, cylinder_vs

        # 원통 투영
        cylinder_focal = dim_U / self._fov_rad

        final_thetas = thetas[valid3]
        final_undist_vs = undist_vs[valid2][valid3]

        final_cylinder_us = (cylinder_focal * final_thetas) + (dim_U / 2.0)

        # 수직 좌표 계산 (단순 비율 변환)
        final_cylinder_vs = ((final_undist_vs - cv) / fv) * (dim_V / 2.0) + (dim_V / 2.0)

        # 결과를 원래 인덱스에 매핑
        valid_indices = np.where(valid)[0]
        valid2_indices = valid_indices[valid2]
        final_indices = valid2_indices[valid3]

        cylinder_us[final_indices] = final_cylinder_us
        cylinder_vs[final_indices] = final_cylinder_vs

        return cylinder_us, cylinder_vs

    def convert_relXYZ_batch_2_cylinder_no_fov_check(self, rel_xs: np.ndarray, rel_ys: np.ndarray, rel_zs: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        여러 점의 상대 World 좌표를 원통 파노라마 좌표로 일괄 변환합니다 (FOV 체크 없음).

        중심점이 FOV 안에 있으면 8개 꼭짓점 모두를 유효한 것으로 투영합니다.
        이를 통해 FOV 경계에서 연속적인 bbox 변화를 보장합니다.

        매개변수:
            rel_xs: 우현 방향 거리 배열 (미터)
            rel_ys: 전방 방향 거리 배열 (미터)
            rel_zs: 높이 배열 (미터, 위쪽이 음수)

        반환값:
            (cylinder_us, cylinder_vs) 원통 파노라마 좌표 배열
        """
        H = self._homography.get_homography()
        cu = self._homography.cu
        fu = self._homography.fu
        cv = self._homography.cv
        fv = self._homography.fv
        dim_U = self._pano_spec.dim_U
        dim_V = self._pano_spec.dim_V

        # 동차좌표 변환 (벡터화) - Z 좌표 포함
        weights = H[2, 0] * rel_ys + H[2, 1] * rel_xs + H[2, 2] * rel_zs + H[2, 3]

        # 유효성 마스크 (카메라 앞에 있는 점만)
        valid = weights > self._eps

        # 초기화 (무효 값)
        cylinder_us = np.full_like(rel_xs, -99999.0)
        cylinder_vs = np.full_like(rel_ys, -99999.0)

        if not np.any(valid):
            return cylinder_us, cylinder_vs

        # 유효한 점만 계산 - Z 좌표 포함
        inv_weights = 1.0 / weights[valid]
        undist_us = inv_weights * (H[0, 0] * rel_ys[valid] + H[0, 1] * rel_xs[valid] + H[0, 2] * rel_zs[valid] + H[0, 3])
        undist_vs = inv_weights * (H[1, 0] * rel_ys[valid] + H[1, 1] * rel_xs[valid] + H[1, 2] * rel_zs[valid] + H[1, 3])

        # undist_v > 0 체크
        valid2 = undist_vs > 0
        if not np.any(valid2):
            return cylinder_us, cylinder_vs

        # 방위각 계산 (FOV 체크 없이)
        thetas = np.arctan2(undist_us[valid2] - cu, fu)

        # 원통 투영 (FOV 체크 없이 모든 점 계산)
        cylinder_focal = dim_U / self._fov_rad

        final_cylinder_us = (cylinder_focal * thetas) + (dim_U / 2.0)

        # 수직 좌표 계산 (단순 비율 변환)
        final_cylinder_vs = ((undist_vs[valid2] - cv) / fv) * (dim_V / 2.0) + (dim_V / 2.0)

        # 결과를 원래 인덱스에 매핑
        valid_indices = np.where(valid)[0]
        valid2_indices = valid_indices[valid2]

        cylinder_us[valid2_indices] = final_cylinder_us
        cylinder_vs[valid2_indices] = final_cylinder_vs

        return cylinder_us, cylinder_vs


@dataclass
class ShipVertices:
    """
    선박 3D 모델의 8개 꼭짓점

    선박 로컬 좌표계 (선체 중심 기준):
        - X축: 전방 (bow 방향이 +)
        - Y축: 우현 (starboard 방향이 +)
        - Z축: 하방 (수면 아래가 +)

    꼭짓점 순서:
        0-3: 수면 레벨 (z=0)
        4-7: 상부 레벨 (z=-height, 위쪽이므로 음수)
    """
    vertices: np.ndarray  # 3x8 배열

    @classmethod
    def from_dimensions(cls, length_m: float, beam_m: float, height_m: float) -> 'ShipVertices':
        """
        선박 치수로부터 꼭짓점을 생성합니다.

        매개변수:
            length_m: 선박 길이 (미터)
            beam_m: 선박 폭 (미터)
            height_m: 수면 위 높이 (미터)

        반환값:
            ShipVertices 인스턴스
        """
        half_L = length_m / 2.0
        half_B = beam_m / 2.0

        # 3x8 배열: [x, y, z] x 8점
        vertices = np.array([
            # 수면 레벨 (z=0)
            [ half_L, -half_B, 0.0],  # 0: bow, port, waterline
            [ half_L,  half_B, 0.0],  # 1: bow, starboard, waterline
            [-half_L, -half_B, 0.0],  # 2: stern, port, waterline
            [-half_L,  half_B, 0.0],  # 3: stern, starboard, waterline
            # 상부 레벨 (z=-height, 위쪽)
            [ half_L, -half_B, -height_m],  # 4: bow, port, top
            [ half_L,  half_B, -height_m],  # 5: bow, starboard, top
            [-half_L, -half_B, -height_m],  # 6: stern, port, top
            [-half_L,  half_B, -height_m],  # 7: stern, starboard, top
        ]).T  # 전치하여 3x8로 변환

        return cls(vertices=vertices)


class DualCameraBboxGenerator:
    """
    EO/IR 듀얼 카메라용 Bounding Box 생성기

    선박의 8개 꼭짓점을 각 카메라로 투영하여 bbox를 계산합니다.

    속성:
        _eo_transform: EO 카메라 변환기
        _ir_transform: IR 카메라 변환기
        _camera_height: 카메라 높이 (미터)
    """

    def __init__(self, camera_height_m: float = 15.0):
        """
        듀얼 카메라 bbox 생성기를 초기화합니다.

        매개변수:
            camera_height_m: 카메라 높이 (미터)
        """
        self._camera_height = camera_height_m

        # EO 카메라 설정
        eo_intrinsic = compute_intrinsic_from_panorama(EO_PANO_SPEC)
        eo_extrinsic = CameraExtrinsic(tz=camera_height_m)
        eo_homography = CameraHomography(eo_intrinsic, eo_extrinsic)
        self._eo_transform = PanoramicFrameTransform(eo_homography, EO_PANO_SPEC)

        # IR 카메라 설정
        ir_intrinsic = compute_intrinsic_from_panorama(IR_PANO_SPEC)
        ir_extrinsic = CameraExtrinsic(tz=camera_height_m)
        ir_homography = CameraHomography(ir_intrinsic, ir_extrinsic)
        self._ir_transform = PanoramicFrameTransform(ir_homography, IR_PANO_SPEC)

        # 상부 점 투영용 복사본
        self._eo_transform_top = None
        self._ir_transform_top = None

    def update_camera_height(self, height_m: float):
        """카메라 높이를 업데이트합니다."""
        self._camera_height = height_m
        self._eo_transform.get_homography().update_cam_height(height_m)
        self._ir_transform.get_homography().update_cam_height(height_m)

    def generate_bbox(
        self,
        target_ship_dims: Tuple[float, float, float],  # (length, beam, height)
        target_rel_x: float,  # 우현 방향 상대 거리 (미터)
        target_rel_y: float,  # 전방 방향 상대 거리 (미터)
        target_heading_rel: float,  # Own 기준 상대 헤딩 (라디안)
        camera_type: str  # "EO" 또는 "IR"
    ) -> Optional[List[float]]:
        """
        선박 bbox를 생성합니다.

        중심점이 FOV 안에 있으면 bbox를 생성하고, 밖이면 None을 반환합니다.
        FOV 체크 없이 모든 꼭짓점을 투영하여 연속적인 bbox 변화를 보장합니다.

        매개변수:
            target_ship_dims: (길이, 폭, 높이) 미터
            target_rel_x: 자선 기준 우현 방향 상대 거리 (미터)
            target_rel_y: 자선 기준 전방 방향 상대 거리 (미터)
            target_heading_rel: 자선 기준 상대 헤딩 (라디안)
            camera_type: "EO" 또는 "IR"

        반환값:
            [left, top, right, bottom] 또는 None (중심점이 FOV 밖)
        """
        transform = self._eo_transform if camera_type == "EO" else self._ir_transform
        pano_spec = transform.get_pano_spec()

        # 먼저 중심점이 FOV 안에 있는지 확인 (방위각 기반)
        # rel_x = 우현(+), rel_y = 전방(+)
        # 방위각: atan2(rel_x, rel_y) → 우현이 양수 각도
        center_bearing_rad = math.atan2(target_rel_x, target_rel_y)
        half_fov_rad = math.radians(pano_spec.fov_deg / 2.0)

        if abs(center_bearing_rad) > half_fov_rad:
            return None  # 중심점이 FOV 밖이면 bbox 생성 안함

        length_m, beam_m, height_m = target_ship_dims

        # 선박 꼭짓점 생성
        ship_vertices = ShipVertices.from_dimensions(length_m, beam_m, height_m)
        vertices = ship_vertices.vertices  # 3x8

        # 상대 헤딩으로 회전
        cos_h = math.cos(target_heading_rel)
        sin_h = math.sin(target_heading_rel)
        rot_mat = np.array([
            [cos_h, -sin_h, 0],
            [sin_h,  cos_h, 0],
            [0,      0,     1]
        ])

        # 회전 적용
        rotated = rot_mat @ vertices

        # 상대 위치로 이동
        rotated[0, :] += target_rel_y  # X = front
        rotated[1, :] += target_rel_x  # Y = right

        # 8개 점 모두 투영 (FOV 체크 없이)
        rel_xs = rotated[1, :]  # right
        rel_ys = rotated[0, :]  # front
        rel_zs = rotated[2, :]  # height

        us, vs = transform.convert_relXYZ_batch_2_cylinder_no_fov_check(rel_xs, rel_ys, rel_zs)

        # 유효한 점 필터링 (카메라 앞에 있는 점만)
        valid_mask = (us > -90000) & (vs > -90000)
        valid_count = np.sum(valid_mask)

        if valid_count < 2:
            return None

        valid_us = us[valid_mask]
        valid_vs = vs[valid_mask]

        left = float(np.min(valid_us))
        right = float(np.max(valid_us))
        top = float(np.min(valid_vs))
        bottom = float(np.max(valid_vs))

        # 파노라마 범위로 클램핑
        left = max(0, left)
        right = min(pano_spec.dim_U, right)
        top = max(0, top)
        bottom = min(pano_spec.dim_V, bottom)

        if right <= left or bottom <= top:
            return None

        return [left, top, right, bottom]

    def generate_detection_dict(
        self,
        ship_idx: int,
        ship_name: str,
        ship_class: str,
        rel_bearing: float,
        bbox: List[float],
        camera_type: str
    ) -> dict:
        """
        파노라마 뷰용 탐지 딕셔너리를 생성합니다.

        매개변수:
            ship_idx: 선박 인덱스
            ship_name: 선박 이름
            ship_class: 선박 클래스
            rel_bearing: 상대 방위각 (도)
            bbox: [left, top, right, bottom]
            camera_type: "EO" 또는 "IR"

        반환값:
            탐지 정보 딕셔너리
        """
        left, top, right, bottom = bbox
        cx_px = (left + right) / 2.0
        cy_px = (top + bottom) / 2.0
        w_px = right - left
        h_px = bottom - top

        pano_spec = self._eo_transform.get_pano_spec() if camera_type == "EO" else self._ir_transform.get_pano_spec()

        return {
            'ship_idx': ship_idx,
            'ship_name': ship_name,
            'ship_class': ship_class,
            'rel_bearing': rel_bearing,
            'cx_px': cx_px,
            'cy_px': cy_px,
            'w_px': w_px,
            'h_px': h_px,
            'left': left,
            'top': top,
            'right': right,
            'bottom': bottom,
            'camera_type': camera_type,
            'pano_w': pano_spec.dim_U,
            'pano_h': pano_spec.dim_V
        }


# 선박 클래스 ID 매핑 (Redis 전송용)
SHIP_CLASS_TO_ID = {
    "CONTAINER": 0,
    "TANKER": 1,
    "CARGO": 2,
    "PASSENGER": 3,
    "FISHING": 4,
    "BUOY": 5,
    "OTHER": 6,
}


def get_ship_class_id(ship_class: str) -> int:
    """선박 클래스 문자열을 ID로 변환합니다."""
    return SHIP_CLASS_TO_ID.get(ship_class, 6)  # 기본값: OTHER
