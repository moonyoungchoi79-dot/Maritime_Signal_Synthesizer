"""
지도 정보 모델 모듈

이 모듈은 지도 표시에 필요한 좌표 변환 정보를 관리합니다.

클래스:
    MapInfo: 지도 좌표 변환 및 표시 설정을 담는 데이터 클래스
"""

from dataclasses import dataclass


@dataclass
class MapInfo:
    """
    지도 좌표 변환 및 표시 설정을 담는 데이터 클래스입니다.

    픽셀 좌표와 지리 좌표(위도/경도) 간 변환에 필요한 정보와
    UTM 좌표계 정보를 포함합니다.

    속성:
        center_lat: 지도 중심 위도 (도 단위)
        center_lon: 지도 중심 경도 (도 단위)
        pixels_per_degree: 1도당 픽셀 수 (줌 레벨)
        utm_zone: UTM 존 번호 (1-60)
        utm_zone_letter: UTM 존 문자
        utm_hemisphere: UTM 반구 ('N' 또는 'S')
    """
    center_lat: float = 35.0  # 지도 중심 위도 (기본값: 대한민국 부근)
    center_lon: float = 129.0  # 지도 중심 경도 (기본값: 대한민국 부근)
    pixels_per_degree: float = 2000.0  # 축척 (픽셀/도)

    # UTM 좌표계 정보
    utm_zone: int = 52  # UTM 존 번호
    utm_zone_letter: str = 'S'  # UTM 존 문자
    utm_hemisphere: str = 'N'  # 반구 ('N': 북반구, 'S': 남반구)

    def update_utm_from_latlon(self):
        """
        중심 좌표를 기반으로 UTM 존 정보를 업데이트합니다.

        현재는 구현되지 않음 (필요시 구현 예정).
        """
        pass
