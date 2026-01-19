"""
NMEA 신호 처리 모듈

이 모듈은 NMEA 0183 프로토콜 관련 유틸리티 함수를 제공합니다.

주요 기능:
- NMEA 체크섬 계산
- AIS 페이로드 인코딩
- NMEA 문장 파싱 및 필드 추출

지원하는 NMEA 문장 유형:
- RMC: Recommended Minimum Navigation Information (위치, 속도, 방향)
- GGA: Global Positioning System Fix Data (위치 정보)
- VTG: Track Made Good and Ground Speed (방향, 속도)
- GLL: Geographic Position - Latitude/Longitude (위치)
- HDT: Heading True (진방위)
- TLL: Target Latitude and Longitude (레이더 목표 위치)
- TTM: Tracked Target Message (추적 목표 정보)
- CAMERA: 카메라 탐지 신호 (사용자 정의)
"""

import random


def calculate_checksum(nmea_str):
    """
    NMEA 문장의 체크섬을 계산합니다.

    NMEA 체크섬은 '$' 또는 '!' 다음부터 '*' 이전까지의
    모든 문자를 XOR 연산하여 계산합니다.

    매개변수:
        nmea_str: '$'와 '*' 사이의 NMEA 문자열

    반환값:
        2자리 16진수 체크섬 문자열 (예: "4A")
    """
    calc_cksum = 0
    for s in nmea_str:
        calc_cksum ^= ord(s)
    return hex(calc_cksum)[2:].upper().zfill(2)


def encode_ais_payload(mmsi):
    """
    AIS 페이로드를 인코딩합니다.

    실제 AIS 페이로드 인코딩은 6비트 ASCII와 특정 필드 구조를 사용하지만,
    시뮬레이션 목적으로 간소화된 형식을 사용합니다.

    매개변수:
        mmsi: Maritime Mobile Service Identity (9자리 해상이동업무식별번호)

    반환값:
        28자 길이의 인코딩된 페이로드 문자열
    """
    # MMSI를 9자리로 패딩
    mmsi_str = str(mmsi).zfill(9)
    dummy_payload_start = f"MMSI:{mmsi_str}"

    # 나머지를 랜덤 문자로 채움
    # AIS 6비트 문자 집합에서 선택
    chars = "0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVW`abcdefghijklmnopqrstuvw"
    remaining_len = 28 - len(dummy_payload_start)

    if remaining_len > 0:
        padding = "".join(random.choices(chars, k=remaining_len))
    else:
        padding = ""

    return (dummy_payload_start + padding)[:28]


def parse_nmea_fields(raw):
    """
    NMEA 문장을 파싱하여 필드 딕셔너리를 반환합니다.

    다양한 NMEA 문장 유형을 지원하며, 각 유형에 맞는 필드를 추출합니다.

    매개변수:
        raw: 원본 NMEA 문장 문자열

    반환값:
        파싱된 필드가 담긴 딕셔너리
        - talker: 토커 ID (예: "GP", "AI", "RA")
        - sentence_type: 문장 유형 (예: "RMC", "GGA")
        - raw: 원본 문장
        - 문장 유형에 따른 추가 필드들
    """
    fields = {}
    raw_str = raw.strip()

    # NMEA 문장 형식 확인 ($ 또는 !로 시작)
    if not raw_str.startswith("$") and not raw_str.startswith("!"):
        return fields

    # 체크섬 분리 및 필드 분할
    parts = raw_str.split('*')[0].split(',')

    header = parts[0]
    if header.startswith('$') or header.startswith('!'):
        header = header[1:]

    # CAMERA 문장 특별 처리 (표준 토커 ID 없음)
    if header == 'CAMERA':
        fields['talker'] = ''
        fields['sentence_type'] = 'CAMERA'
    else:
        fields['talker'] = header[:2]  # 토커 ID (2자)
        fields['sentence_type'] = header[2:]  # 문장 유형
    fields['raw'] = raw_str

    stype = fields['sentence_type']

    try:
        if stype == 'RMC':
            # RMC: Recommended Minimum Navigation Information
            # 위도, 경도, 대지속력(SOG), 진방위(COG) 추출
            if len(parts) > 9:
                fields['lat_deg'] = _parse_lat(parts[3], parts[4])
                fields['lon_deg'] = _parse_lon(parts[5], parts[6])
                fields['sog_knots'] = float(parts[7]) if parts[7] else 0.0
                fields['cog_true_deg'] = float(parts[8]) if parts[8] else 0.0

        elif stype == 'GGA':
            # GGA: Global Positioning System Fix Data
            # 위치 정보만 추출
            if len(parts) > 5:
                fields['lat_deg'] = _parse_lat(parts[2], parts[3])
                fields['lon_deg'] = _parse_lon(parts[4], parts[5])

        elif stype == 'VTG':
            # VTG: Track Made Good and Ground Speed
            # 방향과 속도 추출
            if len(parts) > 7:
                fields['cog_true_deg'] = float(parts[1]) if parts[1] else 0.0
                fields['sog_knots'] = float(parts[5]) if parts[5] else 0.0

        elif stype == 'GLL':
            # GLL: Geographic Position - Latitude/Longitude
            if len(parts) > 4:
                fields['lat_deg'] = _parse_lat(parts[1], parts[2])
                fields['lon_deg'] = _parse_lon(parts[3], parts[4])

        elif stype == 'HDT':
            # HDT: Heading True (진방위)
            if len(parts) > 1:
                fields['heading_true_deg'] = float(parts[1]) if parts[1] else 0.0

        elif stype == 'TLL':
            # TLL: Target Latitude and Longitude (레이더 목표 위치)
            if len(parts) > 5:
                try:
                    fields['target_no'] = int(parts[1])
                except:
                    pass
                fields['lat_deg'] = _parse_lat(parts[2], parts[3])
                fields['lon_deg'] = _parse_lon(parts[4], parts[5])

        elif stype == 'TTM':
            # TTM: Tracked Target Message (추적 목표 메시지)
            if len(parts) > 1:
                try:
                    fields['target_no'] = int(parts[1])
                except:
                    pass

        elif stype == 'CAMERA':
            # CAMERA: 카메라 탐지 신호 (사용자 정의 형식)
            # 형식: $CAMERA,track_id,class,rel_bearing,pano_w,pano_h,cx,cy,w,h*CS
            if len(parts) >= 10:
                try:
                    fields['track_id'] = parts[1]  # 추적 ID
                    fields['ship_class'] = parts[2]  # 선박 유형
                    fields['rel_bearing_deg'] = float(parts[3]) if parts[3] else 0.0  # 상대 방위
                    fields['pano_w_px'] = int(parts[4]) if parts[4] else 1920  # 파노라마 너비
                    fields['pano_h_px'] = int(parts[5]) if parts[5] else 320  # 파노라마 높이
                    fields['bbox_cx_px'] = float(parts[6]) if parts[6] else 0.0  # 바운딩박스 중심 X
                    fields['bbox_cy_px'] = float(parts[7]) if parts[7] else 0.0  # 바운딩박스 중심 Y
                    fields['bbox_w_px'] = float(parts[8]) if parts[8] else 0.0  # 바운딩박스 너비

                    # 마지막 필드에 체크섬이 포함될 수 있음
                    bbox_h_str = parts[9].split('*')[0] if '*' in parts[9] else parts[9]
                    fields['bbox_h_px'] = float(bbox_h_str) if bbox_h_str else 0.0  # 바운딩박스 높이
                except:
                    pass

        # UTM 좌표 자리표시자 (필요시 계산)
        if 'lat_deg' in fields and 'lon_deg' in fields:
            fields['utm_easting_m'] = 0.0
            fields['utm_northing_m'] = 0.0
            fields['utm_zone'] = 0
            fields['utm_hemisphere'] = 'N'

    except:
        pass

    return fields


def _parse_lat(val, ns):
    """
    NMEA 위도 문자열을 도(degree) 단위로 변환합니다.

    매개변수:
        val: NMEA 형식 위도 (DDMM.MMMM)
        ns: 북위/남위 표시 ('N' 또는 'S')

    반환값:
        도 단위 위도 (남위는 음수)
    """
    if not val:
        return 0.0
    try:
        d = float(val[:2])  # 도
        m = float(val[2:])  # 분
        v = d + m / 60.0
        return v if ns == 'N' else -v
    except:
        return 0.0


def _parse_lon(val, ew):
    """
    NMEA 경도 문자열을 도(degree) 단위로 변환합니다.

    매개변수:
        val: NMEA 형식 경도 (DDDMM.MMMM)
        ew: 동경/서경 표시 ('E' 또는 'W')

    반환값:
        도 단위 경도 (서경은 음수)
    """
    if not val:
        return 0.0
    try:
        d = float(val[:3])  # 도
        m = float(val[3:])  # 분
        v = d + m / 60.0
        return v if ew == 'E' else -v
    except:
        return 0.0
