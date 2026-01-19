"""
NMEA 신호 처리 모듈

이 모듈은 NMEA 0183 프로토콜 관련 유틸리티 함수를 제공합니다.

주요 기능:
- NMEA 체크섬 계산
- AIS 페이로드 인코딩 (AIVDM Message Type 1)
- NMEA 문장 파싱 및 필드 추출

지원하는 NMEA 문장 유형:
- TTM: Tracked Target Message (추적 목표 정보)
- CAMERA: 카메라 탐지 신호 (사용자 정의)
"""

import datetime


def calculate_checksum(nmea_str):
    
    calc_cksum = 0
    for s in nmea_str:
        calc_cksum ^= ord(s)
    return hex(calc_cksum)[2:].upper().zfill(2)


def _int_to_twos_complement(value, bits):
    
    if value >= 0:

        return value

    else:
        
        return (1 << bits) + value


def _bits_to_armored_string(bit_string):
    result = []
    for i in range(0, len(bit_string), 6):
        chunk = bit_string[i:i+6]
        
        if len(chunk) < 6:
            chunk = chunk + '0' * (6 - len(chunk))

        v = int(chunk, 2)

        if v < 40:
            ascii_val = v + 48
        else:
            ascii_val = v + 56

        result.append(chr(ascii_val))

    return ''.join(result)


def encode_ais_payload(mmsi, lat_deg=0.0, lon_deg=0.0, sog_knots=0.0,
                       cog_deg=0.0, heading_deg=0.0, nav_status=0,
                       rot=0, utc_second=None):
    """
    AIS Message Type 1 (Class A Position Report) 페이로드를 인코딩합니다.

    ITU-R M.1371-5 표준에 따라 168비트 페이로드를 생성하고,
    6비트 Armoring 알고리즘으로 ASCII 문자열로 변환합니다.

    [비트맵 구조 - 총 168비트]
    - Message ID (6 bits): 메시지 타입 = 1
    - Repeat Indicator (2 bits): 반복 지시자 = 0
    - MMSI (30 bits): 선박 식별번호
    - Navigation Status (4 bits): 항해 상태
    - Rate of Turn (8 bits): 선회율 (부호 있는 정수)
    - Speed Over Ground (10 bits): 대지속력 * 10
    - Position Accuracy (1 bit): 위치 정확도 = 1 (고정밀)
    - Longitude (28 bits): 경도 (분 * 10000, 2의 보수)
    - Latitude (27 bits): 위도 (분 * 10000, 2의 보수)
    - Course Over Ground (12 bits): 대지침로 * 10
    - True Heading (9 bits): 진방위
    - Time Stamp (6 bits): UTC 초
    - Special Manoeuvre (2 bits): 특수 조종 = 0
    - Spare (3 bits): 예비 = 0
    - RAIM flag (1 bit): RAIM = 0
    - Radio Status (19 bits): 라디오 상태 = 0

    매개변수:
        mmsi: Maritime Mobile Service Identity (9자리 해상이동업무식별번호)
        lat_deg: 위도 (도 단위, -90 ~ 90, 북위 양수)
        lon_deg: 경도 (도 단위, -180 ~ 180, 동경 양수)
        sog_knots: 대지속력 (노트, 0 ~ 102.2)
        cog_deg: 대지침로 (도, 0 ~ 359.9)
        heading_deg: 진방위 (도, 0 ~ 359, 511 = 없음)
        nav_status: 항해 상태 (0=항해중, 1=정박, 5=계류 등)
        rot: 선회율 (-127 ~ 127, 0=선회 안함, -128=없음)
        utc_second: UTC 초 (0~59, None이면 현재 시간 사용)

    반환값:
        28자 길이의 인코딩된 페이로드 문자열

    [디코딩 방법]
    1. 각 ASCII 문자를 10진수로 변환:
       - 문자코드 >= 88 이면: V = 문자코드 - 56
       - 그 외: V = 문자코드 - 48
    2. 각 V를 6비트 이진수로 변환하여 이어 붙임
    3. 비트맵에 따라 필드 추출:
       - bits 0-5: Message ID
       - bits 6-7: Repeat Indicator
       - bits 8-37: MMSI
       - bits 38-41: Navigation Status
       - bits 42-49: Rate of Turn
       - bits 50-59: Speed Over Ground (값 / 10 = 노트)
       - bit 60: Position Accuracy
       - bits 61-88: Longitude (2의 보수, 값 / 10000 / 60 = 도)
       - bits 89-115: Latitude (2의 보수, 값 / 10000 / 60 = 도)
       - bits 116-127: Course Over Ground (값 / 10 = 도)
       - bits 128-136: True Heading
       - bits 137-142: Time Stamp
       - bits 143-144: Special Manoeuvre
       - bits 145-147: Spare
       - bit 148: RAIM flag
       - bits 149-167: Radio Status
    """
    bits = ''

    # 1. Message ID (6 bits) - Type 1 Position Report
    bits += format(1, '06b')

    # 2. Repeat Indicator (2 bits) - 0 = default
    bits += format(0, '02b')

    # 3. MMSI (30 bits)
    mmsi_int = int(mmsi) % (2**30)  # 30비트 범위로 제한
    bits += format(mmsi_int, '030b')

    # 4. Navigation Status (4 bits)
    nav_status = max(0, min(15, nav_status))
    bits += format(nav_status, '04b')

    # 5. Rate of Turn (8 bits) - 부호 있는 정수
    # ROT 공식: ROT_AIS = 4.733 * sqrt(ROT_degrees_per_min)
    # 여기서는 직접 값 사용, -128=없음
    rot = max(-128, min(127, rot))
    rot_twos = _int_to_twos_complement(rot, 8)
    bits += format(rot_twos, '08b')

    # 6. Speed Over Ground (10 bits) - 노트 * 10
    # 최대값 102.2 노트 = 1022, 1023 = 없음
    sog_encoded = min(1022, max(0, int(round(sog_knots * 10))))
    bits += format(sog_encoded, '010b')

    # 7. Position Accuracy (1 bit) - 1 = high accuracy (<10m)
    bits += '1'

    # 8. Longitude (28 bits) - 분 * 10000, 2의 보수
    # 경도: -180 ~ 180도 -> 분으로 변환 (* 60) -> * 10000
    # 181도 = 0x6791AC0 = 없음
    lon_minutes = lon_deg * 60.0
    lon_encoded = int(round(lon_minutes * 10000))
    lon_encoded = max(-180 * 60 * 10000, min(180 * 60 * 10000, lon_encoded))
    lon_twos = _int_to_twos_complement(lon_encoded, 28)
    bits += format(lon_twos, '028b')

    # 9. Latitude (27 bits) - 분 * 10000, 2의 보수
    # 위도: -90 ~ 90도 -> 분으로 변환 (* 60) -> * 10000
    # 91도 = 0x3412140 = 없음
    lat_minutes = lat_deg * 60.0
    lat_encoded = int(round(lat_minutes * 10000))
    lat_encoded = max(-90 * 60 * 10000, min(90 * 60 * 10000, lat_encoded))
    lat_twos = _int_to_twos_complement(lat_encoded, 27)
    bits += format(lat_twos, '027b')

    # 10. Course Over Ground (12 bits) - 도 * 10
    # 0 ~ 359.9도, 3600 = 없음
    cog_encoded = min(3599, max(0, int(round(cog_deg * 10))))
    bits += format(cog_encoded, '012b')

    # 11. True Heading (9 bits) - 0 ~ 359도, 511 = 없음
    if heading_deg is None or heading_deg < 0:
        hdg_encoded = 511
    else:
        hdg_encoded = min(359, max(0, int(round(heading_deg))))
    bits += format(hdg_encoded, '09b')

    # 12. Time Stamp (6 bits) - UTC 초 (0~59), 60=없음, 61=수동, 62=추정, 63=비작동
    if utc_second is None:
        utc_second = datetime.datetime.now(datetime.timezone.utc).second
    utc_second = max(0, min(59, utc_second))
    bits += format(utc_second, '06b')

    # 13. Special Manoeuvre (2 bits) - 0 = 없음
    bits += format(0, '02b')

    # 14. Spare (3 bits) - 예비, 0으로 설정
    bits += format(0, '03b')

    # 15. RAIM flag (1 bit) - 0 = RAIM 미사용
    bits += '0'

    # 16. Radio Status (19 bits) - SOTDMA 상태, 간소화하여 0
    bits += format(0, '019b')

    # 총 168비트 확인
    assert len(bits) == 168, f"Bit length mismatch: {len(bits)} != 168"

    # 비트 스트림을 Armored ASCII로 변환
    payload = _bits_to_armored_string(bits)

    return payload


def decode_ais_payload(payload):
    """
    AIS 페이로드를 디코딩하여 필드 값을 반환합니다.

    [디코딩 과정]
    1. 각 ASCII 문자를 6비트 값으로 변환 (역 Armoring)
    2. 비트 스트림 생성
    3. 비트맵에 따라 필드 추출

    매개변수:
        payload: 인코딩된 AIS 페이로드 문자열

    반환값:
        디코딩된 필드가 담긴 딕셔너리:
        - message_id: 메시지 타입
        - repeat_indicator: 반복 지시자
        - mmsi: 선박 식별번호
        - nav_status: 항해 상태
        - rot: 선회율
        - sog_knots: 대지속력 (노트)
        - position_accuracy: 위치 정확도
        - lon_deg: 경도 (도)
        - lat_deg: 위도 (도)
        - cog_deg: 대지침로 (도)
        - heading_deg: 진방위 (도)
        - utc_second: UTC 초
        - special_manoeuvre: 특수 조종
        - raim: RAIM 플래그
        - radio_status: 라디오 상태
    """
    # 1. ASCII를 6비트 값으로 변환 (역 Armoring)
    bits = ''
    for char in payload:
        ascii_val = ord(char)
        # 역 Armoring: ASCII >= 88이면 -56, 아니면 -48
        if ascii_val >= 88:
            v = ascii_val - 56
        else:
            v = ascii_val - 48
        bits += format(v, '06b')

    # 2. 비트맵에 따라 필드 추출
    def extract_bits(start, length):
        return bits[start:start+length]

    def bits_to_uint(bit_str):
        return int(bit_str, 2)

    def bits_to_signed(bit_str, bits_len):
        """2의 보수를 부호 있는 정수로 변환"""
        val = int(bit_str, 2)
        if val >= (1 << (bits_len - 1)):
            val -= (1 << bits_len)
        return val

    result = {}

    # Message ID (bits 0-5)
    result['message_id'] = bits_to_uint(extract_bits(0, 6))

    # Repeat Indicator (bits 6-7)
    result['repeat_indicator'] = bits_to_uint(extract_bits(6, 2))

    # MMSI (bits 8-37)
    result['mmsi'] = bits_to_uint(extract_bits(8, 30))

    # Navigation Status (bits 38-41)
    result['nav_status'] = bits_to_uint(extract_bits(38, 4))

    # Rate of Turn (bits 42-49) - 부호 있는 8비트
    result['rot'] = bits_to_signed(extract_bits(42, 8), 8)

    # Speed Over Ground (bits 50-59)
    sog_raw = bits_to_uint(extract_bits(50, 10))
    result['sog_knots'] = sog_raw / 10.0

    # Position Accuracy (bit 60)
    result['position_accuracy'] = bits_to_uint(extract_bits(60, 1))

    # Longitude (bits 61-88) - 2의 보수, 28비트
    lon_raw = bits_to_signed(extract_bits(61, 28), 28)
    result['lon_deg'] = lon_raw / 10000.0 / 60.0

    # Latitude (bits 89-115) - 2의 보수, 27비트
    lat_raw = bits_to_signed(extract_bits(89, 27), 27)
    result['lat_deg'] = lat_raw / 10000.0 / 60.0

    # Course Over Ground (bits 116-127)
    cog_raw = bits_to_uint(extract_bits(116, 12))
    result['cog_deg'] = cog_raw / 10.0

    # True Heading (bits 128-136)
    result['heading_deg'] = bits_to_uint(extract_bits(128, 9))

    # Time Stamp (bits 137-142)
    result['utc_second'] = bits_to_uint(extract_bits(137, 6))

    # Special Manoeuvre (bits 143-144)
    result['special_manoeuvre'] = bits_to_uint(extract_bits(143, 2))

    # Spare (bits 145-147) - 무시

    # RAIM flag (bit 148)
    result['raim'] = bits_to_uint(extract_bits(148, 1))

    # Radio Status (bits 149-167)
    result['radio_status'] = bits_to_uint(extract_bits(149, 19))

    return result


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
        if stype == 'TTM':
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

    except:
        pass

    return fields
