"""
NMEA 신호 생성 및 처리 모듈

이 모듈은 해상 통신에 사용되는 NMEA 0183 형식의 메시지를 생성하고 파싱합니다.
AIS(Automatic Identification System) 신호의 인코딩/디코딩과 NMEA 체크섬 계산을 제공합니다.

주요 기능:
- NMEA 체크섬 계산 (XOR 기반)
- AIS Message Type 1 페이로드 인코딩 (선박 위치 보고)
- AIS 페이로드 디코딩 (Armored ASCII -> 이진 -> 필드 추출)
- NMEA 메시지 필드 파싱 (TTM, CAMERA 등)

AIS 인코딩 과정:
    1. 각 필드를 지정된 비트 수로 이진 변환
    2. 음수 값은 2의 보수(Two's Complement)로 변환
    3. 168비트 비트스트림을 6비트 단위로 분할
    4. 각 6비트 청크를 Armored ASCII 문자로 변환
       - 값 0~39: ASCII 48~87 ('0'~'W')
       - 값 40~63: ASCII 96~119 ('`'~'w')

AIS 디코딩 과정:
    1. Armored ASCII 문자를 6비트 값으로 역변환
    2. 6비트 값들을 연결하여 비트스트림 복원
    3. 비트스트림에서 각 필드를 지정된 위치/길이로 추출
    4. 2의 보수로 인코딩된 필드는 부호 있는 정수로 변환

NMEA 0183 형식:
    - 시작 문자: '$' (일반) 또는 '!' (AIS)
    - 필드 구분: ','
    - 체크섬 구분: '*'
    - 체크섬: 시작 문자와 '*' 사이 모든 문자의 XOR (16진수 2자리)
    - 예: !AIVDM,1,1,,A,13u@DR0P00PRD6=PNR2P00000000,0*5A

참조:
    - ITU-R M.1371-5: AIS 기술 표준
    - IEC 61162-1: NMEA 0183 인터페이스 표준
"""

import datetime


def calculate_checksum(nmea_str):
    """
    NMEA 메시지의 체크섬을 계산합니다.

    NMEA 체크섬은 메시지의 모든 문자에 대해 XOR 연산을 수행하여 계산됩니다.
    시작 문자('$' 또는 '!')와 체크섬 구분자('*') 이후는 제외됩니다.

    계산 과정:
        1. 초기값 0으로 시작
        2. 각 문자의 ASCII 값을 순차적으로 XOR
        3. 결과를 2자리 대문자 16진수로 변환

    매개변수:
        nmea_str: 체크섬을 계산할 NMEA 문자열
                  (시작 문자 '$' 또는 '!' 제외, '*' 이전까지)

    반환값:
        2자리 대문자 16진수 체크섬 문자열 (예: "5A", "0F")

    예시:
        >>> calculate_checksum("AIVDM,1,1,,A,13u@DR0P00PRD6=PNR2P00000000,0")
        '5A'
    """
    calc_cksum = 0
    for s in nmea_str:
        calc_cksum ^= ord(s)
    return hex(calc_cksum)[2:].upper().zfill(2)


def _int_to_twos_complement(value, bits):
    """
    정수를 2의 보수(Two's Complement) 표현으로 변환합니다.

    AIS 메시지에서 위도, 경도, 선회율(ROT) 등 음수가 가능한 필드에 사용됩니다.

    변환 과정:
        - 양수: 그대로 반환
        - 음수: 2^bits + value (예: -1을 8비트로 → 255)

    매개변수:
        value: 변환할 정수 값 (양수 또는 음수)
        bits: 비트 수 (표현 범위 결정)

    반환값:
        2의 보수로 변환된 부호 없는 정수

    예시:
        >>> _int_to_twos_complement(-1, 8)
        255  # 이진수: 11111111
        >>> _int_to_twos_complement(127, 8)
        127  # 이진수: 01111111
        >>> _int_to_twos_complement(-128, 8)
        128  # 이진수: 10000000
    """
    if value >= 0:
        return value
    else:
        return (1 << bits) + value


def _bits_to_armored_string(bit_string):
    """
    비트 문자열을 AIS Armored ASCII 문자열로 변환합니다.

    AIS 페이로드는 6비트 단위로 인코딩되어 Armored ASCII로 표현됩니다.
    이 인코딩 방식은 NMEA 메시지에서 안전하게 전송 가능한 문자만 사용합니다.

    Armored ASCII 변환 규칙:
        1. 비트 문자열을 6비트 청크로 분할
        2. 마지막 청크가 6비트 미만이면 '0'으로 패딩
        3. 각 6비트 값(0~63)을 ASCII 문자로 변환:
           - 값 0~39 → ASCII 48~87 (문자 '0'~'W')
             공식: ascii_val = value + 48
           - 값 40~63 → ASCII 96~119 (문자 '`'~'w')
             공식: ascii_val = value + 56

    이 인코딩의 특징:
        - 숫자 0-9 (ASCII 48-57): 값 0-9
        - 문자 :;<=>?@ (ASCII 58-64): 값 10-16
        - 대문자 A-W (ASCII 65-87): 값 17-39
        - 문자 `abcdefghijklmnopqrstuvw (ASCII 96-119): 값 40-63

    매개변수:
        bit_string: '0'과 '1'로 구성된 비트 문자열

    반환값:
        Armored ASCII로 인코딩된 문자열

    예시:
        >>> _bits_to_armored_string("000001000010")  # 값 1, 값 2
        '12'  # ASCII 49, 50
    """
    result = []
    for i in range(0, len(bit_string), 6):
        chunk = bit_string[i:i+6]
        # 마지막 청크가 6비트 미만이면 0으로 패딩
        if len(chunk) < 6:
            chunk = chunk + '0' * (6 - len(chunk))
        v = int(chunk, 2)
        # Armored ASCII 변환
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
    AIS Message Type 1 (Class A 위치 보고) 페이로드를 인코딩합니다.

    Message Type 1은 Class A 선박용 예정된 위치 보고로,
    선박의 실시간 위치, 속도, 방향 정보를 담습니다.
    총 168비트(28개 Armored ASCII 문자)로 구성됩니다.

    비트 필드 구조 (ITU-R M.1371-5 기준):
        Bit 0-5 (6비트): Message ID = 1
        Bit 6-7 (2비트): Repeat Indicator (기본값 0)
        Bit 8-37 (30비트): MMSI (해상이동업무식별번호)
        Bit 38-41 (4비트): Navigation Status (항해 상태)
        Bit 42-49 (8비트): Rate of Turn (선회율, 2의 보수)
        Bit 50-59 (10비트): Speed Over Ground (대지속력 × 10)
        Bit 60 (1비트): Position Accuracy
        Bit 61-88 (28비트): Longitude (경도 × 60 × 10000, 2의 보수)
        Bit 89-115 (27비트): Latitude (위도 × 60 × 10000, 2의 보수)
        Bit 116-127 (12비트): Course Over Ground (대지침로 × 10)
        Bit 128-136 (9비트): True Heading (진방위)
        Bit 137-142 (6비트): Time Stamp (UTC 초)
        Bit 143-144 (2비트): Special Manoeuvre
        Bit 145-147 (3비트): Spare
        Bit 148 (1비트): RAIM flag
        Bit 149-167 (19비트): Radio Status

    위치 인코딩 상세:
        - 경도: 도(degree) → 분(minute) 변환 (×60) → 10000배 (×10000)
          범위: -180° ~ +180° (-108000000 ~ +108000000)
          181° (0x6791AC0) = 사용 불가/없음
        - 위도: 도(degree) → 분(minute) 변환 (×60) → 10000배 (×10000)
          범위: -90° ~ +90° (-54000000 ~ +54000000)
          91° (0x3412140) = 사용 불가/없음

    속도 인코딩:
        - SOG: 실제 속도(노트) × 10, 범위 0~1022 (0~102.2노트)
          1023 = 사용 불가/없음

    방향 인코딩:
        - COG: 실제 각도 × 10, 범위 0~3599 (0.0°~359.9°)
          3600 = 사용 불가/없음
        - Heading: 정수 각도, 범위 0~359
          511 = 사용 불가/없음

    매개변수:
        mmsi: 9자리 MMSI 번호 (0~999999999)
        lat_deg: 위도 (도 단위, -90 ~ +90)
        lon_deg: 경도 (도 단위, -180 ~ +180)
        sog_knots: 대지속력 (노트, 0 ~ 102.2)
        cog_deg: 대지침로 (도 단위, 0 ~ 359.9)
        heading_deg: 진방위 (도 단위, 0 ~ 359, None이면 511=없음)
        nav_status: 항해 상태 (0=항해중, 1=정박, 5=계류 등)
        rot: 선회율 (도/분, -128~127, 0=선회 안함)
        utc_second: UTC 초 (0~59, None이면 현재 시간 사용)

    반환값:
        Armored ASCII로 인코딩된 28자 페이로드 문자열

    예시:
        >>> encode_ais_payload(123456789, lat_deg=37.5, lon_deg=126.9, sog_knots=10.5)
        '13u@DR0P00PRD6=PNR2P00000000'  # 예시 출력
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
    AIS 페이로드를 디코딩하여 각 필드를 추출합니다.

    Armored ASCII로 인코딩된 AIS 페이로드를 비트스트림으로 변환한 후,
    ITU-R M.1371-5 표준에 따라 각 필드를 추출합니다.

    디코딩 과정:
        1. Armored ASCII 역변환:
           - ASCII 88 이상: value = ascii_val - 56
           - ASCII 88 미만: value = ascii_val - 48
        2. 각 6비트 값을 이진 문자열로 변환
        3. 모든 이진 문자열을 연결하여 비트스트림 생성
        4. 비트스트림에서 각 필드의 위치와 길이에 따라 값 추출
        5. 2의 보수로 인코딩된 필드는 부호 있는 정수로 변환

    위치 디코딩:
        - 비트스트림에서 28비트(경도) 또는 27비트(위도) 추출
        - 2의 보수 변환으로 부호 있는 정수 획득
        - 10000으로 나누어 분(minute) 단위로 변환
        - 60으로 나누어 도(degree) 단위로 변환

    매개변수:
        payload: Armored ASCII로 인코딩된 AIS 페이로드 문자열

    반환값:
        디코딩된 필드가 담긴 딕셔너리:
        - message_id: 메시지 타입 (1, 2, 3 등)
        - repeat_indicator: 반복 지시자 (0~3)
        - mmsi: 해상이동업무식별번호 (9자리)
        - nav_status: 항해 상태 (0~15)
        - rot: 선회율 (도/분, -128~127)
        - sog_knots: 대지속력 (노트)
        - position_accuracy: 위치 정확도 (0=낮음, 1=높음)
        - lon_deg: 경도 (도 단위)
        - lat_deg: 위도 (도 단위)
        - cog_deg: 대지침로 (도 단위)
        - heading_deg: 진방위 (도 단위, 511=없음)
        - utc_second: UTC 초
        - special_manoeuvre: 특수 조종 (0~2)
        - raim: RAIM 플래그 (0 또는 1)
        - radio_status: 라디오 상태

    예시:
        >>> result = decode_ais_payload("13u@DR0P00PRD6=PNR2P00000000")
        >>> print(result['mmsi'], result['lat_deg'], result['lon_deg'])
    """
    # Armored ASCII를 비트스트림으로 역변환
    bits = ''
    for char in payload:
        ascii_val = ord(char)
        # 역변환: ASCII 88 이상이면 -56, 미만이면 -48
        if ascii_val >= 88:
            v = ascii_val - 56
        else:
            v = ascii_val - 48
        bits += format(v, '06b')

    def extract_bits(start, length):
        """비트스트림에서 지정된 위치의 비트를 추출합니다."""
        return bits[start:start+length]

    def bits_to_uint(bit_str):
        """비트 문자열을 부호 없는 정수로 변환합니다."""
        return int(bit_str, 2)

    def bits_to_signed(bit_str, bits_len):
        """
        2의 보수를 부호 있는 정수로 변환합니다.

        MSB가 1이면 음수로 처리합니다.
        """
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
    NMEA 메시지를 파싱하여 필드를 추출합니다.

    다양한 NMEA 메시지 형식을 지원하며, 메시지 타입에 따라
    적절한 필드를 추출합니다.

    지원하는 메시지 타입:
        - TTM (Tracked Target Message): 레이더 추적 목표 정보
        - CAMERA: 카메라 탐지 신호 (커스텀 형식)

    NMEA 메시지 구조:
        - 시작 문자: '$' (일반) 또는 '!' (AIS)
        - Talker ID: 2자 (예: 'RA'=레이더, 'AI'=AIS)
        - Sentence Type: 메시지 타입 (예: 'TTM', 'VDM')
        - 필드들: 쉼표로 구분
        - 체크섬: '*' 뒤 2자리 16진수

    CAMERA 메시지 형식:
        $CAMERA,track_id,class,rel_bearing,pano_w,pano_h,cx,cy,w,h*CS
        - track_id: 추적 대상 ID
        - class: 선박 유형 (CONTAINER, TANKER 등)
        - rel_bearing: 상대 방위각 (도)
        - pano_w/h: 파노라마 이미지 크기 (픽셀)
        - cx/cy: 바운딩 박스 중심 좌표 (픽셀)
        - w/h: 바운딩 박스 크기 (픽셀)

    매개변수:
        raw: 원본 NMEA 메시지 문자열

    반환값:
        파싱된 필드를 담은 딕셔너리:
        - talker: Talker ID (예: 'RA', 'AI', '')
        - sentence_type: 메시지 타입 (예: 'TTM', 'CAMERA')
        - raw: 원본 메시지 문자열
        - (메시지 타입별 추가 필드)

    예시:
        >>> fields = parse_nmea_fields("$CAMERA,1,CONTAINER,45.0,1920,320,960,192,50,30*AB")
        >>> print(fields['ship_class'], fields['rel_bearing_deg'])
        'CONTAINER' 45.0
    """
    fields = {}
    raw_str = raw.strip()

    # NMEA 메시지 형식 확인
    if not raw_str.startswith("$") and not raw_str.startswith("!"):
        return fields

    # 체크섬 제거 및 필드 분리
    parts = raw_str.split('*')[0].split(',')

    # 헤더 파싱 (Talker ID + Sentence Type)
    header = parts[0]
    if header.startswith('$') or header.startswith('!'):
        header = header[1:]

    if header == 'CAMERA':
        fields['talker'] = ''
        fields['sentence_type'] = 'CAMERA'
    else:
        fields['talker'] = header[:2]
        fields['sentence_type'] = header[2:]
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
            # CAMERA: 카메라 탐지 신호
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
                    # 바운딩박스 높이 (체크섬 분리 처리)
                    bbox_h_str = parts[9].split('*')[0] if '*' in parts[9] else parts[9]
                    fields['bbox_h_px'] = float(bbox_h_str) if bbox_h_str else 0.0
                except:
                    pass
    except:
        pass

    return fields
