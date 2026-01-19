"""
상수 정의 모듈

이 모듈은 애플리케이션 전반에서 사용되는 상수와 설정값을 정의합니다.

주요 상수:
- APP_NAME: 애플리케이션 이름
- DEFAULT_WINDOW_SIZE: 기본 윈도우 크기
- CSV_HEADER: NMEA 신호 로그 CSV 파일 헤더
- logger: 로깅 인스턴스
"""

import logging

# 애플리케이션 기본 정보
APP_NAME = "Maritime Signal Synthesizer"  # 애플리케이션 표시 이름
DEFAULT_WINDOW_SIZE = (1600, 900)  # 기본 윈도우 크기 (너비, 높이) 픽셀

# 로깅 설정
# 로그 레벨 INFO, 타임스탬프-레벨-메시지 형식
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger("NMEA_Gen")  # NMEA 생성기 전용 로거

# NMEA 신호 로그 CSV 파일 헤더 정의
# 각 필드 설명:
# - id: 신호 고유 식별자
# - rx_time: 수신 시간 (UTC)
# - receiver_ship_index: 수신 선박 인덱스
# - receiver_ship_name: 수신 선박 이름
# - talker: NMEA 토커 ID (예: AI, RA 등)
# - sentence_type: NMEA 문장 유형 (예: VDM, TTM 등)
# - raw: 원본 NMEA 문장
CSV_HEADER = [
    "id", "rx_time", "receiver_ship_index", "receiver_ship_name", "talker", "sentence_type", "raw"
]