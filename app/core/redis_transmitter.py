"""
Redis Bbox 전송 모듈

이 모듈은 카메라 탐지 Bounding Box 정보를 Redis로 전송합니다.

클래스:
    RedisConfig: Redis 연결 설정
    RedisBboxTransmitter: Bbox 전송 클래스

전송 형식:
    Key: "infer:eo" 또는 "infer:ir"
    Value: {"prediction": [[left, top, right, bottom, confidence, class_id], ...]}
"""

import json
from dataclasses import dataclass
from typing import List, Optional

try:
    import redis
    REDIS_AVAILABLE = True
except ImportError:
    REDIS_AVAILABLE = False


@dataclass
class RedisConfig:
    """
    Redis 연결 설정

    속성:
        enabled: Redis 전송 활성화 여부
        host: Redis 서버 호스트
        port: Redis 서버 포트
        password: Redis 비밀번호 (선택)
        eo_key: EO 카메라 bbox 키
        ir_key: IR 카메라 bbox 키
        use_tls: TLS 사용 여부
        timeout_sec: 연결 타임아웃 (초)
    """
    enabled: bool = False
    host: str = "127.0.0.1"
    port: int = 6379
    password: Optional[str] = None
    eo_key: str = "infer:eo"
    ir_key: str = "infer:ir"
    use_tls: bool = False
    timeout_sec: float = 5.0


class RedisBboxTransmitter:
    """
    Redis로 Bbox 정보를 전송하는 클래스

    속성:
        _config: Redis 설정
        _client: Redis 클라이언트
        _connected: 연결 상태
    """

    def __init__(self, config: RedisConfig):
        """
        RedisBboxTransmitter를 초기화합니다.

        매개변수:
            config: Redis 설정
        """
        self._config = config
        self._client: Optional['redis.Redis'] = None
        self._connected = False

        if config.enabled and REDIS_AVAILABLE:
            self.connect()

    def connect(self) -> bool:
        """
        Redis 서버에 연결합니다.

        반환값:
            연결 성공 여부
        """
        if not REDIS_AVAILABLE:
            return False

        if not self._config.enabled:
            return False

        try:
            if self._config.use_tls:
                pool = redis.ConnectionPool(
                    host=self._config.host,
                    port=self._config.port,
                    db=0,
                    socket_connect_timeout=self._config.timeout_sec,
                    socket_timeout=self._config.timeout_sec,
                    connection_class=redis.SSLConnection,
                    ssl_cert_reqs=None,
                    password=self._config.password,
                )
            else:
                pool = redis.ConnectionPool(
                    host=self._config.host,
                    port=self._config.port,
                    db=0,
                    socket_connect_timeout=self._config.timeout_sec,
                    socket_timeout=self._config.timeout_sec,
                    password=self._config.password,
                )

            self._client = redis.Redis(connection_pool=pool, decode_responses=True)
            # 연결 테스트
            self._client.ping()
            self._connected = True
            return True

        except Exception as e:
            self._connected = False
            self._client = None
            return False

    def disconnect(self):
        """Redis 연결을 종료합니다."""
        if self._client:
            try:
                self._client.close()
            except Exception:
                pass
        self._client = None
        self._connected = False

    def is_connected(self) -> bool:
        """연결 상태를 반환합니다."""
        return self._connected and self._client is not None

    def send_bboxes(self, camera_type: str, bboxes: List[dict]) -> bool:
        """
        Bbox 정보를 Redis로 전송합니다.

        매개변수:
            camera_type: "EO" 또는 "IR"
            bboxes: Bbox 딕셔너리 목록
                각 딕셔너리는 'left', 'top', 'right', 'bottom', 'confidence', 'class_id' 키를 가짐

        반환값:
            전송 성공 여부
        """
        if not self.is_connected():
            # 재연결 시도
            if not self.connect():
                return False

        key = self._config.eo_key if camera_type.upper() == "EO" else self._config.ir_key

        # 전송 형식 구성
        prediction = []
        for bbox in bboxes:
            prediction.append([
                bbox.get('left', 0),
                bbox.get('top', 0),
                bbox.get('right', 0),
                bbox.get('bottom', 0),
                bbox.get('confidence', 1.0),
                bbox.get('class_id', 0)
            ])

        payload = {"prediction": prediction}

        try:
            dump_data = json.dumps(payload)
            self._client.set(key, dump_data)
            return True
        except Exception as e:
            self._connected = False
            return False

    def send_detections(self, camera_type: str, detections: List[dict]) -> bool:
        """
        탐지 딕셔너리 목록을 Redis로 전송합니다.

        매개변수:
            camera_type: "EO" 또는 "IR"
            detections: 탐지 딕셔너리 목록 (generate_detection_dict 출력 형식)

        반환값:
            전송 성공 여부
        """
        from app.core.camera_projection import get_ship_class_id

        bboxes = []
        for det in detections:
            bboxes.append({
                'left': det.get('left', 0),
                'top': det.get('top', 0),
                'right': det.get('right', 0),
                'bottom': det.get('bottom', 0),
                'confidence': 1.0,  # 시뮬레이션이므로 항상 1.0
                'class_id': get_ship_class_id(det.get('ship_class', 'OTHER'))
            })

        return self.send_bboxes(camera_type, bboxes)

    def update_config(self, config: RedisConfig):
        """설정을 업데이트하고 재연결합니다."""
        self.disconnect()
        self._config = config
        if config.enabled:
            self.connect()
