"""
시간 입력 위젯 모듈

이 모듈은 일/시/분/초 형식으로 시간을 입력받는 커스텀 위젯을 제공합니다.

클래스:
    TimeInputWidget: 시간 입력 위젯

사용 예시:
    widget = TimeInputWidget()
    widget.set_seconds(3661)  # 1시간 1분 1초
    total = widget.get_seconds()  # 3661
"""

from PyQt6.QtWidgets import (
    QWidget, QHBoxLayout, QSpinBox, QDoubleSpinBox
)
from PyQt6.QtCore import pyqtSignal


class TimeInputWidget(QWidget):
    """
    일/시/분/초 형식으로 시간을 입력받는 위젯입니다.

    시뮬레이션 시간이나 이벤트 트리거 시간을 설정할 때 사용됩니다.
    총 시간은 초 단위로 변환되어 처리됩니다.

    시그널:
        valueChanged(float): 값이 변경될 때 총 초 단위 값을 전달
    """

    # 값 변경 시 발생하는 시그널 (총 초 단위)
    valueChanged = pyqtSignal(float)

    def __init__(self, parent=None):
        """
        시간 입력 위젯을 초기화합니다.

        매개변수:
            parent: 부모 위젯
        """
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)

        # 일 입력 (0~365)
        self.spin_d = QSpinBox()
        self.spin_d.setRange(0, 365)
        self.spin_d.setSuffix("d")

        # 시 입력 (0~23)
        self.spin_h = QSpinBox()
        self.spin_h.setRange(0, 23)
        self.spin_h.setSuffix("h")

        # 분 입력 (0~59)
        self.spin_m = QSpinBox()
        self.spin_m.setRange(0, 59)
        self.spin_m.setSuffix("m")

        # 초 입력 (0~59.99)
        self.spin_s = QDoubleSpinBox()
        self.spin_s.setRange(0, 59.99)
        self.spin_s.setSuffix("s")

        # 값 변경 시 시그널 연결
        self.spin_d.valueChanged.connect(self.emit_value)
        self.spin_h.valueChanged.connect(self.emit_value)
        self.spin_m.valueChanged.connect(self.emit_value)
        self.spin_s.valueChanged.connect(self.emit_value)

        # 레이아웃에 추가
        layout.addWidget(self.spin_d)
        layout.addWidget(self.spin_h)
        layout.addWidget(self.spin_m)
        layout.addWidget(self.spin_s)

    def emit_value(self):
        """현재 값을 시그널로 전달합니다."""
        self.valueChanged.emit(self.get_seconds())

    def set_seconds(self, total_seconds):
        """
        총 초를 일/시/분/초로 변환하여 설정합니다.

        매개변수:
            total_seconds: 총 시간 (초 단위)
        """
        # 일 계산
        d = int(total_seconds // 86400)
        rem = total_seconds % 86400

        # 시 계산
        h = int(rem // 3600)
        rem = rem % 3600

        # 분 계산
        m = int(rem // 60)

        # 초 계산
        s = rem % 60

        # 값 설정
        self.spin_d.setValue(d)
        self.spin_h.setValue(h)
        self.spin_m.setValue(m)
        self.spin_s.setValue(s)

    def get_seconds(self):
        """
        현재 설정된 시간을 초 단위로 반환합니다.

        반환값:
            총 시간 (초 단위)
        """
        d = self.spin_d.value()
        h = self.spin_h.value()
        m = self.spin_m.value()
        s = self.spin_s.value()
        return d * 86400 + h * 3600 + m * 60 + s
