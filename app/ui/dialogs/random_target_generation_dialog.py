"""
랜덤 타겟 생성(RTG) 다이얼로그 모듈

이 모듈은 시뮬레이션 중 무작위 선박을 생성하기 위한 다이얼로그를 제공합니다.
자선 주변 원형 영역 또는 지정된 Area 내에 랜덤 타겟을 생성할 수 있습니다.

클래스:
    RTGDialog: 랜덤 타겟 생성 설정 다이얼로그
"""

from PyQt6.QtWidgets import (
    QDialog, QFormLayout, QComboBox, QDoubleSpinBox, QLabel, QSpinBox, QDialogButtonBox
)

from app.models.project import current_project
from app.models.ship import SHIP_CLASS_DIMENSIONS


class RTGDialog(QDialog):
    """
    랜덤 타겟 생성 설정 다이얼로그 클래스입니다.

    생성 구역, 선박 클래스, 신호 유형별 타겟 수를 설정할 수 있습니다.

    속성:
        combo_mode: 생성 구역 선택 (원형 또는 Area)
        spin_R: 원형 생성 시 반경(NM)
        combo_ship_class: 선박 클래스 선택
        spin_nai: AIS 전용 타겟 수
        spin_nra: RATTM 전용 타겟 수
        spin_nboth: AIS+RATTM 모두 발신하는 타겟 수
    """

    def __init__(self, parent=None):
        """
        RTGDialog를 초기화합니다.

        매개변수:
            parent: 부모 위젯 (기본값: None)
        """
        super().__init__(parent)
        self.setWindowTitle("Random Target Generate Settings")
        self.resize(420, 320)

        l = QFormLayout(self)

        # 생성 구역 선택 (원형 또는 정의된 Area)
        self.combo_mode = QComboBox()
        self.combo_mode.addItem("Circle around Own Ship", -1)
        for area in current_project.areas:
            self.combo_mode.addItem(f"Area: {area.name}", area.id)
        self.combo_mode.currentIndexChanged.connect(self.on_mode_changed)
        l.addRow("Generation Zone:", self.combo_mode)

        # 원형 생성 시 반경 설정
        self.spin_R = QDoubleSpinBox()
        self.spin_R.setRange(0.1, 1000.0)
        self.spin_R.setValue(10.0)
        self.spin_R.setSingleStep(1.0)
        self.spin_R.setDecimals(1)

        self.label_R = QLabel("Distance R (nm):")
        l.addRow(self.label_R, self.spin_R)

        # 선박 클래스 선택
        self.combo_ship_class = QComboBox()
        for ship_class in SHIP_CLASS_DIMENSIONS.keys():
            dims = SHIP_CLASS_DIMENSIONS[ship_class]
            self.combo_ship_class.addItem(f"{ship_class} ({dims[0]:.0f}m)", ship_class)
        self.combo_ship_class.setCurrentIndex(0)
        l.addRow("Ship Class:", self.combo_ship_class)

        # AIS 전용 타겟 수
        self.spin_nai = QSpinBox()
        self.spin_nai.setRange(0, 100)
        self.spin_nai.setValue(1)
        l.addRow("Targets (AI only):", self.spin_nai)

        # RATTM 전용 타겟 수
        self.spin_nra = QSpinBox()
        self.spin_nra.setRange(0, 100)
        self.spin_nra.setValue(1)
        l.addRow("Targets (RA only):", self.spin_nra)

        # AIS+RATTM 모두 발신하는 타겟 수
        self.spin_nboth = QSpinBox()
        self.spin_nboth.setRange(0, 100)
        self.spin_nboth.setValue(1)
        l.addRow("Targets (Both AI & RA):", self.spin_nboth)

        bb = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        l.addWidget(bb)

    def on_mode_changed(self):
        """
        생성 구역 모드 변경 시 UI를 업데이트합니다.

        원형 모드일 때만 반경(R) 입력 필드를 표시합니다.
        Area 모드에서는 해당 Area의 경계 내에 생성되므로 반경이 필요 없습니다.
        """
        is_circle = (self.combo_mode.currentData() == -1)
        self.spin_R.setVisible(is_circle)
        self.label_R.setVisible(is_circle)

    def get_data(self):
        """
        입력된 설정 데이터를 반환합니다.

        반환값:
            dict: 랜덤 타겟 생성 설정
                - area_id: 생성 구역 ID (-1이면 원형 모드)
                - R: 원형 생성 시 반경 (NM)
                - ship_class: 선박 클래스
                - N_AI_only: AIS 전용 타겟 수
                - N_RA_only: RATTM 전용 타겟 수
                - N_both: AIS+RATTM 타겟 수
        """
        return {
            "area_id": self.combo_mode.currentData(),
            "R": self.spin_R.value(),
            "ship_class": self.combo_ship_class.currentData(),
            "N_AI_only": self.spin_nai.value(),
            "N_RA_only": self.spin_nra.value(),
            "N_both": self.spin_nboth.value()
        }
