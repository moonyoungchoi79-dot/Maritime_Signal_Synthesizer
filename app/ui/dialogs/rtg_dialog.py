from PyQt6.QtWidgets import (
    QDialog, QFormLayout, QComboBox, QDoubleSpinBox, QLabel, QSpinBox, QDialogButtonBox
)

from app.core.models.project import current_project
from app.core.models.ship import SHIP_CLASS_DIMENSIONS

class RTGDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Random Target Generate Settings")
        self.resize(420, 320)

        l = QFormLayout(self)

        self.combo_mode = QComboBox()
        self.combo_mode.addItem("Circle around Own Ship", -1)
        for area in current_project.areas:
            self.combo_mode.addItem(f"Area: {area.name}", area.id)
        self.combo_mode.currentIndexChanged.connect(self.on_mode_changed)
        l.addRow("Generation Zone:", self.combo_mode)

        self.spin_R = QDoubleSpinBox()
        self.spin_R.setRange(0.1, 1000.0)  # R in nautical miles
        self.spin_R.setValue(10.0)
        self.spin_R.setSingleStep(1.0)
        self.spin_R.setDecimals(1)

        self.label_R = QLabel("Distance R (nm):")
        l.addRow(self.label_R, self.spin_R)

        # Ship class selection
        self.combo_ship_class = QComboBox()
        for ship_class in SHIP_CLASS_DIMENSIONS.keys():
            dims = SHIP_CLASS_DIMENSIONS[ship_class]
            self.combo_ship_class.addItem(f"{ship_class} ({dims[0]:.0f}m)", ship_class)
        self.combo_ship_class.setCurrentIndex(0)  # Default: CONTAINER
        l.addRow("Ship Class:", self.combo_ship_class)

        self.spin_nai = QSpinBox()
        self.spin_nai.setRange(0, 100)
        self.spin_nai.setValue(1)
        l.addRow("Targets (AI only):", self.spin_nai)

        self.spin_nra = QSpinBox()
        self.spin_nra.setRange(0, 100)
        self.spin_nra.setValue(1)
        l.addRow("Targets (RA only):", self.spin_nra)

        self.spin_nboth = QSpinBox()
        self.spin_nboth.setRange(0, 100)
        self.spin_nboth.setValue(1)
        l.addRow("Targets (Both AI & RA):", self.spin_nboth)

        bb = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        l.addWidget(bb)

    def on_mode_changed(self):
        is_circle = (self.combo_mode.currentData() == -1)
        self.spin_R.setVisible(is_circle)
        self.label_R.setVisible(is_circle)

    def get_data(self):
        return {
            "area_id": self.combo_mode.currentData(),
            "R": self.spin_R.value(),
            "ship_class": self.combo_ship_class.currentData(),
            "N_AI_only": self.spin_nai.value(),
            "N_RA_only": self.spin_nra.value(),
            "N_both": self.spin_nboth.value()
        }
