from PyQt6.QtWidgets import (
    QWidget, QHBoxLayout, QSpinBox, QDoubleSpinBox
)

class TimeInputWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0,0,0,0)
        layout.setSpacing(5)
        
        self.spin_d = QSpinBox(); self.spin_d.setRange(0, 365); self.spin_d.setSuffix("d")
        self.spin_h = QSpinBox(); self.spin_h.setRange(0, 23); self.spin_h.setSuffix("h")
        self.spin_m = QSpinBox(); self.spin_m.setRange(0, 59); self.spin_m.setSuffix("m")
        self.spin_s = QDoubleSpinBox(); self.spin_s.setRange(0, 59.99); self.spin_s.setSuffix("s")
        
        layout.addWidget(self.spin_d)
        layout.addWidget(self.spin_h)
        layout.addWidget(self.spin_m)
        layout.addWidget(self.spin_s)
        
    def set_seconds(self, total_seconds):
        d = int(total_seconds // 86400)
        rem = total_seconds % 86400
        h = int(rem // 3600)
        rem = rem % 3600
        m = int(rem // 60)
        s = rem % 60
        
        self.spin_d.setValue(d)
        self.spin_h.setValue(h)
        self.spin_m.setValue(m)
        self.spin_s.setValue(s)
        
    def get_seconds(self):
        d = self.spin_d.value()
        h = self.spin_h.value()
        m = self.spin_m.value()
        s = self.spin_s.value()
        return d * 86400 + h * 3600 + m * 60 + s
