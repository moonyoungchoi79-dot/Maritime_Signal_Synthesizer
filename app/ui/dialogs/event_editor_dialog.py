import uuid
from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout, QLineEdit,
    QComboBox, QDialogButtonBox, QLabel, QDoubleSpinBox, QMessageBox,
    QGroupBox, QCheckBox, QPushButton, QListWidget, QListWidgetItem
)
from PyQt6.QtCore import Qt
from app.core.models.event import SimEvent, EventCondition
from app.core.models.project import current_project

class EventEditorDialog(QDialog):
    def __init__(self, parent=None, event=None):
        super().__init__(parent)
        self.setWindowTitle("Edit Event" if event else "New Event")
        self.event = event
        self.resize(500, 600)
        self.init_ui()
        if event:
            self.load_event()
        else:
            self.trigger_type_combo.setCurrentIndex(0)
            self.update_trigger_ui()
            self.action_type_combo.setCurrentIndex(0)
            self.update_action_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        
        form = QFormLayout()
        self.name_edit = QLineEdit("New Event")
        form.addRow("Name:", self.name_edit)
        
        self.enabled_chk = QCheckBox("Enabled")
        self.enabled_chk.setChecked(True)
        form.addRow("", self.enabled_chk)
        
        layout.addLayout(form)
        
        grp_trigger = QGroupBox("Trigger")
        trig_layout = QFormLayout(grp_trigger)
        
        self.trigger_type_combo = QComboBox()
        self.trigger_type_combo.addItems(["TIME", "AREA_ENTER", "AREA_LEAVE", "CPA_UNDER", "CPA_OVER", "DIST_UNDER", "DIST_OVER"])
        self.trigger_type_combo.currentIndexChanged.connect(self.update_trigger_ui)
        trig_layout.addRow("Type:", self.trigger_type_combo)
        
        self.target_ship_combo = QComboBox() 
        self.populate_ships(self.target_ship_combo)
        trig_layout.addRow("Target Ship:", self.target_ship_combo)
        
        self.ref_ship_combo = QComboBox() 
        self.populate_ships(self.ref_ship_combo, include_own=True) 
        self.ref_ship_label = QLabel("Ref Ship:")
        trig_layout.addRow(self.ref_ship_label, self.ref_ship_combo)
        
        self.condition_val_spin = QDoubleSpinBox()
        self.condition_val_spin.setRange(0, 9999999)
        self.condition_val_label = QLabel("Value:")
        trig_layout.addRow(self.condition_val_label, self.condition_val_spin)
        
        self.area_combo = QComboBox()
        self.populate_areas()
        self.area_label = QLabel("Area:")
        trig_layout.addRow(self.area_label, self.area_combo)
        
        layout.addWidget(grp_trigger)
        
        grp_action = QGroupBox("Action")
        act_layout = QFormLayout(grp_action)
        
        self.action_type_combo = QComboBox()
        self.action_type_combo.addItems(["STOP", "CHANGE_SPEED", "CHANGE_HEADING", "MANEUVER", "CHANGE_DESTINATION_LOC"])
        self.action_type_combo.currentIndexChanged.connect(self.update_action_ui)
        act_layout.addRow("Type:", self.action_type_combo)
        
        self.action_val_spin = QDoubleSpinBox()
        self.action_val_spin.setRange(0, 10000) 
        self.action_val_label = QLabel("Value:")
        act_layout.addRow(self.action_val_label, self.action_val_spin)

        self.action_opt_combo = QComboBox()
        self.action_opt_combo.addItems(["ReturnToOriginalPath_ShortestDistance", "ChangeDestination_ToOriginalFinal"])
        self.action_opt_label = QLabel("Option:")
        act_layout.addRow(self.action_opt_label, self.action_opt_combo)
        
        self.lat_spin = QDoubleSpinBox()
        self.lat_spin.setRange(-90, 90)
        self.lat_spin.setDecimals(6)
        self.lat_label = QLabel("Lat:")
        act_layout.addRow(self.lat_label, self.lat_spin)
        
        self.lon_spin = QDoubleSpinBox()
        self.lon_spin.setRange(-180, 180)
        self.lon_spin.setDecimals(6)
        self.lon_label = QLabel("Lon:")
        act_layout.addRow(self.lon_label, self.lon_spin)
        
        layout.addWidget(grp_action)

        # 조건부 이벤트 그룹
        grp_prereq = QGroupBox("Prerequisite Events (조건부 이벤트)")
        prereq_layout = QVBoxLayout(grp_prereq)

        # 논리 연산자 선택
        logic_layout = QHBoxLayout()
        logic_label = QLabel("Logic:")
        self.logic_combo = QComboBox()
        self.logic_combo.addItems(["AND (모든 조건 충족)", "OR (하나라도 충족)"])
        logic_layout.addWidget(logic_label)
        logic_layout.addWidget(self.logic_combo)
        logic_layout.addStretch()
        prereq_layout.addLayout(logic_layout)

        # 조건 추가 행
        add_layout = QHBoxLayout()
        self.prereq_event_combo = QComboBox()
        self.prereq_event_combo.setMinimumWidth(200)
        self._populate_event_combo()
        add_layout.addWidget(self.prereq_event_combo)

        self.prereq_mode_combo = QComboBox()
        self.prereq_mode_combo.addItems(["TRIGGERED (발동됨)", "NOT_TRIGGERED (발동 안됨)"])
        add_layout.addWidget(self.prereq_mode_combo)

        self.btn_add_prereq = QPushButton("+")
        self.btn_add_prereq.setFixedWidth(30)
        self.btn_add_prereq.clicked.connect(self._add_prerequisite)
        add_layout.addWidget(self.btn_add_prereq)
        prereq_layout.addLayout(add_layout)

        # 조건 목록
        self.prereq_list = QListWidget()
        self.prereq_list.setMaximumHeight(80)
        prereq_layout.addWidget(self.prereq_list)

        # 삭제 버튼
        self.btn_remove_prereq = QPushButton("Remove Selected")
        self.btn_remove_prereq.clicked.connect(self._remove_prerequisite)
        prereq_layout.addWidget(self.btn_remove_prereq)

        layout.addWidget(grp_prereq)

        btns = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        btns.accepted.connect(self.on_ok)
        btns.rejected.connect(self.reject)
        layout.addWidget(btns)

    def populate_ships(self, combo, include_own=True):
        combo.clear()
        if include_own:
             combo.addItem(f"Own Ship (Default)", -1)
             
        own_idx = current_project.settings.own_ship_idx
        for s in current_project.ships:
            tag = "Own" if s.idx == own_idx else ("R" if s.idx >= 1000 else "T")
            combo.addItem(f"[{tag}] {s.name}", s.idx)

    def populate_areas(self):
        self.area_combo.clear()
        for a in current_project.areas:
            self.area_combo.addItem(a.name, a.id)

    def _populate_event_combo(self):
        """다른 이벤트들로 콤보박스 채우기"""
        self.prereq_event_combo.clear()
        current_id = self.event.id if self.event else None
        for evt in current_project.events:
            if evt.id != current_id:  # 자기 자신 제외
                self.prereq_event_combo.addItem(evt.name, evt.id)

    def _add_prerequisite(self):
        """선행 조건 추가"""
        if self.prereq_event_combo.count() == 0:
            return

        event_id = self.prereq_event_combo.currentData()
        event_name = self.prereq_event_combo.currentText()
        mode = "TRIGGERED" if self.prereq_mode_combo.currentIndex() == 0 else "NOT_TRIGGERED"
        mode_text = "발동됨" if mode == "TRIGGERED" else "발동 안됨"

        # 중복 체크
        for i in range(self.prereq_list.count()):
            item = self.prereq_list.item(i)
            if item.data(Qt.ItemDataRole.UserRole) == event_id:
                QMessageBox.warning(self, "Warning", "이미 추가된 이벤트입니다.")
                return

        item = QListWidgetItem(f"{event_name} → {mode_text}")
        item.setData(Qt.ItemDataRole.UserRole, event_id)
        item.setData(Qt.ItemDataRole.UserRole + 1, mode)
        self.prereq_list.addItem(item)

    def _remove_prerequisite(self):
        """선택된 선행 조건 제거"""
        current_row = self.prereq_list.currentRow()
        if current_row >= 0:
            self.prereq_list.takeItem(current_row)

    def update_trigger_ui(self):
        ttype = self.trigger_type_combo.currentText()
        
        self.ref_ship_combo.setVisible(False)
        self.ref_ship_label.setVisible(False)
        self.area_combo.setVisible(False)
        self.area_label.setVisible(False)
        self.condition_val_spin.setVisible(False)
        self.condition_val_label.setVisible(False)
        
        if ttype == "TIME":
            self.condition_val_label.setText("Seconds:")
            self.condition_val_spin.setVisible(True)
            self.condition_val_label.setVisible(True)
        elif "AREA" in ttype:
            self.area_combo.setVisible(True)
            self.area_label.setVisible(True)
        elif "DIST" in ttype or "CPA" in ttype:
            self.ref_ship_combo.setVisible(True)
            self.ref_ship_label.setVisible(True)
            self.condition_val_label.setText("Dist(NM)/Time(min):")
            self.condition_val_spin.setVisible(True)
            self.condition_val_label.setVisible(True)

    def update_action_ui(self):
        atype = self.action_type_combo.currentText()
        self.action_val_spin.setVisible(False)
        self.action_val_label.setVisible(False)
        self.action_opt_combo.setVisible(False)
        self.action_opt_label.setVisible(False)
        self.lat_spin.setVisible(False)
        self.lat_label.setVisible(False)
        self.lon_spin.setVisible(False)
        self.lon_label.setVisible(False)
        
        if atype == "CHANGE_SPEED":
            self.action_val_label.setText("Speed (kn):")
            self.action_val_spin.setRange(0, 10000) 
            self.action_val_spin.setVisible(True)
            self.action_val_label.setVisible(True)
        elif atype == "CHANGE_HEADING":
            self.action_val_label.setText("Heading (deg):")
            self.action_val_spin.setRange(0, 360)
            self.action_val_spin.setVisible(True)
            self.action_val_label.setVisible(True)
        elif atype == "MANEUVER":
            self.action_opt_combo.setVisible(True)
            self.action_opt_label.setVisible(True)
        elif atype == "CHANGE_DESTINATION_LOC":
            self.lat_spin.setVisible(True)
            self.lat_label.setVisible(True)
            self.lon_spin.setVisible(True)
            self.lon_label.setVisible(True)

    def load_event(self):
        self.name_edit.setText(self.event.name)
        self.enabled_chk.setChecked(self.event.enabled)
        
        idx = self.target_ship_combo.findData(self.event.target_ship_idx)
        if idx >= 0: self.target_ship_combo.setCurrentIndex(idx)
        
        self.trigger_type_combo.setCurrentText(self.event.trigger_type)
        self.update_trigger_ui()
        
        if self.event.trigger_type in ["TIME", "DIST_UNDER", "DIST_OVER", "CPA_UNDER", "CPA_OVER"]:
            self.condition_val_spin.setValue(self.event.condition_value)
        elif "AREA" in self.event.trigger_type:
            idx = self.area_combo.findData(int(self.event.condition_value))
            if idx >= 0: self.area_combo.setCurrentIndex(idx)
            
        idx = self.ref_ship_combo.findData(self.event.reference_ship_idx)
        if idx >= 0: self.ref_ship_combo.setCurrentIndex(idx)
        
        self.action_type_combo.setCurrentText(self.event.action_type)
        self.update_action_ui()
        
        if self.event.action_type in ["CHANGE_SPEED", "CHANGE_HEADING"]:
            self.action_val_spin.setValue(self.event.action_value)
        elif self.event.action_type == "MANEUVER":
            idx = self.action_opt_combo.findText(self.event.action_option)
            if idx >= 0: self.action_opt_combo.setCurrentIndex(idx)
        elif self.event.action_type == "CHANGE_DESTINATION_LOC":
            try:
                lat, lon = map(float, self.event.action_option.split(","))
                self.lat_spin.setValue(lat)
                self.lon_spin.setValue(lon)
            except:
                pass

        # 조건부 이벤트 로드
        logic = getattr(self.event, 'prerequisite_logic', 'AND')
        self.logic_combo.setCurrentIndex(0 if logic == "AND" else 1)

        prereqs = getattr(self.event, 'prerequisite_events', [])
        self.prereq_list.clear()
        for cond in prereqs:
            # cond가 EventCondition 객체 또는 dict일 수 있음
            if isinstance(cond, dict):
                event_id = cond.get('event_id', '')
                mode = cond.get('mode', 'TRIGGERED')
            else:
                event_id = cond.event_id
                mode = cond.mode

            # 이벤트 이름 찾기
            event_name = event_id
            for evt in current_project.events:
                if evt.id == event_id:
                    event_name = evt.name
                    break

            mode_text = "발동됨" if mode == "TRIGGERED" else "발동 안됨"
            item = QListWidgetItem(f"{event_name} → {mode_text}")
            item.setData(Qt.ItemDataRole.UserRole, event_id)
            item.setData(Qt.ItemDataRole.UserRole + 1, mode)
            self.prereq_list.addItem(item)

    def on_ok(self):
        name = self.name_edit.text().strip()
        if not name:
            QMessageBox.warning(self, "Error", "Name required")
            return
            
        if not self.event:
            self.event = SimEvent(str(uuid.uuid4()), name)
        
        self.event.name = name
        self.event.enabled = self.enabled_chk.isChecked()
        self.event.target_ship_idx = self.target_ship_combo.currentData()
        self.event.trigger_type = self.trigger_type_combo.currentText()
        
        if self.event.trigger_type in ["TIME", "DIST_UNDER", "DIST_OVER", "CPA_UNDER", "CPA_OVER"]:
            self.event.condition_value = self.condition_val_spin.value()
        elif "AREA" in self.event.trigger_type:
            self.event.condition_value = float(self.area_combo.currentData())
            
        self.event.reference_ship_idx = self.ref_ship_combo.currentData()
        
        self.event.action_type = self.action_type_combo.currentText()
        if self.event.action_type in ["CHANGE_SPEED", "CHANGE_HEADING"]:
            self.event.action_value = self.action_val_spin.value()
        elif self.event.action_type == "MANEUVER":
            self.event.action_option = self.action_opt_combo.currentText()
        elif self.event.action_type == "CHANGE_DESTINATION_LOC":
            self.event.action_option = f"{self.lat_spin.value()},{self.lon_spin.value()}"

        # 조건부 이벤트 저장
        self.event.prerequisite_logic = "AND" if self.logic_combo.currentIndex() == 0 else "OR"
        self.event.prerequisite_events = []
        for i in range(self.prereq_list.count()):
            item = self.prereq_list.item(i)
            event_id = item.data(Qt.ItemDataRole.UserRole)
            mode = item.data(Qt.ItemDataRole.UserRole + 1)
            self.event.prerequisite_events.append(EventCondition(event_id=event_id, mode=mode))

        self.accept()

    def get_event(self):
        return self.event