import uuid

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QPushButton, QLabel, QLineEdit, 
    QComboBox, QCheckBox, QDoubleSpinBox, QListWidget, QListWidgetItem, QGroupBox, 
    QAbstractItemView
)
from PyQt6.QtCore import (
    Qt
)
from PyQt6.QtGui import (
    QBrush
)

from app.core.models.project import current_project, EventTrigger
from app.ui.widgets.time_input_widget import TimeInputWidget

class EventEditorDialog(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        # self.setWindowTitle("Scenario Event Scripting")
        
        layout = QHBoxLayout(self)
        
        # Left: List of Events
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        
        self.event_list = QListWidget()
        self.event_list.setDragDropMode(QAbstractItemView.DragDropMode.InternalMove)
        self.event_list.model().rowsMoved.connect(self.on_list_reordered)
        self.event_list.currentRowChanged.connect(self.on_event_selected)
        left_layout.addWidget(self.event_list)
        
        btn_box = QHBoxLayout()
        self.btn_add = QPushButton("Add Event")
        self.btn_dup = QPushButton("Duplicate")
        self.btn_del = QPushButton("Delete Event")
        self.btn_add.clicked.connect(self.add_event)
        self.btn_dup.clicked.connect(self.duplicate_event)
        self.btn_del.clicked.connect(self.delete_event)
        btn_box.addWidget(self.btn_add)
        btn_box.addWidget(self.btn_dup)
        btn_box.addWidget(self.btn_del)
        left_layout.addLayout(btn_box)
        
        layout.addWidget(left_panel, 1)
        
        # Right: Editor
        self.editor_group = QGroupBox("Event Details")
        self.editor_group.setEnabled(False)
        form = QFormLayout(self.editor_group)
        
        self.edit_name = QLineEdit()
        self.chk_enabled = QCheckBox("Enabled")
        
        self.combo_trigger = QComboBox()
        self.combo_trigger.addItems(["TIME", "AREA_ENTER", "AREA_LEAVE", "CPA_UNDER", "CPA_OVER", "DIST_UNDER", "DIST_OVER"])
        self.combo_trigger.currentIndexChanged.connect(self.update_ui_state)
        
        # Time Reference (Start vs End)
        self.combo_time_ref = QComboBox()
        self.combo_time_ref.addItems(["Elapsed Time (Since Start)", "Remaining Time (Until End)"])
        self.combo_time_ref.currentIndexChanged.connect(self.on_time_ref_changed)

        # Time Input (Day, Hour, Min, Sec)
        self.time_input = TimeInputWidget()
        
        # CPA Distance Input
        self.spin_cpa = QDoubleSpinBox()
        self.spin_cpa.setRange(0, 50000)
        self.spin_cpa.setSuffix(" m")
        
        self.combo_ref = QComboBox() # For Reference Ship
        
        self.combo_area = QComboBox() # For Area
        
        self.combo_action = QComboBox()
        self.combo_action.addItems(["STOP", "CHANGE_SPEED", "CHANGE_HEADING"])
        self.combo_action.currentIndexChanged.connect(self.update_ui_state)
        
        self.combo_target = QComboBox()
        
        self.spin_action_val = QDoubleSpinBox()
        self.spin_action_val.setRange(0, 1000)
        
        form.addRow("Name:", self.edit_name)
        form.addRow("", self.chk_enabled)
        form.addRow("Trigger Type:", self.combo_trigger)
        form.addRow("Time Ref:", self.combo_time_ref)
        
        form.addRow("Ref Ship:", self.combo_ref)
        form.addRow("Time:", self.time_input)
        form.addRow("CPA Dist:", self.spin_cpa)
        form.addRow("Area:", self.combo_area)
        
        form.addRow("Action Type:", self.combo_action)
        form.addRow("Target Ship:", self.combo_target)
        
        self.lbl_act_val = QLabel("Value:")
        form.addRow(self.lbl_act_val, self.spin_action_val)
        
        btn_save = QPushButton("Save Changes")
        btn_save.clicked.connect(self.save_current_event)
        form.addRow(btn_save)
        
        layout.addWidget(self.editor_group, 2)
        
        self.current_event_id = None
        self.refresh_list()
        self.update_combos()

    def update_combos(self):
        self.combo_area.clear()
        for a in current_project.areas:
            self.combo_area.addItem(f"{a.name} (ID: {a.id})", a.id)
            
        self.combo_target.clear()
        for s in current_project.ships:
            self.combo_target.addItem(f"{s.name} (ID: {s.idx})", s.idx)
            
        self.combo_ref.clear()
        for s in current_project.ships:
            self.combo_ref.addItem(f"{s.name} (ID: {s.idx})", s.idx)
            
    def refresh_all(self):
        self.refresh_list()
        self.update_combos()

    def on_list_reordered(self, parent, start, end, destination, row):
        new_events = []
        for i in range(self.event_list.count()):
            eid = self.event_list.item(i).data(Qt.ItemDataRole.UserRole)
            evt = next((e for e in current_project.events if e.id == eid), None)
            if evt:
                new_events.append(evt)
        current_project.events = new_events

    def refresh_list(self):
        self.event_list.clear()
        for e in current_project.events:
            item = QListWidgetItem(f"{e.name} ({e.trigger_type} -> {e.action_type})")
            item.setData(Qt.ItemDataRole.UserRole, e.id)
            if not e.enabled:
                item.setForeground(QBrush(Qt.GlobalColor.gray))
            self.event_list.addItem(item)

    def add_event(self):
        eid = str(uuid.uuid4())
        evt = EventTrigger(id=eid, name="New Event")
        current_project.events.append(evt)
        self.refresh_list()
        self.event_list.setCurrentRow(self.event_list.count()-1)

    def duplicate_event(self):
        row = self.event_list.currentRow()
        if row < 0: return
        eid = self.event_list.item(row).data(Qt.ItemDataRole.UserRole)
        original_evt = next((e for e in current_project.events if e.id == eid), None)
        if not original_evt: return
        
        new_id = str(uuid.uuid4())
        new_evt = EventTrigger(
            id=new_id,
            name=f"{original_evt.name} (Copy)",
            enabled=original_evt.enabled,
            trigger_type=original_evt.trigger_type,
            condition_value=original_evt.condition_value,
            action_type=original_evt.action_type,
            target_ship_idx=original_evt.target_ship_idx,
            action_value=original_evt.action_value,
            is_relative_to_end=original_evt.is_relative_to_end,
            reference_ship_idx=original_evt.reference_ship_idx
        )
        current_project.events.append(new_evt)
        self.refresh_list()
        self.event_list.setCurrentRow(self.event_list.count() - 1)

    def delete_event(self):
        row = self.event_list.currentRow()
        if row < 0: return
        eid = self.event_list.item(row).data(Qt.ItemDataRole.UserRole)
        current_project.events = [e for e in current_project.events if e.id != eid]
        self.refresh_list()
        self.editor_group.setEnabled(False)
        self.current_event_id = None

    def on_event_selected(self, row):
        if row < 0: 
            self.editor_group.setEnabled(False)
            return
            
        eid = self.event_list.item(row).data(Qt.ItemDataRole.UserRole)
        evt = next((e for e in current_project.events if e.id == eid), None)
        if not evt: return
        
        self.current_event_id = eid
        self.editor_group.setEnabled(True)
        
        self.edit_name.setText(evt.name)
        self.chk_enabled.setChecked(evt.enabled)
        self.combo_trigger.setCurrentText(evt.trigger_type)
        
        if evt.trigger_type == "TIME":
            # Set Target Ship first to calculate duration correctly
            idx = self.combo_target.findData(evt.target_ship_idx)
            if idx >= 0: self.combo_target.setCurrentIndex(idx)

            self.combo_time_ref.blockSignals(True)
            if evt.is_relative_to_end:
                self.combo_time_ref.setCurrentIndex(1) # Remaining
                self._display_time_as_remaining(evt.condition_value)
            else:
                self.combo_time_ref.setCurrentIndex(0) # Elapsed
                self.time_input.set_seconds(evt.condition_value)
            self.combo_time_ref.blockSignals(False)
        elif evt.trigger_type in ["CPA_UNDER", "CPA_OVER", "DIST_UNDER", "DIST_OVER"]:
            self.spin_cpa.setValue(evt.condition_value)
        else:
            idx = self.combo_area.findData(int(evt.condition_value))
            if idx >= 0: self.combo_area.setCurrentIndex(idx)
            
        self.combo_action.setCurrentText(evt.action_type)
        
        idx = self.combo_target.findData(evt.target_ship_idx)
        if idx >= 0: self.combo_target.setCurrentIndex(idx)
        
        idx = self.combo_ref.findData(evt.reference_ship_idx)
        if idx >= 0: self.combo_ref.setCurrentIndex(idx)
        
        self.spin_action_val.setValue(evt.action_value)
        self.update_ui_state()

    def _display_time_as_remaining(self, elapsed_sec):
        dur = self.get_current_ship_duration()
        rem = max(0, dur - elapsed_sec)
        self.time_input.set_seconds(rem)

    def get_current_ship_duration(self):
        sid = self.combo_target.currentData()
        ship = current_project.get_ship_by_idx(sid)
        if ship and ship.is_generated:
            return ship.total_duration_sec
        return 0.0

    def on_time_ref_changed(self):
        # Convert currently displayed value to the other format
        current_val = self.time_input.get_seconds()
        dur = self.get_current_ship_duration()
        
        # If we just switched modes, the value in the box represents the OLD mode's value.
        # Elapsed + Remaining = Duration
        # So, NewValue = Duration - OldValue
        new_val = max(0, dur - current_val)
        self.time_input.set_seconds(new_val)

    def update_ui_state(self):
        trig = self.combo_trigger.currentText()
        if trig == "TIME":
            self.combo_time_ref.setVisible(True)
            self.time_input.setVisible(True)
            self.spin_cpa.setVisible(False)
            self.combo_area.setVisible(False)
            self.combo_ref.setVisible(False)
            self.editor_group.layout().labelForField(self.combo_time_ref).setVisible(True)
            self.editor_group.layout().labelForField(self.time_input).setVisible(True)
            self.editor_group.layout().labelForField(self.spin_cpa).setVisible(False)
            self.editor_group.layout().labelForField(self.combo_area).setVisible(False)
            self.editor_group.layout().labelForField(self.combo_ref).setVisible(False)
        elif trig in ["CPA_UNDER", "CPA_OVER", "DIST_UNDER", "DIST_OVER"]:
            self.combo_time_ref.setVisible(False)
            self.time_input.setVisible(False)
            self.spin_cpa.setVisible(True)
            self.combo_area.setVisible(False)
            self.combo_ref.setVisible(True)
            self.editor_group.layout().labelForField(self.combo_time_ref).setVisible(False)
            self.editor_group.layout().labelForField(self.time_input).setVisible(False)
            self.editor_group.layout().labelForField(self.spin_cpa).setVisible(True)
            self.editor_group.layout().labelForField(self.combo_area).setVisible(False)
            self.editor_group.layout().labelForField(self.combo_ref).setVisible(True)
        else:
            self.combo_time_ref.setVisible(False)
            self.time_input.setVisible(False)
            self.spin_cpa.setVisible(False)
            self.combo_area.setVisible(True)
            self.combo_ref.setVisible(False)
            self.editor_group.layout().labelForField(self.combo_time_ref).setVisible(False)
            self.editor_group.layout().labelForField(self.time_input).setVisible(False)
            self.editor_group.layout().labelForField(self.spin_cpa).setVisible(False)
            self.editor_group.layout().labelForField(self.combo_area).setVisible(True)
            self.editor_group.layout().labelForField(self.combo_ref).setVisible(False)
            
        act = self.combo_action.currentText()
        if act == "STOP":
            self.lbl_act_val.setVisible(False)
            self.spin_action_val.setVisible(False)
        elif act == "CHANGE_SPEED":
            self.lbl_act_val.setText("New Speed (kn):")
            self.lbl_act_val.setVisible(True)
            self.spin_action_val.setVisible(True)
        elif act == "CHANGE_HEADING":
            self.lbl_act_val.setText("New Heading (deg):")
            self.lbl_act_val.setVisible(True)
            self.spin_action_val.setVisible(True)

    def save_current_event(self):
        if not self.current_event_id: return
        evt = next((e for e in current_project.events if e.id == self.current_event_id), None)
        if not evt: return
        
        evt.name = self.edit_name.text()
        evt.enabled = self.chk_enabled.isChecked()
        evt.trigger_type = self.combo_trigger.currentText()
        
        if evt.trigger_type == "TIME":
            evt.is_relative_to_end = (self.combo_time_ref.currentIndex() == 1)
            input_val = float(self.time_input.get_seconds())
            
            if evt.is_relative_to_end:
                dur = self.get_current_ship_duration()
                evt.condition_value = max(0, dur - input_val)
            else:
                evt.condition_value = input_val
        elif evt.trigger_type in ["CPA_UNDER", "CPA_OVER", "DIST_UNDER", "DIST_OVER"]:
            evt.condition_value = self.spin_cpa.value()
        else:
            evt.condition_value = self.combo_area.currentData()
            
        evt.action_type = self.combo_action.currentText()
        evt.target_ship_idx = self.combo_target.currentData()
        evt.reference_ship_idx = self.combo_ref.currentData()
        evt.action_value = self.spin_action_val.value()
        
        self.refresh_list()
        # Restore selection
        for i in range(self.event_list.count()):
            if self.event_list.item(i).data(Qt.ItemDataRole.UserRole) == self.current_event_id:
                self.event_list.setCurrentRow(i)
                break
