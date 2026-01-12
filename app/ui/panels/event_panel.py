import uuid

from PyQt6.QtWidgets import (
    QWidget, QHBoxLayout, QSpinBox, QDoubleSpinBox, QVBoxLayout, QListWidget, 
    QAbstractItemView, QPushButton, QGroupBox, QFormLayout, QLineEdit, QCheckBox, 
    QComboBox, QLabel, QListWidgetItem, QTableWidget, QHeaderView, QTableWidgetItem,
    QFrame
)
from PyQt6.QtCore import (
    Qt
)
from PyQt6.QtGui import (
    QBrush
)
from app.core.models.project import current_project, EventTrigger
from app.ui.widgets.time_input_widget import TimeInputWidget

class EventScriptPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        
        layout = QHBoxLayout(self)
        
        # Left: List of Events
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        
        # Priority Filter
        left_layout.addWidget(QLabel("Priority Filter (Move to Top):"))
        self.combo_priority = QComboBox()
        self.combo_priority.currentIndexChanged.connect(self.refresh_table)
        left_layout.addWidget(self.combo_priority)

        # Sort Criteria
        left_layout.addWidget(QLabel("Sort By:"))
        self.combo_sort = QComboBox()
        self.combo_sort.addItems(["Name", "Target Ship", "Trigger Type"])
        self.combo_sort.currentIndexChanged.connect(self.refresh_table)
        left_layout.addWidget(self.combo_sort)

        self.event_table = QTableWidget()
        self.event_table.setColumnCount(6)
        self.event_table.setHorizontalHeaderLabels(["Name", "Enabled", "Trigger", "Condition", "Action", "Target Ship"])
        self.event_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self.event_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        self.event_table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.event_table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.event_table.itemSelectionChanged.connect(self.on_table_selection_changed)
        left_layout.addWidget(self.event_table)
        
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
        self.combo_trigger.addItems(["TIME", "AREA_ENTER", "AREA_LEAVE", "CPA_UNDER"])
        self.combo_trigger.currentIndexChanged.connect(self.update_ui_state)
        
        # Time Reference (Start vs End)
        self.combo_time_ref = QComboBox()
        self.combo_time_ref.addItems(["Elapsed Time (Since Start)", "Remaining Time (Until End)"])
        self.combo_time_ref.currentIndexChanged.connect(self.on_time_ref_changed)

        # Time Input (Day, Hour, Min, Sec)
        self.time_input = TimeInputWidget()
        
        # CPA Distance Input
        self.spin_cpa = QDoubleSpinBox()
        self.spin_cpa.setRange(0, 1000)
        self.spin_cpa.setSuffix(" nm")
        
        self.combo_ref = QComboBox()
        
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
        self.refresh_table()
        self.update_filter_combo()
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

    def update_filter_combo(self):
        current_data = self.combo_priority.currentData()
        self.combo_priority.blockSignals(True)
        self.combo_priority.clear()
        
        self.combo_priority.addItem("None", "NONE")
        self.combo_priority.addItem("Target: Manual Ships", "MANUAL")
        self.combo_priority.addItem("Target: Random Targets", "RANDOM")
        
        # Sort ships
        ships = sorted(current_project.ships, key=lambda s: s.idx)
        for s in ships:
            self.combo_priority.addItem(f"Target: {s.name} (ID: {s.idx})", s.idx)
            
        # Restore selection
        idx = self.combo_priority.findData(current_data)
        if idx >= 0:
            self.combo_priority.setCurrentIndex(idx)
        else:
            self.combo_priority.setCurrentIndex(0)
        self.combo_priority.blockSignals(False)
            
    def refresh_all(self):
        self.refresh_table()
        self.update_filter_combo()
        self.update_combos()

    def on_list_reordered(self, parent, start, end, destination, row):
        pass # Table sorting replaces manual reordering

    def refresh_table(self):
        self.event_table.blockSignals(True)
        self.event_table.setRowCount(0)
        
        prio_data = self.combo_priority.currentData()
        sort_crit = self.combo_sort.currentText()
        
        def get_sort_key(evt):
            # 1. Priority (True comes first, so use False for sort)
            is_prio = False
            if prio_data == "MANUAL":
                if evt.target_ship_idx < 1000: is_prio = True
            elif prio_data == "RANDOM":
                if evt.target_ship_idx >= 1000: is_prio = True
            elif isinstance(prio_data, int):
                if evt.target_ship_idx == prio_data: is_prio = True
            
            # 2. Sort Criteria
            val = ""
            if sort_crit == "Name": val = evt.name
            elif sort_crit == "Target Ship": val = evt.target_ship_idx
            elif sort_crit == "Trigger Type": val = evt.trigger_type
            
            return (not is_prio, val)

        sorted_events = sorted(current_project.events, key=get_sort_key)
        
        self.event_table.setRowCount(len(sorted_events))
        
        for row, e in enumerate(sorted_events):
            # Name
            name_item = QTableWidgetItem(e.name)
            name_item.setData(Qt.ItemDataRole.UserRole, e.id)
            if not e.enabled:
                name_item.setForeground(QBrush(Qt.GlobalColor.gray))
            self.event_table.setItem(row, 0, name_item)
            
            # Enabled
            self.event_table.setItem(row, 1, QTableWidgetItem("Yes" if e.enabled else "No"))
            
            # Trigger
            self.event_table.setItem(row, 2, QTableWidgetItem(e.trigger_type))
            
            # Condition
            cond_str = f"{e.condition_value}"
            if e.trigger_type == "TIME": cond_str += "s"
            elif e.trigger_type in ["CPA_UNDER", "DIST_UNDER", "DIST_OVER"]: cond_str += "nm"
            self.event_table.setItem(row, 3, QTableWidgetItem(cond_str))
            
            # Action
            self.event_table.setItem(row, 4, QTableWidgetItem(e.action_type))
            
            # Target Ship
            ship = current_project.get_ship_by_idx(e.target_ship_idx)
            ship_name = ship.name if ship else f"ID:{e.target_ship_idx}"
            self.event_table.setItem(row, 5, QTableWidgetItem(ship_name))
            
        self.event_table.blockSignals(False)

    def add_event(self):
        eid = str(uuid.uuid4())
        evt = EventTrigger(id=eid, name="New Event", enabled=False)
        current_project.events.append(evt)
        self.refresh_table()
        # Select new
        self.select_event_by_id(eid)

    def duplicate_event(self):
        row = self.event_table.currentRow()
        if row < 0: return
        eid = self.event_table.item(row, 0).data(Qt.ItemDataRole.UserRole)
        original_evt = next((e for e in current_project.events if e.id == eid), None)
        if not original_evt: return
        
        new_id = str(uuid.uuid4())
        new_evt = EventTrigger(
            id=new_id,
            name=f"{original_evt.name} (Copy)",
            enabled=False,
            trigger_type=original_evt.trigger_type,
            condition_value=original_evt.condition_value,
            action_type=original_evt.action_type,
            target_ship_idx=original_evt.target_ship_idx,
            action_value=original_evt.action_value,
            is_relative_to_end=original_evt.is_relative_to_end,
            reference_ship_idx=original_evt.reference_ship_idx
        )
        current_project.events.append(new_evt)
        self.refresh_table()
        self.select_event_by_id(new_id)

    def delete_event(self):
        row = self.event_table.currentRow()
        if row < 0: return
        eid = self.event_table.item(row, 0).data(Qt.ItemDataRole.UserRole)
        current_project.events = [e for e in current_project.events if e.id != eid]
        self.refresh_table()
        self.editor_group.setEnabled(False)
        self.current_event_id = None

    def select_event_by_id(self, eid):
        for row in range(self.event_table.rowCount()):
            if self.event_table.item(row, 0).data(Qt.ItemDataRole.UserRole) == eid:
                self.event_table.selectRow(row)
                break

    def on_table_selection_changed(self):
        items = self.event_table.selectedItems()
        if not items:
            self.editor_group.setEnabled(False)
            return
        
        row = items[0].row()
        eid = self.event_table.item(row, 0).data(Qt.ItemDataRole.UserRole)
        self.on_event_selected(eid)

    def on_event_selected(self, eid):
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
        elif evt.trigger_type in ["CPA_UNDER", "DIST_UNDER", "DIST_OVER"]:
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
            self.set_row_visible(self.combo_time_ref, True)
            self.set_row_visible(self.time_input, True)
            self.set_row_visible(self.spin_cpa, False)
            self.set_row_visible(self.combo_area, False)
            self.set_row_visible(self.combo_ref, False)
        elif trig == "CPA_UNDER":
            self.set_row_visible(self.combo_time_ref, False)
            self.set_row_visible(self.time_input, False)
            self.set_row_visible(self.spin_cpa, True)
            self.set_row_visible(self.combo_area, False)
            self.set_row_visible(self.combo_ref, False)
        elif trig in ["DIST_UNDER", "DIST_OVER"]:
            self.set_row_visible(self.combo_time_ref, False)
            self.set_row_visible(self.time_input, False)
            self.set_row_visible(self.spin_cpa, True)
            self.set_row_visible(self.combo_area, False)
            self.set_row_visible(self.combo_ref, True)
        else:
            self.set_row_visible(self.combo_time_ref, False)
            self.set_row_visible(self.time_input, False)
            self.set_row_visible(self.spin_cpa, False)
            self.set_row_visible(self.combo_area, True)
            self.set_row_visible(self.combo_ref, False)
            
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

    def set_row_visible(self, widget, visible):
        widget.setVisible(visible)
        label = self.editor_group.layout().labelForField(widget)
        if label:
            label.setVisible(visible)

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
        elif evt.trigger_type in ["CPA_UNDER", "DIST_UNDER", "DIST_OVER"]:
            evt.condition_value = self.spin_cpa.value()
        else:
            evt.condition_value = self.combo_area.currentData()
            
        evt.action_type = self.combo_action.currentText()
        evt.target_ship_idx = self.combo_target.currentData()
        evt.reference_ship_idx = self.combo_ref.currentData()
        evt.action_value = self.spin_action_val.value()
        
        # Regenerate ID to force re-evaluation in SimulationWorker
        # This ensures that if the event was previously triggered, it can trigger again
        # with the new parameters.
        new_id = str(uuid.uuid4())
        evt.id = new_id
        self.current_event_id = new_id
        
        self.refresh_table()
        # Restore selection
        self.select_event_by_id(new_id)