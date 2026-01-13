import os
import uuid
import copy
import re
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QLineEdit, 
    QComboBox, QListWidget, QListWidgetItem, QGroupBox, QFormLayout, 
    QFileDialog, QMessageBox, QCheckBox, QAbstractItemView, QDoubleSpinBox, QSpinBox
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QBrush, QColor

from app.core.models.project import current_project, EventTrigger
from app.core.models.scenario import Scenario
from app.ui.widgets.time_input_widget import TimeInputWidget
import app.core.state as app_state

class ScenarioPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Initialize global state if None
        if app_state.current_scenario is None:
            app_state.current_scenario = Scenario()
            
        self.is_dirty = False
        self.loading_event = False
        self.last_selected_row = -1
            
        self.init_ui()
        self.refresh_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 1. File Management
        file_box = QHBoxLayout()
        self.btn_new = QPushButton("New Scenario")
        self.btn_open = QPushButton("Open Scenario")
        self.btn_save = QPushButton("Save Scenario")
        self.btn_save_as = QPushButton("Save As...")
        
        self.btn_new.clicked.connect(self.new_scenario)
        self.btn_open.clicked.connect(self.open_scenario)
        self.btn_save.clicked.connect(self.save_scenario)
        self.btn_save_as.clicked.connect(self.save_scenario_as)
        
        file_box.addWidget(self.btn_new)
        file_box.addWidget(self.btn_open)
        file_box.addWidget(self.btn_save)
        file_box.addWidget(self.btn_save_as)
        layout.addLayout(file_box)
        
        # 2. Meta Data
        meta_group = QGroupBox("Scenario Metadata")
        meta_layout = QFormLayout(meta_group)
        
        self.edit_name = QLineEdit()
        self.edit_name.textChanged.connect(self.on_meta_changed)
        self.edit_desc = QLineEdit()
        self.edit_desc.textChanged.connect(self.on_meta_changed)
        
        self.combo_scope = QComboBox()
        self.combo_scope.addItems(["ALL_SHIPS", "OWN_ONLY", "TARGET_ONLY", "SELECTED_SHIPS"])
        self.combo_scope.currentTextChanged.connect(self.on_scope_changed)
        
        self.list_scope_ships = QListWidget()
        self.list_scope_ships.setSelectionMode(QAbstractItemView.SelectionMode.MultiSelection)
        self.list_scope_ships.setFixedHeight(100)
        self.list_scope_ships.itemSelectionChanged.connect(self.on_ship_selection_changed)
        
        meta_layout.addRow("Name:", self.edit_name)
        meta_layout.addRow("Description:", self.edit_desc)
        meta_layout.addRow("Scope Mode:", self.combo_scope)
        meta_layout.addRow("Select Ships:", self.list_scope_ships)
        
        layout.addWidget(meta_group)
        
        # 3. Event Composition
        event_group = QGroupBox("Event Composition")
        event_layout = QHBoxLayout(event_group)
        
        # Available Events (from Project)
        v1 = QVBoxLayout()
        v1.addWidget(QLabel("Available Events (Project)"))
        self.list_avail = QListWidget()
        v1.addWidget(self.list_avail)
        event_layout.addLayout(v1)
        
        # Buttons
        btn_layout = QVBoxLayout()
        self.btn_add = QPushButton("Add ->")
        self.btn_remove = QPushButton("<- Remove")
        self.btn_add.clicked.connect(self.add_event_to_scenario)
        self.btn_remove.clicked.connect(self.remove_event_from_scenario)
        btn_layout.addStretch()
        btn_layout.addWidget(self.btn_add)
        btn_layout.addWidget(self.btn_remove)
        btn_layout.addStretch()
        event_layout.addLayout(btn_layout)
        
        # Scenario Events
        v2 = QVBoxLayout()
        v2.addWidget(QLabel("Scenario Events (Included)"))
        self.list_scen_events = QListWidget()
        self.list_scen_events.setDragDropMode(QAbstractItemView.DragDropMode.InternalMove)
        self.list_scen_events.model().rowsMoved.connect(self.on_scen_list_reordered)
        self.list_scen_events.itemChanged.connect(self.on_scen_event_item_changed)
        v2.addWidget(self.list_scen_events)
        
        self.btn_dup_scen = QPushButton("Duplicate Selected")
        self.btn_dup_scen.clicked.connect(self.duplicate_scenario_event)
        v2.addWidget(self.btn_dup_scen)
        
        event_layout.addLayout(v2)
        
        layout.addWidget(event_group)
        
        # 5. Event Editor
        self.editor_group = QGroupBox("Selected Event Details")
        self.editor_group.setEnabled(False)
        form = QFormLayout(self.editor_group)
        
        self.edit_evt_name = QLineEdit()
        self.chk_evt_enabled = QCheckBox("Enabled")
        
        self.combo_trigger = QComboBox()
        self.combo_trigger.addItems(["TIME", "AREA_ENTER", "AREA_LEAVE", "CPA_UNDER", "CPA_OVER", "DIST_UNDER", "DIST_OVER"])
        self.combo_trigger.currentIndexChanged.connect(self.update_editor_ui_state)
        
        self.combo_time_ref = QComboBox()
        self.combo_time_ref.addItems(["Elapsed Time (Since Start)", "Remaining Time (Until End)"])
        self.combo_time_ref.currentIndexChanged.connect(self.on_time_ref_changed)
        
        self.time_input = TimeInputWidget()
        
        self.spin_cpa = QDoubleSpinBox()
        self.spin_cpa.setRange(0, 50000)
        self.spin_cpa.setSuffix(" m")
        
        self.combo_ref = QComboBox()
        self.combo_area = QComboBox()
        
        self.combo_action = QComboBox()
        self.combo_action.addItems(["STOP", "CHANGE_SPEED", "CHANGE_HEADING"])
        self.combo_action.currentIndexChanged.connect(self.update_editor_ui_state)
        
        self.combo_target = QComboBox()
        
        self.spin_action_val = QDoubleSpinBox()
        self.spin_action_val.setRange(0, 1000)
        
        self.lbl_act_val = QLabel("Value:")
        
        self._build_editor_form(form)
        layout.addWidget(self.editor_group)
        self.list_scen_events.itemSelectionChanged.connect(self.on_scen_event_selected)
        
        # 4. Enable
        self.chk_enable_scen = QCheckBox("Enable Scenario")
        self.chk_enable_scen.setStyleSheet("font-weight: bold; font-size: 14px;")
        self.chk_enable_scen.toggled.connect(self.on_enable_toggled)
        layout.addWidget(self.chk_enable_scen)
        
        self.current_filepath = None

    def _build_editor_form(self, form):
        form.addRow("Name:", self.edit_evt_name)
        form.addRow("", self.chk_evt_enabled)
        form.addRow("Trigger Type:", self.combo_trigger)
        form.addRow("Time Ref:", self.combo_time_ref)
        form.addRow("Time:", self.time_input)
        form.addRow("CPA Dist:", self.spin_cpa)
        form.addRow("Ref Ship:", self.combo_ref)
        form.addRow("Area:", self.combo_area)
        form.addRow("Action Type:", self.combo_action)
        form.addRow("Target Ship:", self.combo_target)
        form.addRow(self.lbl_act_val, self.spin_action_val)
        
        btn_save_evt = QPushButton("Apply Changes")
        btn_save_evt.clicked.connect(self.save_event_changes)
        form.addRow(btn_save_evt)
        
        # Connect change signals for dirty check
        self.edit_evt_name.textChanged.connect(self.mark_dirty)
        self.chk_evt_enabled.toggled.connect(self.mark_dirty)
        self.combo_trigger.currentIndexChanged.connect(self.mark_dirty)
        self.combo_time_ref.currentIndexChanged.connect(self.mark_dirty)
        self.spin_cpa.valueChanged.connect(self.mark_dirty)
        self.combo_ref.currentIndexChanged.connect(self.mark_dirty)
        self.combo_area.currentIndexChanged.connect(self.mark_dirty)
        self.combo_action.currentIndexChanged.connect(self.mark_dirty)
        self.combo_target.currentIndexChanged.connect(self.mark_dirty)
        self.spin_action_val.valueChanged.connect(self.mark_dirty)
        
        # Try to connect TimeInputWidget internal spinboxes
        for sb in self.time_input.findChildren(QSpinBox):
            sb.valueChanged.connect(self.mark_dirty)

    def refresh_ui(self):
        scen = app_state.current_scenario
        if not scen: return
        
        self.update_editor_combos()
        
        self.edit_name.blockSignals(True)
        self.edit_name.setText(scen.name)
        self.edit_name.blockSignals(False)
        
        self.edit_desc.blockSignals(True)
        self.edit_desc.setText(scen.description)
        self.edit_desc.blockSignals(False)
        
        self.combo_scope.blockSignals(True)
        self.combo_scope.setCurrentText(scen.scope_mode)
        self.combo_scope.blockSignals(False)
        
        self.chk_enable_scen.blockSignals(True)
        self.chk_enable_scen.setChecked(scen.enabled)
        self.chk_enable_scen.blockSignals(False)
        
        # Refresh Ship List
        self.list_scope_ships.blockSignals(True)
        self.list_scope_ships.clear()
        for s in current_project.ships:
            item = QListWidgetItem(f"{s.name} (ID: {s.idx})")
            item.setData(Qt.ItemDataRole.UserRole, s.idx)
            if s.idx in scen.selected_ships:
                item.setSelected(True)
            self.list_scope_ships.addItem(item)
        self.list_scope_ships.setVisible(scen.scope_mode == "SELECTED_SHIPS")
        self.list_scope_ships.blockSignals(False)
        
        # Refresh Available Events
        self.list_avail.clear()
        if not current_project.events:
            self.list_avail.addItem("No events in Project. Create in Event Tab first.")
            self.list_avail.item(0).setFlags(Qt.ItemFlag.NoItemFlags)
        else:
            for e in current_project.events:
                item = QListWidgetItem(f"{e.name} ({e.trigger_type})")
                item.setData(Qt.ItemDataRole.UserRole, e.id)
                self.list_avail.addItem(item)
                
        # Refresh Scenario Events
        self.list_scen_events.blockSignals(True)
        self.list_scen_events.clear()
        for e in scen.events:
            item = QListWidgetItem(f"{e.name}")
            item.setData(Qt.ItemDataRole.UserRole, e.id) # Note: ID might be same as project event or new
            item.setFlags(item.flags() | Qt.ItemFlag.ItemIsUserCheckable)
            item.setCheckState(Qt.CheckState.Checked if e.enabled else Qt.CheckState.Unchecked)
            self.list_scen_events.addItem(item)
        self.list_scen_events.blockSignals(False)
        self.editor_group.setEnabled(False)
        self.is_dirty = False
        self.last_selected_row = -1
        self.editor_group.setTitle("Selected Event Details")

    def update_editor_combos(self):
        self.combo_area.clear()
        for a in current_project.areas:
            self.combo_area.addItem(f"{a.name} (ID: {a.id})", a.id)
            
        self.combo_target.clear()
        for s in current_project.ships:
            self.combo_target.addItem(f"{s.name} (ID: {s.idx})", s.idx)
            
        self.combo_ref.clear()
        for s in current_project.ships:
            self.combo_ref.addItem(f"{s.name} (ID: {s.idx})", s.idx)

    def on_meta_changed(self):
        app_state.current_scenario.name = self.edit_name.text()
        app_state.current_scenario.description = self.edit_desc.text()

    def on_scope_changed(self, text):
        app_state.current_scenario.scope_mode = text
        self.list_scope_ships.setVisible(text == "SELECTED_SHIPS")

    def on_ship_selection_changed(self):
        selected = [item.data(Qt.ItemDataRole.UserRole) for item in self.list_scope_ships.selectedItems()]
        app_state.current_scenario.selected_ships = selected

    def on_enable_toggled(self, checked):
        app_state.current_scenario.enabled = checked

    def add_event_to_scenario(self):
        row = self.list_avail.currentRow()
        if row < 0: return
        eid = self.list_avail.item(row).data(Qt.ItemDataRole.UserRole)
        if not eid: return
        
        # Check for duplicates in current scenario
        if any(e.id == eid for e in app_state.current_scenario.events):
            QMessageBox.warning(self, "Duplicate", "This event is already added to the scenario.")
            return
        
        # Find original event
        orig = next((e for e in current_project.events if e.id == eid), None)
        if not orig: return
        
        # Copy event to scenario
        # We keep the ID to identify it as the "same logic" or generate new?
        # Requirement says "Scenario includes event definition copy".
        # To allow "Same Event duplicated -> apply once", keeping ID is safer if we treat them as same source.
        # But if we want independence, maybe new ID?
        # Let's keep ID for now to allow dedup logic in worker.
        new_evt = copy.deepcopy(orig)
        app_state.current_scenario.events.append(new_evt)
        self.refresh_ui()

    def remove_event_from_scenario(self):
        row = self.list_scen_events.currentRow()
        if row < 0: return
        app_state.current_scenario.events.pop(row)
        self.refresh_ui()

    def duplicate_scenario_event(self):
        if self.is_dirty:
            ret = QMessageBox.question(
                self, "Unsaved Changes", 
                "You have unsaved changes. Save them before duplicating?",
                QMessageBox.StandardButton.Save | QMessageBox.StandardButton.Discard | QMessageBox.StandardButton.Cancel
            )
            if ret == QMessageBox.StandardButton.Cancel:
                return
            elif ret == QMessageBox.StandardButton.Save:
                self.save_event_changes(target_row=self.last_selected_row)

        row = self.list_scen_events.currentRow()
        if row < 0: return
        
        orig_evt = app_state.current_scenario.events[row]
        new_evt = copy.deepcopy(orig_evt)
        new_evt.id = str(uuid.uuid4())
        
        base_name = orig_evt.name
        match = re.match(r"(.*) \(Copy \d+\)$", base_name)
        if match:
            base_name = match.group(1)
        
        existing_names = {e.name for e in app_state.current_scenario.events}
        count = 1
        new_name = f"{base_name} (Copy {count})"
        while new_name in existing_names:
            count += 1
            new_name = f"{base_name} (Copy {count})"
        new_evt.name = new_name
        
        app_state.current_scenario.events.insert(row + 1, new_evt)
        self.refresh_ui()
        self.list_scen_events.setCurrentRow(row + 1)

    def on_scen_event_item_changed(self, item):
        row = self.list_scen_events.row(item)
        if row < 0: return
        is_checked = (item.checkState() == Qt.CheckState.Checked)
        app_state.current_scenario.events[row].enabled = is_checked

    def on_scen_list_reordered(self, parent, start, end, destination, row):
        new_events = []
        for i in range(self.list_scen_events.count()):
            eid = self.list_scen_events.item(i).data(Qt.ItemDataRole.UserRole)
            evt = next((e for e in app_state.current_scenario.events if e.id == eid), None)
            if evt:
                new_events.append(evt)
        app_state.current_scenario.events = new_events

    def new_scenario(self):
        app_state.current_scenario = Scenario()
        self.current_filepath = None
        self.refresh_ui()

    def open_scenario(self):
        path, _ = QFileDialog.getOpenFileName(self, "Open Scenario", "", "Scenario Files (*.json)")
        if path:
            try:
                app_state.current_scenario = Scenario.load_from_file(path)
                self.current_filepath = path
                self.refresh_ui()
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load scenario: {e}")

    def save_scenario(self):
        if not self.current_filepath:
            self.save_scenario_as()
        else:
            try:
                app_state.current_scenario.save_to_file(self.current_filepath)
                QMessageBox.information(self, "Saved", "Scenario saved.")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to save: {e}")

    def save_scenario_as(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save Scenario As", "", "Scenario Files (*.json)")
        if path:
            self.current_filepath = path
            self.save_scenario()

    def on_scen_event_selected(self):
        # Check for unsaved changes
        if self.is_dirty:
            ret = QMessageBox.question(
                self, "Unsaved Changes", 
                "You have unsaved changes in the current event. Do you want to save them?",
                QMessageBox.StandardButton.Save | QMessageBox.StandardButton.Discard | QMessageBox.StandardButton.Cancel
            )
            
            if ret == QMessageBox.StandardButton.Cancel:
                self.list_scen_events.blockSignals(True)
                if self.last_selected_row != -1 and self.last_selected_row < self.list_scen_events.count():
                    self.list_scen_events.setCurrentRow(self.last_selected_row)
                else:
                    self.list_scen_events.clearSelection()
                self.list_scen_events.blockSignals(False)
                return
            elif ret == QMessageBox.StandardButton.Save:
                # Save to the PREVIOUS row
                self.save_event_changes(target_row=self.last_selected_row)
            else:
                # Discard
                self.is_dirty = False
                self.editor_group.setTitle("Selected Event Details")

        items = self.list_scen_events.selectedItems()
        if not items:
            self.editor_group.setEnabled(False)
            self.last_selected_row = -1
            return
        
        row = self.list_scen_events.row(items[0])
        if row < 0 or row >= len(app_state.current_scenario.events):
            self.editor_group.setEnabled(False)
            self.last_selected_row = -1
            return
            
        evt = app_state.current_scenario.events[row]
        self.loading_event = True
        self.last_selected_row = row
        self.editor_group.setEnabled(True)
        
        self.edit_evt_name.setText(evt.name)
        self.chk_evt_enabled.setChecked(evt.enabled)
        self.combo_trigger.setCurrentText(evt.trigger_type)
        
        if evt.trigger_type == "TIME":
            idx = self.combo_target.findData(evt.target_ship_idx)
            if idx >= 0: self.combo_target.setCurrentIndex(idx)

            self.combo_time_ref.blockSignals(True)
            if evt.is_relative_to_end:
                self.combo_time_ref.setCurrentIndex(1)
                self._display_time_as_remaining(evt.condition_value)
            else:
                self.combo_time_ref.setCurrentIndex(0)
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
        self.update_editor_ui_state()
        self.loading_event = False

    def update_editor_ui_state(self):
        trig = self.combo_trigger.currentText()
        if trig == "TIME":
            self.set_row_visible(self.combo_time_ref, True)
            self.set_row_visible(self.time_input, True)
            self.set_row_visible(self.spin_cpa, False)
            self.set_row_visible(self.combo_area, False)
            self.set_row_visible(self.combo_ref, False)
        elif trig in ["CPA_UNDER", "CPA_OVER"]:
            self.set_row_visible(self.combo_time_ref, False)
            self.set_row_visible(self.time_input, False)
            self.set_row_visible(self.spin_cpa, True)
            self.set_row_visible(self.combo_area, False)
            self.set_row_visible(self.combo_ref, True)
            self.spin_cpa.setSuffix(" m")
            self.spin_cpa.setRange(0, 50000)
        elif trig in ["DIST_UNDER", "DIST_OVER"]:
            self.set_row_visible(self.combo_time_ref, False)
            self.set_row_visible(self.time_input, False)
            self.set_row_visible(self.spin_cpa, True)
            self.set_row_visible(self.combo_area, False)
            self.set_row_visible(self.combo_ref, True)
            self.spin_cpa.setSuffix(" nm")
            self.spin_cpa.setRange(0, 1000)
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
        if label: label.setVisible(visible)

    def save_event_changes(self, target_row=None):
        if target_row is None:
            items = self.list_scen_events.selectedItems()
            if not items: return
            row = self.list_scen_events.row(items[0])
        else:
            row = target_row
        
        evt = app_state.current_scenario.events[row]
        evt.name = self.edit_evt_name.text()
        evt.enabled = self.chk_evt_enabled.isChecked()
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
        
        self.list_scen_events.blockSignals(True)
        item = self.list_scen_events.item(row)
        if item:
            item.setText(evt.name)
            item.setCheckState(Qt.CheckState.Checked if evt.enabled else Qt.CheckState.Unchecked)
        self.list_scen_events.blockSignals(False)
        
        self.is_dirty = False
        self.editor_group.setTitle("Selected Event Details")

    def mark_dirty(self):
        if not self.loading_event:
            self.is_dirty = True
            self.editor_group.setTitle("Selected Event Details *")

    def get_current_ship_duration(self):
        sid = self.combo_target.currentData()
        ship = current_project.get_ship_by_idx(sid)
        if ship and ship.is_generated:
            return ship.total_duration_sec
        return 0.0

    def _display_time_as_remaining(self, elapsed_sec):
        dur = self.get_current_ship_duration()
        rem = max(0, dur - elapsed_sec)
        self.time_input.set_seconds(rem)

    def on_time_ref_changed(self):
        current_val = self.time_input.get_seconds()
        dur = self.get_current_ship_duration()
        new_val = max(0, dur - current_val)
        self.time_input.set_seconds(new_val)