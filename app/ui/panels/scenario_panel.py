import os
import uuid
import copy
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QLineEdit, 
    QComboBox, QListWidget, QListWidgetItem, QGroupBox, QFormLayout, 
    QFileDialog, QMessageBox, QCheckBox, QAbstractItemView
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QBrush, QColor

from app.core.models.project import current_project, EventTrigger
from app.core.models.scenario import Scenario
import app.core.state as app_state

class ScenarioPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Initialize global state if None
        if app_state.current_scenario is None:
            app_state.current_scenario = Scenario()
            
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
        self.list_scen_events.itemChanged.connect(self.on_scen_event_item_changed)
        v2.addWidget(self.list_scen_events)
        event_layout.addLayout(v2)
        
        layout.addWidget(event_group)
        
        # 4. Enable
        self.chk_enable_scen = QCheckBox("Enable Scenario")
        self.chk_enable_scen.setStyleSheet("font-weight: bold; font-size: 14px;")
        self.chk_enable_scen.toggled.connect(self.on_enable_toggled)
        layout.addWidget(self.chk_enable_scen)
        
        self.current_filepath = None

    def refresh_ui(self):
        scen = app_state.current_scenario
        if not scen: return
        
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

    def on_scen_event_item_changed(self, item):
        row = self.list_scen_events.row(item)
        if row < 0: return
        is_checked = (item.checkState() == Qt.CheckState.Checked)
        app_state.current_scenario.events[row].enabled = is_checked

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