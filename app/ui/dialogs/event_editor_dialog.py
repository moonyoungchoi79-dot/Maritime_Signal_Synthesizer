from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout, QLabel, 
    QLineEdit, QComboBox, QDoubleSpinBox, QPushButton, QGroupBox,
    QStackedWidget, QWidget, QMessageBox
)
from PyQt6.QtCore import Qt
from app.core.models.event import Event
from app.core.models.project import current_project

class EventEditorDialog(QDialog):
    def __init__(self, parent=None, event=None):
        super().__init__(parent)
        self.setWindowTitle("Event Editor")
        self.resize(450, 600)
        self.event = event
        
        # UI Components
        self.edt_name = QLineEdit()
        
        # Trigger UI
        self.combo_trigger_type = QComboBox()
        # [수정] 콤보박스에 아이템 추가 (순서 중요: TIME, AREA_ENTER, AREA_LEAVE, DISTANCE_TO_SHIP)
        self.combo_trigger_type.addItems(["Time (sec)", "Area Enter", "Area Leave", "Distance to Ship"])
        
        self.stack_trigger_params = QStackedWidget()
        
        # Action UI
        self.combo_target_ship = QComboBox()
        self.combo_action_type = QComboBox()
        self.combo_action_type.addItems(["Change Speed (kn)", "Change Heading (deg)", "Stop"])
        self.spin_action_val = QDoubleSpinBox()
        self.spin_action_val.setRange(0, 10000)
        
        self.init_ui()
        
        if self.event:
            self.load_data()
        else:
            self.event = Event() # Temp for default
            self.load_data()
            
    def init_ui(self):
        main_layout = QVBoxLayout(self)
        
        # Common Info
        grp_info = QGroupBox("General")
        l_info = QFormLayout(grp_info)
        l_info.addRow("Event Name:", self.edt_name)
        main_layout.addWidget(grp_info)
        
        # Trigger Group
        grp_trig = QGroupBox("Trigger Condition")
        l_trig = QVBoxLayout(grp_trig)
        
        row_type = QHBoxLayout()
        row_type.addWidget(QLabel("Type:"))
        row_type.addWidget(self.combo_trigger_type)
        l_trig.addLayout(row_type)
        
        l_trig.addWidget(self.stack_trigger_params)
        
        # --- Stack Page 0: Time ---
        p_time = QWidget()
        l_time = QFormLayout(p_time)
        self.spin_trig_time = QDoubleSpinBox()
        self.spin_trig_time.setRange(0, 1e7)
        l_time.addRow("Time (sec):", self.spin_trig_time)
        self.stack_trigger_params.addWidget(p_time)
        
        # --- Stack Page 1 & 2: Area (Shared UI, separate logic index) ---
        # We can reuse the same widget logic or create two similar ones. 
        # For simplicity, let's create one widget for Area selection and reuse.
        p_area = QWidget()
        l_area = QFormLayout(p_area)
        self.combo_trig_area = QComboBox()
        # Populate Areas
        for area in current_project.areas:
            self.combo_trig_area.addItem(f"{area.name} (ID: {area.id})", area.id)
        l_area.addRow("Select Area:", self.combo_trig_area)
        self.stack_trigger_params.addWidget(p_area) 
        # Note: Index 1 is Enter, Index 2 is Leave. 
        # We need another page for Leave or reuse. 
        # Let's add the same widget instance again? No, widgets can only have one parent.
        # Create another identical one for Index 2.
        p_area2 = QWidget()
        l_area2 = QFormLayout(p_area2)
        self.combo_trig_area2 = QComboBox()
        for area in current_project.areas:
            self.combo_trig_area2.addItem(f"{area.name} (ID: {area.id})", area.id)
        l_area2.addRow("Select Area:", self.combo_trig_area2)
        self.stack_trigger_params.addWidget(p_area2)

        # --- [추가] Stack Page 3: Distance to Ship ---
        p_dist = QWidget()
        l_dist = QFormLayout(p_dist)
        
        self.combo_dist_target_ship = QComboBox()
        # Populate Ships (exclude can be done later, just list all for now)
        for ship in current_project.ships:
             self.combo_dist_target_ship.addItem(f"{ship.name} (ID: {ship.idx})", ship.idx)
             
        self.spin_dist_val = QDoubleSpinBox()
        self.spin_dist_val.setRange(0, 1000000) # Max 1000km
        self.spin_dist_val.setSuffix(" m")
        
        self.combo_dist_cond = QComboBox()
        self.combo_dist_cond.addItems(["UNDER (Distance < Value)", "OVER (Distance > Value)"])
        
        l_dist.addRow("Target Ship:", self.combo_dist_target_ship)
        l_dist.addRow("Distance:", self.spin_dist_val)
        l_dist.addRow("Condition:", self.combo_dist_cond)
        
        self.stack_trigger_params.addWidget(p_dist)

        main_layout.addWidget(grp_trig)
        
        # Action Group
        grp_act = QGroupBox("Action")
        l_act = QFormLayout(grp_act)
        
        # Populate Target Ships
        for ship in current_project.ships:
            self.combo_target_ship.addItem(f"{ship.name} (ID: {ship.idx})", ship.idx)
            
        l_act.addRow("Apply to Ship:", self.combo_target_ship)
        l_act.addRow("Action Type:", self.combo_action_type)
        l_act.addRow("Value:", self.spin_action_val)
        
        main_layout.addWidget(grp_act)
        
        # Buttons
        btn_box = QHBoxLayout()
        btn_ok = QPushButton("OK")
        btn_cancel = QPushButton("Cancel")
        btn_ok.clicked.connect(self.accept_data)
        btn_cancel.clicked.connect(self.reject)
        btn_box.addStretch()
        btn_box.addWidget(btn_ok)
        btn_box.addWidget(btn_cancel)
        main_layout.addLayout(btn_box)
        
        self.combo_trigger_type.currentIndexChanged.connect(self.update_ui_state)
        self.update_ui_state()

    def update_ui_state(self):
        idx = self.combo_trigger_type.currentIndex()
        self.stack_trigger_params.setCurrentIndex(idx)

    def load_data(self):
        self.edt_name.setText(self.event.name)
        
        # Trigger Type
        if self.event.trigger_type == Event.TRIGGER_TIME:
            self.combo_trigger_type.setCurrentIndex(0)
        elif self.event.trigger_type == Event.TRIGGER_AREA_ENTER:
            self.combo_trigger_type.setCurrentIndex(1)
        elif self.event.trigger_type == Event.TRIGGER_AREA_LEAVE:
            self.combo_trigger_type.setCurrentIndex(2)
        elif self.event.trigger_type == Event.TRIGGER_DISTANCE:
            self.combo_trigger_type.setCurrentIndex(3)
            
        # Trigger Params
        self.spin_trig_time.setValue(self.event.trigger_time)
        
        # Set Area combos
        idx = self.combo_trig_area.findData(self.event.trigger_area_id)
        if idx >= 0: self.combo_trig_area.setCurrentIndex(idx)
        
        idx2 = self.combo_trig_area2.findData(self.event.trigger_area_id)
        if idx2 >= 0: self.combo_trig_area2.setCurrentIndex(idx2)
        
        # [추가] Distance Params
        d_idx = self.combo_dist_target_ship.findData(self.event.trigger_target_ship_idx)
        if d_idx >= 0: self.combo_dist_target_ship.setCurrentIndex(d_idx)
        self.spin_dist_val.setValue(self.event.trigger_distance)
        if self.event.trigger_condition == "OVER":
            self.combo_dist_cond.setCurrentIndex(1)
        else:
            self.combo_dist_cond.setCurrentIndex(0)

        # Action Params
        t_idx = self.combo_target_ship.findData(self.event.target_ship_idx)
        if t_idx >= 0: self.combo_target_ship.setCurrentIndex(t_idx)
        
        if self.event.action_type == Event.ACTION_CHANGE_SPEED:
            self.combo_action_type.setCurrentIndex(0)
        elif self.event.action_type == Event.ACTION_CHANGE_HEADING:
            self.combo_action_type.setCurrentIndex(1)
        elif self.event.action_type == Event.ACTION_STOP:
            self.combo_action_type.setCurrentIndex(2)
            
        self.spin_action_val.setValue(self.event.action_value)

    def accept_data(self):
        # Save to self.event
        self.event.name = self.edt_name.text()
        
        t_idx = self.combo_trigger_type.currentIndex()
        if t_idx == 0:
            self.event.trigger_type = Event.TRIGGER_TIME
            self.event.trigger_time = self.spin_trig_time.value()
        elif t_idx == 1:
            self.event.trigger_type = Event.TRIGGER_AREA_ENTER
            self.event.trigger_area_id = self.combo_trig_area.currentData()
        elif t_idx == 2:
            self.event.trigger_type = Event.TRIGGER_AREA_LEAVE
            self.event.trigger_area_id = self.combo_trig_area2.currentData()
        elif t_idx == 3: # [추가] Distance
            self.event.trigger_type = Event.TRIGGER_DISTANCE
            self.event.trigger_target_ship_idx = self.combo_dist_target_ship.currentData()
            self.event.trigger_distance = self.spin_dist_val.value()
            self.event.trigger_condition = "OVER" if self.combo_dist_cond.currentIndex() == 1 else "UNDER"
            
        # Action
        self.event.target_ship_idx = self.combo_target_ship.currentData()
        a_idx = self.combo_action_type.currentIndex()
        if a_idx == 0:
            self.event.action_type = Event.ACTION_CHANGE_SPEED
        elif a_idx == 1:
            self.event.action_type = Event.ACTION_CHANGE_HEADING
        elif a_idx == 2:
            self.event.action_type = Event.ACTION_STOP
            
        self.event.action_value = self.spin_action_val.value()
        
        self.accept()