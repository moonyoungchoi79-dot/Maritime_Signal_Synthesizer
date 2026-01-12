import os
import datetime

from PyQt6.QtWidgets import (
    QHBoxLayout, QFormLayout, QPushButton, QLineEdit, QFileDialog, QDialog, 
    QDialogButtonBox, QDateTimeEdit
)

class NewProjectDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("New Project")
        self.resize(400, 250)
        l = QFormLayout(self)
        
        self.title_edit = QLineEdit("MyProject")
        self.path_edit = QLineEdit(os.getcwd())
        self.btn_path = QPushButton("...")
        self.btn_path.clicked.connect(self.browse)
        
        path_box = QHBoxLayout()
        path_box.addWidget(self.path_edit)
        path_box.addWidget(self.btn_path)
        
        self.start_dt = QDateTimeEdit(datetime.datetime.now())
        self.start_dt.setDisplayFormat("yyyy-MM-dd HH:mm:ss")
        
        l.addRow("Title:", self.title_edit)
        l.addRow("Folder:", path_box)
        l.addRow("Start Time:", self.start_dt)
        
        bb = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        l.addWidget(bb)
        
    def browse(self):
        d = QFileDialog.getExistingDirectory(self, "Select Directory")
        if d: self.path_edit.setText(d)

    def get_data(self):
        return (self.title_edit.text(), self.path_edit.text(), 
                0.0, 0.0, 
                self.start_dt.dateTime().toPyDateTime())
