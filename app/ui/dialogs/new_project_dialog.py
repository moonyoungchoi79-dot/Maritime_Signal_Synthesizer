"""
새 프로젝트 다이얼로그 모듈

이 모듈은 새 프로젝트를 생성하기 위한 다이얼로그를 제공합니다.
프로젝트 이름, 저장 폴더, 시뮬레이션 시작 시간을 설정할 수 있습니다.

클래스:
    NewProjectDialog: 새 프로젝트 생성 다이얼로그
"""

import os
import datetime

from PyQt6.QtWidgets import (
    QHBoxLayout, QFormLayout, QPushButton, QLineEdit, QFileDialog, QDialog,
    QDialogButtonBox, QDateTimeEdit
)


class NewProjectDialog(QDialog):
    """
    새 프로젝트를 생성하기 위한 다이얼로그 클래스입니다.

    프로젝트의 기본 정보를 입력받습니다:
    - 프로젝트 제목
    - 저장 폴더 경로
    - 시뮬레이션 시작 시간

    속성:
        title_edit: 프로젝트 제목 입력 필드
        path_edit: 저장 폴더 경로 입력 필드
        start_dt: 시뮬레이션 시작 시간 선택 위젯
    """

    def __init__(self, parent=None):
        """
        NewProjectDialog를 초기화합니다.

        매개변수:
            parent: 부모 위젯 (기본값: None)
        """
        super().__init__(parent)
        self.setWindowTitle("New Project")
        self.resize(560, 350)
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
        """
        폴더 선택 다이얼로그를 표시합니다.

        사용자가 선택한 디렉토리 경로를 path_edit에 설정합니다.
        """
        d = QFileDialog.getExistingDirectory(self, "Select Directory")
        if d:
            self.path_edit.setText(d)

    def get_data(self):
        """
        입력된 프로젝트 정보를 반환합니다.

        반환값:
            tuple: (제목, 폴더 경로, 위도, 경도, 시작 시간)
                - 위도와 경도는 현재 0.0으로 고정
        """
        return (self.title_edit.text(), self.path_edit.text(),
                0.0, 0.0,
                self.start_dt.dateTime().toPyDateTime())
