"""
Maritime Signal Synthesizer - 메인 진입점

애플리케이션의 진입점입니다.
PyQt6 기반의 GUI 애플리케이션을 초기화하고 실행합니다.

주요 기능:
- QApplication 인스턴스 생성 및 초기화
- 테마 스타일 적용 (System/Light/Dark 모드 지원)
- 메인 윈도우 생성 및 표시
"""

import sys
from PyQt6.QtWidgets import QApplication
from PyQt6.QtGui import QFont
from app.ui.main_window import MainWindow
from app.ui.styles import apply_modern_style
from app.models.project import current_project

if __name__ == "__main__":
    # PyQt6 애플리케이션 인스턴스 생성
    app = QApplication(sys.argv)
    app.setFont(QFont("Segoe UI", 10))

    # 프로젝트의 속성을 포함하는 json파일에서 테마 모드를 가져와 스타일 적용
    # 설정이 없으면 기본값 "System" 사용
    theme_mode = current_project.settings.theme_mode if hasattr(current_project.settings, 'theme_mode') else "System"
    apply_modern_style(app, theme_mode)

    # 메인 윈도우 생성 및 표시
    w = MainWindow()
    w.show()

    # 이벤트 루프 실행 및 종료 코드 반환
    sys.exit(app.exec())
