import sys
from PyQt6.QtWidgets import QApplication
from app.ui.windows.main_window import MainWindow
from app.ui.styles import apply_modern_style
from app.core.models.project import current_project

if __name__ == "__main__":
    app = QApplication(sys.argv)

    # Apply modern style based on theme setting
    theme_mode = current_project.settings.theme_mode if hasattr(current_project.settings, 'theme_mode') else "System"
    apply_modern_style(app, theme_mode)

    w = MainWindow()
    w.show()
    sys.exit(app.exec())
