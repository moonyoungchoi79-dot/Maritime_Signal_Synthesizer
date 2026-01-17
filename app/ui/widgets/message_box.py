from PyQt6.QtWidgets import QMessageBox

# Export StandardButton for convenience
StandardButton = QMessageBox.StandardButton


class ResizableMessageBox(QMessageBox):
    """QMessageBox that resizes itself after being shown"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumWidth(400)


def show_information(parent, title: str, text: str):
    """Show information message box with larger size"""
    msg = ResizableMessageBox(parent)
    msg.setIcon(QMessageBox.Icon.Information)
    msg.setWindowTitle(title)
    msg.setText(text)
    msg.setStandardButtons(QMessageBox.StandardButton.Ok)
    return msg.exec()


def show_warning(parent, title: str, text: str):
    """Show warning message box with larger size"""
    msg = ResizableMessageBox(parent)
    msg.setIcon(QMessageBox.Icon.Warning)
    msg.setWindowTitle(title)
    msg.setText(text)
    msg.setStandardButtons(QMessageBox.StandardButton.Ok)
    return msg.exec()


def show_critical(parent, title: str, text: str):
    """Show critical message box with larger size"""
    msg = ResizableMessageBox(parent)
    msg.setIcon(QMessageBox.Icon.Critical)
    msg.setWindowTitle(title)
    msg.setText(text)
    msg.setStandardButtons(QMessageBox.StandardButton.Ok)
    return msg.exec()


def show_question(parent, title: str, text: str,
                  buttons=QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                  default_button=QMessageBox.StandardButton.No):
    """Show question message box with larger size"""
    msg = ResizableMessageBox(parent)
    msg.setIcon(QMessageBox.Icon.Question)
    msg.setWindowTitle(title)
    msg.setText(text)
    msg.setStandardButtons(buttons)
    msg.setDefaultButton(default_button)
    return msg.exec()
