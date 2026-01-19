"""
메시지 박스 위젯 모듈

이 모듈은 애플리케이션에서 사용되는 커스텀 메시지 박스를 제공합니다.
기본 QMessageBox보다 더 큰 크기로 표시됩니다.

클래스:
    ResizableMessageBox: 크기 조절이 가능한 메시지 박스

함수:
    show_information: 정보 메시지 표시
    show_warning: 경고 메시지 표시
    show_critical: 오류 메시지 표시
    show_question: 질문 메시지 표시 (Yes/No)
"""

from PyQt6.QtWidgets import QMessageBox

# 편의를 위해 StandardButton 내보내기
StandardButton = QMessageBox.StandardButton


class ResizableMessageBox(QMessageBox):
    """
    크기 조절이 가능한 메시지 박스 클래스입니다.

    기본 QMessageBox의 최소 너비를 400픽셀로 설정하여
    더 나은 가독성을 제공합니다.
    """

    def __init__(self, parent=None):
        """메시지 박스를 초기화합니다."""
        super().__init__(parent)
        self.setMinimumWidth(400)  # 최소 너비 설정


def show_information(parent, title: str, text: str):
    """
    정보 메시지 박스를 표시합니다.

    매개변수:
        parent: 부모 위젯
        title: 메시지 박스 제목
        text: 메시지 내용

    반환값:
        사용자가 클릭한 버튼
    """
    msg = ResizableMessageBox(parent)
    msg.setIcon(QMessageBox.Icon.Information)
    msg.setWindowTitle(title)
    msg.setText(text)
    msg.setStandardButtons(QMessageBox.StandardButton.Ok)
    return msg.exec()


def show_warning(parent, title: str, text: str):
    """
    경고 메시지 박스를 표시합니다.

    매개변수:
        parent: 부모 위젯
        title: 메시지 박스 제목
        text: 메시지 내용

    반환값:
        사용자가 클릭한 버튼
    """
    msg = ResizableMessageBox(parent)
    msg.setIcon(QMessageBox.Icon.Warning)
    msg.setWindowTitle(title)
    msg.setText(text)
    msg.setStandardButtons(QMessageBox.StandardButton.Ok)
    return msg.exec()


def show_critical(parent, title: str, text: str):
    """
    오류(Critical) 메시지 박스를 표시합니다.

    매개변수:
        parent: 부모 위젯
        title: 메시지 박스 제목
        text: 메시지 내용

    반환값:
        사용자가 클릭한 버튼
    """
    msg = ResizableMessageBox(parent)
    msg.setIcon(QMessageBox.Icon.Critical)
    msg.setWindowTitle(title)
    msg.setText(text)
    msg.setStandardButtons(QMessageBox.StandardButton.Ok)
    return msg.exec()


def show_question(parent, title: str, text: str,
                  buttons=QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                  default_button=QMessageBox.StandardButton.No):
    """
    질문 메시지 박스를 표시합니다.

    매개변수:
        parent: 부모 위젯
        title: 메시지 박스 제목
        text: 질문 내용
        buttons: 표시할 버튼 (기본값: Yes | No)
        default_button: 기본 선택 버튼 (기본값: No)

    반환값:
        사용자가 클릭한 버튼 (StandardButton.Yes 또는 StandardButton.No 등)
    """
    msg = ResizableMessageBox(parent)
    msg.setIcon(QMessageBox.Icon.Question)
    msg.setWindowTitle(title)
    msg.setText(text)
    msg.setStandardButtons(buttons)
    msg.setDefaultButton(default_button)
    return msg.exec()
