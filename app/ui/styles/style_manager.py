"""
스타일 관리자 모듈

이 모듈은 애플리케이션의 UI 테마와 QSS 스타일시트를 관리합니다.

클래스:
    StyleManager: 스타일 관리 싱글톤 클래스

함수:
    apply_modern_style: 모던 스타일 적용
    get_style_manager: 스타일 매니저 인스턴스 반환

지원 테마:
    - Light: 밝은 테마
    - Dark: 어두운 테마
    - System: 시스템 설정 따름
"""

import os
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt


class StyleManager:
    """
    애플리케이션 스타일과 테마를 관리하는 싱글톤 클래스입니다.

    QSS 스타일시트 파일을 로드하고 테마를 적용합니다.
    Light, Dark, System 세 가지 테마 모드를 지원합니다.

    사용법:
        manager = StyleManager.instance()
        manager.apply_theme(app, "Dark")
    """

    _instance = None  # 싱글톤 인스턴스
    _styles_dir = os.path.dirname(os.path.abspath(__file__))  # 스타일 파일 디렉토리

    @classmethod
    def instance(cls):
        """
        싱글톤 인스턴스를 반환합니다.

        반환값:
            StyleManager 인스턴스
        """
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def __init__(self):
        """스타일 매니저를 초기화하고 스타일 파일을 로드합니다."""
        self._current_theme = None  # 현재 적용된 테마
        self._light_style = None  # 라이트 테마 스타일시트
        self._dark_style = None  # 다크 테마 스타일시트
        self._load_styles()

    def _load_styles(self):
        """
        디스크에서 QSS 스타일시트 파일을 로드합니다.

        modern_light.qss와 modern_dark.qss 파일을 읽어옵니다.
        파일이 없거나 읽기 실패 시 빈 문자열로 대체합니다.
        """
        light_path = os.path.join(self._styles_dir, 'modern_light.qss')
        dark_path = os.path.join(self._styles_dir, 'modern_dark.qss')

        # 라이트 테마 로드
        try:
            with open(light_path, 'r', encoding='utf-8') as f:
                self._light_style = f.read()
        except Exception as e:
            print(f"Warning: Could not load light theme: {e}")
            self._light_style = ""

        # 다크 테마 로드
        try:
            with open(dark_path, 'r', encoding='utf-8') as f:
                self._dark_style = f.read()
        except Exception as e:
            print(f"Warning: Could not load dark theme: {e}")
            self._dark_style = ""

    def apply_theme(self, app: QApplication, theme_mode: str = "System"):
        """
        애플리케이션에 테마를 적용합니다.

        매개변수:
            app: QApplication 인스턴스
            theme_mode: 테마 모드 ("Light", "Dark", "System")
        """
        is_dark = self._resolve_theme(theme_mode)

        if is_dark:
            app.setStyleSheet(self._dark_style)
            self._current_theme = "dark"
        else:
            app.setStyleSheet(self._light_style)
            self._current_theme = "light"

    def _resolve_theme(self, theme_mode: str) -> bool:
        """
        테마 모드에 따라 다크 테마 사용 여부를 결정합니다.

        매개변수:
            theme_mode: 테마 모드 문자열

        반환값:
            True: 다크 테마 사용
            False: 라이트 테마 사용
        """
        if theme_mode == "Dark":
            return True
        elif theme_mode == "Light":
            return False
        else:  # System - 시스템 설정 따름
            try:
                hints = QApplication.styleHints()
                if hints is not None:
                    return hints.colorScheme() == Qt.ColorScheme.Dark
            except:
                pass
            return False

    def get_current_theme(self) -> str:
        """
        현재 적용된 테마 이름을 반환합니다.

        반환값:
            'light' 또는 'dark'
        """
        return self._current_theme or "light"

    def is_dark_theme(self) -> bool:
        """
        현재 다크 테마가 적용되어 있는지 확인합니다.

        반환값:
            True: 다크 테마
            False: 라이트 테마
        """
        return self._current_theme == "dark"

    def refresh_styles(self):
        """
        디스크에서 스타일을 다시 로드합니다.

        개발 중 스타일 파일 수정 시 유용합니다.
        """
        self._load_styles()


def apply_modern_style(app: QApplication, theme_mode: str = "System"):
    """
    애플리케이션에 모던 스타일을 적용하는 편의 함수입니다.

    매개변수:
        app: QApplication 인스턴스
        theme_mode: 테마 모드 ("Light", "Dark", "System")
    """
    StyleManager.instance().apply_theme(app, theme_mode)


def get_style_manager() -> StyleManager:
    """
    StyleManager 싱글톤 인스턴스를 반환합니다.

    반환값:
        StyleManager 인스턴스
    """
    return StyleManager.instance()
