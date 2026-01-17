"""
Modern Style Manager for Maritime Signal Synthesizer
Manages loading and applying QSS stylesheets based on theme settings
"""

import os
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt


class StyleManager:
    """Manages application styling and themes"""

    _instance = None
    _styles_dir = os.path.dirname(os.path.abspath(__file__))

    @classmethod
    def instance(cls):
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def __init__(self):
        self._current_theme = None
        self._light_style = None
        self._dark_style = None
        self._load_styles()

    def _load_styles(self):
        """Load QSS files from disk"""
        light_path = os.path.join(self._styles_dir, 'modern_light.qss')
        dark_path = os.path.join(self._styles_dir, 'modern_dark.qss')

        try:
            with open(light_path, 'r', encoding='utf-8') as f:
                self._light_style = f.read()
        except Exception as e:
            print(f"Warning: Could not load light theme: {e}")
            self._light_style = ""

        try:
            with open(dark_path, 'r', encoding='utf-8') as f:
                self._dark_style = f.read()
        except Exception as e:
            print(f"Warning: Could not load dark theme: {e}")
            self._dark_style = ""

    def apply_theme(self, app: QApplication, theme_mode: str = "System"):
        """
        Apply theme to the application

        Args:
            app: QApplication instance
            theme_mode: "Light", "Dark", or "System"
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
        Determine if dark theme should be used

        Returns:
            True if dark theme, False if light theme
        """
        if theme_mode == "Dark":
            return True
        elif theme_mode == "Light":
            return False
        else:  # System
            try:
                hints = QApplication.styleHints()
                if hints is not None:
                    return hints.colorScheme() == Qt.ColorScheme.Dark
            except:
                pass
            return False

    def get_current_theme(self) -> str:
        """Get current theme name ('light' or 'dark')"""
        return self._current_theme or "light"

    def is_dark_theme(self) -> bool:
        """Check if current theme is dark"""
        return self._current_theme == "dark"

    def refresh_styles(self):
        """Reload styles from disk (useful for development)"""
        self._load_styles()


def apply_modern_style(app: QApplication, theme_mode: str = "System"):
    """
    Convenience function to apply modern style to application

    Args:
        app: QApplication instance
        theme_mode: "Light", "Dark", or "System"
    """
    StyleManager.instance().apply_theme(app, theme_mode)


def get_style_manager() -> StyleManager:
    """Get the StyleManager singleton instance"""
    return StyleManager.instance()
