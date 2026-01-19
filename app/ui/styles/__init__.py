"""
스타일 관리 패키지

이 패키지는 애플리케이션의 UI 테마와 스타일을 관리합니다.

내보내기:
    StyleManager: 스타일 관리 싱글톤 클래스
    apply_modern_style: 모던 스타일 적용 함수
    get_style_manager: 스타일 매니저 인스턴스 반환 함수
"""

from .style_manager import StyleManager, apply_modern_style, get_style_manager

__all__ = ['StyleManager', 'apply_modern_style', 'get_style_manager']
