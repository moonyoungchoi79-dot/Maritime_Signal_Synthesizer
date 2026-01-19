"""
유틸리티 함수 모듈

이 모듈은 애플리케이션 전반에서 사용되는 공통 유틸리티 함수를 제공합니다.

주요 기능:
- 파일명 정제 (운영체제 호환)
"""

import re


def sanitize_filename(name: str) -> str:
    """
    파일명으로 사용할 수 없는 문자를 제거하고 안전한 파일명으로 변환합니다.

    Windows, macOS, Linux 등 모든 운영체제에서 사용 가능한
    안전한 파일명을 생성합니다.

    매개변수:
        name: 원본 파일명 문자열

    반환값:
        정제된 파일명 (빈 문자열일 경우 "Unnamed" 반환)

    예시:
        sanitize_filename("my file:test.txt") -> "my_filetest.txt"
        sanitize_filename("A/B\\C") -> "ABC"
    """
    # 파일명에 사용할 수 없는 문자 제거: \ / * ? : " < > |
    s = re.sub(r'[\\/*?:"<>|]', "", name)

    # 공백을 언더스코어로 변환
    s = s.replace(" ", "_")

    # 빈 문자열이면 기본값 반환
    return s if s else "Unnamed"
