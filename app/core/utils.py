"""
유틸리티 함수 모듈

이 모듈은 프로젝트 전반에서 사용되는 범용 유틸리티 함수를 제공합니다.

주요 기능:
    - 파일명 정규화 (sanitize_filename)

함수:
    sanitize_filename: 파일 시스템에서 안전하게 사용할 수 있는 파일명 생성
"""

import re


def sanitize_filename(name: str) -> str:
    """
    파일 시스템에서 안전하게 사용할 수 있도록 파일명을 정규화합니다.

    Windows, macOS, Linux 등 다양한 운영체제에서 파일명으로 사용할 수 없는
    문자들을 제거하고, 공백을 언더스코어로 변환합니다.

    처리 과정:
        1. Windows에서 금지된 문자 제거: \\ / * ? : " < > |
        2. 공백을 언더스코어(_)로 변환
        3. 결과가 빈 문자열이면 "Unnamed" 반환

    금지 문자 설명:
        - \\ : 경로 구분자 (Windows)
        - /  : 경로 구분자 (Unix/Mac)
        - *  : 와일드카드
        - ?  : 와일드카드
        - :  : 드라이브 구분자 (Windows), 대체 데이터 스트림
        - "  : 명령줄 인용 충돌
        - <  : 리디렉션 연산자
        - >  : 리디렉션 연산자
        - |  : 파이프 연산자

    매개변수:
        name: 정규화할 원본 파일명 문자열

    반환값:
        정규화된 안전한 파일명 문자열

    예시:
        >>> sanitize_filename("Project: Test/2024")
        'Project_Test2024'
        >>> sanitize_filename("My Ship <Tanker>")
        'My_Ship_Tanker'
        >>> sanitize_filename("   ")
        'Unnamed'
        >>> sanitize_filename("정상파일명.json")
        '정상파일명.json'
    """
    # 금지 문자 제거 (정규 표현식 사용)
    s = re.sub(r'[\\/*?:"<>|]', "", name)
    # 공백을 언더스코어로 변환
    s = s.replace(" ", "_")
    # 빈 문자열이면 기본값 반환
    return s if s else "Unnamed"
