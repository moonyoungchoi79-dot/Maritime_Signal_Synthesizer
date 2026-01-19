"""
도움말 다이얼로그 모듈

이 모듈은 애플리케이션의 사용자 가이드를 제공하는 다이얼로그를 정의합니다.
좌측의 토픽 목록과 우측의 상세 내용을 보여주는 구조로 되어 있습니다.

클래스:
    HelpDialog: 사용자 가이드 다이얼로그
"""

from PyQt6.QtWidgets import (
    QDialog, QHBoxLayout, QListWidget, QTextBrowser, QFrame
)

class HelpDialog(QDialog):
    """
    사용자 가이드를 표시하는 다이얼로그 클래스입니다.

    QListWidget을 사용하여 도움말 목차를 표시하고,
    QTextBrowser를 사용하여 선택된 목차의 상세 내용을 HTML 형식으로 표시합니다.
    """

    def __init__(self, parent=None):
        """
        HelpDialog를 초기화합니다.

        UI 레이아웃을 구성하고 도움말 데이터를 로드합니다.

        매개변수:
            parent: 부모 위젯 (기본값: None)
        """
        super().__init__(parent)
        self.setWindowTitle("User Guide")
        self.resize(1120, 840)
        
        layout = QHBoxLayout(self)
        self.list = QListWidget()         # 토픽 목록 위젯
        self.list.setFixedWidth(200)
        self.content = QTextBrowser()     # 내용 표시 위젯
        self.content.setReadOnly(True)
        
        line = QFrame()                   # 구분선
        line.setFrameShape(QFrame.Shape.VLine)
        line.setFrameShadow(QFrame.Shadow.Sunken)
        
        layout.addWidget(self.list)
        layout.addWidget(line)
        layout.addWidget(self.content)
        
        # 도움말 데이터 (토픽 제목: HTML 내용)
        self.topics = {
            "Introduction": """
                
                <h3>프로그램 개요</h3>
                <p><b>NMEA 0183 Signal Synthesizer</b>는 선박의 항해 데이터를 시뮬레이션하고, 이를 표준 NMEA 0183 프로토콜(UDP)로 송출하는 도구입니다. 실제 선박이나 장비 없이도 ECDIS, VDR, 레이더 등의 해양 전자 장비를 테스트하거나 교육용 시나리오를 구성할 수 있습니다.</p>
                
                <h3>프로젝트 기반 작업</h3>
                <p>모든 작업은 <b>프로젝트(.json)</b> 단위로 관리됩니다. 프로젝트 파일에는 선박의 경로, 속도 설정, 이벤트 스크립트, 화면 설정 등 모든 시나리오 정보가 저장됩니다.</p>
                
                <h3>시작하기 (Getting Started)</h3>
                <ol>
                    <li><b>새 프로젝트 생성:</b> 상단 메뉴의 <code>File > New Project</code>를 클릭합니다. 프로젝트 이름, 저장 폴더, 시뮬레이션 시작 시간(Start Time)을 설정합니다.</li>
                    <li><b>프로젝트 열기:</b> <code>File > Open Project</code>를 통해 기존에 작업하던 .json 파일을 불러올 수 있습니다.</li>
                    <li><b>저장:</b> 작업 중간중간 <code>File > Save Project</code>를 눌러 데이터를 저장하세요. 프로그램 종료 시에도 저장 여부를 묻습니다.</li>
                </ol>
            """,
            "Path Editor": """
                
                <p>시뮬레이션의 무대가 되는 지도와 선박의 이동 경로를 편집하는 화면입니다.</p>
                
                <h3>기본 조작</h3>
                <ul>
                    <li><b>이동 (Pan):</b> 마우스 휠(Middle Button)을 누른 채 드래그하거나, 빈 공간을 좌클릭 드래그합니다.</li>
                    <li><b>확대/축소 (Zoom):</b> 마우스 휠을 굴립니다. 화면 중앙을 기준으로 확대/축소됩니다.</li>
                    <li><b>좌표 확인:</b> 화면 좌측 하단에 현재 마우스 커서 위치의 위도(Lat), 경도(Lon)가 실시간으로 표시됩니다.</li>
                </ul>
                
                <h3>객체 관리 (Object)</h3>
                <ul>
                    <li><b>Add Ship:</b> 새로운 선박을 추가합니다. 선박 이름과 MMSI가 자동 할당됩니다.</li>
                    <li><b>Add Area:</b> 지도상에 다각형 영역(Area)을 그립니다. 위험 구역이나 이벤트 트리거용 구역으로 활용됩니다.</li>
                    <li><b>Select Object:</b> 우측 패널의 콤보박스에서 편집할 선박이나 영역을 선택합니다. 선택된 객체는 지도상에서 강조 표시됩니다.</li>
                </ul>
                
                <h3>경로 편집 (Path)</h3>
                <p>선박을 선택한 후 다음 기능을 사용할 수 있습니다.</p>
                <ul>
                    <li><b>Add Point:</b> 지도를 클릭하여 경로점(Waypoint)을 추가합니다. 각 점마다 목표 속도(Speed)를 설정할 수 있습니다.</li>
                    <li><b>Move Point:</b> 기존 경로점을 드래그하여 위치를 수정합니다.</li>
                    <li><b>Delete Point:</b> 경로점을 클릭하여 삭제합니다.</li>
                    <li><b>우클릭 메뉴:</b> 경로점 위에서 우클릭하면 이동/삭제/중간 삽입 등의 메뉴를 빠르게 사용할 수 있습니다.</li>
                </ul>
            """,
            "Speed Generator": """
                
                <p>Path Editor에서 설정한 경로점과 목표 속도를 바탕으로, 초 단위의 정밀한 위치/속도 데이터를 생성하는 단계입니다.</p>
                
                <ol>
                    <li><b>대상 선택:</b> Path Editor에서 선박을 선택한 상태여야 합니다.</li>
                    <li><b>Variance (분산):</b> 속도 그래프에 자연스러운 노이즈(변동)를 추가합니다. 값이 클수록 속도가 불규칙하게 변합니다.</li>
                    <li><b>Generate Speed:</b> 버튼을 누르면 전체 경로에 대한 시계열 데이터가 계산됩니다.</li>
                    <li><b>그래프 확인:</b> 생성된 속도 프로파일이 그래프로 표시됩니다. 마우스를 올리면 해당 지점의 시간과 속도 값을 툴팁으로 확인할 수 있으며, 드래그하여 확대할 수 있습니다.</li>
                </ol>
            """,
            "Simulator": """
                
                <p>실제 NMEA 데이터를 생성하고 전송하는 핵심 화면입니다.</p>
                
                <h3>주요 기능</h3>
                <ul>
                    <li><b>UDP 전송:</b> 설정된 IP와 Port로 NMEA 데이터를 실시간 전송합니다.</li>
                    <li><b>재생 제어:</b> Start, Pause, Stop 버튼으로 시뮬레이션을 제어합니다. 배속(Speed x)을 조절하여 시간을 빠르게 흐르게 할 수 있습니다.</li>
                    <li><b>Follow 모드:</b> 특정 선박을 선택하면 카메라가 해당 선박을 자동으로 따라갑니다.</li>
                    <li><b>Time Remaining:</b> 현재 Follow 중인 선박(또는 자선)의 목적지 도착까지 남은 시간을 표시합니다.</li>
                </ul>
                
                <h3>RTG (Random Target Generate)</h3>
                <p>시뮬레이션 중 무작위 선박들을 생성하여 교통량을 늘릴 수 있습니다.</p>
                <ul>
                    <li><b>생성:</b> 'Random Target Generate' 버튼을 눌러 반경(nm)과 생성할 선박 수(AI/RA/Both)를 입력합니다.</li>
                    <li><b>특징:</b> 랜덤 타겟은 자선 주변에서 생성되며 직선 운동을 합니다.</li>
                    <li><b>삭제:</b> 'Clear Random Targets' 버튼으로 일괄 삭제하거나, 지도상에서 더블 클릭하여 개별 삭제할 수 있습니다.</li>
                </ul>
                
                <h3>신호 제어 (Signal Control)</h3>
                <p>각 선박별로 발신할 신호 종류(AIVDM, RATTM, Camera)를 켜거나 끌 수 있으며, 발신 주기를 개별적으로 설정할 수 있습니다.</p>
            """,
            "Event Scripter": """
                
                <p>단순한 경로 이동을 넘어, 특정 조건에 따라 선박의 상태를 동적으로 변경하는 시나리오를 작성합니다.</p>
                
                <h3>트리거 (Trigger)</h3>
                <ul>
                    <li><b>TIME:</b> 시뮬레이션 시작 후 특정 시간이 지났을 때 발생합니다.</li>
                    <li><b>AREA_ENTER:</b> 특정 선박이 지정된 영역(Area)에 진입했을 때 발생합니다.</li>
                    <li><b>CPA_UNDER:</b> 기준 선박과의 거리가 설정된 값(NM) 이하로 좁혀졌을 때 발생합니다.</li>
                    <li><b>CPA_OVER:</b> 기준 선박과의 거리가 설정된 값(NM) 이상으로 멀어졌을 때 발생합니다.</li>
                </ul>
                
                <h3>액션 (Action)</h3>
                <ul>
                    <li><b>STOP:</b> 해당 선박을 즉시 정지시킵니다.</li>
                    <li><b>CHANGE_SPEED:</b> 선박의 속도를 지정된 값(kn)으로 변경합니다.</li>
                    <li><b>CHANGE_HEADING:</b> 선박의 침로를 지정된 각도(deg)로 변경합니다.</li>
                    <li><b>MANEUVER:</b> 특수 기동을 수행합니다.
                        <ul>
                            <li><b>ReturnToOriginalPath_ShortestDistance:</b> 원래 경로로 최단거리 복귀합니다.</li>
                            <li><b>ChangeDestination_ToOriginalFinal:</b> 원래 목적지로 향하도록 침로를 변경합니다.</li>
                        </ul>
                    </li>
                </ul>
            """,
            "Settings": """
                
                <p>프로그램의 전반적인 동작과 외관을 설정합니다.</p>
                <ul>
                    <li><b>Appearance:</b> 테마(System/Light/Dark), 선박 및 경로 색상/두께 등을 사용자 정의할 수 있습니다.</li>
                    <li><b>Dropout:</b> 통신 환경 악화를 모사하기 위해 각 신호(AIVDM, RATTM 등)의 유실 확률을 설정합니다.</li>
                    <li><b>Object:</b> 자선(Own Ship)으로 사용할 선박의 인덱스를 지정하거나, 랜덤 타겟 생성 시의 속도 분산 등을 설정합니다.</li>
                </ul>
            """
        }
        
        self.list.addItems(self.topics.keys())
        self.list.currentRowChanged.connect(self.show_topic)
        self.list.setCurrentRow(0)

    def show_topic(self, row):
        """
        선택된 토픽의 내용을 화면에 표시합니다.

        QListWidget의 행 변경 시그널에 연결되는 슬롯 함수입니다.

        매개변수:
            row: 선택된 행 인덱스
        """
        if row < 0: return
        key = self.list.item(row).text()
        self.content.setText(f"<h2>{key}</h2><div>{self.topics[key]}</div>")
