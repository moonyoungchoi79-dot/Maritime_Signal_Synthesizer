# Maritime Signal Synthesizer

**Maritime Signal Synthesizer**는 선박의 위치, 항로, 이벤트에 따라 실제 항해 상황과 유사한 신호 데이터를 생성하는 시뮬레이션 소프트웨어입니다. AIS(AIVDM), 레이더(RATTM), 카메라 감지 신호 등을 NMEA 0183 및 JSON 형식으로 생성하여 UDP 네트워크와 Redis를 통해 전송합니다.

---

## Introduction

이 프로그램은 사용자가 GUI 환경에서 직접 지도 위에 선박과 경로를 배치하고, 특정 상황(이벤트)을 정의하여 시뮬레이션을 수행하는 도구입니다. 경로 이동뿐만 아니라, 선박 간의 조우(CPA), 특정 구역 진입/이탈 등 다양한 조건에 따라 선박이 자동으로 속도를 줄이거나 변침하는 등의 시나리오를 구성할 수 있습니다. 생성된 데이터는 프로젝트 단위로 관리되며, 실시간으로 외부 시스템에 신호를 송출합니다.

---

## Major Components

### 1. 신호 생성 및 전송
- **GPS (GPGGA):** 자선(Own Ship)의 GPS 좌표 데이터 송출.
- **AIS (AIVDM):** 타겟 선박 식별 부호(MMSI)를 포함한 위치, 속도, 침로 정보 송출.
- **Radar (RATTM):** 레이더 추적 타겟 메시지 생성.
- **Camera (Redis):** 가상 카메라 뷰에서 탐지된 객체의 bbox 정보를 Redis 채널로 송출.
  - **Format:** `{"prediction": [[left, top, right, bottom, conf, class], ...]}`
- **수신 모델 적용:** 거리 및 환경 설정에 따른 Signal Dropout 및 노이즈 시뮬레이션.

### 2. 항로 및 선박 제어
- **경로 편집:** 지도상 클릭으로 Waypoint 추가/이동/삭제.
- **항법 모델:** 대권항로 및 등각항로 지원.
- **속도 프로필:** 구간별 목표 속도 설정 가능.
- **실시간 제어:** 시뮬레이션 중 특정 선박의 속도, 침로를 수동으로 즉시 오버라이드 가능.

### 3. 이벤트 및 시나리오 관리
- **Trigger:** 시간 경과, Area 진입/이탈, CPA(최근접점) 거리, 선박 간 거리 등 다양한 조건 감지.
- **Action:** 정지, 속도 변경, 침로 변경, 회피 기동 등 자동 수행.
- **시나리오 구성:** 여러 이벤트를 조합하고 Prerequisite(조건부 발동 조건)를 지정하여 복합적인 상황 연출.

### 4. 데이터 관리 및 분석
- **프로젝트 시스템:** 모든 설정(선박, 경로, 이벤트)을 JSON 기반 프로젝트 폴더로 저장/로드.
- **로그 및 내보내기:** 실시간 NMEA 로그 모니터링 및 CSV 파일 내보내기 지원.
- **랜덤 타겟 생성:** 배경 트래픽을 위한 무작위 선박 생성 기능.

---

## Run Environment

### 요구 사항
- Python 3.8 이상
- 필수 라이브러리: requirements.txt 참조

### 설치 및 실행

1. **리포지토리 클론**
   ```bash
   git clone https://github.com/moonyoungchoi79-dot/Maritime_Signal_Synthesizer
   cd Maritime_Signal_Synthesizer
   ```

2. **필수 라이브러리 설치**
   ```bash
   pip install -r requirements.txt
   ```

3. **프로그램 실행**
   ```bash
   python main.py
   ```

### 기본 사용 흐름
1. **새 프로젝트 생성:** `File > New Project` 메뉴를 통해 프로젝트 폴더와 시작 시간을 설정합니다.
2. **자선 및 타선 추가:** `Path` 탭에서 `Add Ship`을 눌러 선박을 생성하고 지도에 경로를 그립니다.
3. **이벤트 설정 (선택):** `Event` 탭에서 특정 조건(예: 10분 후 감속)을 정의합니다.
4. **시나리오 구성 (선택):** `Scenario` 탭에서 정의된 이벤트를 시나리오에 추가하고 활성화합니다.
5. **시뮬레이션 시작:** `Simulation` 탭으로 이동하여 UDP IP/Port를 설정하고 `Start` 버튼을 누릅니다.

---

## Program Structure

이 프로젝트는 **Model-View-Controller** 패턴으로 설계되어 있습니다.

### Root Directory
- **`main.py`**: 프로그램의 진입점입니다. `QApplication`을 생성하고 테마를 적용한 뒤 `MainWindow`를 실행합니다.
- **`requirements.txt`**: 프로젝트 실행에 필요한 Python 패키지 의존성 목록입니다.
- **`README.md`**: 프로젝트 설명 문서입니다.
- **`LICENSE`**: 프로젝트 라이선스 정보 파일입니다.

### `app/` (Main Package)
애플리케이션의 핵심 소스 코드가 포함된 패키지입니다.

#### 1. `app/core/` (Core Logic)
프로그램 전반에서 사용되는 핵심 로직과 유틸리티입니다.
- **`camera_projection.py`**: 3D 좌표를 2D 카메라 뷰로 투영하고 Bounding Box를 생성하는 로직.
- **`constants.py`**: 앱 이름, 윈도우 크기, 로그 포맷 등 전역 상수 정의.
- **`geometry.py`**: 좌표 변환(위경도 ↔ 픽셀), 대권항로 계산, 방위각 계산 등 기하학적 연산 함수 모음.
- **`nmea.py`**: NMEA 0183 메시지 생성 및 체크섬 계산 유틸리티.
- **`redis_transmitter.py`**: 카메라 탐지 데이터를 Redis 채널로 전송하는 모듈.
- **`state.py`**: 전역 상태 관리 (현재 로드된 시나리오 목록 등).
- **`utils.py`**: 파일명 정규화 등 범용 유틸리티 함수.

#### 2. `app/models/` (Data Models)
데이터 구조를 정의하고 JSON 직렬화/역직렬화를 담당합니다.
- **`project.py`**: `Project` 클래스 정의. 선박, 영역, 설정 등 시뮬레이션 전체 데이터를 총괄하는 인스턴스(`current_project`)를 포함합니다.
- **`ship.py`**: `ShipData` 클래스. 선박의 제원, Waypoint, 속도 프로필, 신호 설정 등을 정의합니다.
- **`area.py`**: `Area` 클래스. 지도상의 다각형 구역(이벤트 트리거용) 데이터를 정의합니다.
- **`event.py`**: `EventTrigger` 클래스. 트리거 조건(시간, 거리 등)과 액션(변침, 변속 등)을 정의합니다.
- **`scenario.py`**: `Scenario` 클래스. 여러 이벤트를 그룹화하고 적용 범위를 관리합니다.
- **`settings.py`**: 프로젝트별 환경 설정(테마, 색상, 수신 모델 파라미터 등) 모델.
- **`map_info.py`**: 지도의 중심 좌표 및 축척 정보를 담는 모델.

#### 3. `app/ui/` (User Interface)
PyQt6 기반의 사용자 인터페이스 구성요소입니다.
- **`main_window.py`**: 애플리케이션의 메인 윈도우. 탭 관리, 툴바, 메뉴, 그리고 전체적인 UI 레이아웃을 조율합니다.

(1) **`app/ui/dialogs/` (Dialog Windows)**: 팝업 형태로 뜨는 대화상자들입니다.
- **`new_project_dialog.py`**: 프로젝트 생성 시 이름, 경로, 시작 시간을 입력받는 창.
- **`event_editor_dialog.py`**: 이벤트 편집 및 상세 설정을 위한 대화상자.
- **`random_target_generation_dialog.py`**: 랜덤 타겟 생성 설정(수량/속도/분포)을 입력받는 창.
- **`help_dialog.py`**: 사용자 가이드 및 도움말을 보여주는 창.
- **`settings_dialog.py`**: 프로그램 및 시뮬레이션 환경 설정을 변경하는 창.

(2) **`app/ui/map/` (Map Visualization)**
- **`map_view.py`**: `QGraphicsView`를 상속받아 지도를 렌더링합니다. 마우스 이벤트를 처리하여 지도 이동, 줌, 객체 선택 및 편집 기능을 수행합니다.
- **`sim_map_view.py`**: 시뮬레이션 실행 중 지도 화면. 경로 표시 및 실시간 선박 이동을 시각화합니다.
- **`ruler_overlay.py`**: 지도 상의 거리/방위 측정을 위한 오버레이 도구.

(3) **`app/ui/panels/` (Functional Tabs)**: 메인 윈도우의 각 탭에 해당하는 패널들입니다.
- **`simulation_panel.py`**: 시뮬레이션 실행/제어, UDP 설정, 로그 모니터링, 수동 선박 제어 기능을 담당합니다.
- **`event_panel.py`**: 이벤트를 생성하고 트리거/액션 조건을 편집하는 UI입니다.
- **`scenario_panel.py`**: 시나리오를 생성하고 이벤트를 배치/정렬하는 UI입니다.

(4) **`app/ui/styles/` (Theme & Styles)**: 애플리케이션의 테마와 스타일시트를 관리합니다.
- **`__init__.py`**: 스타일 패키지 초기화 및 `apply_modern_style` 함수 제공.
- **`style_manager.py`**: 테마(Light/Dark/System) 로드 및 적용을 담당하는 관리자 클래스.
- **`modern_dark.qss`**: 다크 모드용 QSS 스타일시트 파일.
- **`modern_light.qss`**: 라이트 모드용 QSS 스타일시트 파일.

(5) **`app/ui/widgets/` (Custom Widgets)**
- **`message_box.py`**: 사용자에게 알림이나 확인 메시지를 띄우는 기능을 담당합니다.
- **`time_input_widget.py`**: 시/분/초 단위 시간 입력 위젯(총 초 단위 변환 지원).
- **`panorama_view.py`**: EO/IR 카메라 뷰 및 탐지된 bbox 시각화(팬/틸트 제어).
- **`colored_tab_bar.py`**: 테마에 따라 색상이 변경되는 커스텀 탭바.

#### 4.**`app/workers/` (Background Workers)**
백그라운드 작업을 처리하는 워커 스레드입니다.
- **`simulation_worker.py`**: 시뮬레이션 루프를 별도 스레드에서 실행하며 신호 생성 및 전송을 담당합니다.
- **`random_target_generation_worker.py`**: 랜덤 타겟 생성 작업을 별도 스레드에서 수행하여 UI 프리징을 방지합니다.


---

##  Project Data Structure

프로젝트를 저장하면 지정된 폴더 내에 다음과 같은 구조로 데이터가 저장됩니다.
###### <span style="color: red;">주의! 프로젝트 폴더를 선택하면 그 폴더가 아래 구조에서 MyProject가 됩니다. Desktop 등 다른 파일이 함께 있는 폴더를 선택하면 MyProject.json 파일과 Object, Event, Scenario 폴더가 사용자의 다른 파일들과 뒤섞이게 됩니다.</span>

```text
MyProject/
├── MyProject.json       # 프로젝트 메인 파일 (설정, 맵 정보, 객체 목록 참조)
├── Object/              # 선박 및 구역 데이터 폴더
│   ├── Ship_1.json      # 개별 선박 데이터 (경로 포함)
│   ├── Area_0.json      # 개별 구역 데이터
│   └── ...
├── Event/               # 이벤트 정의 폴더
│   ├── EventName.json   # 개별 이벤트 설정
│   └── ...
└── Scenario/            # 시나리오 정의 폴더
    └── Scenario1.json   # 시나리오 구성 파일
```

---

## License

###### This project is licensed under the GPL-3.0 license - see the LICENSE file for details.
