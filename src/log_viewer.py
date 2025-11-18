# src/log_viewer.py
import pandas as pd
import os
import matplotlib.pyplot as plt
# Matplotlib 3D 플롯을 위한 모듈 임포트
from mpl_toolkits.mplot3d import Axes3D 
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
# 🚨 Matplotlib 툴바 임포트 추가
from matplotlib.backends.backend_qtagg import NavigationToolbar2QT as NavigationToolbar

from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QPushButton, QFileDialog, QSizePolicy, QLabel, QScrollArea
)
from PySide6.QtCore import Slot, Qt, QTimer

# -----------------
# 3D 플롯 전용 새 창 클래스 (변화 없음)
# -----------------
class Plot3DWindow(QMainWindow):
    """
    로봇의 엔드 이펙터 3D 경로를 표시하는 독립된 창
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("로봇 End-Effector 3D 경로")
        self.setGeometry(150, 150, 800, 700)
        
        self.figure = plt.figure(figsize=(7, 6))
        self.ax_3d = self.figure.add_subplot(111, projection='3d')
        
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # 🚨 3D 뷰어 창에도 툴바 추가
        self.toolbar = NavigationToolbar(self.canvas, self)
        
        main_layout = QVBoxLayout()
        main_layout.addWidget(self.toolbar) # 툴바를 캔버스 위에 추가
        main_layout.addWidget(self.canvas)
        
        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

    def plot_3d(self, X, Y, Z):
        """
        주어진 X, Y, Z 데이터를 사용하여 3D 경로를 그립니다.
        """
        self.ax_3d.clear()
        
        # 3D 플롯
        self.ax_3d.plot(X, Y, Z, label='End-Effector Path', linewidth=2, marker='.', markersize=3)
        
        # 시작점과 끝점 마커
        self.ax_3d.scatter(X.iloc[0], Y.iloc[0], Z.iloc[0], 
                            color='green', marker='o', s=100, label='Start')
        self.ax_3d.scatter(X.iloc[-1], Y.iloc[-1], Z.iloc[-1], 
                            color='red', marker='x', s=100, label='End')
        
        self.ax_3d.set_xlabel('End_X(m)')
        self.ax_3d.set_ylabel('End_Y(m)')
        self.ax_3d.set_zlabel('End_Z(m)')
        self.ax_3d.set_title("Robot End-Effector 3D Path")
        self.ax_3d.legend()
        
        self.figure.tight_layout()
        self.canvas.draw()
        
# -----------------
# 메인 뷰어 창 클래스 수정: 2D Axes 2개 + 툴바 추가
# -----------------
class LogViewerWindow(QMainWindow):
    # FSM 상태별 배경색 정의
    STATE_COLORS = {
        'OPERATING': '#4CAF5030',   # 연한 초록색 (실행)
        'IDLE': '#9E9E9E30',        # 연한 회색 (대기)
        'MAINTENANCE': '#FF980030', # 연한 주황색 (정비)
        'EMERGENCY_STOP': '#F4433630' # 연한 빨간색 (긴급 정지)
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("ROS2 로봇 상태 로그 분석 (2D View)")
        self.setGeometry(100, 100, 1000, 700)
        
        self.plot_3d_window = Plot3DWindow(self)
        self.loaded_data = None 

        # Matplotlib Figure 및 Axes 초기화
        self.figure = plt.figure(figsize=(10, 8))
        self.ax_joint = self.figure.add_subplot(2, 1, 1) 
        self.ax_pos_2d = self.figure.add_subplot(2, 1, 2) 
        
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # 🚨 핵심 수정: 2D 그래프용 툴바 추가
        # NavigationToolbar2QT를 사용하여 캔버스에 줌, 이동, 저장 등의 기능을 제공합니다.
        self.toolbar = NavigationToolbar(self.canvas, self)
        
        # UI 구성 요소
        self.load_button = QPushButton("CSV 파일 불러오기")
        self.load_button.clicked.connect(self.load_csv_dialog)
        
        self.view_3d_button = QPushButton("3D 경로 보기")
        self.view_3d_button.setEnabled(False) 
        self.view_3d_button.clicked.connect(self.show_3d_viewer)
        
        self.status_label = QLabel("상태: CSV 파일을 로드하십시오.")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        # 레이아웃 설정
        control_layout = QHBoxLayout()
        control_layout.addWidget(self.load_button)
        control_layout.addWidget(self.view_3d_button) 
        control_layout.addWidget(self.status_label)
        
        main_layout = QVBoxLayout()
        # 🚨 툴바를 메인 레이아웃의 상단에 추가
        main_layout.addWidget(self.toolbar)
        main_layout.addLayout(control_layout)
        main_layout.addWidget(self.canvas)

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)
        
    @Slot()
    def load_csv_dialog(self):
        """파일 대화 상자를 열어 CSV 파일을 선택하고 로드합니다."""
        file_name, _ = QFileDialog.getOpenFileName(
            self, 
            "로그 파일 선택", 
            os.path.expanduser("~"),
            "CSV Files (*.csv)"
        )
        if file_name:
            self.plot_data(file_name)

    @Slot()
    def show_3d_viewer(self):
        """3D 뷰어 창을 표시하고 데이터를 플롯합니다."""
        if self.loaded_data is not None:
            df = self.loaded_data
            pos_cols = ['End_X(m)', 'End_Y(m)', 'End_Z(m)']
            
            if not all(col in df.columns for col in pos_cols):
                self.status_label.setText("🚨 오류 발생: 3D 플롯에 필요한 X, Y, Z 컬럼이 없습니다.")
                return
                
            X = df[pos_cols[0]]
            Y = df[pos_cols[1]]
            Z = df[pos_cols[2]]
            
            # 3D 뷰어 창을 띄우기 전에 3D 플롯을 수행합니다.
            self.plot_3d_window.plot_3d(X, Y, Z)
            self.plot_3d_window.show() 
        else:
            self.status_label.setText("🚨 오류 발생: 로드된 데이터가 없습니다.")


    def plot_data(self, file_path):
        """CSV 파일을 읽고 FSM 상태 색상 배경과 2D 그래프 (Joint/Pos 분리)를 그립니다."""
        try:
            # 1. 데이터 로드 
            df = pd.read_csv(file_path)
            self.loaded_data = df 
            self.status_label.setText(f"상태: {os.path.basename(file_path)} 로드 완료. 그래프 업데이트 중...")
            self.view_3d_button.setEnabled(True) 
            
            time_col = 'Time(s)'
            if time_col not in df.columns:
                 raise ValueError(f"CSV 파일에 '{time_col}' 컬럼이 없습니다.")
            
            # 2. Axes 초기화
            self.ax_joint.clear()
            self.ax_pos_2d.clear()
            
            # 3. FSM 상태 배경색 하이라이팅 (두 그래프 모두에 적용)
            self._highlight_fsm_states(df, [self.ax_joint, self.ax_pos_2d], time_col)
            
            # 4. 데이터 플롯
            
            # 4-1. Joint Angles (상단 그래프 - ax_joint)
            joint_cols = [f'Joint{i}' for i in range(1, 7)]
            df.plot(y=joint_cols, x=time_col, ax=self.ax_joint, legend=False)
            
            self.ax_joint.set_ylabel("Joint Angles (rad)")
            self.ax_joint.set_xlabel("") 
            self.ax_joint.grid(True, linestyle='--')
            self.ax_joint.legend(loc='upper right', ncol=3, title="Joints") 
            self.ax_joint.set_title("Robot Joint Angles Over Time")

            # 4-2. End Position (하단 그래프 - ax_pos_2d)
            pos_cols = ['End_X(m)', 'End_Y(m)', 'End_Z(m)']
            
            df.plot(y=pos_cols, x=time_col, ax=self.ax_pos_2d, legend=False)
            
            self.ax_pos_2d.set_ylabel("End-Effector Position (m)")
            self.ax_pos_2d.set_xlabel(time_col)
            self.ax_pos_2d.grid(True, linestyle='--')
            self.ax_pos_2d.legend(loc='upper right', ncol=3, title="End Position") 
            self.ax_pos_2d.set_title("Robot End-Effector X, Y, Z Position Over Time")


            # 5. Canvas 갱신
            self.figure.tight_layout()
            self.canvas.draw()
            self.status_label.setText(f"상태: 2D 그래프 업데이트 완료. '3D 경로 보기' 버튼 활성화. (File: {os.path.basename(file_path)})")
            
        except Exception as e:
            self.loaded_data = None
            self.view_3d_button.setEnabled(False)
            self.status_label.setText(f"🚨 오류 발생: 데이터를 로드하거나 플롯할 수 없습니다. ({e})")
            print(f"Plotting Error: {e}")

    def _highlight_fsm_states(self, df, axs, time_col):
        """FSM 상태 변화에 따라 그래프 배경에 색상을 칠합니다."""
        
        state_col = 'FSM_State'
        
        if df.empty or state_col not in df.columns or df[time_col].empty:
            return

        state_changes = df[state_col].ne(df[state_col].shift()).fillna(True)
        change_indices = state_changes[state_changes].index.tolist()
        
        end_time_of_data = df[time_col].iloc[-1]
        
        for i in range(len(change_indices)):
            start_index = change_indices[i]
            
            start_time = df[time_col].iloc[start_index]
            
            raw_state = df[state_col].iloc[start_index]
            current_state = str(raw_state).split('.')[-1]
            
            # 종료 시간 설정
            if i + 1 < len(change_indices):
                end_index = change_indices[i+1]
                stop_time = df[time_col].iloc[end_index]
            else:
                stop_time = end_time_of_data

            color = self.STATE_COLORS.get(current_state, '#80808030') 

            # axvspan을 사용하여 두 Axes 모두에 수직 배경색 칠하기
            for ax in axs:
                ax.axvspan(start_time, stop_time, color=color, alpha=1.0, zorder=-1)