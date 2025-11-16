# src/log_viewer.py
import pandas as pd
import os
import matplotlib.pyplot as plt
# Matplotlib 3D 플롯을 위한 모듈 임포트
from mpl_toolkits.mplot3d import Axes3D 
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QPushButton, QFileDialog, QSizePolicy, QLabel, QScrollArea
)
from PySide6.QtCore import Slot, Qt, QTimer

# 3D 플롯을 위해 matplotlib 기본 설정을 건드리지 않도록 합니다.

class LogViewerWindow(QMainWindow):
    # 🚨 FSM 상태별 배경색 정의
    STATE_COLORS = {
        'OPERATING': '#4CAF5030',   # 연한 초록색 (실행)
        'IDLE': '#9E9E9E30',        # 연한 회색 (대기)
        'MAINTENANCE': '#FF980030', # 연한 주황색 (정비)
        'EMERGENCY_STOP': '#F4433630' # 연한 빨간색 (긴급 정지)
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("ROS2 로봇 상태 로그 분석")
        self.setGeometry(100, 100, 1000, 700)

        # Matplotlib Figure 및 Axes 초기화
        # 1x2 그리드로 Axes를 생성합니다.
        # axs[0]: Joint Angles (2D)
        # axs[1]: End-Effector Position (3D)
        self.figure = plt.figure(figsize=(10, 8))
        self.axs = [
            self.figure.add_subplot(2, 1, 1), # 2D Axes
            self.figure.add_subplot(2, 1, 2, projection='3d') # 3D Axes
        ]
        
        self.canvas = FigureCanvas(self.figure)
        
        # UI 구성 요소
        self.load_button = QPushButton("CSV 파일 불러오기 (경로 설정)")
        self.load_button.clicked.connect(self.load_csv_dialog)
        
        self.status_label = QLabel("상태: CSV 파일을 로드하십시오.")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        # 레이아웃 설정
        control_layout = QHBoxLayout()
        control_layout.addWidget(self.load_button)
        control_layout.addWidget(self.status_label)
        
        main_layout = QVBoxLayout()
        main_layout.addLayout(control_layout)
        main_layout.addWidget(self.canvas)

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)
        
    @Slot()
    def load_csv_dialog(self):
        """파일 대화 상자를 열어 CSV 파일을 선택하고 로드합니다."""
        # 🚨 사용자 로컬 환경에 맞게 경로를 설정하세요.
        file_name, _ = QFileDialog.getOpenFileName(
            self, 
            "로그 파일 선택", 
            os.path.expanduser("~"),
            "CSV Files (*.csv)"
        )
        if file_name:
            self.plot_data(file_name)

    def plot_data(self, file_path):
        """CSV 파일을 읽고 FSM 상태 색상 배경과 3D 경로 그래프를 그립니다."""
        try:
            # 1. 데이터 로드 (헤더는 사용자 정의와 일치해야 함)
            df = pd.read_csv(file_path)
            self.status_label.setText(f"상태: {os.path.basename(file_path)} 로드 완료. 그래프 업데이트 중...")
            
            time_col = 'Time(s)'
            if time_col not in df.columns:
                 raise ValueError(f"CSV 파일에 '{time_col}' 컬럼이 없습니다.")
            
            # 2. Axes 초기화
            # 2D Axes 초기화
            self.axs[0].clear()
            # 3D Axes 초기화
            self.figure.delaxes(self.axs[1]) # 기존 3D Axes 삭제
            self.axs[1] = self.figure.add_subplot(2, 1, 2, projection='3d') # 새 3D Axes 생성
            
            # 3. FSM 상태 배경색 하이라이팅 (2D Joint Angle 그래프에만 적용)
            self._highlight_fsm_states(df, [self.axs[0]], time_col)
            
            # 4. 데이터 플롯
            
            # 4-1. Joint Angles (상단 그래프 - 2D)
            joint_cols = [f'Joint{i}' for i in range(1, 7)]
            df.plot(y=joint_cols, x=time_col, ax=self.axs[0], legend=False)
            self.axs[0].set_ylabel("Joint Angles (rad)")
            self.axs[0].grid(True, linestyle='--')
            self.axs[0].legend(loc='upper right', ncol=3)
            self.axs[0].set_title("Robot Joint Angles Over Time")

            # 4-2. End Position (하단 그래프 - 3D)
            pos_cols = ['End_X(m)', 'End_Y(m)', 'End_Z(m)']
            
            X = df[pos_cols[0]]
            Y = df[pos_cols[1]]
            Z = df[pos_cols[2]]
            
            # 3D 플롯
            self.axs[1].plot(X, Y, Z, label='End-Effector Path', linewidth=2)
            
            # 시작점과 끝점 마커
            self.axs[1].scatter(X.iloc[0], Y.iloc[0], Z.iloc[0], 
                                color='green', marker='o', s=50, label='Start')
            self.axs[1].scatter(X.iloc[-1], Y.iloc[-1], Z.iloc[-1], 
                                color='red', marker='x', s=50, label='End')
            
            self.axs[1].set_xlabel(pos_cols[0])
            self.axs[1].set_ylabel(pos_cols[1])
            self.axs[1].set_zlabel(pos_cols[2])
            self.axs[1].set_title("Robot End-Effector 3D Path")
            self.axs[1].legend()

            # 5. Canvas 갱신
            self.figure.tight_layout()
            self.canvas.draw()
            self.status_label.setText(f"상태: 그래프 업데이트 완료. (File: {os.path.basename(file_path)})")
            
        except Exception as e:
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
            
            # 상태 이름 추출
            raw_state = df[state_col].iloc[start_index]
            current_state = str(raw_state).split('.')[-1]
            
            # 종료 시간 설정
            if i + 1 < len(change_indices):
                end_index = change_indices[i+1]
                stop_time = df[time_col].iloc[end_index]
            else:
                stop_time = end_time_of_data

            color = self.STATE_COLORS.get(current_state, '#80808030') 

            # axvspan을 사용하여 2D 그래프(axs)에만 수직 배경색 칠하기
            for ax in axs:
                # 3D axes는 axvspan을 지원하지 않으므로, 2D axes만 처리해야 함.
                # 현재 axs 리스트에 2D axes만 전달되므로 문제가 없음
                ax.axvspan(start_time, stop_time, color=color, alpha=1.0, zorder=-1)