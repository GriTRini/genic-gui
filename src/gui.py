import sys
import os
import time
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QVBoxLayout, QHBoxLayout, 
    QWidget, QLabel, QSpacerItem, QSizePolicy, QMessageBox, QScrollArea,
    QStackedWidget
)
from PySide6.QtCore import (
    Slot, Qt, QSize, QTimer
)
from PySide6.QtGui import QCloseEvent, QIcon, QFont

# [사용자 정의 모듈 Import]
from src.ros2_node import RclpyThread 
from src.data_logger import DataLogger
from src.log_viewer import LogViewerWindow 

class MainWindow(QMainWindow):
    BASE_HEIGHT = 450
    BASE_FONT_SIZE = 12

    # 버튼 상태 정의 상수
    STATE_ACTION_RUN = "ACTION_RUN"       
    STATE_MAINTENANCE = "MAINTENANCE"     
    STATE_EMERGENCY = "EMERGENCY"         
    STATE_IDLE = "IDLE"                   

    def __init__(self):
        super().__init__()
        self.setWindowTitle("DSRPY 로봇 제어 시스템")
        
        # [주의] __init__에서 showFullScreen()을 바로 호출하지 않고, UI 생성 후 마지막에 호출합니다.

        self.current_action_state = self.STATE_IDLE 
        self.current_fsm_state_text = "N/A"
        self.next_cmd_value = 31 
        
        self.is_manual_mode = False
        self.last_data_time = 0.0  
        
        # 연결 상태 체크 타이머
        self.connection_check_timer = QTimer(self)
        self.connection_check_timer.timeout.connect(self._check_connection_status)
        self.connection_check_timer.start(1000) 

        # 긴급 정지 깜빡임 타이머
        self.emergency_timer = QTimer(self)
        self.emergency_timer.timeout.connect(self._toggle_emergency_style) 
        self.emergency_blink_on = False

        self.log_viewer = None 
        
        if QIcon.hasThemeIcon("robot"):
            self.setWindowIcon(QIcon.fromTheme("robot"))
            
        self.ros_thread = RclpyThread()
        self.data_logger = DataLogger() 
        self.ros_thread.start()
        
        self.ros_thread.fsm_state_updated.connect(self.update_fsm_state)
        self.ros_thread.pose_updated.connect(self.update_end_pose) 
        self.ros_thread.joint_angles_updated.connect(self.update_joint_angles)
        
        self.central_stack = QStackedWidget()
        self.setCentralWidget(self.central_stack)

        # 1. 사용자 모드 UI 생성
        self.user_mode_widget = QWidget()
        self._setup_user_mode_ui()
        self.central_stack.addWidget(self.user_mode_widget)

        # 2. 관리자 모드 UI 생성 (버튼 객체들이 여기서 생성됨)
        self.admin_mode_widget = QWidget()
        self._setup_admin_mode_ui() 
        self.central_stack.addWidget(self.admin_mode_widget)

        self.central_stack.setCurrentIndex(0)
        
        # UI 생성 완료 후 전체 화면 적용
        self.showFullScreen()
        
        # 초기 스타일 적용
        self._apply_dynamic_style(self.height())
        self.statusBar().showMessage("시스템 준비 완료", 5000)

    # ============================================================
    # 연결 상태 확인 로직
    # ============================================================
    def _check_connection_status(self):
        current_time = time.time()
        time_diff = current_time - self.last_data_time
        
        if self.last_data_time == 0.0 or time_diff > 3.0:
            self.lbl_connection_status.setText("NOT CONNECTED")
            self.lbl_connection_status.setStyleSheet("background-color: #F44336; color: white; border-radius: 10px; padding: 10px; font-weight: bold; font-size: 16px;")
        else:
            self.lbl_connection_status.setText("CONNECTED")
            self.lbl_connection_status.setStyleSheet("background-color: #4CAF50; color: white; border-radius: 10px; padding: 10px; font-weight: bold; font-size: 16px;")

    # ============================================================
    # 사용자 모드 UI 설정 (종료 버튼 추가됨)
    # ============================================================
    def _setup_user_mode_ui(self):
        layout = QVBoxLayout(self.user_mode_widget)
        layout.setContentsMargins(30, 30, 30, 30)
        layout.setSpacing(20)
        
        # 1. 상단 영역
        top_layout = QHBoxLayout()
        self.lbl_connection_status = QLabel("WAITING...")
        self.lbl_connection_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_connection_status.setFixedSize(200, 60)
        self.lbl_connection_status.setStyleSheet("background-color: #9E9E9E; color: white; border-radius: 10px; padding: 10px; font-weight: bold;")
        top_layout.addWidget(self.lbl_connection_status)
        
        top_layout.addStretch() 
        
        self.btn_user_manual = QPushButton("수동 모드 OFF")
        self.btn_user_manual.setIcon(QIcon.fromTheme("input-mouse"))
        self.btn_user_manual.setCheckable(True)
        self.btn_user_manual.setFixedSize(180, 60)
        self.btn_user_manual.setStyleSheet("""
            QPushButton { font-size: 16px; font-weight: bold; background-color: #607D8B; color: white; border-radius: 10px; } 
            QPushButton:checked { background-color: #FFC107; color: black; border: 3px solid #FF5722; }
        """)
        self.btn_user_manual.clicked.connect(self.on_manual_mode_toggle)
        top_layout.addWidget(self.btn_user_manual)
        
        layout.addLayout(top_layout)

        # 2. 중앙 영역 (실행/정지 버튼 꽉 채우기)
        center_layout = QHBoxLayout()
        self.btn_user_toggle = QPushButton("\n동작 실행\n(RUN)")
        self.btn_user_toggle.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.btn_user_toggle.setIcon(QIcon.fromTheme("media-playback-start"))
        self.btn_user_toggle.setIconSize(QSize(150, 150))
        self.btn_user_toggle.setStyleSheet("""
            QPushButton { background-color: #4CAF50; color: white; border-radius: 40px; font-size: 60px; font-weight: bold; } 
            QPushButton:hover { background-color: #45a049; }
        """)
        self.btn_user_toggle.clicked.connect(self.on_user_toggle_click)
        center_layout.addWidget(self.btn_user_toggle)
        
        layout.addLayout(center_layout, 1) # 비율 1로 남은 공간 차지

        # 3. 하단 영역
        bottom_layout = QHBoxLayout()
        
        # [NEW] 왼쪽 아래: 프로그램 종료 버튼
        self.btn_user_exit = QPushButton(" 프로그램 종료")
        self.btn_user_exit.setIcon(QIcon.fromTheme("application-exit"))
        self.btn_user_exit.setFixedSize(160, 50)
        self.btn_user_exit.setStyleSheet("""
            QPushButton { font-size: 16px; font-weight: bold; background-color: #546E7A; color: white; border-radius: 8px; }
            QPushButton:hover { background-color: #455A64; }
        """)
        self.btn_user_exit.clicked.connect(self.on_exit_button_click)
        bottom_layout.addWidget(self.btn_user_exit)
        
        # 중앙: 수동 모드 상태 표시 (여백으로 중앙 정렬)
        bottom_layout.addStretch()
        self.lbl_manual_indicator = QLabel("")
        self.lbl_manual_indicator.setStyleSheet("color: #FF5722; font-size: 20px; font-weight: bold;")
        bottom_layout.addWidget(self.lbl_manual_indicator)
        bottom_layout.addStretch()
        
        # 오른쪽 아래: 관리자 모드 버튼
        self.btn_go_admin = QPushButton(" 관리자 모드")
        self.btn_go_admin.setIcon(QIcon.fromTheme("preferences-system"))
        self.btn_go_admin.setFixedSize(160, 50)
        self.btn_go_admin.setStyleSheet("""
            QPushButton { font-size: 16px; font-weight: bold; background-color: #333; color: white; border-radius: 8px; }
            QPushButton:hover { background-color: #000; }
        """)
        self.btn_go_admin.clicked.connect(self.switch_to_admin)
        bottom_layout.addWidget(self.btn_go_admin)

        layout.addLayout(bottom_layout)

    # ============================================================
    # 로직 메서드들
    # ============================================================
    @Slot()
    def on_manual_mode_toggle(self):
        self.is_manual_mode = self.btn_user_manual.isChecked()
        if self.is_manual_mode:
            self.on_publish_command('i', "수동 모드 진입")
            self.btn_user_manual.setText("수동 모드 ON")
            self.lbl_manual_indicator.setText("⚠️ 현재 수동 제어 모드입니다.")
            if self.current_action_state == self.STATE_ACTION_RUN:
                self.current_action_state = self.STATE_IDLE
                self.on_publish_command('s', "수동 전환에 따른 정지")
        else:
            self.btn_user_manual.setText("수동 모드 OFF")
            self.lbl_manual_indicator.setText("")
        self._update_button_ui()

    @Slot()
    def on_user_toggle_click(self):
        if self.is_manual_mode:
            self.statusBar().showMessage("⚠️ 수동 모드 중입니다. 자동 실행이 불가능합니다.", 2000)
            return
        if self.current_action_state == self.STATE_EMERGENCY or self.current_action_state == self.STATE_MAINTENANCE:
            self.statusBar().showMessage(f"🔴 현재 {self.current_action_state} 상태입니다.", 3000)
            return
        if self.current_action_state == self.STATE_ACTION_RUN:
            self.current_action_state = self.STATE_IDLE
            self.on_publish_command('s', "동작 정지")
        else:
            self.current_action_state = self.STATE_ACTION_RUN
            self.on_publish_command('t', "동작 실행")
        self._update_button_ui()

    @Slot()
    def switch_to_admin(self):
        reply = QMessageBox.question(self, '관리자 인증', "관리자 모드로 진입하시겠습니까?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.central_stack.setCurrentIndex(1)
            self.statusBar().showMessage("관리자 모드로 전환되었습니다.", 3000)
            self._update_button_ui()

    @Slot()
    def switch_to_user(self):
        self.central_stack.setCurrentIndex(0)
        self.statusBar().showMessage("사용자 모드로 전환되었습니다.", 3000)
        self._update_button_ui()

    def _update_button_ui(self):
        # 수동 모드 UI 처리
        if self.is_manual_mode:
            self.btn_user_toggle.setEnabled(False)
            self.btn_user_toggle.setText("\n수동 모드\n(MANUAL)")
            self.btn_user_toggle.setStyleSheet("""
                QPushButton { background-color: #555555; color: #AAAAAA; border-radius: 40px; font-size: 50px; font-weight: bold; border: 4px dashed #FF5722; }
            """)
            self.btn_user_toggle.setIcon(QIcon.fromTheme("input-mouse"))
            
            # 관리자 버튼 비활성
            if hasattr(self, 'btn_t'): self.btn_t.setEnabled(False)
            return

        # 일반 모드 UI 처리
        self.btn_user_toggle.setEnabled(True)
        
        # 관리자 버튼 복구
        if hasattr(self, 'btn_t'):
            super_btns = [self.btn_t, self.btn_i, self.btn_m, self.btn_e]
            for btn in super_btns: btn.setEnabled(True)

            if self.current_action_state == self.STATE_ACTION_RUN:
                self.btn_t.setText("정지")
                self.btn_t.setStyleSheet("background-color: orange; color: white; font-size: 14pt; font-weight: bold;")
                self.btn_t.setIcon(QIcon.fromTheme("media-playback-pause"))
                
                self.btn_user_toggle.setText("\n동작 정지\n(STOP)")
                self.btn_user_toggle.setIcon(QIcon.fromTheme("media-playback-pause"))
                self.btn_user_toggle.setStyleSheet("""
                    QPushButton { background-color: #FF9800; color: white; border-radius: 40px; font-size: 60px; font-weight: bold; } 
                    QPushButton:hover { background-color: #F57C00; }
                """)
            elif self.current_action_state == self.STATE_IDLE:
                self.btn_t.setText("동작 실행")
                self.btn_t.setStyleSheet(f"background-color: {self.btn_t.property('color')}; color: white; font-size: 14pt; font-weight: bold;")
                self.btn_t.setIcon(QIcon.fromTheme("media-playback-start"))
                
                self.btn_user_toggle.setText("\n동작 실행\n(RUN)")
                self.btn_user_toggle.setIcon(QIcon.fromTheme("media-playback-start"))
                self.btn_user_toggle.setStyleSheet("""
                    QPushButton { background-color: #4CAF50; color: white; border-radius: 40px; font-size: 60px; font-weight: bold; } 
                    QPushButton:hover { background-color: #45a049; }
                """)

            if self.current_action_state == self.STATE_EMERGENCY:
                self.btn_e.setText("초기화")
                self._toggle_emergency_style()
                self.btn_t.setEnabled(False); self.btn_i.setEnabled(False); self.btn_m.setEnabled(False)
                
                self.btn_user_toggle.setEnabled(False)
                self.btn_user_toggle.setText("\n긴급 정지\n(EMERGENCY)")
                self.btn_user_toggle.setStyleSheet("background-color: #F44336; color: white; border-radius: 40px; font-size: 50px;")
                self.btn_user_manual.setEnabled(False)
            elif self.current_action_state == self.STATE_MAINTENANCE:
                self.btn_m.setText("대기 모드"); self.btn_m.setStyleSheet("background-color: orange;")
                self.btn_t.setEnabled(False); self.btn_i.setEnabled(False)
                
                self.btn_user_toggle.setEnabled(False)
                self.btn_user_toggle.setText("\n정비 모드\n(MAINTENANCE)")
                self.btn_user_toggle.setStyleSheet("background-color: #2196F3; color: white; border-radius: 40px; font-size: 50px;")
                self.btn_user_manual.setEnabled(False)

    def _setup_admin_mode_ui(self):
        main_h_layout = QHBoxLayout(self.admin_mode_widget)
        self.status_panel_layout = self._setup_status_panel()
        main_h_layout.addLayout(self.status_panel_layout, 2) 
        self.command_panel_layout = self._setup_command_panel()
        main_h_layout.addLayout(self.command_panel_layout, 1) 

    def _setup_status_panel(self):
        v_layout = QVBoxLayout()
        v_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.title_status = QLabel("--- 🤖 로봇 실시간 상태 ---")
        v_layout.addWidget(self.title_status)
        self.status_label = QLabel("FSM State: N/A")
        v_layout.addWidget(self.status_label)
        self.pose_label = QLabel("End Position (X, Y, Z):\n X: N/A\n Y: N/A\n Z: N/A")
        self.pose_label.setStyleSheet("padding: 5px; margin-top: 10px; border: 1px solid #CCC;")
        v_layout.addWidget(self.pose_label)
        joint_scroll_area = QScrollArea()
        joint_scroll_area.setWidgetResizable(True)
        joint_container = QWidget() 
        self.joint_v_layout = QVBoxLayout(joint_container)
        self.joint_v_layout.setContentsMargins(0, 0, 0, 0)
        num_joints = 6
        self.joint_labels = [QLabel(f"Joint {i+1} Angle: N/A") for i in range(num_joints)]
        for label in self.joint_labels: self.joint_v_layout.addWidget(label)
        joint_scroll_area.setWidget(joint_container)
        self.title_joint = QLabel("\n--- 관절 각도 (Joint Angles) ---")
        v_layout.addWidget(self.title_joint)
        v_layout.addWidget(joint_scroll_area)
        v_layout.addItem(QSpacerItem(20, 20, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding))
        return v_layout

    def _setup_command_panel(self):
        v_layout = QVBoxLayout()
        v_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        top_bar = QHBoxLayout()
        self.btn_back_user = QPushButton(" 사용자 모드 복귀")
        self.btn_back_user.setIcon(QIcon.fromTheme("go-home"))
        self.btn_back_user.setStyleSheet("background-color: #009688; color: white; padding: 5px; font-weight: bold;")
        self.btn_back_user.clicked.connect(self.switch_to_user)
        top_bar.addWidget(self.btn_back_user)
        v_layout.addLayout(top_bar)
        
        self.title_command = QLabel(" 로봇 제어 명령 ")
        self.title_command.setAlignment(Qt.AlignmentFlag.AlignCenter) 
        v_layout.addWidget(self.title_command)
        
        self.btn_i = self._create_command_button("대기 모드", 'i', "#607D8B", icon_name="process-stop", is_toggle=False)
        self.btn_i.clicked.connect(lambda: self.on_publish_command('i', "대기 모드"))
        v_layout.addWidget(self.btn_i)

        self.btn_t = self._create_command_button("동작 실행", 't', "#4CAF50", icon_name="media-playback-start", is_toggle=True)
        self.btn_t.clicked.connect(self.on_run_toggle) 
        v_layout.addWidget(self.btn_t)
        
        self.btn_m = self._create_command_button("정비 모드", 'm', "#2196F3", icon_name="preferences-system", is_toggle=True)
        self.btn_m.clicked.connect(lambda: self.on_maintenance_toggle('m', "정비 모드"))
        v_layout.addWidget(self.btn_m)

        self.btn_e = self._create_command_button("긴급 정지", 'e', "#F44336", is_bold=True, icon_name="media-playback-stop", is_toggle=True)
        self.btn_e.clicked.connect(lambda: self.on_emergency_toggle('e', "긴급 정지"))
        v_layout.addWidget(self.btn_e)
        
        self.btn_test_toggle = self._create_command_button(f"Test CMD ({self.next_cmd_value})", 'TOGGLE', "#9C27B0", color_text="white", icon_name="input-gaming", is_toggle=False, height_ratio=1.0)
        self.btn_test_toggle.clicked.connect(self.send_ethercat_toggle_cmd)
        v_layout.addWidget(self.btn_test_toggle)
        v_layout.addItem(QSpacerItem(20, 20, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding))
        
        self.btn_log = self._create_command_button("📊 로그 분석", 'L', "#800080", color_text="white", icon_name="document-open", is_toggle=False)
        self.btn_log.clicked.connect(self.show_log_viewer)
        v_layout.addWidget(self.btn_log)

        self.btn_fullscreen = self._create_command_button("전체 화면 토글", 'F', "#FFC107", color_text="black", icon_name="view-fullscreen", is_toggle=False)
        self.btn_fullscreen.clicked.connect(self.toggle_fullscreen)
        v_layout.addWidget(self.btn_fullscreen)

        self.btn_exit = self._create_command_button("프로그램 종료", 'X', "#607D8B", is_bold=True, icon_name="application-exit", is_toggle=False)
        self.btn_exit.clicked.connect(self.on_exit_button_click) 
        v_layout.addWidget(self.btn_exit)
        return v_layout

    def _create_command_button(self, text, command_char, color, is_bold=False, icon_name=None, color_text="white", is_toggle=True, height_ratio=1.2):
        btn = QPushButton(text)
        btn.setProperty("is_command", True)
        btn.setProperty("command_char", command_char)
        btn.setProperty("color", color)
        btn.setProperty("color_text", color_text)
        btn.setProperty("is_bold", is_bold)
        btn.setProperty("icon_name", icon_name)
        btn.setProperty("is_toggle", is_toggle)
        btn.setProperty("height_ratio", height_ratio)
        if icon_name: btn.setIcon(QIcon.fromTheme(icon_name)) 
        return btn

    @Slot()
    def show_log_viewer(self):
        if self.log_viewer is None: self.log_viewer = LogViewerWindow(self)
        self.log_viewer.show()

    @Slot()
    def _toggle_emergency_style(self):
        if self.current_action_state != self.STATE_EMERGENCY:
            self.emergency_timer.stop()
            return
        if self.emergency_blink_on:
             self.btn_e.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; font-size: 14pt;")
        else:
             self.btn_e.setStyleSheet("background-color: #2E8B57; color: white; font-weight: bold; font-size: 14pt;")
        self.emergency_blink_on = not self.emergency_blink_on

    def _apply_dynamic_style(self, current_height):
        # Admin UI가 생성되었는지 확인
        if not hasattr(self, 'btn_t'): return

        ratio = max(1.0, current_height / self.BASE_HEIGHT)
        all_btns = [
            self.btn_t, self.btn_i, self.btn_m, self.btn_e, 
            self.btn_fullscreen, self.btn_exit, self.btn_log,
            self.btn_test_toggle 
        ]
        self._update_button_ui()

    @Slot()
    def on_run_toggle(self):
        if self.current_action_state == self.STATE_EMERGENCY or self.current_action_state == self.STATE_MAINTENANCE:
            self.statusBar().showMessage(f"🔴 현재 {self.current_action_state} 상태입니다.", 3000)
            return
        if self.current_action_state == self.STATE_ACTION_RUN:
            self.current_action_state = self.STATE_IDLE
            self.on_publish_command('s', "동작 정지")
        else:
            self.current_action_state = self.STATE_ACTION_RUN
            self.on_publish_command('t', "동작 실행")
        self._update_button_ui()

    @Slot(str, str)
    def on_maintenance_toggle(self, command_char, command_name):
        if self.current_action_state == self.STATE_EMERGENCY: return
        if self.current_action_state == self.STATE_MAINTENANCE:
            self.current_action_state = self.STATE_IDLE
            self.on_publish_command('i', "정비 모드 복귀") 
            self._update_button_ui()
        elif command_char == 'm':
            msg = QMessageBox.question(self, '정비 모드', "원점으로 이동합니다. 진행하시겠습니까?", QMessageBox.Yes | QMessageBox.No)
            if msg == QMessageBox.Yes:
                self.current_action_state = self.STATE_MAINTENANCE
                self.on_publish_command('m', command_name)
                self._update_button_ui()

    @Slot(str, str)
    def on_emergency_toggle(self, command_char, command_name):
        if self.current_action_state == self.STATE_EMERGENCY:
            self.current_action_state = self.STATE_IDLE
            self.on_publish_command('r', "긴급 정지 취소")
            self.emergency_timer.stop()
        else:
            self.current_action_state = self.STATE_EMERGENCY
            self.on_publish_command('e', command_name)
            self.emergency_timer.start(1000)
        self._update_button_ui()

    def resizeEvent(self, event):
        self._apply_dynamic_style(event.size().height())
        super().resizeEvent(event)
        
    @Slot()
    def toggle_fullscreen(self):
        if self.isFullScreen(): self.showNormal()
        else: self.showFullScreen()

    @Slot(float, float, float)
    def update_end_pose(self, x, y, z):
        self.last_data_time = time.time()
        self.pose_label.setText(f"End Position (X, Y, Z):\n X: {x:.4f}\n Y: {y:.4f}\n Z: {z:.4f}")

    @Slot(list)
    def update_joint_angles(self, angles):
        self.last_data_time = time.time()
        curr_cnt = len(self.joint_labels)
        if len(angles) > curr_cnt:
             for i in range(curr_cnt, len(angles)):
                 l = QLabel("N/A"); self.joint_v_layout.addWidget(l); self.joint_labels.append(l)
        for i, a in enumerate(angles):
            if i < len(self.joint_labels): self.joint_labels[i].setText(f"Joint {i+1}: {a:.4f} deg"); self.joint_labels[i].show()

    @Slot(str)
    def update_fsm_state(self, status_text):
        self.current_fsm_state_text = status_text
        self.status_label.setText(f"FSM {status_text}")

    @Slot(str, str)
    def on_publish_command(self, char, name):
        pub = self.ros_thread.get_publisher()
        if pub and self.ros_thread.running:
            pub.publish_command(char)
            self.statusBar().showMessage(f"'{name}' ({char}) 발행됨", 2000)

    @Slot()
    def send_ethercat_toggle_cmd(self):
        pub = self.ros_thread.get_publisher()
        if pub and self.ros_thread.running:
            pub.publish_ethercat_cmd(self.next_cmd_value)
            self.next_cmd_value = 30 if self.next_cmd_value == 31 else 31
            self.btn_test_toggle.setText(f"Test CMD ({self.next_cmd_value})")

    @Slot()
    def on_exit_button_click(self):
        if QMessageBox.question(self, '종료', "종료하시겠습니까?", QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            self.close()

    def closeEvent(self, event: QCloseEvent):
        self.ros_thread.stop()
        self.emergency_timer.stop() 
        self.connection_check_timer.stop() 
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())