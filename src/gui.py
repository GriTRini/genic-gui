import sys
import os
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QVBoxLayout, QHBoxLayout, 
    QWidget, QLabel, QSpacerItem, QSizePolicy, QMessageBox, QScrollArea
)
from PySide6.QtCore import (
    Slot, Qt, QSize, QTimer
)
from PySide6.QtGui import QCloseEvent, QIcon

# [사용자 정의 모듈 Import]
# src 폴더가 main.py와 같은 경로에 있어야 합니다.
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
        self.setWindowTitle("DSRPY 로봇 제어 및 상태 GUI")
        self.setGeometry(100, 100, 800, self.BASE_HEIGHT)

        self.current_action_state = self.STATE_IDLE 
        self.current_fsm_state_text = "N/A"
        
        # [NEW] 토글 상태 저장을 위한 변수 (최초 전송값: 31)
        self.next_cmd_value = 31 
        
        self.emergency_timer = QTimer(self)
        self.emergency_timer.timeout.connect(self._toggle_emergency_style) 
        self.emergency_blink_on = False

        self.log_viewer = None 
        
        if QIcon.hasThemeIcon("robot"):
            self.setWindowIcon(QIcon.fromTheme("robot"))
            
        # 1. ROS2 스레드 시작
        self.ros_thread = RclpyThread()
        self.data_logger = DataLogger() 
        self.ros_thread.start()
        
        # 2. ROS2 Signals 연결
        self.ros_thread.fsm_state_updated.connect(self.update_fsm_state)
        self.ros_thread.pose_updated.connect(self.update_end_pose) 
        self.ros_thread.joint_angles_updated.connect(self.update_joint_angles)
        
        # --- UI 레이아웃 설정 ---
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_h_layout = QHBoxLayout()

        self.status_panel_layout = self._setup_status_panel()
        main_h_layout.addLayout(self.status_panel_layout, 2) 

        self.command_panel_layout = self._setup_command_panel()
        main_h_layout.addLayout(self.command_panel_layout, 1) 

        central_widget.setLayout(main_h_layout)
        
        self._apply_dynamic_style(self.BASE_HEIGHT) 
        self.statusBar().showMessage("ROS2 통신 스레드 시작 및 대기 중...", 5000)
    
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
        for label in self.joint_labels:
             self.joint_v_layout.addWidget(label)

        joint_scroll_area.setWidget(joint_container)
        
        self.title_joint = QLabel("\n--- 관절 각도 (Joint Angles) ---")
        v_layout.addWidget(self.title_joint)
        v_layout.addWidget(joint_scroll_area)
        
        v_layout.addItem(QSpacerItem(20, 20, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding))
        
        return v_layout

    def _setup_command_panel(self):
        v_layout = QVBoxLayout()
        v_layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        self.title_command = QLabel(" 로봇 제어 명령 ")
        self.title_command.setAlignment(Qt.AlignmentFlag.AlignCenter) 
        v_layout.addWidget(self.title_command)
        
        # I: 대기 모드
        self.btn_i = self._create_command_button("대기 모드", 'i', "#607D8B", icon_name="process-stop", is_toggle=False)
        self.btn_i.clicked.connect(lambda: self.on_publish_command('i', "대기 모드"))
        v_layout.addWidget(self.btn_i)

        # T: 동작 실행
        self.btn_t = self._create_command_button("동작 실행", 't', "#4CAF50", icon_name="media-playback-start", is_toggle=True)
        self.btn_t.clicked.connect(self.on_run_toggle) 
        v_layout.addWidget(self.btn_t)
        
        # M: 정비 모드
        self.btn_m = self._create_command_button("정비 모드", 'm', "#2196F3", icon_name="preferences-system", is_toggle=True)
        self.btn_m.clicked.connect(lambda: self.on_maintenance_toggle('m', "정비 모드"))
        v_layout.addWidget(self.btn_m)

        # E: 긴급 정지
        self.btn_e = self._create_command_button("긴급 정지", 'e', "#F44336", is_bold=True, icon_name="media-playback-stop", is_toggle=True)
        self.btn_e.clicked.connect(lambda: self.on_emergency_toggle('e', "긴급 정지"))
        v_layout.addWidget(self.btn_e)
        
        # ============================================================
        # [NEW] EtherCAT Test Button (Toggle 31 <-> 30)
        # ============================================================
        self.btn_test_toggle = self._create_command_button(
            f"Test CMD ({self.next_cmd_value})", # 초기 텍스트: Test CMD (31)
            'TOGGLE', 
            "#9C27B0",      # 보라색 계열
            color_text="white", 
            icon_name="input-gaming", 
            is_toggle=False,
            height_ratio=1.0 
        )
        self.btn_test_toggle.clicked.connect(self.send_ethercat_toggle_cmd)
        v_layout.addWidget(self.btn_test_toggle)
        # ============================================================
        
        # (참고: 연결 버튼이 삭제되었습니다)

        v_layout.addItem(QSpacerItem(20, 20, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding))
        
        # 기타 버튼
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
        
        if icon_name:
            btn.setIcon(QIcon.fromTheme(icon_name)) 
            
        return btn
    
    @Slot()
    def show_log_viewer(self):
        if self.log_viewer is None:
            self.log_viewer = LogViewerWindow(self)
        self.log_viewer.show()

    @Slot()
    def _toggle_emergency_style(self):
        if self.current_action_state != self.STATE_EMERGENCY:
            self.emergency_timer.stop()
            return
            
        current_ratio = self.height() / self.BASE_HEIGHT
        height = int(50 * min(current_ratio, 2.0))
        btn_font_size = self.BASE_FONT_SIZE * 1.2 * min(current_ratio, 2.0)
        
        height_ratio_t = self.btn_t.property("height_ratio") if self.btn_t.property("height_ratio") is not None else 1.2
        height_ratio_m = self.btn_m.property("height_ratio") if self.btn_m.property("height_ratio") is not None else 1.2
        height_ratio_i = self.btn_i.property("height_ratio") if self.btn_i.property("height_ratio") is not None else 1.2
        
        if self.emergency_blink_on:
            style = f"background-color: #4CAF50; color: white; font-weight: bold; height: {height}px; font-size: {btn_font_size}pt;"
        else:
            style = f"background-color: #2E8B57; color: white; font-weight: bold; height: {height}px; font-size: {btn_font_size}pt;"
            
        self.btn_e.setStyleSheet(style)
        self.emergency_blink_on = not self.emergency_blink_on
        
        for btn, ratio_val in [(self.btn_t, height_ratio_t), (self.btn_m, height_ratio_m), (self.btn_i, height_ratio_i)]:
            height = int(50 * min(current_ratio, 2.0))
            btn_font_size = self.BASE_FONT_SIZE * 1.2 * min(current_ratio, 2.0)
            btn.setStyleSheet(f"background-color: #AAAAAA; color: #666666; height: {height}px; font-size: {btn_font_size}pt;")

    def _apply_dynamic_style(self, current_height):
        ratio = max(1.0, current_height / self.BASE_HEIGHT)
        font_size = int(self.BASE_FONT_SIZE * min(ratio, 2.0))
        icon_size = int(30 * min(ratio, 2.0))
        
        widgets = [
            (self.title_status, font_size * 1.5, True),
            (self.title_joint, font_size * 1.5, True),
            (self.status_label, font_size * 1.2, True),
            (self.pose_label, font_size * 1.0, False), 
            (self.title_command, font_size * 1.7, True),
        ]
        
        for widget, size, is_bold in widgets:
            bold_style = "font-weight: bold;" if is_bold else ""
            if widget == self.status_label:
                widget.setStyleSheet(f"font-size: {size}pt; {bold_style}; padding: 5px; background-color: #F0F0F0; border-radius: 5px;")
            elif widget == self.pose_label:
                widget.setStyleSheet(f"font-size: {size}pt; {bold_style}; padding: 5px; border: 1px solid #CCC;")
            else:
                widget.setStyleSheet(f"font-size: {size}pt; {bold_style};")

        for label in self.joint_labels:
            label.setStyleSheet(f"font-size: {font_size}pt;")

        # [수정됨] 연결 버튼 제거됨
        all_btns = [
            self.btn_t, self.btn_i, self.btn_m, self.btn_e, 
            self.btn_fullscreen, self.btn_exit, self.btn_log,
            self.btn_test_toggle 
        ]
        
        for btn in all_btns:
            color = btn.property("color")
            color_text = btn.property("color_text")
            is_bold = btn.property("is_bold")
            height_ratio = btn.property("height_ratio") if btn.property("height_ratio") is not None else 1.2
            
            if btn in [self.btn_t, self.btn_i, self.btn_m, self.btn_e]:
                height_base = 50
                font_base = self.BASE_FONT_SIZE * 1.2
            else:
                height_base = 35
                font_base = self.BASE_FONT_SIZE
                
            height = int(height_base * min(ratio, 2.0) * (height_ratio / 1.2)) 
            btn_font_size = font_base * min(ratio, 2.0)
            
            bold_style = "font-weight: bold;" if is_bold else ""
            
            btn.setStyleSheet(
                f"background-color: {color}; color: {color_text}; height: {height}px; "
                f"font-size: {btn_font_size}pt; {bold_style};"
            )
            btn.setIconSize(QSize(icon_size, icon_size))

        self._update_button_ui() 

    
    @Slot()
    def on_run_toggle(self):
        if self.current_action_state == self.STATE_EMERGENCY or self.current_action_state == self.STATE_MAINTENANCE:
            self.statusBar().showMessage(f"🔴 현재 {self.current_action_state} 상태입니다. 초기화/복귀 버튼을 먼저 누르십시오.", 3000)
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
        # 1. 긴급 정지 상태인 경우: 동작 차단
        if self.current_action_state == self.STATE_EMERGENCY:
            self.statusBar().showMessage("🔴 긴급 정지 상태입니다. 초기화(긴급 정지 취소) 버튼을 먼저 누르십시오.", 3000)
            return

        # 2. 이미 정비 모드인 경우: 모드 해제 (대기 모드로 복귀)
        if self.current_action_state == self.STATE_MAINTENANCE:
            self.current_action_state = self.STATE_IDLE
            self.on_publish_command('i', "정비 모드 복귀") 
            self._update_button_ui()
            
        # 3. 정비 모드로 진입하려는 경우: '큰' 팝업창 확인 후 진행
        elif command_char == 'm' and (self.current_action_state == self.STATE_IDLE or self.current_action_state == self.STATE_ACTION_RUN):
            
            msg_box = QMessageBox(self)
            msg_box.setWindowTitle('정비 모드 진입 확인')
            msg_box.setText("원점으로 이동합니다.\n진행하시겠습니까?")
            msg_box.setIcon(QMessageBox.Question)
            msg_box.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
            msg_box.setDefaultButton(QMessageBox.No)

            msg_box.setStyleSheet("""
                QLabel {
                    min-width: 400px;
                    min-height: 80px;
                    font-size: 16pt;
                }
                QPushButton {
                    width: 120px;
                    height: 40px;
                    font-size: 14pt;
                }
            """)

            ret = msg_box.exec()

            if ret == QMessageBox.Yes:
                self.current_action_state = self.STATE_MAINTENANCE
                self.on_publish_command('m', command_name)
                self._update_button_ui()
            else:
                self.statusBar().showMessage("취소: 정비 모드 진입을 중단했습니다.", 2000)

    @Slot(str, str)
    def on_emergency_toggle(self, command_char, command_name):
        if self.current_action_state == self.STATE_EMERGENCY:
            self.current_action_state = self.STATE_IDLE
            self.on_publish_command('r', "긴급 정지 취소 (초기화)")
            self.emergency_timer.stop()
        else:
            self.current_action_state = self.STATE_EMERGENCY
            self.on_publish_command('e', command_name)
            self.emergency_timer.start(1000)
            
        self._update_button_ui()
        
    def _update_button_ui(self):
        current_height = self.height()
        current_ratio = current_height / self.BASE_HEIGHT
        
        all_command_btns = [self.btn_t, self.btn_i, self.btn_m, self.btn_e]
        
        for btn in all_command_btns:
            color = btn.property("color")
            is_bold = btn.property("is_bold")
            color_text = btn.property("color_text")
            bold_style = "font-weight: bold;" if is_bold else ""
            
            height = int(50 * min(current_ratio, 2.0))
            btn_font_size = self.BASE_FONT_SIZE * 1.2 * min(current_ratio, 2.0)
            
            btn.setStyleSheet(f"background-color: {color}; color: {color_text}; height: {height}px; font-size: {btn_font_size}pt; {bold_style};")
            btn.setEnabled(True)
            
            if btn == self.btn_t:
                btn.setText("동작 실행")
                btn.setIcon(QIcon.fromTheme("media-playback-start"))
            elif btn == self.btn_m:
                btn.setText("정비 모드")
                btn.setIcon(QIcon.fromTheme("preferences-system"))
            elif btn == self.btn_e:
                btn.setText("긴급 정지")
                btn.setIcon(QIcon.fromTheme("media-playback-stop"))
                if self.emergency_timer.isActive():
                    self.emergency_timer.stop() 
        
        if self.current_action_state == self.STATE_EMERGENCY:
            self.btn_e.setText("초기화")
            self.btn_e.setIcon(QIcon.fromTheme("system-run"))
            self._toggle_emergency_style() 
            self.btn_t.setEnabled(False)
            self.btn_i.setEnabled(False)
            self.btn_m.setEnabled(False)
        
        elif self.current_action_state == self.STATE_MAINTENANCE:
            self.btn_m.setText("대기 모드")
            self.btn_m.setStyleSheet(f"background-color: orange; color: white; height: {int(50 * min(current_ratio, 2.0))}px; font-size: {self.BASE_FONT_SIZE * 1.2 * min(current_ratio, 2.0)}pt;")
            self.btn_m.setIcon(QIcon.fromTheme("process-stop"))
            self.btn_t.setEnabled(False)
            self.btn_i.setEnabled(False)
            self.btn_e.setEnabled(True) 
        
        elif self.current_action_state == self.STATE_ACTION_RUN:
            self.btn_t.setText("정지")
            self.btn_t.setStyleSheet(f"background-color: orange; color: white; height: {int(50 * min(current_ratio, 2.0))}px; font-size: {self.BASE_FONT_SIZE * 1.2 * min(current_ratio, 2.0)}pt;")
            self.btn_t.setIcon(QIcon.fromTheme("media-playback-pause"))
        
        icon_size = int(30 * min(current_ratio, 2.0))
        
        # [수정됨] 연결 버튼 제거됨
        all_btns = [
            self.btn_t, self.btn_i, self.btn_m, self.btn_e, 
            self.btn_fullscreen, self.btn_exit, self.btn_log, 
            self.btn_test_toggle
        ]
        
        for btn in all_btns:
            height_ratio = btn.property("height_ratio") if btn.property("height_ratio") is not None else 1.0
            
            if btn in all_command_btns:
                height_base = 50
                font_base = self.BASE_FONT_SIZE * 1.2
            else:
                height_base = 35
                font_base = self.BASE_FONT_SIZE
                    
            height = int(height_base * min(current_ratio, 2.0) * (height_ratio / 1.2)) 
            btn_font_size = font_base * min(current_ratio, 2.0)
            
            if not btn.property("is_toggle") or btn in all_command_btns: 
                bold_style = "font-weight: bold;" if btn.property("is_bold") else ""
                btn.setStyleSheet(f"background-color: {btn.property('color')}; color: {btn.property('color_text')}; height: {height}px; font-size: {btn_font_size}pt; {bold_style};")
            
            btn.setIconSize(QSize(icon_size, icon_size))
            
            # [수정됨] 연결 버튼 관련 UI 업데이트 로직 삭제

    def resizeEvent(self, event):
        current_height = event.size().height()
        self._apply_dynamic_style(current_height)
        super().resizeEvent(event)
        
    @Slot()
    def toggle_fullscreen(self):
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()

    @Slot(float, float, float)
    def update_end_pose(self, x, y, z):
        self.pose_label.setText(f"End Position (X, Y, Z):\n X: {x:.4f}\n Y: {y:.4f}\n Z: {z:.4f}")

    @Slot(list)
    def update_joint_angles(self, angles):
        current_joint_count = len(self.joint_labels)
        
        if len(angles) > current_joint_count and len(angles) <= 20:
             for i in range(current_joint_count, len(angles)):
                 new_label = QLabel(f"Joint {i+1} Angle: N/A")
                 self.joint_v_layout.addWidget(new_label)
                 self.joint_labels.append(new_label)

        for i, angle in enumerate(angles):
            if i < len(self.joint_labels):
                self.joint_labels[i].setText(f"Joint {i+1} Angle: {angle:.4f} deg")
                self.joint_labels[i].show()
            
        for i in range(len(angles), len(self.joint_labels)):
            self.joint_labels[i].hide()
        
        self._update_button_ui()

    @Slot(str)
    def update_fsm_state(self, status_text):
        if not status_text or status_text.strip() == "":
            return

        self.current_fsm_state_text = status_text
        display_text = f"FSM {self.current_fsm_state_text}"
        self.status_label.setText(display_text)
        print(f"[GUI] FSM State Updated & Stored: {self.current_fsm_state_text}")

    @Slot(str, str)
    def on_publish_command(self, command_char, command_name):
        publisher = self.ros_thread.get_publisher()
        if publisher and self.ros_thread.running:
            publisher.publish_command(command_char)
            self.statusBar().showMessage(f"'{command_name}' 명령 ({command_char}) 발행됨", 2000)
        else:
            self.statusBar().showMessage("ROS2 Publisher가 준비되지 않았거나 종료되었습니다.", 2000)

    # =================================================================
    # [NEW] EtherCAT 토글 명령 발행 슬롯 (31 <-> 30)
    # =================================================================
    @Slot()
    def send_ethercat_toggle_cmd(self):
        publisher = self.ros_thread.get_publisher()
        
        if publisher and self.ros_thread.running:
            # 1. 현재 저장된 값을 보냅니다 (처음: 31)
            cmd_to_send = self.next_cmd_value
            publisher.publish_ethercat_cmd(cmd_to_send)
            
            self.statusBar().showMessage(f"EtherCAT 명령 ({cmd_to_send}) 발행됨", 2000)
            
            # 2. 다음 값을 위해 토글 (31이면 30으로, 30이면 31로)
            if self.next_cmd_value == 31:
                self.next_cmd_value = 30
            else:
                self.next_cmd_value = 31
            
            # 3. 버튼 텍스트 업데이트 (사용자에게 다음에 뭐가 나갈지 보여줌)
            self.btn_test_toggle.setText(f"Test CMD ({self.next_cmd_value})")
            
        else:
            self.statusBar().showMessage("ROS2 통신이 연결되지 않았습니다.", 2000)

    @Slot()
    def on_exit_button_click(self):
        reply = QMessageBox.question(self, '프로그램 종료 확인', "ROS2 통신을 종료하고 프로그램을 닫으시겠습니까?", QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        if reply == QMessageBox.StandardButton.Yes:
            self.close()

    def closeEvent(self, event: QCloseEvent):
        print("Stopping ROS2 thread and closing application...")
        self.ros_thread.stop()
        self.emergency_timer.stop() 
        
        # [수정됨] 로봇 노드 프로세스 종료 로직 삭제됨
        
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())