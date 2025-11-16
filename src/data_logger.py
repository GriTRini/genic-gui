# data_logger.py
import csv
from datetime import datetime

class DataLogger:
    """로봇 상태 데이터를 CSV 파일로 기록하는 클래스."""
    
    def __init__(self, filename="robot_log.csv"):
        self.filename = filename
        self._initialize_csv()

    def _initialize_csv(self):
        """CSV 파일을 초기화하고 헤더를 작성합니다."""
        with open(self.filename, 'w', newline='') as f:
            writer = csv.writer(f)
            # FSM 상태, 각도, Tmat 위치, Tmat 회전 정보 등을 기록할 헤더
            header = ['Timestamp', 'FSM_State', 'Joint_Angles', 'Pose_X', 'Pose_Y', 'Pose_Z', 'Quat_X', 'Quat_Y', 'Quat_Z', 'Quat_W']
            writer.writerow(header)

    def log_data(self, fsm_state, angles, pose_data):
        """실시간 데이터를 파일에 기록합니다."""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        
        # pose_data: (x, y, z, qx, qy, qz, qw) 튜플을 가정
        
        try:
            row = [
                timestamp,
                fsm_state,
                # 관절 각도 리스트를 문자열로 변환하여 저장
                str(angles),
                f"{pose_data[0]:.4f}", f"{pose_data[1]:.4f}", f"{pose_data[2]:.4f}",
                f"{pose_data[3]:.4f}", f"{pose_data[4]:.4f}", f"{pose_data[5]:.4f}", f"{pose_data[6]:.4f}",
            ]
            with open(self.filename, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(row)
        except Exception as e:
            print(f"Error logging data: {e}")