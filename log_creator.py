import numpy as np
import pandas as pd
import time
import os

# --- 1. 상수 정의 ---
# CSV 헤더 정의 (로그 뷰어에서 사용할 헤더와 일치해야 함)
HEADERS = [
    'Time(s)', 'FSM_State', 
    'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 
    'End_X(m)', 'End_Y(m)', 'End_Z(m)'
]

# 로봇 상태 매핑 (로그 뷰어의 STATE_MAPPING을 따름)
FSM_STATES = [
    'IDLE', 
    'OPERATING', 
    'EMERGENCY', 
    'MAINTENANCE'
]

# --- 2. 데이터 생성 함수 ---
def generate_robot_log_data(duration_sec=10, sample_rate_hz=100, filename="test_robot_log.csv"):
    """
    임의의 로봇 상태 시계열 데이터를 생성하여 CSV로 저장합니다.
    
    Args:
        duration_sec (int): 전체 시뮬레이션 시간 (초).
        sample_rate_hz (int): 데이터 샘플링 빈도 (Hz).
        filename (str): 저장할 파일 이름.
    """
    num_samples = duration_sec * sample_rate_hz
    dt = 1.0 / sample_rate_hz
    data = []
    current_time = 0.0
    
    # 임의의 초기 관절 각도 (라디안) 및 위치
    joint_angles = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    end_position = np.array([0.5, 0.0, 0.3])
    
    # 상태 전환 시점 (예: 2초, 4.5초, 7초)
    transition_times = [2.0, 4.5, 7.0]
    state_index = 0
    current_state = FSM_STATES[state_index]

    print(f"⏳ Generating {duration_sec}s of data ({num_samples} samples)...")

    for i in range(num_samples):
        # 2-1. 시간 진행 및 상태 전환 체크
        current_time = i * dt
        
        if state_index < len(transition_times) and current_time >= transition_times[state_index]:
            state_index += 1
            if state_index < len(FSM_STATES):
                current_state = FSM_STATES[state_index]

        # 2-2. 임의의 데이터 변화 (노이즈와 추세 추가)
        noise = np.random.randn(6) * 0.005 
        trend = np.sin(current_time / 2) * 0.001
        
        # OPERATING 상태일 때 관절 및 위치를 움직임
        if 'OPERATING' in current_state:
            joint_angles += (noise + trend) * 5
            end_position[0] += np.sin(current_time) * 0.001
            end_position[2] += np.cos(current_time) * 0.0005
        
        # EMERGENCY 상태일 때 움직임 정지
        elif 'EMERGENCY' in current_state:
            joint_angles += noise * 0.1 # 미세한 떨림만 허용
            
        # 2-3. 데이터 로우 구성
        row = [current_time, current_state]
        row.extend(joint_angles.tolist())
        row.extend(end_position.tolist())
        
        data.append(row)

    # 3. DataFrame으로 변환 및 CSV 저장
    df = pd.DataFrame(data, columns=HEADERS)
    
    # 🚨 저장 경로 설정 (사용자 지정 경로에 맞게 조정 필요)
    # 현재는 프로젝트 루트에 저장하도록 설정
    output_path = os.path.join(os.getcwd(), filename)
    df.to_csv(output_path, index=False)
    
    print(f"✅ Data generated successfully and saved to: {output_path}")
    print("-" * 30)
    print(df.head())

# --- 3. 실행 ---
if __name__ == "__main__":
    # 10초 동안 100Hz로 데이터를 생성하여 'test_robot_log.csv' 파일로 저장
    generate_robot_log_data(duration_sec=10, sample_rate_hz=100, filename="test_robot_log.csv")