import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String, Int32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from PySide6.QtCore import Slot, QThread, Signal

# ====================================================================
# 1. ROS2 Publisher 노드 (명령 발행)
# ====================================================================
class CommandPublisher(Node):
    def __init__(self):
        super().__init__('pyside6_command_publisher') 
        
        # 1. 기존 문자열 명령 (/user_command)
        self.publisher_ = self.create_publisher(String, '/user_command', 10)

        # 2. EtherCAT 명령 (/ethercat/cmd)
        self.ethercat_publisher_ = self.create_publisher(Int32, '/ethercat/cmd', 10)
        
        # [수정 4] 수동 조작 명령 (/manual_cmd) - Int32 사용
        # 프로토콜 예시: 1:전진, 2:후진, 3:좌, 4:우
        self.manual_publisher_ = self.create_publisher(Int32, '/manual_cmd', 10)

        self.get_logger().info('ROS2 Command Publisher Node Started.')

    def publish_command(self, command_char):
        msg = String()
        msg.data = command_char
        self.publisher_.publish(msg)
        self.get_logger().info(f"Command: '{msg.data}'")

    def publish_ethercat_cmd(self, value: int):
        msg = Int32()
        msg.data = value
        self.ethercat_publisher_.publish(msg)
        self.get_logger().info(f"EtherCAT Cmd: {value}")

    # [수정 5] 수동 조작 발행 함수 추가
    def publish_manual_cmd(self, value: int):
        msg = Int32()
        msg.data = value
        self.manual_publisher_.publish(msg)
        # 로그가 너무 많이 찍히는게 싫으면 아래 줄은 주석 처리하세요
        self.get_logger().info(f"Manual Cmd: {value}") 


# ====================================================================
# 2. ROS2 통신 백그라운드 스레드
# ====================================================================
class RclpyThread(QThread):
    fsm_state_updated = Signal(str)
    pose_updated = Signal(float, float, float)
    joint_angles_updated = Signal(list) 

    def __init__(self):
        super().__init__()
        self.publisher_node = None
        self.subscriber_node = None
        self.running = True

    def run(self):
        rclpy.init(args=None) 
        self.publisher_node = CommandPublisher()
        self.subscriber_node = self._setup_subscriber_node()

        executor = SingleThreadedExecutor()
        executor.add_node(self.publisher_node)
        executor.add_node(self.subscriber_node)
        
        while rclpy.ok() and self.running:
            executor.spin_once(timeout_sec=0.01)
            QThread.msleep(1) 
        
        if self.publisher_node:
             self.publisher_node.destroy_node()
        if self.subscriber_node:
             self.subscriber_node.destroy_node()
        rclpy.shutdown()

    def _setup_subscriber_node(self):
        sub_node = rclpy.create_node('pyside6_status_subscriber')
        
        sub_node.create_subscription(
            String, '/robot/fsm_state', 
            lambda msg: self.fsm_state_updated.emit(f"State: {msg.data}"), 10
        )

        sub_node.create_subscription(
            Pose, '/robot/end_pose',
            lambda msg: self.pose_updated.emit(msg.position.x, msg.position.y, msg.position.z), 10
        )

        sub_node.create_subscription(
            JointState, '/robot/joint_states',
            lambda msg: self.joint_angles_updated.emit(list(msg.position)), 10
        )
        
        return sub_node

    @Slot()
    def stop(self):
        self.running = False
        self.quit()
        self.wait() 

    # Publisher 노드 접근용 헬퍼
    def get_publisher(self):
        return self.publisher_node