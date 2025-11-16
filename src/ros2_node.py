# src/ros2_node.py
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from PySide6.QtCore import Slot, QThread, Signal, Qt

# ====================================================================
# 1. ROS2 Publisher 노드 (명령 발행)
# ====================================================================
class CommandPublisher(Node):
    def __init__(self):
        super().__init__('pyside6_command_publisher') 
        # /user_command 토픽 발행
        self.publisher_ = self.create_publisher(
            String, 
            '/user_command', 
            10
        )
        self.get_logger().info('ROS2 Command Publisher Node Started.')

    def publish_command(self, command_char):
        msg = String()
        msg.data = command_char
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing Command: '{msg.data}' on '/user_command'")


# ====================================================================
# 2. ROS2 통신 백그라운드 스레드 (상태 구독 및 명령 발행)
# ====================================================================
class RclpyThread(QThread):
    fsm_state_updated = Signal(str)
    pose_updated = Signal(float, float, float) # 위치 3개(x, y, z)만 전달
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
        
        # 1. FSM 상태 구독
        sub_node.create_subscription(
            String, '/fsm_state', 
            lambda msg: self.fsm_state_updated.emit(f"State: {msg.data}"), 10
        )

        # 2. 끝단 위치 구독 (/robot/end_pose)
        sub_node.create_subscription(
            Pose, '/robot/end_pose',
            lambda msg: self.pose_updated.emit(
                msg.position.x, msg.position.y, msg.position.z
            ), 10
        )

        # 3. 관절 상태 구독 (/robot/joint_states)
        sub_node.create_subscription(
            JointState, '/robot/joint_states',
            lambda msg: self.joint_angles_updated.emit(list(msg.position)), 10
        )
        
        return sub_node

    @Slot()
    def stop(self):
        print('ROS2 Node Destroying...') 
        self.running = False
        self.quit()
        self.wait() 

    def get_publisher(self):
        return self.publisher_node