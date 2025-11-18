import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger # 💡 연결 요청을 위한 표준 서비스 임포트

from PySide6.QtCore import Slot, QThread, Signal, Qt

# ====================================================================
# 1. ROS 2 Publisher 노드 (명령 발행)
# ====================================================================
class CommandPublisher(Node):
    def __init__(self):
        # 노드 이름을 클라이언트 기능까지 포함하도록 변경
        super().__init__('pyside6_command_publisher_client_node') 
        
        # /user_command 토픽 발행
        self.publisher_ = self.create_publisher(
            String, 
            '/user_command', 
            10
        )
        self.get_logger().info('ROS2 Command Publisher Node Started.')

    def publish_command(self, command_char):
        """GUI로부터 명령을 받아 ROS 2 토픽으로 발행"""
        msg = String()
        msg.data = command_char
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing Command: '{msg.data}' on '/user_command'")


# ====================================================================
# 2. ROS 2 Service Client 노드 (연결 요청 처리)
# ====================================================================
class ConnectionClient(Node):
    # 연결 결과를 GUI로 보내기 위한 시그널 (QThread에서 연결될 예정)
    connection_result = Signal(bool, str)

    def __init__(self):
        super().__init__('robot_connection_client')
        # /connect_robot 서비스에 연결하는 클라이언트 생성
        self.client = self.create_client(Trigger, 'connect_robot')
        self.get_logger().info('ROS2 Connection Client Node Created.')

    def wait_for_service(self):
        """서비스 서버가 준비될 때까지 대기"""
        if not self.client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('Connection service not available on /connect_robot.')
            return False
        return True

    def send_request(self):
        """비동기 서비스 요청을 보내고 Future 객체를 반환"""
        if not self.wait_for_service():
            # 서비스 준비 실패 시 즉시 피드백 시그널 방출
            self.connection_result.emit(False, "Service unavailable or not ready.")
            return None

        request = Trigger.Request()
        future = self.client.call_async(request)
        
        self.get_logger().info('Sending connection request...')
        return future

    # QThread에서 시그널 연결을 위해 사용하는 메서드
    def set_signal_target(self, signal_target):
        self.connection_result = signal_target


# ====================================================================
# 3. ROS 2 통신 백그라운드 스레드 (상태 구독, 명령 발행 및 연결 요청 관리)
# ====================================================================
class RclpyThread(QThread):
    # ------------------------------------
    # ROS 2 데이터를 GUI로 전달하는 시그널
    # ------------------------------------
    fsm_state_updated = Signal(str)
    pose_updated = Signal(float, float, float) 
    joint_angles_updated = Signal(list) 
    connection_feedback = Signal(bool, str) # 💡 연결 요청 결과 시그널

    def __init__(self):
        super().__init__()
        self.publisher_node = None
        self.subscriber_node = None
        self.client_node = None
        self.running = True
        self.pending_service_future = None # 비동기 서비스 처리를 위한 Future 저장

    def run(self):
        rclpy.init(args=None) 
        
        # 1. 노드 생성
        self.publisher_node = CommandPublisher()
        self.subscriber_node = self._setup_subscriber_node()
        self.client_node = ConnectionClient()
        
        # 2. 클라이언트 노드의 시그널을 이 스레드의 시그널에 연결 (GUI 전달 경로 설정)
        self.client_node.set_signal_target(self.connection_feedback) 

        # 3. Executor 설정
        executor = SingleThreadedExecutor()
        executor.add_node(self.publisher_node)
        executor.add_node(self.subscriber_node)
        executor.add_node(self.client_node) # 클라이언트 노드 추가
        
        # 4. ROS 2 스핀 루프
        while rclpy.ok() and self.running:
            # 💡 펜딩된 서비스 요청 결과 처리
            if self.pending_service_future and self.pending_service_future.done():
                try:
                    response = self.pending_service_future.result()
                    if response is not None:
                        # 결과를 GUI로 전달
                        self.connection_feedback.emit(
                            response.success,
                            response.message
                        )
                    else:
                        self.connection_feedback.emit(False, "Service failed (no response data).")
                except Exception as e:
                    self.client_node.get_logger().error(f'Service call error: {e}')
                    self.connection_feedback.emit(False, f"Exception during service call: {e}")
                finally:
                    self.pending_service_future = None # Future 처리 완료
            
            executor.spin_once(timeout_sec=0.01)
            QThread.msleep(1) 
        
        # 5. 노드 정리 및 종료
        if self.publisher_node: self.publisher_node.destroy_node()
        if self.subscriber_node: self.subscriber_node.destroy_node()
        if self.client_node: self.client_node.destroy_node()
        rclpy.shutdown()

    def _setup_subscriber_node(self):
        """상태 구독을 위한 임시 노드 생성 및 구독 설정"""
        sub_node = rclpy.create_node('pyside6_status_subscriber')
        
        # FSM 상태 구독
        sub_node.create_subscription(
            String, '/fsm_state', 
            lambda msg: self.fsm_state_updated.emit(f"State: {msg.data}"), 10
        )
        # 끝단 위치 구독
        sub_node.create_subscription(
            Pose, '/robot/end_pose',
            lambda msg: self.pose_updated.emit(
                msg.position.x, msg.position.y, msg.position.z
            ), 10
        )
        # 관절 상태 구독
        sub_node.create_subscription(
            JointState, '/robot/joint_states',
            lambda msg: self.joint_angles_updated.emit(list(msg.position)), 10
        )
        
        return sub_node

    @Slot()
    def request_connection(self):
        """
        [GUI Slot] GUI 버튼 클릭 시 호출되어 연결 요청을 시작합니다.
        """
        if self.client_node:
            future = self.client_node.send_request()
            if future:
                self.pending_service_future = future # Future를 저장하여 run()에서 결과 확인
        else:
            self.connection_feedback.emit(False, "ROS2 Thread not fully initialized.")

    @Slot()
    def stop(self):
        """스레드 및 ROS 2 환경 안전 종료"""
        print('ROS2 Node Destroying...') 
        self.running = False
        self.quit()
        self.wait() 

    def get_publisher(self):
        return self.publisher_node