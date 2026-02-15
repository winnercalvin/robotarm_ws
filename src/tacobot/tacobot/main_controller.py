import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from tacobot_interfaces.action import RobotTask

class TaskController(Node):
    def __init__(self):
        super().__init__('task_controller')
        # [수정] 클라이언트는 하나만 있으면 됩니다!
        self.cli_universal = ActionClient(self, RobotTask, '/dsr01/action_server')

    def send_task(self, joints, task_type):
        """
        :param task_type: 1(집기), 2(놓기), 3(붓기)
        """
        if not self.cli_universal.wait_for_server(timeout_sec=20.0):
            self.get_logger().error("서버를 찾을 수 없습니다!")
            return None

        goal_msg = RobotTask.Goal()
        goal_msg.target_joints = joints
        goal_msg.task_type = task_type

        print(f"명령 전송: Type {task_type}")
        
        return self.cli_universal.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

    def feedback_callback(self, feedback_msg):
        print(f"   📢 상태: {feedback_msg.feedback.status}")

def main(args=None):
    rclpy.init(args=args)
    controller = TaskController()

    # --- [시나리오] ---
    
    # 1. 스쿠퍼 잡기 (Type 1) -> [주석 처리됨: 실행 안 함]
    pos_scooper = [-21.679, 31.319, 73.279, 4.451, 61.702, -0.439]
    future = controller.send_task(pos_scooper, task_type=1)
    if future is None:
        print("❌ 명령 전송 실패: 서버가 응답하지 않습니다.")
        rclpy.shutdown()
        return
    rclpy.spin_until_future_complete(controller, future)
    
    # 결과 확인 로직 -> [주석 처리됨]
    goal_handle = future.result()
    if goal_handle.accepted:
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(controller, res_future)
        print("✅ 잡기 완료!\n")
    
    print("⏳ [System] 로봇 상태 정리 중... (3초 대기)")
    time.sleep(3.0)
    
    # 2. 튀김기 가서 붓기 (Type 3) -> [이것만 실행됨!]
    pos_fryer = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0] 
    future = controller.send_task(pos_fryer, task_type=3) # 3번이 붓기!
    rclpy.spin_until_future_complete(controller, future)

    goal_handle = future.result()
    if goal_handle.accepted:
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(controller, res_future)
        print("✅ 붓기 완료!\n")

    print("🏁 종료")
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()