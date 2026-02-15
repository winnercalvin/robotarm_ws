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
    
    print("=== [Scenario] 튀김 요리 프로세스 시작 ===")

    # ============================================================
    # 1. 스쿠퍼 잡기 (Task 1)
    # ============================================================
    print("\n▶ Step 1: 스쿠퍼 잡기 이동")
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

    # ============================================================
    # 2. 스쿠핑 동작 (Task 5)
    # ============================================================
    print("\n▶ Step 2: 스쿠핑(Scoop) 실행")

    # [좌표 설정] 스쿱 동작에 필요한 4가지 포인트 정의
    scoop_p0 = [0.0, 0.0, 90.0, 0.0, 90.0, -90.0]           # 시작(관절)
    scoop_p1 = [367.34, 4.86, 125.75, 111.67, 179.79, 120.88] # 경유(좌표)
    scoop_p2 = [589.75, 16.62, 90.35, 152.95, 174.37, 70.14]  # 목표(좌표)
    scoop_p3 = [589.75, 16.62, 120.35, 152.95, 174.37, 70.14] # 상승(좌표)

    # 데이터 합치기 (24개)
    full_scoop_data = scoop_p0 + scoop_p1 + scoop_p2 + scoop_p3

    future = controller.send_task(full_scoop_data, task_type=5)
    rclpy.spin_until_future_complete(controller, future)

    goal_handle = future.result()
    if goal_handle.accepted:
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(controller, res_future)
        print("✅ 스쿠핑 완료!")

    print("⏳ [System] 로봇 상태 정리 중... (3초 대기)")
    time.sleep(3.0)
    
    # ============================================================
    # 3. 튀김기 가서 붓기 (Task 3)
    # ============================================================
    print("\n▶ Step 3: 튀김기로 이동 및 붓기")
    
    # [좌표 확인] 붓기 작업을 할 위치 (튀김기 앞)
    pos_fryer = [0.0, 0.0, 0.0, 0.0, 90.0, 0.0] 
    
    future = controller.send_task(pos_fryer, task_type=3)
    rclpy.spin_until_future_complete(controller, future)

    goal_handle = future.result()
    if goal_handle.accepted:
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(controller, res_future)
        print("✅ 붓기 완료!")

    print("⏳ [System] 3초 대기...")
    time.sleep(3.0)

    # ------------------------------------------------------------
    # 4. 기름 털기 (Task 4)
    # ------------------------------------------------------------
    print("\n▶ Step 4: 기름 털기 (이동 -> 잡기 -> Shake)")
    
    # 서버는 해당 위치로 이동한 후 -> Grip을 수행하고 -> Shake를 합니다.
    pos_shake = [-13.679, 21.319, 73.279, 4.451, 61.702, -0.439]
    
    future = controller.send_task(pos_shake, task_type=4)
    rclpy.spin_until_future_complete(controller, future)
    print("✅ 털기 완료!")

    print("\n🏁 모든 시나리오 종료")
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()