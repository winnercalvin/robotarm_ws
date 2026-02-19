import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from tacobot_interfaces.action import RobotTask

class TaskController(Node):
    def __init__(self):
        super().__init__('task_controller')
        self.cli_universal = ActionClient(self, RobotTask, '/dsr01/action_server')

    def send_task(self, joints, task_type):
        """
        :param task_type: 1(잡기), 3(붓기), 4(놓고 새로잡아 흔들기), 5(스쿱), 6(기름털기)
        """
        if not self.cli_universal.wait_for_server(timeout_sec=20.0):
            self.get_logger().error("서버를 찾을 수 없습니다!")
            return None

        goal_msg = RobotTask.Goal()
        goal_msg.target_joints = joints
        goal_msg.task_type = task_type

        print(f"   🚀 명령 전송: Type {task_type} (데이터 개수: {len(joints)})")
        
        return self.cli_universal.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

    def feedback_callback(self, feedback_msg):
        # print(f"   📢 상태: {feedback_msg.feedback.status}")
        pass

def main(args=None):
    rclpy.init(args=args)
    controller = TaskController()

    try:
        # 🚨 [핵심] rclpy가 살아있는 동안 계속 반복 (무한 루프)
        while rclpy.ok():
            print("\n" + "="*50)
            print("🍟 감자튀김 조리 대기 중... (새 주문을 기다립니다)")
            print("="*50)
            
            # 사용자 입력 대기 (y 누르면 시작, n 누르면 종료)
            user_cmd = input("👉 조리를 시작할까요? (y 누르면 시작 / n 누르면 종료): ")
            
            if user_cmd.lower() == 'n':
                print("🛑 프로그램을 안전하게 종료합니다.")
                break # while문을 빠져나가서 프로그램 종료
            elif user_cmd.lower() != 'y':
                print("⚠️ 잘못된 입력입니다. 'y' 또는 'n'을 입력해주세요.")
                continue # 다시 입력 대기로 돌아감

            # 여기서부터 기존 시나리오 쭈욱 진행
            print("\n=== [Scenario] 🍟 감자튀김 요리 프로세스 시작 ===")

            # ============================================================
            # Step 1. 소분된 용기를 잡는다 (Task 1)
            # ============================================================
            print("\n▶ Step 1: 소분된 용기를 잡는다")
            pos_scooper = [39.09, 47.41, 73.43, 10.34, 60.66, -15.75]
            future = controller.send_task(pos_scooper, task_type=1)
            if future is None:
                print("❌ 명령 전송 실패: 서버가 응답하지 않습니다.")
                break
            rclpy.spin_until_future_complete(controller, future)
            
            goal_handle = future.result()
            if goal_handle.accepted:
                res_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, res_future)
                print("✅ 잡기 완료!\n")
            
            time.sleep(3.0)

            # # ============================================================
            # # 2. 스쿠핑 동작 (Task 5)
            # # ============================================================
            # print("\n▶ Step 2: 스쿠핑(Scoop) 실행")

            # # [좌표 설정] 스쿱 동작에 필요한 4가지 포인트 정의
            # scoop_p0 = [0.0, 0.0, 90.0, 0.0, 90.0, -90.0]           # 시작(관절)
            # scoop_p1 = [367.34, 4.86, 125.75, 111.67, 179.79, 120.88] # 경유(좌표)
            # scoop_p2 = [589.75, 16.62, 90.35, 152.95, 174.37, 70.14]  # 목표(좌표)
            # scoop_p3 = [589.75, 16.62, 120.35, 152.95, 174.37, 70.14] # 상승(좌표)

            # # 데이터 합치기 (24개)
            # full_scoop_data = scoop_p0 + scoop_p1 + scoop_p2 + scoop_p3

            # future = controller.send_task(full_scoop_data, task_type=5)
            # rclpy.spin_until_future_complete(controller, future)

            # goal_handle = future.result()
            # if goal_handle.accepted:
            #     res_future = goal_handle.get_result_async()
            #     rclpy.spin_until_future_complete(controller, res_future)
            #     print("✅ 스쿠핑 완료!")

            # print("⏳ [System] 로봇 상태 정리 중... (3초 대기)")
            # time.sleep(3.0)
    
            # ============================================================
            # Step 2. 용기에 담긴 감자를 튀김트레이에 붓는다 (Task 3)
            # ============================================================
            print("\n▶ Step 2: 감자를 튀김트레이에 붓는다")
            pos_pour_potato = [-0.3, 16.99, 59.02, -6.54, 67.63, -15.75] 
            future = controller.send_task(pos_pour_potato, task_type=3)
            rclpy.spin_until_future_complete(controller, future)

            goal_handle = future.result()
            if goal_handle.accepted:
                res_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, res_future)
                print("✅ 감자 붓기 완료!")
            time.sleep(3.0)

            # ============================================================
            # Step 3. 다 부은 빈 용기를 지정된 위치에 놓는다 (Task 2)
            # ============================================================
            print("\n▶ Step 3: 빈 용기를 지정된 위치에 내려놓는다 (놓기)")
            pos_place_container = [33.77, -21.1, 101.85, 2.98, 81.93, -15.75] 
            future = controller.send_task(pos_place_container, task_type=2)
            rclpy.spin_until_future_complete(controller, future)

            goal_handle = future.result()
            if goal_handle.accepted:
                res_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, res_future)
                print("✅ 용기 내려놓기 완료!")
            time.sleep(3.0)

            # ============================================================
            # Step 4. 튀김트레이를 흔든다 (Task 4)
            # ============================================================
            print("\n▶ Step 4: 튀김트레이를 잡고 흔든다 (잘 튀겨지게)")
            pos_shake_tray = [529.43, 12.41, 147.99, 170.37, -174.75, 81.52] 
            future = controller.send_task(pos_shake_tray, task_type=4)
            rclpy.spin_until_future_complete(controller, future)

            goal_handle = future.result()
            if goal_handle.accepted:
                print("   ⏳ 쉐이크 작업 진행 중... (완료 신호 대기)")
                res_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, res_future)
                print("✅ 튀김트레이 흔들기 완료!")
            time.sleep(3.0)

            # ============================================================
            # Step 5. 튀김트레이를 잡고 기름을 턴다 (Task 6)
            # ============================================================
            print("\n▶ Step 5: 기름을 턴다 (Drain)")
            drain_p1 = [454.94, -3.99, 533.69, 176.66, -118.56, 86.42] 
            drain_p2 = [661.29, 17.23, 246.26, 0.58, 112.04, -86.21] 
            full_drain_data = drain_p1 + drain_p2
            
            future = controller.send_task(full_drain_data, task_type=6)
            rclpy.spin_until_future_complete(controller, future)

            goal_handle = future.result()
            if goal_handle.accepted:
                print("   ⏳ 기름 털기 진행 중... (완료 신호 대기)")
                res_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, res_future)
                print("✅ 확실히 털기(Drain) 완료!")
            else:
                print("❌ 서버가 작업을 거부했습니다.")
            time.sleep(3.0)

            # ============================================================
            # Step 6. 튀김트레이에 담긴 감자칩을 용기에 붓는다 (Task 3)
            # ============================================================
            print("\n▶ Step 6: 감자칩을 최종 용기에 붓는다")
            pos_pour_chips = [-46.97, -26.09, 126.18, 15.81, 30.23, -131.58] 
            future = controller.send_task(pos_pour_chips, task_type=3)
            rclpy.spin_until_future_complete(controller, future)

            goal_handle = future.result()
            if goal_handle.accepted:
                res_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, res_future)
                print("\n✅ 감자칩 붓기 완료!")
            time.sleep(3.0)

            # ============================================================
            # Step 7. 빈 튀김트레이를 제자리에 내려놓는다 (Task 2)
            # ============================================================
            print("\n▶ Step 7: 튀김트레이를 제자리에 내려놓는다 (놓기)")
            pos_place_tray = [0.44, 19.61, 75.85, -0.31, 73.65, -88.48] 
            future = controller.send_task(pos_place_tray, task_type=2)
            rclpy.spin_until_future_complete(controller, future)

            goal_handle = future.result()
            if goal_handle.accepted:
                res_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, res_future)
                print("\n✅ 튀김트레이 내려놓기 완료!")

            print("\n🏁 한 세트 조리 완료! 다음 주문을 대기합니다.")
            # 루프 끝. 다시 while문의 처음(input 대기)으로 돌아갑니다.

    except KeyboardInterrupt:
        # 작업자가 Ctrl+C를 누르면 안전하게 종료
        print("\n사용자에 의해 강제 종료되었습니다.")
    finally:
        # while문을 빠져나왔거나 Ctrl+C를 눌렀을 때 실행되는 뒷정리 코드
        print("로봇 컨트롤러를 종료합니다.")
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()