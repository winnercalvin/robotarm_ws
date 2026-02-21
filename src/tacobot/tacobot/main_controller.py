import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from tacobot_interfaces.action import RobotTask
from std_msgs.msg import String
import json

class TaskController(Node):
    def __init__(self):
        super().__init__('task_controller')
        self.cli_universal = ActionClient(self, RobotTask, '/dsr01/action_server')

        self.order_received = False
        self.current_order_data = {}

        self.subscription = self.create_subscription(
            String,
            '/taco_order', 
            self.order_callback,
            10
        )
        self.get_logger().info("🎧 '/taco_order' 토픽 구독 시작. 주문 대기 중...")

    def order_callback(self, msg):
        try:
            order_data = json.loads(msg.data)
            self.get_logger().info(f"{order_data}")
            self.current_order_data = order_data
            self.order_received = True

        except Exception as e:
            self.get_logger().error(f'에러발생: {str(e)}')

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
            print("🍟 감자튀김 자동 연속 조리 모드 실행 중...")
            print(" (🛑 중지하려면 터미널에서 Ctrl + C 를 누르세요)")
            print("="*50)

            while rclpy.ok() and not controller.order_received:
                rclpy.spin_once(controller, timeout_sec=0.1)
            
            controller.order_received = False

            # 여기서부터 기존 시나리오 쭈욱 진행
            print("\n=== [Scenario] 🍟 감자튀김 요리 프로세스 시작 ===")

            # ============================================================
            # Step 1. 소분된 용기를 잡는다 (Task 1)
            # ============================================================
            print("\n▶ Step 1-1: 용기 근처(안전 경유지)로 이동하며 그리퍼 열기")
            
            # [좌표 수정 필요] 용기 바로 위 또는 앞의 안전한 '경유지' 좌표를 넣으세요!
            pos_approach = [50.66, 30.68, 51.24, 0.21, 97.43, 0.04] 
            
            # task_type=2 (놓기)를 활용: 이동 후 손을 미리 엶
            future = controller.send_task(pos_approach, task_type=2)
            if future is None:
                print("❌ 명령 전송 실패: 서버가 응답하지 않습니다.")
                break
            rclpy.spin_until_future_complete(controller, future)
            
            goal_handle = future.result()
            if goal_handle.accepted:
                res_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, res_future)
                print("✅ 경유지 도착 및 그리퍼 오픈 완료!")
            
            time.sleep(1.0) # 다음 동작 전 1초 대기

            print("\n▶ Step 1-2: 소분된 용기를 잡는다")
            pos_scooper = [42.62, 49.44, 68.33, 4.33, 66.55, -10.6]
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
            
            time.sleep(2.0)

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
            print("\n▶ Step 2: 튀김트레이 경유지를 거쳐 붓기 위치로 이동 (블렌딩)")
            
            # 경유지
            pos_waypoint = [43.08, 31.77, 29.97, 3.24, 114.69, -8.73] 
            # 최종 붓기 도착지
            pos_pour_potato = [-20.41, 30.97, 47.87, 27.2, 94.64, -19.27] 
            
            # 🌟 두 좌표를 합쳐서(12개 데이터) 한 번에 전송합니다!
            combined_pour_data = pos_waypoint + pos_pour_potato
            
            future = controller.send_task(combined_pour_data, task_type=3)
            rclpy.spin_until_future_complete(controller, future)

            goal_handle = future.result()
            if goal_handle.accepted:
                res_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, res_future)
                print("✅ 멈춤 없는 스무스한 이동 및 감자 붓기 완료!")
            time.sleep(2.0)

            # ============================================================
            # Step 3. 다 부은 빈 용기를 지정된 위치에 놓는다 (Task 2)
            # ============================================================
            print("\n▶ Step 3-1: 튀김트레이 위 안전 경유지로 이동")
            # 목표 위치보다 위쪽이나 안전한 각도를 임의로 설정 (값은 실제 로봇에 맞게 수정)
            pos_waypoint = [25.82, -15.82, 87.7, 11.99, 92.98, -8.73] 
            future_wp = controller.send_task(pos_waypoint, task_type=0) # 단순 이동(0)
            rclpy.spin_until_future_complete(controller, future_wp)

            wp_handle = future_wp.result()
            if wp_handle.accepted:
                wp_res_future = wp_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, wp_res_future)
                print("✅ 경유지 도착 완료!")

            time.sleep(1.0) # 안정화 대기
            print("\n▶ Step 3-2: 빈 용기를 지정된 위치에 내려놓는다 (놓기)")
            pos_place_container = [13.66, -2.72, 115.63, 2.60, 66.76, -31.08] 
            future = controller.send_task(pos_place_container, task_type=2)
            rclpy.spin_until_future_complete(controller, future)

            goal_handle = future.result()
            if goal_handle.accepted:
                res_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, res_future)
                print("✅ 용기 내려놓기 완료!")
            time.sleep(2.0)

            # ============================================================
            # Step 4. 튀김트레이를 흔든다 (Task 4)
            # ============================================================
            print("\n▶ Step 4-1")
            # 목표 위치보다 위쪽이나 안전한 각도를 임의로 설정 (값은 실제 로봇에 맞게 수정)
            pos_waypoint = [29.15, -6.78, 79.14, 4.16, 104.55, -7.97] 
            future_wp = controller.send_task(pos_waypoint, task_type=0) # 단순 이동(0)
            rclpy.spin_until_future_complete(controller, future_wp)

            wp_handle = future_wp.result()
            if wp_handle.accepted:
                wp_res_future = wp_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, wp_res_future)
                print("✅ 경유지 도착 완료!")

            time.sleep(1.0) # 안정화 대기

            print("\n▶ Step 4-2")
            # 목표 위치보다 위쪽이나 안전한 각도를 임의로 설정 (값은 실제 로봇에 맞게 수정)
            pos_waypoint = [2.55, 10.11, 71.86, -1.84, 78.37, -87.35] 
            future_wp = controller.send_task(pos_waypoint, task_type=0) # 단순 이동(0)
            rclpy.spin_until_future_complete(controller, future_wp)

            wp_handle = future_wp.result()
            if wp_handle.accepted:
                wp_res_future = wp_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, wp_res_future)
                print("✅ 경유지 도착 완료!")

            time.sleep(1.0) # 안정화 대기

            print("\n▶ Step 4-3")
            # 목표 위치보다 위쪽이나 안전한 각도를 임의로 설정 (값은 실제 로봇에 맞게 수정)
            pos_waypoint = [1.76, 24.14, 67.56, -0.66, 85.16, -87.34] 
            future_wp = controller.send_task(pos_waypoint, task_type=1) 
            rclpy.spin_until_future_complete(controller, future_wp)

            wp_handle = future_wp.result()
            if wp_handle.accepted:
                wp_res_future = wp_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, wp_res_future)
                print("✅ 경유지 도착 완료!")

            time.sleep(1.0) # 안정화 대기

            print("\n▶ Step 4-4")
            # 목표 위치보다 위쪽이나 안전한 각도를 임의로 설정 (값은 실제 로봇에 맞게 수정)
            pos_waypoint = [1.11, 13.8, 72.06, 0.22, 86.3, -87.3] 
            future_wp = controller.send_task(pos_waypoint, task_type=0) 
            rclpy.spin_until_future_complete(controller, future_wp)

            wp_handle = future_wp.result()
            if wp_handle.accepted:
                wp_res_future = wp_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, wp_res_future)
                print("✅ 경유지 도착 완료!")

            time.sleep(1.0) # 안정화 대기

            print("\n▶ Step 4-5")
            # 목표 위치보다 위쪽이나 안전한 각도를 임의로 설정 (값은 실제 로봇에 맞게 수정)
            pos_waypoint = [1.92, 22.69, 40.16, 0.4, 133.09, -87.3] 
            future_wp = controller.send_task(pos_waypoint, task_type=0) 
            rclpy.spin_until_future_complete(controller, future_wp)

            wp_handle = future_wp.result()
            if wp_handle.accepted:
                wp_res_future = wp_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, wp_res_future)
                print("✅ 경유지 도착 완료!")

            time.sleep(1.0) # 안정화 대기

            print("\n▶ Step 4-6")
            # 목표 위치보다 위쪽이나 안전한 각도를 임의로 설정 (값은 실제 로봇에 맞게 수정)
            pos_waypoint = [1.53, 26.24, 55.19, 0.46, 105.61, -87.31] 
            future_wp = controller.send_task(pos_waypoint, task_type=0) 
            rclpy.spin_until_future_complete(controller, future_wp)

            wp_handle = future_wp.result()
            if wp_handle.accepted:
                wp_res_future = wp_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, wp_res_future)
                print("✅ 경유지 도착 완료!")

            time.sleep(1.0) # 안정화 대기

            print("\n▶ Step 4-7")
            # 목표 위치보다 위쪽이나 안전한 각도를 임의로 설정 (값은 실제 로봇에 맞게 수정)
            pos_waypoint = [1.53, 33.27, 49.67, 0.07, 108.93, -87.31] 
            future_wp = controller.send_task(pos_waypoint, task_type=0) 
            rclpy.spin_until_future_complete(controller, future_wp)

            wp_handle = future_wp.result()
            if wp_handle.accepted:
                wp_res_future = wp_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, wp_res_future)
                print("✅ 경유지 도착 완료!")

            time.sleep(1.0) # 안정화 대기

            print("\n▶ Step 4-8: 튀김트레이를 잡고 흔든다 (잘 튀겨지게)")
            pos_shake_tray = [1.27, 43.45, 38.39, 0.1, 117.29, -87.31]
            future = controller.send_task(pos_shake_tray, task_type=5)
            rclpy.spin_until_future_complete(controller, future)

            goal_handle = future.result()
            if goal_handle.accepted:
                print("   ⏳ 쉐이크 작업 진행 중... (완료 신호 대기)")
                res_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, res_future)
                print("✅ 튀김트레이 흔들기 완료!")
            time.sleep(2.0)

            # ============================================================
            # Step 5. 튀김트레이를 잡고 기름을 턴다 (Task 6)
            # ============================================================
            print("\n▶ Step 5-1")
            # 목표 위치보다 위쪽이나 안전한 각도를 임의로 설정 (값은 실제 로봇에 맞게 수정)
            pos_waypoint = [1.53, 33.27, 49.67, 0.07, 108.93, -87.31] 
            future_wp = controller.send_task(pos_waypoint, task_type=0) 
            rclpy.spin_until_future_complete(controller, future_wp)

            wp_handle = future_wp.result()
            if wp_handle.accepted:
                wp_res_future = wp_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, wp_res_future)
                print("✅ 경유지 도착 완료!")

            time.sleep(1.0) # 안정화 대기

            print("\n▶ Step 5-2: 기름을 턴다 (Drain)")
            drain_p1 = [402.16, 9.82, 459.29, 178.03, -148.43, 89.87] 
            drain_p2 = [470.54, 26.17, 186.83, 8.84, 164.47, -78.25] 
            full_drain_data = drain_p1 + drain_p2
            
            future = controller.send_task(full_drain_data, task_type=7)
            rclpy.spin_until_future_complete(controller, future)

            goal_handle = future.result()
            if goal_handle.accepted:
                print("   ⏳ 기름 털기 진행 중... (완료 신호 대기)")
                res_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, res_future)
                print("✅ 확실히 털기(Drain) 완료!")
            else:
                print("❌ 서버가 작업을 거부했습니다.")
            time.sleep(2.0)


            # ============================================================
            # Step 6. 튀김트레이를 흔들기 탁탁탁 (Task 4)
            # ============================================================

            print("\n▶ Step 6-1")
            # 목표 위치보다 위쪽이나 안전한 각도를 임의로 설정 (값은 실제 로봇에 맞게 수정)
            pos_waypoint = [2.15, 5.97, 61.84, -0.47, 100.58, -87.31] 
            future_wp = controller.send_task(pos_waypoint, task_type=0) 
            rclpy.spin_until_future_complete(controller, future_wp)

            wp_handle = future_wp.result()
            if wp_handle.accepted:
                wp_res_future = wp_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, wp_res_future)
                print("✅ 경유지 도착 완료!")

            time.sleep(1.0) # 안정화 대기

            print("\n▶ Step 6-2")
            # 목표 위치보다 위쪽이나 안전한 각도를 임의로 설정 (값은 실제 로봇에 맞게 수정)
            pos_waypoint = [2.15, 18.87, 57.06, -2.26, 89.04, -87.31] 
            future_wp = controller.send_task(pos_waypoint, task_type=4) 
            rclpy.spin_until_future_complete(controller, future_wp)

            wp_handle = future_wp.result() 
            if wp_handle.accepted:
                wp_res_future = wp_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, wp_res_future)
                print("✅ 탁탁탁 시작!")

            time.sleep(1.0) # 안정화 대기

            # ============================================================
            # Step 7. 튀김트레이에 담긴 감자칩을 용기에 붓는다 (Task 3)
            # ============================================================
            print("\n▶ Step 7-1: 튀김트레이 위 안전 경유지로 이동")
            # 목표 위치보다 위쪽이나 안전한 각도를 임의로 설정 (값은 실제 로봇에 맞게 수정)
            pos_waypoint = [27.61, -25.45, 85.36, 13.7, 96.56, -87.3] 
            future_wp = controller.send_task(pos_waypoint, task_type=0) # 단순 이동(0)
            rclpy.spin_until_future_complete(controller, future_wp)

            wp_handle = future_wp.result()
            if wp_handle.accepted:
                wp_res_future = wp_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, wp_res_future)
                print("✅ 경유지 도착 완료!")

            time.sleep(1.0) # 안정화 대기

            print("\n▶ Step 7-2: 감자칩을 최종 용기에 붓는다")
            pos_pour_chips = [-14.32, -33.3, 122.38, 18.34, 76.87, -111.05] 
            future = controller.send_task(pos_pour_chips, task_type=3)
            rclpy.spin_until_future_complete(controller, future)

            goal_handle = future.result()
            if goal_handle.accepted:
                res_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, res_future)
                print("\n✅ 감자칩 붓기 완료!")
            time.sleep(2.0)

            # ============================================================
            # Step 8. 빈 튀김트레이를 제자리에 내려놓는다 (Task 2)
            # ============================================================
            print("\n▶ Step 8-1: 튀김트레이 위 안전 경유지로 이동")
            # 목표 위치보다 위쪽이나 안전한 각도를 임의로 설정 (값은 실제 로봇에 맞게 수정)
            pos_waypoint = [27.6, -6.79, 66.7, 1.72, 117.22, -87.31] 
            future_wp = controller.send_task(pos_waypoint, task_type=0) # 단순 이동(0)
            rclpy.spin_until_future_complete(controller, future_wp)

            wp_handle = future_wp.result()
            if wp_handle.accepted:
                wp_res_future = wp_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, wp_res_future)
                print("✅ 경유지 도착 완료!")

            time.sleep(1.0) # 안정화 대기

            print("\n▶ Step 8-2: 튀김트레이 위 안전 경유지로 이동")
            # 목표 위치보다 위쪽이나 안전한 각도를 임의로 설정 (값은 실제 로봇에 맞게 수정)
            pos_waypoint = [3.33, 4.83, 78.71, -1.82, 85.91, -87.31] 
            future_wp = controller.send_task(pos_waypoint, task_type=0) # 단순 이동(0)
            rclpy.spin_until_future_complete(controller, future_wp)

            wp_handle = future_wp.result()
            if wp_handle.accepted:
                wp_res_future = wp_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, wp_res_future)
                print("✅ 경유지 도착 완료!")

            time.sleep(1.0) # 안정화 대기

            print("\n▶ Step 8-3: 튀김트레이 위 안전 경유지로 이동")
            # 목표 위치보다 위쪽이나 안전한 각도를 임의로 설정 (값은 실제 로봇에 맞게 수정)
            pos_waypoint = [0.95, 14.98, 73.94, 0.22, 77.5, -87.31] 
            future_wp = controller.send_task(pos_waypoint, task_type=0) # 단순 이동(0)
            rclpy.spin_until_future_complete(controller, future_wp)

            wp_handle = future_wp.result()
            if wp_handle.accepted:
                wp_res_future = wp_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, wp_res_future)
                print("✅ 경유지 도착 완료!")

            time.sleep(1.0) # 안정화 대기

            print("\n▶ Step 8-4: 튀김트레이를 제자리에 내려놓는다 (놓기)")
            pos_place_tray = [2.28, 15.08, 80.36, -2.05, 69.63, -86.94] 
            future = controller.send_task(pos_place_tray, task_type=2)
            rclpy.spin_until_future_complete(controller, future)

            goal_handle = future.result()
            if goal_handle.accepted:
                res_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(controller, res_future)
                print("\n✅ 튀김트레이 내려놓기 완료!")

            # ============================================================
            # Step 9. 추가 재료(Toppings) 확인 및 붓기
            # ============================================================
            print("\n============================================================")
            print("Step 9. 추가 재료(Toppings) 확인 및 붓기")
            print("============================================================")
            
            # JSON 구조에서 topping_ids 파싱
            order_tasks = controller.current_order_data.get('tasks', [])
            if order_tasks:
                topping_ids = order_tasks[0].get('topping_ids', [])
                
                if not topping_ids:
                    print("   👉 추가 선택 재료가 없습니다. 바로 서빙으로 넘어갑니다.")
                else:
                    # 토핑이 여러 개면 개수만큼 반복!
                    for topping in topping_ids:
                        print(f"\n▶ [추가 재료] '{topping}' 용기 잡고 붓기 시퀀스 시작!")
                        
                        # 🚨 [TODO] 재료별 좌표 세팅 (직접 티칭해서 값을 채워주세요!)
                        if topping == 'tomato':
                            pos_top_approach = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 토마토 통 안전 접근/놓기
                            pos_top_grab     = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 토마토 통 잡기
                            pos_top_pour_wp  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 감자칩 용기 위 안전 경유지
                            pos_top_pour     = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 토마토 붓기
                        elif topping == 'onion':
                            pos_top_approach = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 양파 통 안전 접근/놓기
                            pos_top_grab     = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 양파 통 잡기
                            pos_top_pour_wp  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 감자칩 용기 위 안전 경유지
                            pos_top_pour     = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 양파 붓기
                        elif topping == 'cabbage':
                            pos_top_approach = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 양배추 통 안전 접근/놓기
                            pos_top_grab     = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 양배추 통 잡기
                            pos_top_pour_wp  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 감자칩 용기 위 안전 경유지
                            pos_top_pour     = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 양배추 붓기
                        else:
                            print(f"⚠️ '{topping}'은(는) 알 수 없는 재료입니다. 패스합니다.")
                            continue
                            
                        # 8-1. 재료 통 접근 (그리퍼 열기)
                        future = controller.send_task(pos_top_approach, task_type=2)
                        rclpy.spin_until_future_complete(controller, future)
                        future.result().get_result_async()
                        time.sleep(1.0)
                        
                        # 8-2. 재료 통 잡기
                        future = controller.send_task(pos_top_grab, task_type=1)
                        rclpy.spin_until_future_complete(controller, future)
                        future.result().get_result_async()
                        time.sleep(2.0)
                        
                        # 8-3. 붓기 경유지 이동
                        future = controller.send_task(pos_top_pour_wp, task_type=0)
                        rclpy.spin_until_future_complete(controller, future)
                        future.result().get_result_async()
                        time.sleep(1.0)
                        
                        # 8-4. 재료 붓기 (최적화 붓기 활용)
                        future = controller.send_task(pos_top_pour, task_type=3)
                        rclpy.spin_until_future_complete(controller, future)
                        future.result().get_result_async()
                        time.sleep(2.0)
                        
                        # 8-5. 다시 제자리 경유지 이동
                        future = controller.send_task(pos_top_approach, task_type=0)
                        rclpy.spin_until_future_complete(controller, future)
                        future.result().get_result_async()
                        time.sleep(1.0)
                        
                        # 8-6. 재료 통 제자리에 내려놓기
                        future = controller.send_task(pos_top_grab, task_type=2)
                        rclpy.spin_until_future_complete(controller, future)
                        future.result().get_result_async()
                        print(f"✅ '{topping}' 투입 완료!\n")
                        time.sleep(2.0)


            # ============================================================
            # 🌟 Step 10. 소스 뿌리기 (Drizzle Sauce)
            # ============================================================
            print("\n============================================================")
            print("Step 10. 소스 뿌리기 (Drizzle Sauce)")
            print("============================================================")
            
            # JSON 구조에서 sauce_id 파싱 (ex: 'tomato_sauce')
            if order_tasks:
                sauce_id = order_tasks[0].get('sauce_id', None)
                
                if not sauce_id:
                    print("   👉 선택된 소스가 없습니다. 바로 서빙으로 넘어갑니다.")
                else:
                    print(f"\n▶ [소스] '{sauce_id}' 용기 잡고 뿌리기 시퀀스 시작!")
                    
                    # 🚨 [TODO] 소스별 좌표 세팅 (직접 티칭해서 값을 채워주세요!)
                    if sauce_id == 'tomato_sauce':
                        pos_sauce_approach = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 소스통 위쪽 안전 경유지
                        pos_sauce_grab     = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 소스통 정확히 잡는 위치
                        pos_sauce_wp       = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 감자칩 용기 위 안전 경유지
                        pos_sauce_drizzle  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 소스 뿌리는 액션 시작 위치
                    else:
                        print(f"⚠️ '{sauce_id}'은(는) 알 수 없는 소스입니다. 패스합니다.")
                        sauce_id = None # 알 수 없는 소스면 아래 동작을 실행하지 않기 위함
                        
                    if sauce_id:
                        # 9-1. 소스통 위 경유지 접근 및 그리퍼 열기 (놓기 활용)
                        print("   >>> 1) 소스통 경유지 접근 및 그리퍼 오픈")
                        future = controller.send_task(pos_sauce_approach, task_type=2)
                        rclpy.spin_until_future_complete(controller, future)
                        future.result().get_result_async()
                        time.sleep(1.0)
                        
                        # 9-2. 소스통 잡기
                        print("   >>> 2) 소스통 그립(Grip)")
                        future = controller.send_task(pos_sauce_grab, task_type=1)
                        rclpy.spin_until_future_complete(controller, future)
                        future.result().get_result_async()
                        time.sleep(1.0)
                        
                        # 9-3. 감자칩 용기 위 경유지로 이동 (단순 이동 task=0)
                        print("   >>> 3) 감자칩 위 경유지로 이동")
                        future = controller.send_task(pos_sauce_wp, task_type=0)
                        rclpy.spin_until_future_complete(controller, future)
                        future.result().get_result_async()
                        time.sleep(1.0)
                        
                        # 9-4. 소스 뿌리기 (새로운 task_type=7 사용)
                        print("   >>> 4) 소스 뿌리기(Drizzle) 액션 실행")
                        future = controller.send_task(pos_sauce_drizzle, task_type=8)
                        rclpy.spin_until_future_complete(controller, future)
                        future.result().get_result_async()
                        time.sleep(2.0)
                        
                        # 9-5. 다시 제자리 경유지 이동 (단순 이동 task=0)
                        print("   >>> 5) 제자리 경유지로 복귀")
                        future = controller.send_task(pos_sauce_approach, task_type=0)
                        rclpy.spin_until_future_complete(controller, future)
                        future.result().get_result_async()
                        time.sleep(1.0)
                        
                        # 9-6. 소스통 제자리에 내려놓기 (놓기 task=2)
                        print("   >>> 6) 소스통 내려놓기")
                        future = controller.send_task(pos_sauce_grab, task_type=2)
                        rclpy.spin_until_future_complete(controller, future)
                        future.result().get_result_async()
                        time.sleep(1.0)

                        # 9-7. 안전하게 허공 경유지로 빠져나오기 (단순 이동 task=0)
                        print("   >>> 7) 안전 경유지로 후퇴")
                        future = controller.send_task(pos_sauce_approach, task_type=0)
                        rclpy.spin_until_future_complete(controller, future)
                        future.result().get_result_async()

                        print(f"✅ '{sauce_id}' 소스 뿌리기 완벽하게 종료!\n")

            # ============================================================
            # 🌟 Step 10. (플로우차트 마무리) 서빙 위치로 이동
            # ============================================================
            print("\n============================================================")
            print("Step 10. 완성된 감자칩 서빙하기")
            print("============================================================")

            # 🚨 [TODO] 서빙 관련 좌표 세팅 (직접 티칭해서 값을 채워주세요!)
            pos_serve_approach = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 완성된 용기 위쪽 안전 접근/후퇴 경유지
            pos_serve_grab     = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 완성된 용기 정확히 잡는 위치
            pos_serve_wp_mid   = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 조리대 -> 서빙대로 넘어가는 중간 경유지 (높게 띄워서)
            pos_serve_wp_drop  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 고객 앞 서빙 테이블 위쪽 안전 경유지
            pos_serve_drop     = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 서빙 테이블에 딱 내려놓는 위치
            pos_home           = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 주문 대기 기본 자세 (Home)

            # 10-1. 완성된 용기 위 경유지 접근 및 그리퍼 열기 (놓기 task=2)
            print("   >>> 1) 완성된 용기 접근 (그리퍼 오픈)")
            future = controller.send_task(pos_serve_approach, task_type=2)
            rclpy.spin_until_future_complete(controller, future)
            future.result().get_result_async()
            time.sleep(1.0)

            # 10-2. 완성된 용기 꽉 잡기 (잡기 task=1)
            print("   >>> 2) 완성된 용기 그립(Grip)")
            future = controller.send_task(pos_serve_grab, task_type=1)
            rclpy.spin_until_future_complete(controller, future)
            future.result().get_result_async()
            time.sleep(1.0)

            # 10-3. 똑바로 위로 들어 올리기 (안전 경유지 task=0)
            print("   >>> 3) 용기 들어올리기 (경유지)")
            future = controller.send_task(pos_serve_approach, task_type=0)
            rclpy.spin_until_future_complete(controller, future)
            future.result().get_result_async()
            time.sleep(1.0)

            # 10-4. 서빙 구역으로 이동 (중간 경유지 task=0)
            # (튀김기나 다른 구조물에 부딪히지 않도록 높은 궤적으로 설정하는 것이 좋습니다)
            print("   >>> 4) 서빙 구역으로 크게 이동 (중간 경유지)")
            future = controller.send_task(pos_serve_wp_mid, task_type=0)
            rclpy.spin_until_future_complete(controller, future)
            future.result().get_result_async()
            time.sleep(1.0)

            # 10-5. 서빙 테이블 바로 위 도착 (경유지 task=0)
            print("   >>> 5) 서빙 테이블 위쪽 도착 (경유지)")
            future = controller.send_task(pos_serve_wp_drop, task_type=0)
            rclpy.spin_until_future_complete(controller, future)
            future.result().get_result_async()
            time.sleep(1.0)

            # 10-6. 서빙 테이블에 용기 내려놓기 (놓기 task=2)
            print("   >>> 6) 고객 앞 서빙 위치에 용기 내려놓기")
            future = controller.send_task(pos_serve_drop, task_type=2)
            rclpy.spin_until_future_complete(controller, future)
            future.result().get_result_async()
            time.sleep(1.0)

            # 10-7. 빈손으로 서빙 테이블 위로 안전하게 빠져나오기 (경유지 task=0)
            print("   >>> 7) 서빙 완료! 빈손으로 안전하게 후퇴")
            future = controller.send_task(pos_serve_wp_drop, task_type=0)
            rclpy.spin_until_future_complete(controller, future)
            future.result().get_result_async()
            time.sleep(1.0)

            # 10-8. 다음 주문을 받을 기본 대기 자세로 이동 (경유지 task=0)
            print("   >>> 8) 대기(Home) 위치로 복귀")
            future = controller.send_task(pos_home, task_type=0)
            rclpy.spin_until_future_complete(controller, future)
            future.result().get_result_async()
            time.sleep(1.0)

            print("\n🎉 모든 서빙이 완료되었습니다! (맛있게 드세요!)")

            print("\n🏁 한 세트 조리 완료! 다음 주문을 대기합니다.")
            # 루프 끝. 다시 while문의 처음(주문 대기)으로 돌아갑니다.


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