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

        self.status_pub = self.create_publisher(String, '/robot_status', 10)

        self.order_received = False
        self.current_order_data = {}

        self.subscription = self.create_subscription(
            String,
            '/taco_order', 
            self.order_callback,
            10
        )
        self.get_logger().info("🎧 '/taco_order' 토픽 구독 시작. 주문 대기 중...")

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        print(f"\n 📢 [웹 UI 전송] : {text}")

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

    def run_task_sync(target_pos, t_type, wait_time=0.5):
        future = controller.send_task(target_pos, task_type=t_type)
        if future is None: return False
        
        # 1. 서버가 명령을 접수할 때까지 대기
        rclpy.spin_until_future_complete(controller, future)
        goal_handle = future.result()
        
        # 2. 서버가 수락했으면, 액션이 '완전히 끝날 때'까지 대기
        if goal_handle.accepted:
            res_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(controller, res_future)
            time.sleep(wait_time) # 동작 완료 후 안정화 대기
            return True
        return False

    try:
        # 🚨 [핵심] rclpy가 살아있는 동안 계속 반복 (무한 루프)
        while rclpy.ok():
            print("\n" + "="*50)
            print("🍟 감자튀김 자동 연속 조리 모드 실행 중...")
            print(" (🛑 중지하려면 터미널에서 Ctrl + C 를 누르세요)")
            print("="*50)

            controller.publish_status("주문을 대기하고 있습니다.")
            while rclpy.ok() and not controller.order_received:
                rclpy.spin_once(controller, timeout_sec=0.1)
            
            controller.order_received = False

            # 🌟 [수정 1] JSON 데이터에서 현재 주문의 첫 번째 task 정보 추출
            order_tasks = controller.current_order_data.get('tasks', [])
            if not order_tasks:
                print("❌ 에러: 주문 데이터에 'tasks' 배열이 없습니다.")
                continue

            current_task = order_tasks[0]
            chip_id = current_task.get('chip_id', 'basic') # 값이 없으면 'basic'으로 간주

            # 🌟 [수정 2] chip_id에 따라 반복 횟수 결정 (double이면 2번, 아니면 1번)
            loop_count = 2 if chip_id == 'double' else 1

            print(f"\n=== [Scenario] 🍟 감자튀김 요리 프로세스 시작 (사이즈: {chip_id.upper()}) ===")

            # ============================================================
            # 🌟 [수정 3] Step 1 ~ Step 3를 loop_count 만큼 반복
            # ============================================================
            for i in range(loop_count):
                current_turn = i + 1
                print(f"\n" + "-"*40)
                print(f"🔄 감자튀김 투입 사이클 {current_turn} / {loop_count}")
                print("-"*40)

                # ------------------------------------------------------------
                # Step 1. 소분된 용기를 잡는다 (Task 1)
                # ------------------------------------------------------------
                controller.publish_status(f"[{current_turn}/{loop_count}] 번째 용기 집는 중입니다.")
                print(f"\n▶ Step 1-1: {current_turn}번째 용기 근처(안전 경유지)로 이동하며 그리퍼 열기")
                
                pos_approach = [50.66, 30.68, 51.24, 0.21, 97.43, 0.04] 
                pos_scooper = [42.62, 49.44, 68.33, 4.33, 66.55, -10.6]

                future = controller.send_task(pos_approach, task_type=2)
                rclpy.spin_until_future_complete(controller, future)
                
                goal_handle = future.result()
                if goal_handle.accepted:
                    res_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(controller, res_future)
                    print(f"✅ {current_turn}번째 경유지 도착 및 그리퍼 오픈 완료!")
                
                time.sleep(0.3)

                print(f"\n▶ Step 1-2: {current_turn}번째 소분된 용기를 잡는다")
                future = controller.send_task(pos_scooper, task_type=1)
                rclpy.spin_until_future_complete(controller, future)
                
                goal_handle = future.result()
                if goal_handle.accepted:
                    res_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(controller, res_future)
                    print(f"✅ {current_turn}번째 용기 잡기 완료!\n")
                
                time.sleep(0.3)

                # ------------------------------------------------------------
                # Step 2. 용기에 담긴 감자를 튀김트레이에 붓는다 (Task 3)
                # ------------------------------------------------------------
                controller.publish_status(f"[{current_turn}/{loop_count}] 번째 감자를 튀김 트레이에 붓는 중입니다.")
                print(f"\n▶ Step 2: 튀김트레이 경유지를 거쳐 붓기 위치로 이동 (블렌딩)")
                
                # 붓는 위치는 동일하므로 공통 좌표 사용
                pos_waypoint_pour = [43.08, 31.77, 29.97, 3.24, 114.69, -8.73] 
                pos_pour_potato = [-20.41, 30.97, 47.87, 27.2, 94.64, -19.27] 
                combined_pour_data = pos_waypoint_pour + pos_pour_potato
                
                future = controller.send_task(combined_pour_data, task_type=3)
                rclpy.spin_until_future_complete(controller, future)

                goal_handle = future.result()
                if goal_handle.accepted:
                    res_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(controller, res_future)
                    print("✅ 멈춤 없는 스무스한 이동 및 감자 붓기 완료!")
                time.sleep(2.0)

                # ------------------------------------------------------------
                # Step 3. 다 부은 빈 용기를 지정된 위치에 놓는다 (Task 2)
                # ------------------------------------------------------------
                controller.publish_status(f"[{current_turn}/{loop_count}] 빈 용기를 반납합니다.")
                print(f"\n▶ Step 3-1: 튀김트레이 위 안전 경유지로 이동")
                
                if current_turn == 1:
                    # [1번째 빈 용기 반납 좌표] (기존 좌표)
                    pos_waypoint_drop = [25.82, -15.82, 87.7, 11.99, 92.98, -8.73] 
                    pos_place_container = [13.66, -5.07, 115.55, 2.69, 67.45, -31.08] 
                else:
                    # 🚨 [2번째 빈 용기 반납 좌표] (TODO: 두 번째 용기 놓을 곳을 새로 티칭해서 넣어야 합니다!)
                    pos_waypoint_drop = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0] 
                    pos_place_container = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0] 

                future_wp = controller.send_task(pos_waypoint_drop, task_type=0)
                rclpy.spin_until_future_complete(controller, future_wp)

                wp_handle = future_wp.result()
                if wp_handle.accepted:
                    wp_res_future = wp_handle.get_result_async()
                    rclpy.spin_until_future_complete(controller, wp_res_future)
                    print(f"✅ {current_turn}번째 놓기 경유지 도착 완료!")

                time.sleep(1.0) 
                
                print(f"\n▶ Step 3-2: {current_turn}번째 빈 용기를 지정된 위치에 내려놓는다")
                future = controller.send_task(pos_place_container, task_type=2)
                rclpy.spin_until_future_complete(controller, future)

                goal_handle = future.result()
                if goal_handle.accepted:
                    res_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(controller, res_future)
                    print(f"✅ {current_turn}번째 용기 내려놓기 완료!")
                time.sleep(2.0)

            # ============================================================
            # Step 4. 튀김트레이를 흔든다 (Task 4)
            # ============================================================
            controller.publish_status("감자가 맛있게 튀겨지도록 흔드는 중입니다.")
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
            pos_waypoint = [2.51, 24.47, 67.4, -1.07, 83.89, -84.59] 
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
            controller.publish_status("기름을 터는 중입니다")
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
            pos_waypoint = [2.15, 19.66, 53.18, -2.26, 92.13, -87.44] 
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
            controller.publish_status("튀겨진 감자칩을 용기에 붓는 중입니다")
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
            # Step 9. 추가 재료(Toppings) 전용 스쿠퍼 잡고 투입
            # ============================================================
            controller.publish_status("주문하신 토핑을 확인합니다.")
            print("\n============================================================")
            print("Step 9. 추가 재료(Toppings) 스쿱(Scoop)")
            print("============================================================")

            topping_kr_map = {
                'cabbage': '양배추',
                'tomato': '토마토',
                'onion': '양파'
            }
            
            order_tasks = controller.current_order_data.get('tasks', [])
            if order_tasks:
                topping_ids = order_tasks[0].get('topping_ids', [])
                print("\n▶ [준비] 안전 구역 공통 접근")
                pos_pre_1 = [2.28, 15.08, 73.07, -2.05, 69.63, -86.94]
                pos_pre_2 = [0.0, 1.11, 76.85, 0.06, 101.96, -0.02]
                
                run_task_sync(pos_pre_1, 0)
                run_task_sync(pos_pre_2, 0)
                
                if not topping_ids:
                    controller.publish_status("추가 토핑 선택이 없으므로 소스 확인으로 넘어갑니다.")
                    print("   👉 추가 선택 재료가 없습니다. 바로 서빙으로 넘어갑니다.")
                else:
                    # --------------------------------------------------------
                    # 9-B. 각 재료별 독립 시퀀스 (잡기 -> 스쿱 -> 붓기 -> 반납)
                    # --------------------------------------------------------
                    for topping in topping_ids:
                        topping_kr = topping_kr_map.get(topping, topping) # 한글 이름 가져오기
                        controller.publish_status(f"{topping_kr} 추가하는 중입니다.") # 🌟 UI 전송
                        print(f"\n▶ [추가 재료] '{topping}' 전용 스쿠퍼 시퀀스 시작!")
                        
                        if topping == 'cabbage':
                            # --- 1. 양배추 스쿠퍼 잡기 ---
                            print("   >>> [1/4] 양배추 스쿠퍼 잡으러 이동 중...")
                            pos_cab_appr = [-26.82, 18.03, 57.39, 0.23, 104.54, -26.75] 
                            pos_cab_grab = [-52.63, 44.38, 46.54, 27.91, 116.43, -46.02] 
                            pos_cab_out1 = [-51.41, 42.92, 46.46, 28.42, 117.79, -46.01] 
                            pos_cab_out2 = [-46.16, 35.33, 51.53, 28.08, 118.82, -40.18] 
                            
                            run_task_sync(pos_cab_appr, 0)
                            run_task_sync(pos_cab_grab, 1, wait_time=1.0) # 🌟 Grip!
                            run_task_sync(pos_cab_out1, 0)
                            run_task_sync(pos_cab_out2, 0)

                            # --- 2. 양배추 스쿱 동작 ---
                            print("   >>> [2/4] 양배추 스쿱(Scoop) 동작 실행")
                            cabbage_scoop_data = [
                                419.65, -55.58, 264.27, 83.22, 150.94, 81.51,
                                435.05, -172.38, 267.64, 46.01, 179.87, 46.39,
                                439.9, -235.13, 266.77, 83.45, 157.49, 88.46,
                                435.98, -246.13, 289.73, 79.46, 142.07, 82.45,
                                440.46, -318.82, 232.56, 79.4, 151.28, 87.55,
                                431.9, -333.83, 176.44, 111.37, -170.33, 115.48,
                                431.89, -327.35, 182.09, 111.37, -170.33, 115.48,
                                431.89, -333.89, 192.02, 111.37, -170.33, 115.49,
                                431.89, -195.27, 196.06, 111.36, -170.32, 115.48
                            ]
                            run_task_sync(cabbage_scoop_data, 6, wait_time=1.0)
                            
                            # --- 3. 용기에 붓기 ---
                            print("   >>> [3/4] 감자칩 용기에 커스텀 붓기 (5단계 + 흔들기)")
                            
                            # 알려주신 5개의 posj (조인트 좌표)를 하나로 합칩니다.
                            custom_pour_data = [
                                -7.45, 1.08, 76.76, -11.9, 100.0, -6.08,     # 1번
                                19.29, -9.53, 104.48, -10.77, 76.22, -6.33,  # 2번
                                39.69, 6.74, 96.6, -15.55, 78.29, 35.44,     # 3번
                                17.82, -1.18, 102.41, 22.97, 70.77, 1.77,    # 4번
                                24.18, -7.34, 78.13, -1.52, 101.08, 24.95    # 5번
                            ]
                            # 한 번에 전송 (task_type=10)
                            run_task_sync(custom_pour_data, 10, wait_time=1.0)

                            # --- 4. 스쿠퍼 반납 (역순) ---
                            print("   >>> [4/4] 양배추 스쿠퍼 반납 중...")
                            run_task_sync(pos_cab_out2, 0)
                            run_task_sync(pos_cab_out1, 0)
                            run_task_sync(pos_cab_grab, 2, wait_time=1.0) # 🌟 Release!
                            run_task_sync(pos_cab_appr, 0)
                            run_task_sync(pos_pre_2, 0) # 공통 대기 장소로 원복
                            
                            print("✅ 'cabbage' 시퀀스 완벽 종료!\n")

                        elif topping == 'tomato':
                            # --- 1. 토마토 스쿠퍼 잡기 ---
                            print("   >>> [1/4] 토마토 스쿠퍼 잡으러 이동 중...")
                            # 🚨 [TODO] 토마토 스쿠퍼 잡기 좌표 티칭 필요
                            pos_tom_appr = [-31.25, 7.15, 71.62, 0.35, 101.55, -31.21] # 위에서 맞추기
                            pos_tom_grab = [-57.81, 37.45, 58.56, 24.29, 114.14, -53.66] # 잡는 위치
                            pos_tom_out1 = [-53.9, 22.19, 69.96, 27.21, 114.57, -53.21] # 나가기 1
                            pos_tom_out2 = [-53.9, 22.18, 69.96, 27.21, 114.56, -53.2] # 나가기 2
                            
                            run_task_sync(pos_tom_appr, 0)
                            run_task_sync(pos_tom_grab, 1, wait_time=1.0) # 🌟 Grip!
                            run_task_sync(pos_tom_out1, 0)
                            run_task_sync(pos_tom_out2, 0)

                            # --- 2. 토마토 스쿱 동작 ---
                            print("   >>> [2/4] 토마토 스쿱(Scoop) 동작 실행 (9 points)")
                            tomato_scoop_data = [
                                340.79, -55.58, 264.27, 83.22, 150.94, 81.51,
                                358.45, -172.38, 267.64, 46.01, 179.87, 46.39,
                                363.3, -235.13, 266.77, 83.45, 157.49, 88.46,
                                359.38, -246.13, 289.73, 79.46, 142.07, 82.45,
                                363.88, -318.82, 229.78, 79.4, 151.28, 87.55,
                                355.31, -333.83, 175.59, 111.38, -170.33, 115.49,
                                355.29, -327.35, 186.83, 111.37, -170.33, 115.48,
                                355.28, -152.38, 186.83, 111.35, -170.32, 115.47,
                                355.28, -20.28, 182.1, 111.35, -170.32, 115.47
                            ]
                            run_task_sync(tomato_scoop_data, 6, wait_time=1.0)
                            
                            # --- 3. 용기에 붓기 ---
                            print("   >>> [3/4] 감자칩 용기에 커스텀 붓기 (5단계 + 흔들기)")
                            custom_pour_data = [
                                -7.45, 1.08, 76.76, -11.9, 100.0, -6.08,
                                19.29, -9.53, 104.48, -10.77, 76.22, -6.33,
                                39.69, 6.74, 96.6, -15.55, 78.29, 35.44,
                                17.82, -1.18, 102.41, 22.97, 70.77, 1.77,
                                24.18, -7.34, 78.13, -1.52, 101.08, 24.95
                            ]
                            run_task_sync(custom_pour_data, 10, wait_time=1.0)

                            # --- 4. 스쿠퍼 반납 (역순) ---
                            print("   >>> [4/4] 토마토 스쿠퍼 반납 중...")
                            run_task_sync(pos_tom_out2, 0)
                            run_task_sync(pos_tom_out1, 0)
                            run_task_sync(pos_tom_grab, 2, wait_time=1.0) # 🌟 Release!
                            run_task_sync(pos_tom_appr, 0)
                            run_task_sync(pos_pre_2, 0) # 공통 대기 장소로 원복
                            
                            print("✅ 'tomato' 시퀀스 완벽 종료!\n")
                            
                        elif topping == 'onion':
                            # --- 1. 양파 스쿠퍼 잡기 ---
                            print("   >>> [1/4] 양파 스쿠퍼 잡으러 이동 중...")
                            # 🚨 [TODO] 양파 스쿠퍼 잡기 좌표 티칭 필요
                            pos_oni_appr = [-37.31, -1.21, 80.96, 0.41, 100.96, -37.24] # 위에서 맞추기
                            pos_oni_grab = [-63.91, 31.87, 67.71, 20.38, 113.12, -61.27] # 잡는 위치
                            pos_oni_out1 = [-60.97, 11.65, 82.39, 24.15, 113.28, -61.27] # 나가기 1
                            pos_oni_out2 = [-60.97, 11.61, 82.39, 24.15, 113.26, -61.25] # 나가기 2
                            
                            run_task_sync(pos_oni_appr, 0)
                            run_task_sync(pos_oni_grab, 1, wait_time=1.0) # 🌟 Grip!
                            run_task_sync(pos_oni_out1, 0)
                            run_task_sync(pos_oni_out2, 0)

                            # --- 2. 양파 스쿱 동작 ---
                            print("   >>> [2/4] 양파 스쿱(Scoop) 동작 실행 (9 points)")
                            onion_scoop_data = [
                                268.69, -55.58, 264.27, 83.22, 150.94, 81.51,
                                286.35, -172.38, 267.64, 46.01, 179.87, 46.39,
                                291.2, -235.13, 266.77, 83.45, 157.49, 88.46,
                                287.28, -246.13, 289.73, 79.46, 142.07, 82.45,
                                284.82, -318.82, 227.84, 79.4, 151.28, 87.55,
                                278.07, -333.83, 173.1, 111.38, -170.33, 115.49,
                                287.01, -333.82, 184.4, 111.38, -170.33, 115.49,
                                281.57, -321.11, 191.61, 111.37, -170.33, 115.48,
                                281.56, -199.76, 191.63, 111.38, -170.33, 115.48
                            ]
                            run_task_sync(onion_scoop_data, 6, wait_time=1.0)
                            
                            # --- 3. 용기에 붓기 ---
                            print("   >>> [3/4] 감자칩 용기에 커스텀 붓기 (5단계 + 흔들기)")
                            custom_pour_data = [
                                -7.45, 1.08, 76.76, -11.9, 100.0, -6.08,
                                19.29, -9.53, 104.48, -10.77, 76.22, -6.33,
                                39.69, 6.74, 96.6, -15.55, 78.29, 35.44,
                                17.82, -1.18, 102.41, 22.97, 70.77, 1.77,
                                24.18, -7.34, 78.13, -1.52, 101.08, 24.95
                            ]
                            run_task_sync(custom_pour_data, 10, wait_time=1.0)

                            # --- 4. 스쿠퍼 반납 (역순) ---
                            print("   >>> [4/4] 양파 스쿠퍼 반납 중...")
                            run_task_sync(pos_oni_out2, 0)
                            run_task_sync(pos_oni_out1, 0)
                            run_task_sync(pos_oni_grab, 2, wait_time=1.0) # 🌟 Release!
                            run_task_sync(pos_oni_appr, 0)
                            # run_task_sync(pos_pre_2, 0) # 공통 대기 장소로 원복
                            
                            print("✅ 'onion' 시퀀스 완벽 종료!\n")
                        else:
                            print(f"⚠️ '{topping}'은(는) 알 수 없는 재료입니다. 패스합니다.")
                            continue

            # ============================================================
            # 🌟 Step 10. 서빙 및 소스 뿌리기 퍼포먼스 (마무리)
            # ============================================================
            controller.publish_status("서빙존으로 용기를 이동중입니다.") 
            print("\n============================================================")
            print("Step 10. 완성된 감자칩 서빙 및 소스 뿌리기")
            print("============================================================")

            # --------------------------------------------------------
            # 10-A. 먼저 손님 앞(서빙 위치)으로 감자칩 배달
            # --------------------------------------------------------
            print("\n▶ [서빙] 완성된 용기를 서빙 존으로 이동")
            pos_serve_1_grip = [12.6, -7.32, 120.14, 2.77, 65.15, -31.92]   # 1번: 잡기
            pos_serve_2_wp   = [9.71, -5.27, 61.94, 3.29, 121.23, -31.84]   # 2번: 이동
            pos_serve_3_wp   = [-65.04, -31.98, 89.65, 1.96, 111.93, -31.84]# 3번: 이동
            pos_serve_4_wp   = [-88.83, -14.95, 91.82, -7.37, 102.0, -31.84]# 4번: 이동
            pos_serve_5_drop = [-95.53, 21.63, 72.16, 2.1, 82.41, -20.47]  # 5번: 놓기

            print("   >>> 1) 완성된 용기 잡기 (Grip)")
            run_task_sync(pos_serve_1_grip, 1, wait_time=1.0)

            print("   >>> 2) 서빙 구역으로 이동 중...")
            run_task_sync(pos_serve_2_wp, 0)
            run_task_sync(pos_serve_3_wp, 0)
            run_task_sync(pos_serve_4_wp, 0)

            print("   >>> 3) 고객 앞 서빙 위치에 용기 내려놓기 (Drop)")
            run_task_sync(pos_serve_5_drop, 2, wait_time=1.0)
            
            # 서빙 후 빈손으로 일단 안전하게 위로 후퇴 (소스 잡으러 가기 위함)
            print("   >>> 4) 용기 내려놓고 안전 구역으로 후퇴")
            run_task_sync(pos_serve_4_wp, 0)
            run_task_sync(pos_serve_3_wp, 0)

            # --------------------------------------------------------
            # 10-B. 서빙된 용기 위에서 소스 뿌리기 퍼포먼스
            # --------------------------------------------------------
            controller.publish_status("소스 뿌리기를 시작합니다.") 
            if order_tasks:
                sauce_id = order_tasks[0].get('sauce_id', None)
                draw_path = order_tasks[0].get('draw_path', None)

                if sauce_id == 'mustard':
                    controller.publish_status("머스타드 뿌리는 중입니다")
                elif sauce_id == 'tomato_sauce':
                    controller.publish_status("케찹 뿌리는 중입니다")
                
                if not sauce_id:
                    print("   👉 선택된 소스가 없습니다. 서빙을 완료합니다.")
                else:
                    print(f"\n▶ [소스] '{sauce_id}' 용기 잡고 뿌리기 시퀀스 시작!")
                    
                    if sauce_id == 'tomato_sauce':
                        # 🍅 토마토 소스 15단계 좌표 (정수에 .0 추가 완료)
                        t1  = [-9.41, -16.46, 91.71, -73.15, 82.61, -19.84]
                        t2  = [-54.88, 0.67, 88.44, -58.15, 32.61, -20.95]
                        t3  = [-55.48, 13.21, 78.47, -64.72, 38.03, -19.15]
                        t4  = [-55.54, 16.62, 52.63, -45.63, 51.08, -45.5]
                        t5  = [-82.24, 17.36, 46.76, -36.9, 45.68, -50.4]
                        t6  = [-82.25, 17.35, 46.76, -36.9, 45.68, -50.43]
                        t7  = [-82.27, 17.37, 46.77, -36.91, 45.69, -50.44]
                        t8  = [-90.29, -20.06, 102.75, -21.93, 37.25, -64.53]
                        t9  = [-80.12, 14.13, 66.48, -32.31, 61.76, -54.27]
                        t10 = [-82.36, -6.31, 102.97, -59.17, 31.8, -213.59]
                        t11 = [-82.24, 17.36, 46.76, -36.9, 45.68, -50.4]
                        t12 = [-58.3, 13.6, 58.6, -37.71, 43.84, -46.24]
                        t13 = [-59.34, 8.59, 82.0, -54.4, 30.14, -25.42]
                        t14 = [-54.88, 0.67, 88.44, -58.15, 32.61, -20.95]
                        t15 = [-9.41, -16.46, 91.71, -73.15, 82.61, -19.84]

                        print("   >>> 1) 토마토 소스통 위로 접근")
                        run_task_sync(t1, 0)
                        run_task_sync(t2, 0)
                        
                        print("   >>> 2) 그립 위치 도착 및 일반 그립!")
                        run_task_sync(t3, 1, wait_time=1.0) 
                        
                        print("   >>> 3) 감자튀김 중앙 상공으로 이동 중...")
                        run_task_sync(t4, 0)
                        run_task_sync(t5, 0)
                        run_task_sync(t6, 0)
                        run_task_sync(t7, 0)
                        run_task_sync(t8, 0)
                        run_task_sync(t9, 0)
                        
                        # 🌟 t10번에 도착 (그리기 시작점) - 복구 완료!
                        run_task_sync(t10, 0, wait_time=1.0) 
                        
                        print("   >>> 4) 좌표에 맞춰 커스텀 그리기 시작!")
                        if draw_path:
                            flat_path_data = []
                            for pt in draw_path:
                                flat_path_data.append(float(pt['xasDouble']))
                                flat_path_data.append(float(pt['yasDouble']))
                            run_task_sync(flat_path_data, 8, wait_time=2.0)
                        else:
                            print("   👉 (draw_path 없음) 기본 지그재그 실행")
                            run_task_sync([], 8, wait_time=2.0) 
                        
                        print("   >>> 5) 소스통 제자리로 복귀 중...")
                        run_task_sync(t11, 0)
                        run_task_sync(t12, 0)

                        print("   >>> 6) 지정된 위치에 소스통 내려놓기")
                        run_task_sync(t13, 2, wait_time=1.0) 
                        
                        print("   >>> 7) 안전하게 후퇴")
                        run_task_sync(t14, 0)
                        run_task_sync(t15, 0)

                        print(f"✅ '{sauce_id}' 퍼포먼스 완벽하게 종료!\n")
                        
                    elif sauce_id == 'mustard':
                        # 🌭 머스타드 소스 15단계 좌표 (정수에 .0 추가 완료)
                        m1  = [-2.85, 5.44, 59.44, -46.58, 80.13, -0.36]
                        m2  = [-38.83, 25.49, 55.79, -59.48, 56.98, -27.36]
                        m3  = [-42.31, 29.05, 53.97, -58.8, 53.13, -27.68]
                        m4  = [-42.47, 43.0, 7.61, -45.42, 73.44, -56.28]
                        m5  = [-9.05, 7.76, 60.16, -63.9, 84.82, -20.48]
                        m6  = [-76.71, -0.88, 42.53, -30.28, 105.75, -54.37]
                        m7  = [-80.12, 14.13, 66.48, -32.31, 61.76, -54.27]
                        m8  = [-82.36, -6.31, 102.97, -59.17, 31.8, -213.59]
                        m9  = [-80.12, 14.13, 66.48, -32.31, 61.76, -54.27]
                        m10 = [-76.71, -0.88, 42.53, -30.28, 105.75, -54.37]
                        m11 = [-9.05, 7.76, 60.16, -63.9, 84.82, -20.48]
                        m12 = [-42.47, 43.0, 7.61, -45.42, 73.44, -56.28]
                        m13 = [-41.54, 27.24, 55.53, -59.04, 53.92, -27.46]
                        m14 = [-38.83, 25.49, 55.79, -59.48, 56.98, -27.36]
                        m15 = [-2.85, 5.44, 59.44, -46.58, 80.13, -0.36]

                        print("   >>> 1) 머스타드 소스통 위로 접근")
                        run_task_sync(m1, 0)
                        run_task_sync(m2, 0)
                        
                        print("   >>> 2) 그립 위치 도착 및 일반 그립!")
                        run_task_sync(m3, 1, wait_time=1.0) 
                        
                        print("   >>> 3) 감자튀김 중앙 상공으로 이동 중...")
                        run_task_sync(m4, 0)
                        run_task_sync(m5, 0)
                        run_task_sync(m6, 0)
                        run_task_sync(m7, 0)
                        
                        # 🌟 m8번에 도착 (그리기 시작점)
                        run_task_sync(m8, 0, wait_time=1.0)
                        
                        print("   >>> 4) 좌표에 맞춰 커스텀 그리기 시작!")
                        if draw_path:
                            flat_path_data = []
                            for pt in draw_path:
                                flat_path_data.append(float(pt['xasDouble']))
                                flat_path_data.append(float(pt['yasDouble']))
                            run_task_sync(flat_path_data, 8, wait_time=2.0)
                        else:
                            print("   👉 (draw_path 없음) 기본 지그재그 실행")
                            run_task_sync([], 8, wait_time=2.0) 
                            
                        print("   >>> 5) 소스통 제자리로 복귀 중...")
                        run_task_sync(m9, 0)
                        run_task_sync(m10, 0)
                        run_task_sync(m11, 0)
                        run_task_sync(m12, 0)
                        
                        print("   >>> 6) 지정된 위치에 소스통 내려놓기")
                        run_task_sync(m13, 2, wait_time=1.0) 
                        
                        print("   >>> 7) 안전하게 후퇴")
                        run_task_sync(m14, 0)
                        run_task_sync(m15, 0)

                        print(f"✅ '{sauce_id}' 퍼포먼스 완벽하게 종료!\n")
                        
                    else:
                        print(f"⚠️ '{sauce_id}'은(는) 알 수 없는 소스입니다. 패스합니다.")

            # --------------------------------------------------------
            # 10-C. 기본 대기 위치(Home)로 복귀 및 마무리
            # --------------------------------------------------------
            print("\n▶ [마무리] 다음 주문 대기 위치로 복귀")
            # 🚨 [TODO] 다음 주문을 기다릴 가장 안전하고 멋진 대기 자세(Home) 좌표를 넣어주세요.
            pos_home = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0] 
            run_task_sync(pos_home, 0, wait_time=1.0)

            # 🌟 UI에 완료 메시지 쏘기
            controller.publish_status("타코가 완성 되었습니다. 서빙 존에서 받아가세요! 맛있게 드세요 ^^")
            
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