import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from tacobot_interfaces.action import RobotTask
from std_msgs.msg import String, Bool, Float64MultiArray, Int32
import json

class TaskController(Node):
    def __init__(self):
        super().__init__('task_controller')
        self.cli_universal = ActionClient(self, RobotTask, '/dsr01/action_server')

        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.color_state_pub = self.create_publisher(Int32, '/dsr01/ui_state', 10)

        # 🌟 [수정 1] 단일 데이터 대신, 주문을 차곡차곡 쌓아둘 대기열(Queue) 리스트 생성!
        self.order_queue = []
        self.is_paused = False  # 정지 상태 플래그
        self.jog_joints = None  # 조그 명령 저장 변수

        self.subscription = self.create_subscription(
            String,
            '/taco_order', 
            self.order_callback,
            10
        )

        # 2. 정지 명령 구독 (/dsr01/stop)
        self.stop_sub = self.create_subscription(
            Bool,
            '/dsr01/stop',
            self.stop_callback,
            10)
        
        # 3. 조그 명령 구독 (/dsr01/jog_command)
        self.jog_sub = self.create_subscription(
            Float64MultiArray,
            '/dsr01/jog_command',
            self.jog_callback,
            10)

        self.get_logger().info("🎧 '/taco_order' 토픽 구독 시작. 주문 대기 중...")

    def send_color_state(self, state_num):
        msg = Int32()
        msg.data = state_num
        self.color_state_pub.publish(msg)    

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        print(f"\n 📢 [웹 UI 전송] : {text}")

    def order_callback(self, msg):
        try:
            new_order = json.loads(msg.data)
            self.get_logger().info(f"🔔 새로운 주문 접수됨! (대기열 추가)")
            
            # 🌟 [수정 2] 들어온 주문을 덮어씌우지 않고, 대기열 맨 끝에 추가(append)
            self.order_queue.append(new_order)
            
            # 현재 대기 중인 총 주문 개수를 UI로 알려줄 수도 있습니다!
            print(f"   현재 대기 중인 주문 수: {len(self.order_queue)}건")

        except Exception as e:
            self.get_logger().error(f'에러발생: {str(e)}')

    def send_task(self, joints, task_type):
        # ... (이하 send_task, feedback_callback 등은 기존과 100% 동일) ...
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
        pass

    def stop_callback(self, msg):
        """정지 신호를 받으면 is_paused 상태를 업데이트"""
        self.is_paused = msg.data
        if self.is_paused:
            self.get_logger().warn("🛑 [STOP] 정지 신호 수신! 조그 모드로 전환합니다.")
            self.publish_status("정지 신호 수신 - 조그 이동 가능 상태")
            self.send_color_state(6)
        else:
            self.get_logger().info("▶️ [RESUME] 정지 해제! 작업을 재개합니다.")
            self.publish_status("작업 재개")
            self.send_color_state(2)

    def jog_callback(self, msg):
        """조그 조인트 값을 수신하여 저장 (6개 값 확인)"""
        if len(msg.data) == 6:
            self.jog_joints = list(msg.data)

def main(args=None):
    rclpy.init(args=args)
    controller = TaskController()

    def run_task_sync(target_pos, t_type, wait_time=0.5):
        # 1. 작업을 시작하기 전 정지 상태라면 대기 (조그 모드 활성화)
        check_pause_and_jog()

        future = controller.send_task(target_pos, task_type=t_type)
        if future is None: return False
        
        rclpy.spin_until_future_complete(controller, future)
        goal_handle = future.result()
        
        if goal_handle.accepted:
            res_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(controller, res_future)
            time.sleep(wait_time) 

            # 2. 작업이 끝난 직후 정지 상태라면 대기 (조그 모드 활성화)
            check_pause_and_jog()

            return True
        return False
    
    def check_pause_and_jog():
        """정지 상태일 때 조그 명령을 받아 로봇을 움직이는 핵심 루프"""
        first_entry = True
        while controller.is_paused and rclpy.ok():
            if first_entry:
                print("⏸️ 일시정지 중... 조그 명령(/dsr01/jog_command) 대기 중")
                first_entry = False
            
            # 조그 명령(좌표)이 들어왔다면 movej(task_type 0) 실행
            if controller.jog_joints is not None:
                print(f"🕹️ 조그 이동 수행: {controller.jog_joints}")
                jog_future = controller.send_task(controller.jog_joints, task_type=0)
                if jog_future:
                    rclpy.spin_until_future_complete(controller, jog_future)
                    j_handle = jog_future.result()
                    if j_handle.accepted:
                        j_res = j_handle.get_result_async()
                        rclpy.spin_until_future_complete(controller, j_res)
                
                # 명령 수행 후 초기화 (중복 실행 방지)
                controller.jog_joints = None

            # 토픽 수신을 위해 스핀
            rclpy.spin_once(controller, timeout_sec=0.1)

    try:
        # 🚨 [핵심] rclpy가 살아있는 동안 계속 반복 (무한 루프)
        while rclpy.ok():
            print("\n" + "="*50)
            print("🍟 감자튀김 자동 연속 조리 모드 실행 중...")
            print(" (🛑 중지하려면 터미널에서 Ctrl + C 를 누르세요)")
            print("="*50)

            # 🌟 [수정 3] 대기열(queue)이 비어있으면 계속 기다림
            if len(controller.order_queue) == 0:
                controller.publish_status("주문을 대기하고 있습니다.")
                controller.send_color_state(1)
                while rclpy.ok() and len(controller.order_queue) == 0:
                    rclpy.spin_once(controller, timeout_sec=0.1)

            controller.send_color_state(2)
            
            # 대기열에 주문이 생겼다! 가장 앞에 있는(오래된) 0번 주문서를 뽑아냄!
            current_order_data = controller.order_queue.pop(0)

            # JSON 데이터에서 장바구니 목록(tasks) 추출
            order_tasks = current_order_data.get('tasks', [])
            if not order_tasks:
                print("❌ 에러: 주문 데이터에 'tasks' 배열이 없습니다.")
                continue
            
            total_menus = len(order_tasks)
            # 현재 처리 중인 주문 번호를 표시해주면 더 좋습니다
            order_id = current_order_data.get('order_id', '알수없음')
            print(f"\n🔔 [주문 처리 시작] 주문번호: {order_id} (총 {total_menus}개 메뉴)")
            
            # ============================================================
            # 장바구니에 담긴 메뉴 개수만큼 전체 시나리오 반복!
            # ============================================================
            for task_idx, current_task in enumerate(order_tasks):
                current_menu_num = task_idx + 1
                
                controller.publish_status(f"총 {total_menus}개 중 {current_menu_num}번째 메뉴를 시작합니다.")
                print(f"\n{'#'*50}")
                print(f"🍔 [{current_menu_num} / {total_menus}] 번째 요리 프로세스 시작!")
                print(f"{'#'*50}")

                # 이번 턴의 chip_id에 따라 감자 투입 반복 횟수 결정
                chip_id = current_task.get('chip_id', 'basic') 
                loop_count = 2 if chip_id == 'double' else 1

                print(f"\n=== [Scenario] 🍟 감자튀김 요리 프로세스 (사이즈: {chip_id.upper()}) ===")

                # ------------------------------------------------------------
                # Step 1 ~ Step 3를 loop_count 만큼 반복 (double이면 2번)
                # ------------------------------------------------------------
                for i in range(loop_count):
                    current_turn = i + 1
                    print(f"\n" + "-"*40)
                    print(f"🔄 감자튀김 투입 사이클 {current_turn} / {loop_count}")
                    print("-"*40)

                    # --- Step 1. 소분된 용기를 잡는다 ---
                    controller.publish_status(f"용기를 집는 중입니다.")
                    print(f"\n▶ Step 1-1: {current_turn}번째 용기 근처(안전 경유지)로 이동하며 그리퍼 열기")
                    
                    # 집는 곳은 항상 동일
                    pos_approach = [50.66, 30.68, 51.24, 0.21, 97.43, 0.04] 
                    pos_scooper = [42.59, 48.76, 69.94, 4.37, 65.78, -10.64]

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

                    # --- Step 2. 용기에 담긴 감자를 튀김트레이에 붓는다 ---
                    controller.publish_status(f"감자를 튀김 트레이에 붓는 중입니다.")
                    print(f"\n▶ Step 2: 튀김트레이 경유지를 거쳐 붓기 위치로 이동 (블렌딩)")
                    
                    # 붓는 위치도 동일
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

                    # --- Step 3. 다 부은 빈 용기를 지정된 위치에 놓는다 ---
                    print(f"\n▶ Step 3-1: 튀김트레이 위 안전 경유지로 이동")
                    
                    # 빈 용기를 버리는 위치는 겹치면 안 되므로 분기 처리!
                    if current_turn == 1:
                        # [1번째 빈 용기 반납 좌표]
                        pos_waypoint_drop = [25.82, -15.82, 87.7, 11.99, 92.98, -8.73] 
                        pos_place_container = [13.66, -5.07, 115.55, 2.69, 67.45, -31.08] 
                    else:
                        # 🚨 [2번째 빈 용기 반납 좌표] 
                        pos_waypoint_drop = [59.65, 11.7, 52.43, 6.49, 101.1, -19.27] 
                        pos_place_container = [65.58, 19.12, 102.35, 2.82, 56.54, -17.65] 

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
                # Step 4. 튀김트레이를 흔든다 (Task 4, 5)
                # ============================================================
                controller.publish_status("감자가 맛있게 튀겨지도록 흔드는 중입니다.")
                print("\n▶ Step 4-1 ~ 4-7 경유지 이동")
                run_task_sync([29.15, -6.78, 79.14, 4.16, 104.55, -7.97], 0, 1.0)
                run_task_sync([2.55, 10.11, 71.86, -1.84, 78.37, -87.35], 0, 1.0)
                run_task_sync([2.51, 24.47, 67.4, -1.07, 83.89, -84.59], 1, 1.0)
                run_task_sync([1.11, 13.8, 72.06, 0.22, 86.3, -87.3], 0, 1.0)
                run_task_sync([1.92, 22.69, 40.16, 0.4, 133.09, -87.3], 0, 1.0)
                run_task_sync([1.53, 26.24, 55.19, 0.46, 105.61, -87.31], 0, 1.0)
                run_task_sync([1.53, 33.27, 49.67, 0.07, 108.93, -87.31], 0, 1.0)

                print("\n▶ Step 4-8: 튀김트레이를 잡고 흔든다 (잘 튀겨지게)")
                pos_shake_tray = [1.27, 43.45, 38.39, 0.1, 117.29, -87.31]
                run_task_sync(pos_shake_tray, 5, 2.0)
                print("✅ 튀김트레이 흔들기 완료!")

                # ============================================================
                # Step 5. 튀김트레이를 잡고 기름을 턴다 (Task 7)
                # ============================================================
                controller.publish_status("기름을 터는 중입니다")
                print("\n▶ Step 5-1: 경유지 도착")
                run_task_sync([1.53, 33.27, 49.67, 0.07, 108.93, -87.31], 0, 1.0)

                print("\n▶ Step 5-2: 기름을 턴다 (Drain)")
                drain_p1 = [402.16, 9.82, 459.29, 178.03, -148.43, 89.87] 
                drain_p2 = [470.54, 26.17, 186.83, 8.84, 164.47, -78.25] 
                run_task_sync(drain_p1 + drain_p2, 7, 2.0)
                print("✅ 확실히 털기(Drain) 완료!")

                # ============================================================
                # Step 6. 튀김트레이를 흔들기 탁탁탁 (Task 4)
                # ============================================================
                print("\n▶ Step 6-1: 경유지 도착")
                run_task_sync([2.15, 5.97, 61.84, -0.47, 100.58, -87.31], 0, 1.0)

                print("\n▶ Step 6-2: 탁탁탁 시작!")
                run_task_sync([2.15, 19.75, 52.76, -2.26, 92.46, -87.45], 4, 1.0)

                # ============================================================
                # Step 7. 튀김트레이에 담긴 감자칩을 용기에 붓는다 (Task 3)
                # ============================================================
                controller.publish_status("튀겨진 감자칩을 용기에 붓는 중입니다")
                print("\n▶ Step 7-1: 튀김트레이 위 안전 경유지로 이동")
                run_task_sync([27.61, -25.45, 85.36, 13.7, 96.56, -87.3], 0, 1.0)

                print("\n▶ Step 7-2: 감자칩을 최종 용기에 붓는다")
                run_task_sync([-14.32, -33.3, 122.38, 18.34, 76.87, -111.05], 3, 2.0)
                print("\n✅ 감자칩 붓기 완료!")

                # ============================================================
                # Step 8. 빈 튀김트레이를 제자리에 내려놓는다 (Task 2)
                # ============================================================
                print("\n▶ Step 8-1 ~ 8-3: 경유지 이동")
                run_task_sync([27.6, -6.79, 66.7, 1.72, 117.22, -87.31], 0, 1.0)
                run_task_sync([3.33, 4.83, 78.71, -1.82, 85.91, -87.31], 0, 1.0)
                run_task_sync([0.95, 14.98, 73.94, 0.22, 77.5, -87.31], 0, 1.0)

                print("\n▶ Step 8-4: 튀김트레이를 제자리에 내려놓는다 (놓기)")
                run_task_sync([3.09, 15.17, 81.71, -2.29, 68.22, -86.02], 2, 1.0)
                print("\n✅ 튀김트레이 내려놓기 완료!")

                # ============================================================
                # Step 9. 추가 재료(Toppings) 전용 스쿠퍼 잡고 투입
                # ============================================================
                controller.publish_status("주문하신 토핑을 확인합니다.")
                print("\n============================================================")
                print("Step 9. 추가 재료(Toppings) 스쿱(Scoop)")
                print("============================================================")

                topping_kr_map = {'cabbage': '양배추', 'tomato': '토마토', 'onion': '양파'}
                
                # 🌟 order_tasks[0]이 아니라 현재 current_task 사용!
                topping_ids = current_task.get('topping_ids', [])
                
                print("\n▶ [준비] 안전 구역 공통 접근")
                pos_pre_1 = [2.28, 15.08, 73.07, -2.05, 69.63, -86.94]
                pos_pre_2 = [0.0, 1.11, 76.85, 0.06, 101.96, -0.02]
                run_task_sync(pos_pre_1, 0)
                run_task_sync(pos_pre_2, 0)
                
                if not topping_ids:
                    controller.publish_status("추가 토핑 선택이 없으므로 소스 확인으로 넘어갑니다.")
                    print("   👉 추가 선택 재료가 없습니다. 바로 서빙으로 넘어갑니다.")
                else:
                    for topping in topping_ids:
                        topping_kr = topping_kr_map.get(topping, topping) 
                        controller.publish_status(f"{topping_kr} 추가하는 중입니다.") 
                        print(f"\n▶ [추가 재료] '{topping}' 전용 스쿠퍼 시퀀스 시작!")
                        
                        if topping == 'cabbage':
                            print("   >>> [1/4] 양배추 스쿠퍼 잡으러 이동 중...")
                            pos_cab_appr = [-26.82, 18.03, 57.39, 0.23, 104.54, -26.75] 
                            pos_cab_grab = [-50.96, 44.71, 46.65, 25.71, 115.94, -46.02] 
                            pos_cab_out1 = [-51.41, 42.92, 46.46, 28.42, 117.79, -46.01] 
                            pos_cab_out2 = [-46.16, 35.33, 51.53, 28.08, 118.82, -40.18] 
                            run_task_sync(pos_cab_appr, 0)
                            run_task_sync(pos_cab_grab, 1, wait_time=1.0) 
                            run_task_sync(pos_cab_out1, 0)
                            run_task_sync(pos_cab_out2, 0)

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
                            
                            print("   >>> [3/4] 감자칩 용기에 커스텀 붓기 (5단계 + 흔들기)")
                            custom_pour_data = [
                                -7.45, 1.08, 76.76, -11.9, 100.0, -6.08,
                                19.29, -9.53, 104.48, -10.77, 76.22, -6.33,
                                39.69, 6.74, 96.6, -15.55, 78.29, 35.44,
                                17.82, -1.18, 102.41, 22.97, 70.77, 1.77,
                                24.18, -7.34, 78.13, -1.52, 101.08, 24.95
                            ]
                            run_task_sync(custom_pour_data, 10, wait_time=1.0)

                            print("   >>> [4/4] 양배추 스쿠퍼 반납 중...")
                            run_task_sync(pos_cab_out2, 0)
                            run_task_sync(pos_cab_out1, 0)
                            run_task_sync(pos_cab_grab, 2, wait_time=1.0) 
                            run_task_sync(pos_cab_appr, 0)
                            run_task_sync(pos_pre_2, 0) 
                            print("✅ 'cabbage' 시퀀스 완벽 종료!\n")

                        elif topping == 'tomato':
                            print("   >>> [1/4] 토마토 스쿠퍼 잡으러 이동 중...")
                            pos_tom_appr = [-31.25, 7.15, 71.62, 0.35, 101.55, -31.21] 
                            pos_tom_grab = [-57.22, 36.32, 60.65, 23.74, 112.18, -53.66] 
                            pos_tom_out1 = [-53.9, 22.19, 69.96, 27.21, 114.57, -53.21] 
                            pos_tom_out2 = [-53.9, 22.18, 69.96, 27.21, 114.56, -53.2] 
                            run_task_sync(pos_tom_appr, 0)
                            run_task_sync(pos_tom_grab, 1, wait_time=1.0) 
                            run_task_sync(pos_tom_out1, 0)
                            run_task_sync(pos_tom_out2, 0)

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
                            
                            print("   >>> [3/4] 감자칩 용기에 커스텀 붓기 (5단계 + 흔들기)")
                            custom_pour_data = [
                                -7.45, 1.08, 76.76, -11.9, 100.0, -6.08,
                                19.29, -9.53, 104.48, -10.77, 76.22, -6.33,
                                39.69, 6.74, 96.6, -15.55, 78.29, 35.44,
                                17.82, -1.18, 102.41, 22.97, 70.77, 1.77,
                                24.18, -7.34, 78.13, -1.52, 101.08, 24.95
                            ]
                            run_task_sync(custom_pour_data, 10, wait_time=1.0)

                            print("   >>> [4/4] 토마토 스쿠퍼 반납 중...")
                            run_task_sync(pos_tom_out2, 0)
                            run_task_sync(pos_tom_out1, 0)
                            run_task_sync(pos_tom_grab, 2, wait_time=1.0) 
                            run_task_sync(pos_tom_appr, 0)
                            run_task_sync(pos_pre_2, 0) 
                            print("✅ 'tomato' 시퀀스 완벽 종료!\n")
                            
                        elif topping == 'onion':
                            print("   >>> [1/4] 양파 스쿠퍼 잡으러 이동 중...")
                            pos_oni_appr = [-37.31, -1.21, 80.96, 0.41, 100.96, -37.24] 
                            pos_oni_grab = [-63.76, 30.72, 70.5, 20.21, 110.62, -61.27] 
                            pos_oni_out1 = [-60.97, 11.65, 82.39, 24.15, 113.28, -61.27] 
                            pos_oni_out2 = [-60.97, 11.61, 82.39, 24.15, 113.26, -61.25] 
                            run_task_sync(pos_oni_appr, 0)
                            run_task_sync(pos_oni_grab, 1, wait_time=1.0) 
                            run_task_sync(pos_oni_out1, 0)
                            run_task_sync(pos_oni_out2, 0)

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
                            
                            print("   >>> [3/4] 감자칩 용기에 커스텀 붓기 (5단계 + 흔들기)")
                            custom_pour_data = [
                                -7.45, 1.08, 76.76, -11.9, 100.0, -6.08,
                                19.29, -9.53, 104.48, -10.77, 76.22, -6.33,
                                39.69, 6.74, 96.6, -15.55, 78.29, 35.44,
                                17.82, -1.18, 102.41, 22.97, 70.77, 1.77,
                                24.18, -7.34, 78.13, -1.52, 101.08, 24.95
                            ]
                            run_task_sync(custom_pour_data, 10, wait_time=1.0)

                            print("   >>> [4/4] 양파 스쿠퍼 반납 중...")
                            run_task_sync(pos_oni_out2, 0)
                            run_task_sync(pos_oni_out1, 0)
                            run_task_sync(pos_oni_grab, 2, wait_time=1.0) 
                            run_task_sync(pos_oni_appr, 0)
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

                print("\n▶ [서빙] 완성된 용기를 서빙 존으로 이동")
                pos_serve_1_grip = [14.52, -5.09, 118.33, 2.82, 64.71, -30.1]   
                pos_serve_2_wp   = [9.71, -5.27, 61.94, 3.29, 121.23, -31.84]   
                pos_serve_3_wp   = [-65.04, -31.98, 89.65, 1.96, 111.93, -31.84]
                pos_serve_4_wp   = [-88.83, -14.95, 91.82, -7.37, 102.0, -31.84]
                pos_serve_5_drop = [-95.53, 21.63, 72.16, 2.1, 82.41, -20.47]  

                print("   >>> 1) 완성된 용기 잡기 (Grip)")
                run_task_sync(pos_serve_1_grip, 1, wait_time=1.0)

                print("   >>> 2) 서빙 구역으로 이동 중...")
                run_task_sync(pos_serve_2_wp, 0)
                run_task_sync(pos_serve_3_wp, 0)
                run_task_sync(pos_serve_4_wp, 0)

                print("   >>> 3) 고객 앞 서빙 위치에 용기 내려놓기 (Drop)")
                run_task_sync(pos_serve_5_drop, 2, wait_time=1.0)
                
                print("   >>> 4) 용기 내려놓고 안전 구역으로 후퇴")
                run_task_sync(pos_serve_4_wp, 0)
                run_task_sync(pos_serve_3_wp, 0)

                # (Step 10-B 소스 뿌리기)
                sauce_id = current_task.get('sauce_id', None)
                draw_path = current_task.get('draw_path', None)

                # (이전에 있던 sauce_id 강제 변경 코드는 완전히 삭제했습니다!)

                controller.publish_status("소스 뿌리기를 시작합니다.") 

                if sauce_id == 'mustard':
                    controller.publish_status("머스타드 뿌리는 중입니다")
                elif sauce_id == 'tomato_sauce':
                    controller.publish_status("케찹 뿌리는 중입니다")
                
                if not sauce_id:
                    print("   👉 선택된 소스가 없습니다. 서빙을 완료합니다.")
                else:
                    print(f"\n▶ [소스] '{sauce_id}' 용기 잡고 뿌리기 시퀀스 시작!")
                    
                    if sauce_id == 'tomato_sauce':
                        # 🍅 토마토 소스
                        t1  = [-9.41, -16.46, 91.71, -73.15, 82.61, -19.84]
                        t2  = [-54.88, 0.67, 88.44, -58.15, 32.61, -20.95]
                        t3  = [-56.01, 12.84, 80.1, -65.91, 37.03, -17.67]
                        t4  = [-56.06, 15.71, 55.06, -46.06, 49.69, -44.56]
                        t5  = [-82.24, 17.36, 46.76, -36.9, 45.68, -50.4]
                        t6  = [-82.25, 17.35, 46.76, -36.9, 45.68, -50.43]
                        t7  = [-82.27, 17.37, 46.77, -36.91, 45.69, -50.44]
                        t8  = [-90.29, -20.06, 102.75, -21.93, 37.25, -64.53]
                        t9  = [-80.12, 14.13, 66.48, -32.31, 61.76, -54.27]
                        t10 = [-82.36, -6.31, 102.97, -59.17, 31.8, -213.59]
                        t11 = [-82.24, 17.36, 46.76, -36.9, 45.68, -50.4]
                        t12 = [-60.46, 12.06, 60.54, -35.55, 42.29, -48.31]
                        t13 = [-59.54, 8.1, 83.47, -55.61, 29.42, -24.02]
                        t14 = [-54.88, 0.67, 88.44, -58.15, 32.61, -20.95]
                        t15 = [-9.41, -16.46, 91.71, -73.15, 82.61, -19.84]

                        run_task_sync(t1, 0)
                        run_task_sync(t2, 0)
                        run_task_sync(t3, 13, wait_time=1.0) 
                        run_task_sync(t4, 0)
                        run_task_sync(t5, 0)
                        run_task_sync(t6, 0)
                        run_task_sync(t7, 0)
                        run_task_sync(t8, 0)
                        run_task_sync(t9, 0)
                        run_task_sync(t10, 11, wait_time=0.1) 

                        print("   >>> [Grip] 소스 짜기(Task 12) 모드로 그랩 변경!")
                        run_task_sync(t10, 12, wait_time=0.5)

                        print("   >>> 좌표에 맞춰 그리기 시작!")
                        if draw_path: # draw_path 데이터가 있을 때
                            flat_path_data = []
                            for pt in draw_path:
                                flat_path_data.append(float(pt['xasDouble']))
                                flat_path_data.append(float(pt['yasDouble']))
                            run_task_sync(flat_path_data, 8, wait_time=2.0)
                        else: # 🌟 draw_path가 아예 없거나 None일 때
                            print("   👉 (draw_path 없음) 사전에 지정된 기본 그림(로고) 도안을 그립니다.")
                            run_task_sync([], 8, wait_time=2.0) 
                        
                        run_task_sync(t11, 11, wait_time=0.1)
                        run_task_sync(t12, 0)
                        run_task_sync(t13, 2, wait_time=1.0) 
                        run_task_sync(t14, 0)
                        run_task_sync(t15, 0)
                        print(f"✅ '{sauce_id}' 퍼포먼스 종료!\n")
                        
                    elif sauce_id == 'mustard':
                        # 🌭 머스타드 소스
                        m1  = [-2.85, 5.44, 59.44, -46.58, 80.13, -0.36]
                        m2  = [-38.83, 25.49, 55.79, -59.48, 56.98, -27.36]
                        m3  = [-42.16, 28.05, 58.06, -60.92, 51.71, -24.24]
                        m4  = [-42.28, 36.5, 22.63, -47.75, 67.68, -49.65]
                        m5  = [-9.05, 7.76, 60.16, -63.9, 84.82, -20.48]
                        m6  = [-76.71, -0.88, 42.53, -30.28, 105.75, -54.37]
                        m7  = [-80.12, 14.13, 66.48, -32.31, 61.76, -54.27]
                        m8  = [-82.36, -6.31, 102.97, -59.17, 31.8, -213.59]
                        m9  = [-80.12, 14.13, 66.48, -32.31, 61.76, -54.27]
                        m10 = [-76.71, -0.88, 42.53, -30.28, 105.75, -54.37]
                        m11 = [-9.05, 7.76, 60.16, -63.9, 84.82, -20.48]
                        m12 = [-42.47, 43.0, 7.61, -45.42, 73.44, -56.28]
                        m13 = [-41.35, 25.5, 60.13, -60.97, 52.66, -24.28]
                        m14 = [-38.83, 25.49, 55.79, -59.48, 56.98, -27.36]
                        m15 = [-2.85, 5.44, 59.44, -46.58, 80.13, -0.36]

                        run_task_sync(m1, 0)
                        run_task_sync(m2, 0)
                        run_task_sync(m3, 13, wait_time=1.0) 
                        run_task_sync(m4, 0)
                        run_task_sync(m5, 0)
                        run_task_sync(m6, 0)
                        run_task_sync(m7, 0)
                        run_task_sync(m8, 11, wait_time=0.1)

                        print("   >>> [Grip] 머스타드 짜기(Task 12) 모드로 그랩 변경!")
                        run_task_sync(m8, 12, wait_time=0.5)
                        
                        print("   >>> 좌표에 맞춰 그리기 시작!")
                        if draw_path: # draw_path 데이터가 있을 때
                            flat_path_data = []
                            for pt in draw_path:
                                flat_path_data.append(float(pt['xasDouble']))
                                flat_path_data.append(float(pt['yasDouble']))
                            run_task_sync(flat_path_data, 8, wait_time=2.0)
                        else: # 🌟 draw_path가 아예 없거나 None일 때
                            print("   👉 (draw_path 없음) 사전에 지정된 기본 그림(로고) 도안을 그립니다.")
                            run_task_sync([], 8, wait_time=2.0) 
                            
                        run_task_sync(m9, 11, wait_time=0.1)
                        run_task_sync(m10, 0)
                        run_task_sync(m11, 0)
                        run_task_sync(m12, 0)
                        run_task_sync(m13, 2, wait_time=1.0) 
                        run_task_sync(m14, 0)
                        run_task_sync(m15, 0)
                        print(f"✅ '{sauce_id}' 퍼포먼스 종료!\n")
                        
                # --------------------------------------------------------
                # 10-C. 다음 메뉴 준비 위치(Home)로 복귀 (for문 안쪽)
                # --------------------------------------------------------
                print(f"\n▶ [{current_menu_num}번째 메뉴 완료] 다음 작업을 위한 준비 위치로 복귀")
                pos_home = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0] 
                run_task_sync(pos_home, 0, wait_time=1.0)

            # ============================================================
            # 🌟 [for문 종료] 장바구니에 담긴 모든 요리가 끝났을 때 1번만 실행!
            # ============================================================
            controller.publish_status("타코가 완성 되었습니다. 서빙 존에서 받아가세요! 맛있게 드세요 ^^")
            
            print(f"\n🎉 총 {total_menus}개의 서빙이 모두 완료되었습니다! (맛있게 드세요!)")
            print("\n🏁 한 세트 조리 완료! 다음 새로운 주문을 대기합니다.")
            
            # 루프 끝. 다시 while문의 처음(주문 대기)으로 돌아갑니다.

    except KeyboardInterrupt:
        print("\n사용자에 의해 강제 종료되었습니다.")
    finally:
        print("로봇 컨트롤러를 종료합니다.")
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()