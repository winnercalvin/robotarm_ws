import rclpy
import DR_init
import time

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 이동 속도 및 가속도
VELOCITY = 30
ACC = 20

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp, get_tool, get_tcp, ROBOT_MODE_MANUAL,ROBOT_MODE_AUTONOMOUS  # 필요한 기능만 임포트
    from DSR_ROBOT2 import get_robot_mode,set_robot_mode

    # Tool과 TCP 설정시 매뉴얼 모드로 변경해서 진행
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(2)  # 설정 안정화를 위해 잠시 대기
    # 설정된 상수 출력
    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {get_tcp()}") 
    print(f"ROBOT_TOOL: {get_tool()}")
    print(f"ROBOT_MODE 0:수동, 1:자동 : {get_robot_mode()}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#" * 50)



def perform_task():
    """붓기 액션"""
    print("[pour node] 붓기 시작...")
    from DSR_ROBOT2 import movej, wait, get_current_posj
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.action import ActionServer
    from tacobot_interfaces.action import RobotTask

    # 붓기 동작 (J6 회전)
    def pour_action():
        print(">>> [Pour Node] 붓기 시작 (Tilting)...")
        
        # 1. 현재 위치(붓기 직전 위치) 저장
        # get_current_posj는 array를 반환하므로 list로 변환 필수!
        current_joints = list(get_current_posj())
        
        # 2. 기울일 각도 계산 (마지막 관절 J6를 -110도 회전)
        # 방향이 반대라면 +110으로 수정하세요.
        target_pour_joints = list(current_joints)
        target_pour_joints[5] = target_pour_joints[5] - 110.0 
        
        # 3. 기울이기 (실행)
        movej(target_pour_joints, vel=VELOCITY, acc=ACC)
        wait(2.0) # 내용물이 다 쏟아질 때까지 2초 대기
        
        # 4. 원위치 복귀 (다시 수평으로)
        print(">>> [Pour Node] 원위치 복귀...")
        movej(current_joints, vel=VELOCITY, acc=ACC)


    # ---------------------------------------------------------
    # 액션 콜백 
    # ---------------------------------------------------------
    def execute_callback(goal_handle):
        try:
            print("\n[Pour Node] 붓기 명령 수신!")
            
            # [중요] list로 변환
            target_joints = list(goal_handle.request.target_joints)
            
            feedback_msg = RobotTask.Feedback()
            
            # [Step 1] 튀김기 앞으로 이동
            feedback_msg.status = "붓기 위치로 이동 중..."
            goal_handle.publish_feedback(feedback_msg)
            
            print(f"이동 목표: {target_joints}")
            movej(target_joints, vel=VELOCITY, acc=ACC)
            wait(0.5)

            # [Step 2] 도착했으면 붓기 실행!
            feedback_msg.status = "내용물 붓는 중..."
            goal_handle.publish_feedback(feedback_msg)
            
            pour_action() # <--- 여기서 붓기 함수 호출
            
            # 성공 처리
            goal_handle.succeed()
            result = RobotTask.Result()
            result.success = True
            result.message = "Pouring Complete"
            return result

        except Exception as e:
            print(f"!!!! 에러 발생 !!!! : {e}")
            goal_handle.abort()
            result = RobotTask.Result()
            result.success = False
            result.message = f"Error: {e}"
            return result

    # ---------------------------------------------------------
    # 서버 실행 (멀티스레드 적용)
    # ---------------------------------------------------------
    node = DR_init.__dsr__node

    # [중요] 콜백 그룹 설정: 이 서버가 바빠도 다른 통신(movej)이 끼어들 수 있게 함
    server = ActionServer(
        node,
        RobotTask,
        '/dsr01/pouring', # 네임스페이스 포함된 이름
        execute_callback,
        callback_group=ReentrantCallbackGroup() # <--- 여기가 핵심!
    )
    
    print("붓기(Pour) 액션 서버 대기 중...")
    
    # [중요] 멀티스레드 실행기로 노드를 돌림
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("종료")


def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("grip_simple", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    # 초기화는 한 번만 수행
    initialize_robot()

    perform_task()
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()