import rclpy
import DR_init
import time

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 이동 속도 및 가속도
VELOCITY = 60
ACC = 60

# 디지털 출력 상태
ON, OFF = 1, 0

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp,get_tool,get_tcp,ROBOT_MODE_MANUAL,ROBOT_MODE_AUTONOMOUS  # 필요한 기능만 임포트
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
    """로봇이 수행할 작업"""
    print("Performing grip task...")
    from DSR_ROBOT2 import (
        set_digital_output,
        get_digital_input,
        movej,wait,
    )
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.action import ActionServer
    from tacobot_interfaces.action import RobotTask

    # 디지털 입력 신호 대기 함수
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            # print("Waiting for digital input...")


    # Release 동작
    def release():
        print("Releasing...")
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait_digital_input(2)

    # Grip 동작
    def grip():
        print("Gripping...")
        # release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait_digital_input(1)

    # current_joints = get_current_posj() # [J1, J2, J3, J4, J5, J6] 리스트 반환
    # print(f"현재 관절 각도: {current_joints}")

    # ---------------------------------------------------------
    # 액션 콜백 (여기가 핵심!)
    # ---------------------------------------------------------
    def execute_callback(goal_handle):
        try:
            print("\n[Action Server] 명령 수신! 작업 시작...")
            
            target_joints = list(goal_handle.request.target_joints)
            task_type = goal_handle.request.task_type
            
            feedback_msg = RobotTask.Feedback()
            
            # [Step 1] 이동
            feedback_msg.status = "목표 위치로 이동 중..."
            goal_handle.publish_feedback(feedback_msg)
            
            print(f"이동 목표: {target_joints}")
            movej(target_joints, vel=VELOCITY, acc=ACC)
            wait(0.5)

            # [Step 2] 동작 수행
            if task_type == 1:
                feedback_msg.status = "집는 중 (Grip)..."
                goal_handle.publish_feedback(feedback_msg)
                
                if get_digital_input(1) == 1:
                    print("! 이미 잡고 있어 먼저 놓습니다.")
                    release()
                    wait(0.5)
                grip()
                
            elif task_type == 2:
                feedback_msg.status = "놓는 중 (Release)..."
                goal_handle.publish_feedback(feedback_msg)
                release()
            
            else:
                feedback_msg.status = "이동 완료"
                goal_handle.publish_feedback(feedback_msg)

            # 성공 처리
            goal_handle.succeed()
            result = RobotTask.Result()
            result.success = True
            result.message = "Success"
            print("[Action Server] 작업 완료!")
            return result

        except Exception as e:
            # 에러 발생 시 프로그램이 죽지 않고 로그를 띄우도록 함
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
        '/dsr01/scooper_grab', # 네임스페이스 포함된 이름
        execute_callback,
        callback_group=ReentrantCallbackGroup() # <--- 여기가 핵심!
    )
    
    print("스쿠퍼 액션 서버 대기 중... (MultiThreaded)")
    
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