import rclpy
import DR_init
import time
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from tacobot_interfaces.action import RobotTask

# 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 초기화 시 출력하거나 기본값으로 사용할 속도
VELOCITY = 50 
ACC = 50

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def initialize_robot():
    """로봇 연결 및 초기 설정"""
    node = DR_init.__dsr__node
    service_name = f"/{ROBOT_ID}/system/set_robot_mode"
    from DSR_ROBOT2 import (
        movej, wait, set_tool, set_tcp, get_tool, get_tcp, 
        get_robot_mode, set_robot_mode, 
        ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    )
   
    time.sleep(5)
    
    try:
        # 매뉴얼 모드 설정 및 도구 설정
        set_robot_mode(ROBOT_MODE_MANUAL)
        wait(0.5)
        
        print(">>> 설정 적용 중 (Tool/TCP)...")
        set_tool(ROBOT_TOOL)
        wait(0.5)
        set_tcp(ROBOT_TCP)
        wait(0.5)
        
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        wait(1.0)
        
        print("=== [Action Server] 로봇 연결 및 설정 완료! ===")
        print(f"TCP: {get_tcp()} | TOOL: {get_tool()}")
        print(f"MODE: {get_robot_mode()}")
    except Exception as e:
        print(f"!!! 초기화 중 오류 발생: {e}")

# 1. 현관문 (Goal Callback)
def goal_callback(goal_request):
    print(f"\n🔔 [CCTV-1] 현관문 도착! Task {goal_request.task_type} 요청 수락.", flush=True)
    return GoalResponse.ACCEPT


# 2. 안방 (Execute Callback)
def execute_callback(goal_handle):
    task_type = goal_handle.request.task_type
    print(f"🎬 [CCTV-2] 작업 시작! Task {task_type}", flush=True)
    # DSR 라이브러리 임포트
    from DSR_ROBOT2 import (
        movej, wait, get_current_posj, # [필수] 현재 위치 확인 함수
        set_tool, set_tcp, get_tool, get_tcp, 
        get_robot_mode, set_robot_mode, 
        ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    )
    import tacobot.grab_tools as grab_tools
    import tacobot.pour_tools as pour_tools

    try:
        target_joints = list(goal_handle.request.target_joints)
        v, a = 50, 50
        if task_type == 1: v, a = 30, 20
        elif task_type == 3: v, a = 60, 40

        # --- 동작 시작 ---
        
        # 1. Release (잡기 전)
        if task_type == 1:
            print("   >>> [Action] Release", flush=True)
            grab_tools.release()
            time.sleep(0.5)

        # 2. 이동 (movej + time.sleep)
        goal_handle.publish_feedback(RobotTask.Feedback(status=f"이동 중..."))
        print(f"   >>> [Move] 이동 명령 전송 (movej)", flush=True)
        movej(target_joints, vel=v, acc=a)
        
        # [매우 중요] 로봇이 움직이는 동안, 파이썬은 그냥 3초 쉽니다.
        # 드라이버를 건드리지 않기 위해 wait() 대신 time.sleep() 씁니다.
        print("   >>> [Wait] 3초 이동 대기...", flush=True)
        time.sleep(3.0)
        print("   >>> [Wait] 이동 완료 간주.", flush=True)

        # 3. 도착 후 동작
        if task_type == 1:   
            print("   >>> [Action] Grip", flush=True)
            grab_tools.grip() 
        elif task_type == 2: 
            grab_tools.release()
        elif task_type == 3: 
            print("   >>> [Action] Pour", flush=True)
            pour_tools.pour_action()

        print("🎉 [CCTV-3] 작업 성공! (Succeed)", flush=True)
        goal_handle.succeed()
        
        # 다음 명령 수신을 위한 통신 안정화 시간
        time.sleep(0.5) 

        return RobotTask.Result(success=True, message="Success")

    except Exception as e:
        print(f"!!!! 에러 발생 !!!! : {e}", flush=True)
        goal_handle.abort()
        return RobotTask.Result(success=False, message=str(e))
    
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("action_server", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    
    initialize_robot()
    
    server = ActionServer(
        node, 
        RobotTask, 
        '/dsr01/action_server',
        execute_callback,
        goal_callback=goal_callback
    )
    
    
    print(">>> 만능 액션 서버가 대기 중입니다...", flush=True)
    try:
        # Executor 없이 그냥 spin() -> 싱글 스레드 강제
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("종료")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()