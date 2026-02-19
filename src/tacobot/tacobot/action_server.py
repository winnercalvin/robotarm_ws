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
    from DSR_ROBOT2 import (
        set_tool, set_tcp, get_tool, get_tcp, 
        get_robot_mode, set_robot_mode, wait,
        ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    )
    
    service_name = f"/{ROBOT_ID}/system/set_robot_mode"
    time.sleep(5.0)
    
    try:
        print(">>> [Init] 로봇 초기화 시작...", flush=True)
        
        # 매뉴얼 모드 설정
        set_robot_mode(ROBOT_MODE_MANUAL)
        wait(0.5)
        
        print(">>> [Init] 툴/TCP 설정 중...", flush=True)
        set_tool(ROBOT_TOOL)
        wait(0.5)
        set_tcp(ROBOT_TCP)
        wait(0.5)
        
        # 자동 모드 변경
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        wait(1.0)
        
        print("=== [System] 로봇 연결 및 설정 완료! ===", flush=True)
        # 확인용 출력 (에러나면 여기서 걸림)
        # print(f"TCP: {get_tcp()} | TOOL: {get_tool()}") 
        # print(f"MODE: {get_robot_mode()}")
        
    except Exception as e:
        print(f"!!! 초기화 중 오류 발생: {e}", flush=True)

# 1. 현관문 (Goal Callback)
def goal_callback(goal_request):
    print(f"\n🔔 [CCTV-1] 현관문 도착! Task {goal_request.task_type} 요청 수락.", flush=True)
    return GoalResponse.ACCEPT


# 2. 안방 (Execute Callback)
def execute_callback(goal_handle):
    task_type = goal_handle.request.task_type
    data = list(goal_handle.request.target_joints)
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
    import tacobot.scoop_tools as scoop_tools
    import tacobot.shake_tools as shake_tools
    import tacobot.drain_tools as drain_tools

    def move_and_wait(target, v, a):
        print(f"   >>> [Move] 이동 명령 전송 (Vel: {v})", flush=True)
        movej(target, vel=v, acc=a)
        
        # 로봇이 실제로 도착할 때까지 파이썬이 감시합니다.
        # (wait(0)보다 훨씬 확실한 방법)
        start_t = time.time()
        while True:
            current = list(get_current_posj())
            diff = sum([abs(target[i] - current[i]) for i in range(6)])
            
            # 오차가 2.0 미만이면 도착으로 인정
            if diff < 2.0:
                print("   >>> [Wait] 도착 확인 완료!", flush=True)
                break
            
            # 15초 넘으면 강제 통과 (무한 대기 방지)
            if time.time() - start_t > 15.0:
                print("   >>> [Warn] 시간 초과! 다음 동작 강제 진행.", flush=True)
                break
                
            time.sleep(0.1) # 0.1초 간격으로 확인

    try:
        goal_handle.publish_feedback(RobotTask.Feedback(status=f"Processing..."))

        # ---------------------------------------------------------
        # Case A: 일반 이동 및 동작 (Grip / Pour) - 데이터 6개
        # ---------------------------------------------------------
        if task_type in [1, 2, 3]: 
            v, a = 50, 50
            if task_type == 1: v, a = 30, 20
            elif task_type == 2: v, a = 20, 20
            elif task_type == 3: v, a = 60, 40

            # 1. Grip일 경우 Release 먼저 수행
            if task_type == 1:
                print("   >>> [Module] Release 실행", flush=True)
                grab_tools.release()
                time.sleep(0.5)
            
            # [안전장치] 3초 대기
            print("   >>> [Wait] 로봇 이동 완료 대기...", flush=True)
            move_and_wait(data, v, a)
            print("   >>> [Wait] 이동 완료 확인됨!", flush=True)

            # 3. 동작 수행
            if task_type == 1:   
                print("   >>> [Module] Grip 실행", flush=True)
                grab_tools.grip() 
            elif task_type == 2:
                # [추가된 부분] 놓기(Task 2): 도착하면 그리퍼를 엽니다.
                print("   >>> [Module] 도착 후 Release(놓기) 실행", flush=True)
                grab_tools.release()
                time.sleep(0.5)
            elif task_type == 3: 
                print("   >>> [Module] Pour 실행", flush=True)
                pour_tools.pour_action()

        # ---------------------------------------------------------
        # Case B: 스쿱 동작 (Scoop) - 데이터 24개 (P0~P3)
        # ---------------------------------------------------------
        elif task_type == 5:
            if len(data) == 24:
                print("   >>> [Data] 스쿱 좌표 데이터(24개) 수신 완료", flush=True)
                # 데이터 쪼개기 (6개씩)
                p0 = data[0:6]
                p1 = data[6:12]
                p2 = data[12:18]
                p3 = data[18:24]
                
                # 스쿱 모듈 실행 (이동 로직이 내부에 있음)
                scoop_tools.scoop_action(p0, p1, p2, p3)
            else:
                print(f"⚠️ [Error] 데이터 개수 오류! (Expected: 24, Got: {len(data)})", flush=True)
                goal_handle.abort()
                return RobotTask.Result(success=False, message="Data Length Error")

        # ---------------------------------------------------------
        # Case C: 쉐이크 동작 (Shake) - [추가됨]
        # ---------------------------------------------------------
        elif task_type == 4:
            print("   >>> [Task] 쉐이크 준비 (이동 -> 잡기 -> 흔들기)", flush=True)

            print("   >>> [Module] 이동 전 Release 실행", flush=True)
            grab_tools.release()
            time.sleep(0.5)
            
            # 1. 쉐이크 위치로 이동
            move_and_wait(data, 60, 40)
            
            # 2. 확실하게 잡기 (요청하신 부분)
            print("   >>> [Module] Grip 재확인", flush=True)
            grab_tools.grip()
            time.sleep(0.5)

            # 3. 흔들기 실행
            shake_tools.shake_action()

        # ---------------------------------------------------------
        # Task 6: 확실하게 털기 (Drain)
        # ---------------------------------------------------------
        elif task_type == 6:
            if len(data) == 12:
                print("   >>> [Data] 털기(Drain) 좌표 데이터(12개) 수신 완료", flush=True)
                p1 = data[0:6]
                p2 = data[6:12]
                
                # 매개변수 넣어서 실행!
                drain_tools.drain_action(p1, p2)
            else:
                print(f"⚠️ [Error] 데이터 개수 오류! (Expected: 12, Got: {len(data)})", flush=True)
                goal_handle.abort()
                return RobotTask.Result(success=False, message="Data Length Error")

        # 성공 처리
        print("🎉 [Success] 작업 완료 신호 전송", flush=True)
        goal_handle.succeed()
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