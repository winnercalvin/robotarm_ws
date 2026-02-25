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
VELOCITY = 30 
ACC = 30

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
        get_robot_mode, set_robot_mode, movesj,
        ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    )
    import tacobot.grab_tools as grab_tools
    import tacobot.pour_tools as pour_tools
    import tacobot.scoop_tools as scoop_tools
    import tacobot.shake_tools as shake_tools
    import tacobot.drain_tools as drain_tools
    import tacobot.drizzle_tools as drizzle_tools

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

    def wait_for_arrival(target):
            start_t = time.time()
            while True:
                current = list(get_current_posj())
                diff = sum([abs(target[i] - current[i]) for i in range(6)])
                if diff < 2.0:
                    print("   >>> [Wait] 최종 목적지 도착 확인 완료!", flush=True)
                    break
                if time.time() - start_t > 15.0:
                    break
                time.sleep(0.1)

    try:
        goal_handle.publish_feedback(RobotTask.Feedback(status=f"Processing..."))

        # ---------------------------------------------------------
        # Case A: 일반 이동 및 동작
        # ---------------------------------------------------------
        # 🌟 리스트에 12(Strong), 13(Middle) 추가
        if task_type in [0, 1, 2, 3, 9, 11, 12, 13]: 
            v, a = 50, 50
            if task_type == 0: v, a = 30, 30   
            elif task_type == 1: v, a = 30, 20 
            elif task_type == 2: v, a = 20, 20 
            elif task_type == 3: v, a = 50, 40 
            elif task_type == 9: v, a = 30, 20  # Weak Grip (111)
            elif task_type == 11: v, a = 120, 100
            elif task_type == 12: v, a = 30, 20 # Strong Grip (000)
            elif task_type == 13: v, a = 30, 20 # Middle Grip (001)

            # 🌟 무언가를 잡는 동작(1, 9, 12, 13)을 하기 전에는 항상 Release 먼저 실행
            if task_type in [1, 9, 12, 13]:
                print("   >>> [Module] Grip 전 Release 안전 실행", flush=True)
                grab_tools.release()
                time.sleep(0.5)
            
            if task_type == 3 and len(data) == 12:
                wp = data[0:6]
                target = data[6:12]
                print("   >>> [Move] 경유지를 거쳐 논스톱(Spline) 이동 중...", flush=True)
                movesj([posj(wp), posj(target)], vel=v, acc=a)
                wait_for_arrival(target)
            else:
                print("   >>> [Wait] 로봇 일반 이동 완료 대기...", flush=True)
                move_and_wait(data, v, a)
                print("   >>> [Wait] 이동 완료 확인됨!", flush=True)

            # 🌟 도착 후 각 Task 번호에 맞는 함수 실행
            if task_type == 0: 
                print("   >>> [Module] 단순 이동(경유지) 완료", flush=True)
            elif task_type == 1: 
                grab_tools.grip() 
            elif task_type == 2:
                grab_tools.release()
                time.sleep(0.5)
            elif task_type == 3: 
                pour_tools.pour_action(move_and_wait)
            elif task_type == 9: 
                grab_tools.weak_grip()    # (111) 기존 sauce_grip
            elif task_type == 12: 
                grab_tools.strong_grip()  # (000)
            elif task_type == 13: 
                grab_tools.middle_grip()  # (001)

        # ---------------------------------------------------------
        # Case C-1: 쉐이크 동작 (Task 4) - Z축 위아래
        # ---------------------------------------------------------
        elif task_type == 4:
            print("   >>> [Task 4] 쉐이크 준비 (위아래 Z축)", flush=True)
            move_and_wait(data, 50, 40)
            shake_tools.shake_action(direction="z")

        # ---------------------------------------------------------
        # Case C-2: 쉐이크 동작 (Task 5) - Y축 좌우 [새로 추가]
        # ---------------------------------------------------------
        elif task_type == 5:
            print("   >>> [Task 5] 쉐이크 준비 (좌우 Y축)", flush=True)
            move_and_wait(data, 50, 40)
            shake_tools.shake_action(direction="y")

        # ---------------------------------------------------------
        # Case B: 스쿱 동작 (Task 6) - 9개의 직교 좌표(posx)
        # ---------------------------------------------------------
        elif task_type == 6:
            if len(data) == 54: # 🌟 48에서 54로 변경 (9개 * 6 = 54)
                print("   >>> [Data] 스쿱 좌표 데이터(9개 포인트) 수신 완료", flush=True)
                
                # 54개의 데이터를 6개씩 9덩어리로 쪼갭니다.
                p1, p2, p3, p4 = data[0:6], data[6:12], data[12:18], data[18:24]
                p5, p6, p7, p8 = data[24:30], data[30:36], data[36:42], data[42:48]
                p9 = data[48:54] # 🌟 9번째 좌표 조각 추가
                
                # scoop_tools로 넘겨 실행 (p9 추가)
                scoop_tools.scoop_action(p1, p2, p3, p4, p5, p6, p7, p8, p9)
            else:
                goal_handle.abort()
                return RobotTask.Result(success=False, message=f"Data Length Error: expected 54, got {len(data)}")

        # ---------------------------------------------------------
        # Case E: 기름 털기 (Task 7) - [기존 6에서 7로 변경]
        # ---------------------------------------------------------
        elif task_type == 7:
            if len(data) == 12:
                print("   >>> [Data] 털기(Drain) 좌표 데이터 수신 완료", flush=True)
                p1, p2 = data[0:6], data[6:12]
                drain_tools.drain_action(p1, p2)
            else:
                goal_handle.abort()
                return RobotTask.Result(success=False, message="Data Length Error")

        # ---------------------------------------------------------
        # Case F: 소스 뿌리기 (Task 8)
        # ---------------------------------------------------------
        elif task_type == 8:
            print("   >>> [Task 8] 소스 뿌리기 시작!", flush=True)
            if len(data) > 0:
                print(f"   >>> [Module] 웹소켓 커스텀 도안 그리기 (데이터 수: {len(data)})", flush=True)
                drizzle_tools.custom_drizzle(data)
            else:
                # 🌟 빈 배열([])이 들어왔다면 기본 로고를 그립니다.
                print("   >>> [Module] 웹소켓 데이터 없음 -> 내장된 기본 로고 그리기 호출", flush=True)
                drizzle_tools.draw_default_logo()


        # =========================================================
        # 커스텀 5단계 붓기 + 쉐이크 중첩 (Task 10)
        # =========================================================
        elif task_type == 10:
            if len(data) == 30: # 좌표 5개 x 데이터 6개 = 30개
                from DSR_ROBOT2 import move_periodic, DR_TOOL
                print("   >>> [Pour] 5단계 커스텀 붓기 시작!", flush=True)
                
                # 데이터 쪼개기
                p1, p2, p3 = data[0:6], data[6:12], data[12:18]
                p4, p5 = data[18:24], data[24:30]
                
                # 1~4번 좌표로 순차 이동
                move_and_wait(p1, 50, 40)
                move_and_wait(p2, 50, 40)
                move_and_wait(p3, 50, 40)
                move_and_wait(p4, 50, 40)
                
                # 5번 좌표 진입 전 흔들기(Periodic) 켜기!
                print("   >>> [Shake] 5번 좌표 진입하며 흔들기 시작!", flush=True)
                move_periodic(
                    amp=[10, 0, 0, 0, 0, 0], # (참고: [X,Y,Z, Rx,Ry,Rz] 이므로 현재 X축 방향 진동입니다)
                    period=0.4,
                    atime=0.2,
                    repeat=5,
                    ref=DR_TOOL
                )
                
                # 흔들기가 켜진 상태로 5번 좌표로 진입 (모션 중첩 발생!)
                move_and_wait(p5, 30, 20) # 붓기 마지막은 살짝 부드럽게 속도 30으로 세팅
            else:
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