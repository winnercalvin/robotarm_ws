import rclpy
import DR_init
import time

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"  # action_server.py에 맞춤

# 이동 속도 및 가속도
VELOCITY = 60
ACC = 60

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp, set_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    
    # 안전한 제어권 획득을 위한 모드 전환
    set_robot_mode(ROBOT_MODE_MANUAL)
    time.sleep(0.5)

    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print("#" * 50)

    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1.0)


def perform_task():
    from DSR_ROBOT2 import movej, get_current_posj
    # 🌟 실제 패키지에 있는 함수를 그대로 불러와서 테스트합니다!
    from tacobot.pour_tools import pour_action 

    # 도착 확인이 포함된 안전한 이동 함수 정의 (테스트 및 pour_tools 복귀용)
    def move_and_wait(target, v, a):
        print(f"   >>> [Move] 목표 위치로 이동 중... (Vel: {v})", flush=True)
        movej(target, vel=v, acc=a)
        while True:
            current = list(get_current_posj())
            diff = sum([abs(target[i] - current[i]) for i in range(6)])
            if diff < 2.0:
                print("   >>> [Wait] 도착 완료!", flush=True)
                break
            time.sleep(0.1)

    # 1. 메인 시나리오에서 가져온 붓기 준비 위치
    pos_pour_chips = [-14.32, -33.3, 122.38, 18.34, 76.87, -111.05]
    
    print("\n==================================================")
    print("🍟 [Unit Test] 감자칩 붓기(Pouring) 동작 테스트 시작")
    print("==================================================")

    # 2. 해당 좌표로 먼저 이동
    print("\n▶ 1단계: 붓기 준비 위치로 이동")
    move_and_wait(pos_pour_chips, VELOCITY, ACC)
    time.sleep(1.0)

    # 3. 붓기 동작 유닛 테스트 실행
    print("\n▶ 2단계: pour_action() 실행")
    # move_and_wait 함수를 주입해주어야 pour_tools 안에서 6번(복귀) 동작을 제대로 수행합니다.
    pour_action(move_and_wait_func=move_and_wait)
    
    print("\n🎉 붓기 유닛 테스트가 무사히 종료되었습니다!")


def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("pour_test_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        initialize_robot()
        perform_task()
    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"!!!! 에러 발생 !!!! : {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()