import rclpy
import DR_init
import time

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
# ROBOT_TCP = "GripperDA_v1" # <--- 이거 쓰면 에러나서 주석 처리함

# 이동 속도 및 가속도
VELOCITY = 40
ACC = 60

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp, get_tool, get_tcp, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    from DSR_ROBOT2 import get_robot_mode, set_robot_mode, set_ref_coord, DR_BASE

    # Tool과 TCP 설정시 매뉴얼 모드로 변경해서 진행
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    
    
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    set_ref_coord(DR_BASE) # 좌표 기준을 베이스로 고정 (좌표 튀는거 방지용 필수 설정)
    time.sleep(2) 
    
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
    print("Performing task...")
    # [수정] DR_TOOL, DR_BASE 누락되면 멈춰서 추가함
    from DSR_ROBOT2 import posx, posj, movec, movej, move_periodic, movel, set_ref_coord, wait, DR_TOOL, DR_BASE

    # 초기 위치 및 목표 위치 설정
    #P0 = posj(0,0,90,0,90,0)
    #P1 = posx(353.51, -91.46, 102.22, 117.92, 179.74, 118.32)
    #P2 = posx(353.91, -91.09, 179.37, 86.6, 179.67, 87)
    P1 = posx(370.95, 320.83, 390.80, 89.67, 122.66, 90.05) #털기 장소
    P2 = posx(369.31, 45.07, 195.11, 88.17, 179.98, 88.54) 

    for a in range(3):
        movel(P1,v=400, a=400)
        wait(1)
        movel(P2,v=2000, a=2000)




def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)
    node = rclpy.create_node("move_basic", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    try:
        initialize_robot()
        perform_task()

    except KeyboardInterrupt:
        print("\nNode interrupted by user.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # 종료 처리
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()