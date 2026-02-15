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



def source_grip():
    """로봇이 수행할 작업"""
    print("Performing grip task...")
    from DSR_ROBOT2 import (
        set_digital_output,
        get_digital_input,
        movej,wait,movel,posx
    )

    # 디지털 입력 신호 대기 함수
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
    


    # Release 동작
    def release():
        print("Releasing...")
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        set_digital_output(3, OFF)
        wait_digital_input(2)

    # Grip 동작
    def soft_grip():
        print("Gripping...")
        set_digital_output(3, ON)
        set_digital_output(2, ON)
        set_digital_output(1, ON)
        wait_digital_input(3)
        wait_digital_input(2)
        wait_digital_input(1)
    

    def grip():
        set_digital_output(3, ON)
        set_digital_output(2, OFF)
        set_digital_output(1, OFF)
        wait_digital_input(3)

    
    def hard_grip():
        set_digital_output(3, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, OFF)




    # 초기 위치로 이동
    release()
    JReady = [-0.09, 37.97, 44.42, 0.42, 97.89, 90.01]
    print("Moving to ready position...")
    movej(JReady, vel=VELOCITY, acc=ACC)

    #소스통로 이동
    P1 = posx(328.19, -675.72, 300.95, 89.01, -97.47, 87.71)
    movel(P1,v=100, a=90)
    grip()
    wait(2)
    soft_grip()
    wait(2)
    hard_grip()




    movel(P1,v=100, a=90)
    release()
    wait(0.5)



def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("grip_simple", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    # 초기화는 한 번만 수행
    initialize_robot()

    source_grip()
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()