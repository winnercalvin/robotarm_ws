import rclpy
import DR_init

# ==========================================
# 1. 로봇 설정 상수
# ==========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)


def perform_task():
    from DSR_ROBOT2 import set_digital_output, get_digital_input, wait, movel, movec, posx, set_velx, set_accx, DR_BASE

    # ---------------------------------------------------------
    # 2. 속도 및 좌표 설정 (보내주신 예제 스타일 적용)
    # ---------------------------------------------------------
    # 전역 태스크 속도/가속도 설정 (보내주신 코드 참조)
    # 속도 60mm/sec, 회전속도 30deg/sec
    set_velx(60, 30)
    # 가속도 100mm/sec2, 회전가속도 60deg/sec2
    set_accx(100, 60)

    #초록(왼)
    # p1 = posx(369.72, 45.37, 271.84, 40.04, 179.89, 40.39)
    # p2 = posx(435.05, -172.38, 267.64, 46.01, 179.87, 46.39)
    # p3 = posx(439.9, -235.13, 266.77, 83.45, 157.49, 88.46)
    # p4 = posx(435.98, -246.13, 289.73, 79.46, 142.07, 82.45)
    # p5 = posx(440.46, -318.82, 223.4, 79.4, 151.28, 87.55)
    # p6 = posx(431.91, -333.83, 167.41, 111.38, -170.33, 115.49) 
    # p7 = posx(431.89, -327.35, 182.09, 111.37, -170.33, 115.48)
    # p8 = posx(431.88, -152.38, 182.1, 111.35, -170.32, 115.47)
        # P0 = posx(334.34, -128.01, 319.25, 85.79, 134.51, 85.15)

    #빨강(중앙)
    # p1 = posx(293.12, 45.37, 271.84, 40.04, 179.89, 40.39)
    # p2 = posx(358.45, -172.38, 267.64, 46.01, 179.87, 46.39)
    # p3 = posx(363.3, -235.13, 266.77, 83.45, 157.49, 88.46)
    # p4 = posx(359.38, -246.13, 289.73, 79.46, 142.07, 82.45)
    # p5 = posx(363.86, -318.82, 223.4, 79.4, 151.28, 87.55)
    # p6 = posx(355.31, -333.83, 167.41, 111.38, -170.33, 115.49)
    # p7 = posx(355.29, -327.35, 182.09, 111.37, -170.33, 115.48)
    # p8 = posx(355.28, -152.38, 182.1, 111.35, -170.32, 115.47)


    #흰색(오른쪽)
    p1 = posx(221.02, 45.37, 271.84, 40.04, 179.89, 40.39)
    p2 = posx(286.35, -172.38, 267.64, 46.01, 179.87, 46.39)
    p3 = posx(291.2, -235.13, 266.77, 83.45, 157.49, 88.46)
    p4 = posx(287.28, -246.13, 289.73, 79.46, 142.07, 82.45)
    p5 = posx(291.76, -318.82, 223.4, 79.4, 151.28, 87.55)
    p6 = posx(283.21, -333.83, 167.41, 111.38, -170.33, 115.49)
    p7 = posx(283.19, -327.35, 182.09, 111.37, -170.33, 115.48)
    p8 = posx(283.18, -152.38, 182.1, 111.35, -170.32, 115.47)


    print(">>> 동작: 스쿱 (Scooping)")
    #movec(P_via, P_final)
    movel(p1)
    movel(p2)
    movel(p3)
    movel(p4) 
    movel(p5)
    movel(p6)
    movel(p7)
    movel(p8)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("move_scoop_demo", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        initialize_robot()
        perform_task()
    except KeyboardInterrupt:
        print("Shutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()