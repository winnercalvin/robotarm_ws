#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# move_pouring_modified.py

import rclpy
import DR_init
import time

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

VELOCITY = 40  # VELOCITY = 40 -> default
ACC = 50       # ACC = 60 -> default

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    from DSR_ROBOT2 import (
        set_tool, set_tcp, get_tool, get_tcp,
        ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS,
        get_robot_mode, set_robot_mode
    )

    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(2)

    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {get_tcp()}")
    print(f"ROBOT_TOOL: {get_tool()}")
    print(f"ROBOT_MODE: {get_robot_mode()}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#" * 50)


###############################################################
# added optimized pouring code
###############################################################

def pour_action():
    from DSR_ROBOT2 import movej, get_current_posj
    import time

    VEL_POUR, ACC_POUR = 20, 15
    VEL_RETURN, ACC_RETURN = 40, 30

    print(">>> [Module] J5+J6 복합 붓기 시작...", flush=True)

    current_joints = list(get_current_posj())

    # --------------------------------
    # 1차 틸트 (기본 기울기 형성)
    # --------------------------------
    first_tilt = list(current_joints)
    first_tilt[4] -= 25.0   # 🔥 J5 추가
    first_tilt[5] -= 60.0   # J6
    movej(first_tilt, vel=VEL_POUR, acc=ACC_POUR)
    time.sleep(2.0)

    # --------------------------------
    # 2차 틸트 (완전 배출 각도)
    # --------------------------------
    second_tilt = list(first_tilt)
    second_tilt[4] -= 15.0   # 🔥 J5 추가 기울기
    second_tilt[5] -= 40.0
    movej(second_tilt, vel=VEL_POUR, acc=ACC_POUR)
    time.sleep(2.0)

    # --------------------------------
    # 오버 틸트 (잔류물 제거)
    # --------------------------------
    over_tilt = list(second_tilt)
    over_tilt[4] -= 5.0     # 🔥 J5 조금 더
    over_tilt[5] -= 20.0
    movej(over_tilt, vel=15, acc=10)
    time.sleep(1.0)

    # --------------------------------
    # 마이크로 쉐이킹 (J6만 진동)
    # --------------------------------
    for _ in range(2):
        shake_up = list(over_tilt)
        shake_up[5] += 6.0
        movej(shake_up, vel=30, acc=20)
        time.sleep(0.4)

        shake_down = list(over_tilt)
        shake_down[5] -= 6.0
        movej(shake_down, vel=30, acc=20)
        time.sleep(0.4)

    # --------------------------------
    # 복귀
    # --------------------------------
    print(">>> [Module] 복귀 중...", flush=True)
    movej(current_joints, vel=VEL_RETURN, acc=ACC_RETURN)
    time.sleep(2.0)

    print(">>> [Module] 붓기 완료 (J5+J6 안정화)", flush=True)
###############################################################

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("move_basic", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        initialize_robot()
        # perform_task()
        # perform_task_unit1()
        # perform_task_unit2()
        pour_action()
        # pour_action_frame(portion=1)

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

