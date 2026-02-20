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

    # 속도 분리 (기울일 때는 느리게, 복귀는 빠르게)
    VEL_POUR, ACC_POUR = 20, 15
    VEL_RETURN, ACC_RETURN = 40, 30

    print(">>> [Module] 최적화 붓기 시작...", flush=True)

    # 1. 현재 위치 저장
    current_joints = list(get_current_posj())

    # -----------------------------
    # 2. 1차 틸팅 (부드럽게)
    # -----------------------------
    first_tilt = list(current_joints)
    first_tilt[5] -= 80.0
    movej(first_tilt, vel=VEL_POUR, acc=ACC_POUR)
    time.sleep(2.0)

    # -----------------------------
    # 3. 2차 틸팅 (완전 배출)
    # -----------------------------
    second_tilt = list(first_tilt)
    second_tilt[5] -= 30.0
    movej(second_tilt, vel=VEL_POUR, acc=ACC_POUR)
    time.sleep(2.0)

    # -----------------------------
    # 4. 오버틸트 (잔류물 제거)
    # -----------------------------
    over_tilt = list(second_tilt)
    over_tilt[5] -= 30.0    # over_tilt[5] -= 10.0
    movej(over_tilt, vel=15, acc=10)
    time.sleep(1.0)

    # -----------------------------
    # 5. 마이크로 쉐이킹 (잔여물 제거)
    # -----------------------------
    for _ in range(2):
        shake_up = list(over_tilt)
        shake_up[5] += 5.0
        movej(shake_up, vel=30, acc=20)
        time.sleep(0.5)

        shake_down = list(over_tilt)
        shake_down[5] -= 5.0
        movej(shake_down, vel=30, acc=20)
        time.sleep(0.5)

    # -----------------------------
    # 6. 원위치 복귀 (빠르게)
    # -----------------------------
    print(">>> [Module] 복귀 중...", flush=True)
    movej(current_joints, vel=VEL_RETURN, acc=ACC_RETURN)
    time.sleep(2.0)

    print(">>> [Module] 붓기 완료 (잔류 최소화)", flush=True)
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
        # pour_action()
        pour_action_frame(portion=1)

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

