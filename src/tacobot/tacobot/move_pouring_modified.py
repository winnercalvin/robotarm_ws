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
    from DSR_ROBOT2 import movej, get_current_posj, set_digital_output
    import time

    VEL_POUR, ACC_POUR = 15, 10
    VEL_RETURN, ACC_RETURN = 40, 30

    print(">>> [Module] 강한 붓기 시작...", flush=True)

    current_joints = list(get_current_posj())

    # --------------------------------
    # 🔥 0️⃣ 시작 시 그리퍼 동작 (OPEN → CLOSE/ 필요시 생략가능)
    # --------------------------------
    print(">>> [Gripper] Closing...", flush=True)
    set_digital_output(2, 0)  # OPEN OFF
    set_digital_output(1, 1)  # CLOSE ON
    time.sleep(1.0)

    # --------------------------------
    # 1️⃣ 강한 기울기
    # --------------------------------
    pour_pose = list(current_joints)
    pour_pose[4] = current_joints[4] - 110.0
    pour_pose[5] = current_joints[5] - 140.0

    movej(pour_pose, vel=VEL_POUR, acc=ACC_POUR)
    time.sleep(3.0)

    # --------------------------------
    # 2️⃣ 오버 틸트 (필요시 생략 or 추가)
    # --------------------------------
    # over_pose = list(pour_pose)
    # over_pose[4] -= 20.0
    # over_pose[5] -= 20.0

    # movej(over_pose, vel=10, acc=8)
    # time.sleep(2.0)

    # --------------------------------
    # 3️⃣ 쉐이킹 (필요시 생략 or 추가)
    # --------------------------------
    """
        for _ in range(3):
        shake = list(over_pose)
        shake[5] += 8.0
        movej(shake, vel=25, acc=20)
        time.sleep(0.3)

        movej(over_pose, vel=25, acc=20)
        time.sleep(0.3)

    """


    # --------------------------------
    # 4️⃣ 복귀 (for testing -> 필요시 생략 가능)
    # --------------------------------
    print(">>> [Module] 복귀 중...", flush=True)
    movej(current_joints, vel=VEL_RETURN, acc=ACC_RETURN)
    time.sleep(2.0)

    # --------------------------------
    # 🔥 5️⃣ 복귀 후 그리퍼 OPEN (필요시 생략가능)
    # --------------------------------
    print(">>> [Gripper] Opening...", flush=True)
    set_digital_output(1, 0)  # CLOSE OFF
    set_digital_output(2, 1)  # OPEN ON
    time.sleep(1.0)

    print(">>> [Module] 붓기 완료 (Gripper 포함)", flush=True)
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
        # pour_action_frame(portion=1)
        pour_action()

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
