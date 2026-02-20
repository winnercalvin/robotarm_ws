#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# move_second_ pour.py

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
# second pour
###############################################################
def pour_action_second():
    from DSR_ROBOT2 import (
        movej,
        movel,
        get_current_posj,
        get_current_posx,
        set_digital_output
    )
    import time

    VEL_POUR, ACC_POUR = 15, 10
    VEL_MOVE, ACC_MOVE = 40, 30

    print(">>> [Task2] 2차 Pour 시작", flush=True)

    # 현재 joint / tcp 저장
    current_joints = list(get_current_posj())
    current_posx = list(get_current_posx())

    # --------------------------------------------------
    # 0️⃣ 그리퍼 CLOSE (안전하게 다시 잡기)
    # --------------------------------------------------
    # print(">>> [Gripper] Closing...", flush=True)
    # set_digital_output(2, 0)
    # set_digital_output(1, 1)
    # time.sleep(1.0)

    # --------------------------------------------------
    # 1️⃣ +Z 방향 40mm 상승
    # --------------------------------------------------
    print(">>> [Step1] Z축 40mm 상승", flush=True)

    lift_pose = list(current_posx)
    lift_pose[2] += 40.0   # Z + 40mm

    movel(lift_pose, vel=VEL_MOVE, acc=ACC_MOVE)
    time.sleep(1.5)

    # --------------------------------------------------
    # 2️⃣ 2차 Pour Joint 상태로 이동
    # --------------------------------------------------
    print(">>> [Step2] 2차 Pour 위치로 이동", flush=True)

    second_pour_joint = [
        9.83,
        -18.83,
        123.52,
        -33.14,
        55.37,
        -112.2
    ]

    movej(second_pour_joint, vel=VEL_MOVE, acc=ACC_MOVE)
    time.sleep(2.0)

    # --------------------------------------------------
    # 3️⃣ 해당 위치에서 트레이 기울이기 (J5 + J6)
    # --------------------------------------------------
    print(">>> [Step3] 트레이 기울이기", flush=True)

    base_pour_pose = list(second_pour_joint)

    # 🔥 강한 기울기
    pour_pose = list(base_pour_pose)
    pour_pose[4] -= 100.0   # J5
    pour_pose[5] -= 120.0   # J6

    movej(pour_pose, vel=VEL_POUR, acc=ACC_POUR)
    time.sleep(3.0)

    # 🔥 오버 틸트
    # over_pose = list(pour_pose)
    # over_pose[4] -= 20.0
    # over_pose[5] -= 20.0

    # movej(over_pose, vel=10, acc=8)
    # time.sleep(2.0)

    # 🔥 쉐이킹
    """
    for _ in range(3):
        shake = list(over_pose)
        shake[5] += 8.0
        movej(shake, vel=25, acc=20)
        time.sleep(0.3)

        movej(over_pose, vel=25, acc=20)
        time.sleep(0.3)

    """
    
    # --------------------------------------------------
    # 4️⃣ 원위치 복귀
    # --------------------------------------------------
    print(">>> [Step4] 원위치 복귀", flush=True)

    movej(second_pour_joint, vel=VEL_MOVE, acc=ACC_MOVE)
    time.sleep(1.5)

    movej(current_joints, vel=VEL_MOVE, acc=ACC_MOVE)
    time.sleep(2.0)

    # --------------------------------------------------
    # 5️⃣ Gripper OPEN
    # --------------------------------------------------
    print(">>> [Gripper] Opening...", flush=True)
    set_digital_output(1, 0)
    set_digital_output(2, 1)
    time.sleep(1.0)

    print(">>> [Task2] 2차 Pour 완료", flush=True)



def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("move_basic", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        initialize_robot()
        pour_action_second()

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

