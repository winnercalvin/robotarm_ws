# tacobot/pour_tools.py
import time

def pour_action(move_and_wait_func=None):
    from DSR_ROBOT2 import movej, get_current_posj, move_periodic, DR_TOOL

    # action_server에서 move_and_wait를 넘겨받지 못한 경우를 대비한 기본 함수
    if move_and_wait_func is None:
        def default_move(target, v, a):
            movej(target, vel=v, acc=a)
            time.sleep(2.0)
        move_and_wait_func = default_move

    # 속도 분리 (기울일 때는 느리게, 복귀는 빠르게)
    VEL_POUR, ACC_POUR = 15, 10
    VEL_RETURN, ACC_RETURN = 40, 30

    PERIOD = 2.0
    REPEAT = 3
    WAIT_TIME = (PERIOD * REPEAT) + 0.5 # 여유 시간 0.5초 추가

    print("   >>> [Pour] 최적화 붓기(Pouring) 시작...", flush=True)

    # 1. 현재 조인트 위치 저장
    current_joints = list(get_current_posj())

    # -----------------------------
    # 2. 1차 틸팅 (부드럽게)
    # -----------------------------
    pour_pose = list(current_joints)
    pour_pose[4] = current_joints[4] - 110.0
    pour_pose[5] = current_joints[5] - 140.0

    movej(pour_pose, vel=VEL_POUR, acc=ACC_POUR)
    time.sleep(3.0)

    # 6. Y축 방향 흔들기 (좌우)
    print(f"   >>> [Shake] 좌우 흔들기 시작 ({WAIT_TIME}s wait)", flush=True)
    move_periodic(
        amp=[0, 0, 50, 0, 0, 0],
        period=PERIOD,
        atime=0.2,
        repeat=REPEAT,
        ref=DR_TOOL
    )
    time.sleep(WAIT_TIME)

    # -----------------------------
    # 6. 원위치 복귀 
    # -----------------------------
    print("   >>> [Pour] 붓기 완료. 원위치 복귀 중...", flush=True)
    move_and_wait_func(current_joints, VEL_RETURN, ACC_RETURN)

    print("   ✅ [Pour] 최적화 붓기 시퀀스 종료!", flush=True)