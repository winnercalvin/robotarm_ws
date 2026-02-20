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
    VEL_POUR, ACC_POUR = 20, 15
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
    first_tilt = list(current_joints)
    first_tilt[5] -= 90.0
    print("   >>> [Pour] 1차 틸팅 진행 중...", flush=True)
    move_and_wait_func(first_tilt, VEL_POUR, ACC_POUR)


    # # -----------------------------
    # # 3. 2차 틸팅 (완전 배출)
    # # -----------------------------
    # second_tilt = list(first_tilt)
    # second_tilt[5] -= 30.0
    # print("   >>> [Pour] 2차 틸팅 진행 중...", flush=True)
    # move_and_wait_func(second_tilt, VEL_POUR, ACC_POUR)

    # # -----------------------------
    # # 4. 오버틸트 (잔류물 모으기)
    # # -----------------------------
    # over_tilt = list(second_tilt)
    # over_tilt[5] -= 30.0    
    # print("   >>> [Pour] 오버틸트 진행 중...", flush=True)
    # move_and_wait_func(over_tilt, 15, 10)

    # # -----------------------------
    # # 5. 마이크로 쉐이킹 (잔여물 털기)
    # # -----------------------------
    # print("   >>> [Pour] 마이크로 쉐이킹 시작!", flush=True)
    # for _ in range(2):
    #     shake_up = list(over_tilt)
    #     shake_up[5] += 5.0
    #     # 쉐이킹은 매우 짧은 동작이므로 move_and_wait 대신 기본 movej + sleep 사용
    #     movej(shake_up, vel=30, acc=20) 
    #     time.sleep(0.5)

    #     shake_down = list(over_tilt)
    #     shake_down[5] -= 5.0
    #     movej(shake_down, vel=30, acc=20)
    #     time.sleep(0.5)

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
    # 6. 원위치 복귀 (빠르게)
    # -----------------------------
    print("   >>> [Pour] 붓기 완료. 원위치 복귀 중...", flush=True)
    move_and_wait_func(current_joints, VEL_RETURN, ACC_RETURN)

    print("   ✅ [Pour] 최적화 붓기 시퀀스 종료!", flush=True)