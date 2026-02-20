import time

def shake_action(direction="z"):
    from DSR_ROBOT2 import move_periodic, DR_TOOL
    
    print(f">>> [Module] 쉐이크 동작 시작 (모드: {direction})...", flush=True)
    
    PERIOD = 2.0
    REPEAT = 3
    WAIT_TIME = (PERIOD * REPEAT) + 0.5 
    
    # -----------------------------
    # 1. Z축 방향 흔들기 (위아래)
    # -----------------------------
    if direction == "z":
        print(f"   >>> [Shake] 위아래(Z축) 흔들기 시작 ({WAIT_TIME}s wait)", flush=True)
        move_periodic(
            amp=[0,0,30,0,0,0],
            period=0.4,
            atime=0.2,
            repeat=5,
            ref=DR_TOOL
        )
        time.sleep(WAIT_TIME)

    # -----------------------------
    # 2. Y축 방향 흔들기 (좌우)
    # -----------------------------
    elif direction == "y":
        print(f"   >>> [Shake] 좌우(Y축) 흔들기 시작 ({WAIT_TIME}s wait)", flush=True)
        move_periodic(
            amp=[20,0,0,0,0,0],
            period=0.4,
            atime=0.2,
            repeat=5,
            ref=DR_TOOL
        )
        time.sleep(WAIT_TIME)
    
    print(">>> [Module] 쉐이크 완료", flush=True)