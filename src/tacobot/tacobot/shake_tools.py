# shake_tools.py
import time

def shake_action():
    from DSR_ROBOT2 import move_periodic, DR_TOOL
    print(">>> [Module] 쉐이크 동작 시작 (Shaking)...", flush=True)
    
    # 시간 계산 (2초 * 3회 = 6초)
    PERIOD = 2.0
    REPEAT = 3
    WAIT_TIME = (PERIOD * REPEAT) + 0.5 # 여유 시간 0.5초 추가
    
    # 1. Z축 방향 흔들기 (위아래)
    print(f"   >>> [Shake] 위아래 흔들기 시작 ({WAIT_TIME}s wait)", flush=True)
    move_periodic(
        amp=[0, 0, 50, 0, 0, 0],
        period=PERIOD,
        atime=0.2,
        repeat=REPEAT,
        ref=DR_TOOL
    )
    
    # 🚨 [중요] 1번 동작이 끝날 때까지 여기서 파이썬이 기다려줍니다.
    time.sleep(WAIT_TIME)
    
    # 2. Y축 방향 흔들기 (좌우)
    print(f"   >>> [Shake] 좌우 흔들기 시작 ({WAIT_TIME}s wait)", flush=True)
    move_periodic(
        amp=[0, 50, 0, 0, 0, 0],
        period=PERIOD,
        atime=0.2,
        repeat=REPEAT,
        ref=DR_TOOL
    )
    
    # 🚨 2번 동작도 끝날 때까지 기다려야 다음 동작(이동)과 꼬이지 않습니다.
    time.sleep(WAIT_TIME)
    
    print(">>> [Module] 쉐이크 완료", flush=True)