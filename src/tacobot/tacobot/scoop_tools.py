# scoop_tools.py
import time

def scoop_action(p0_list, p1_list, p2_list, p3_list):
    # 함수 안에서 import
    from DSR_ROBOT2 import posx, posj, movec, movej, movel

    print(">>> [Module] 스쿠핑 작업 시작 (Scooping)...", flush=True)

    P0 = posj(*p0_list) # 시작 관절
    P1 = posx(*p1_list) # 경유점
    P2 = posx(*p2_list) # 목표점
    P3 = posx(*p3_list) # 들어올리기

    # 2. 시작 위치로 이동
    movej(P0, vel=80, acc=70)
    time.sleep(2.0) # 이동 대기

    # 3. 반복 동작 (3회)
    for i in range(3): 
        print(f"   >>> [Scoop] Cycle {i+1}/3", flush=True)
        
        # P1 -> P2 원호 이동
        movec(P1, P2, vel=120, acc=100)
        time.sleep(1.5) # [추가] 원 그리는 시간 대기
        
        # P3 수직 상승
        movel(P3, vel=100, acc=90)
        time.sleep(1.0) # [추가] 들어올리는 시간 대기
        
        # P0 원위치 복귀
        movej(P0, vel=100, acc=90)
        time.sleep(1.5) # [추가] 복귀 시간 대기
        
    print(">>> [Module] 스쿠핑 완료", flush=True)