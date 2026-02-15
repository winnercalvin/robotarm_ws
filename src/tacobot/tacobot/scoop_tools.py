# scoop_tools.py
from DSR_ROBOT2 import posx, posj, movec, movej, movel, wait
import time

# 인자로 p0(관절), p1, p2, p3(좌표)를 받습니다.
def scoop_action(p0_list, p1_list, p2_list, p3_list):
    print(">>> [Module] 스쿠핑 작업 시작 (Scooping)...", flush=True)

    # 1. 리스트로 들어온 값을 DSR 전용 객체(posj, posx)로 변환
    # *p0_list는 리스트의 껍질을 벗겨서 인자로 넣는 문법입니다.
    P0 = posj(*p0_list) # 시작 관절
    P1 = posx(*p1_list) # 경유점
    P2 = posx(*p2_list) # 목표점
    P3 = posx(*p3_list) # 들어올리기

    # 2. 시작 위치로 이동
    # 초기 위치 잡기
    movej(P0, vel=80, acc=70)
    time.sleep(2.0) # 이동 안정화

    # 3. 반복 동작 (3회)
    for i in range(3): 
        print(f"   >>> [Scoop] Cycle {i+1}/3", flush=True)
        
        # P1을 거쳐 P2로 둥글게 이동
        movec(P1, P2, vel=120, acc=100)
        
        # P3로 수직 상승
        movel(P3, vel=100, acc=90)
        
        # P0(원위치)로 복귀
        movej(P0, vel=100, acc=90)
        
    print(">>> [Module] 스쿠핑 완료", flush=True)