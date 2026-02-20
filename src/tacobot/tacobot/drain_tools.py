import time

def drain_action(p1_list, p2_list):
    # 안전하게 함수 내부에서 import
    from DSR_ROBOT2 import posx, movel

    print(">>> [Module] 기름 털기(Drain) 동작 시작...", flush=True)

    # 리스트로 들어온 값을 DSR 전용 객체(posx)로 변환
    P1 = posx(*p1_list)
    P2 = posx(*p2_list)

    # 3회 반복 털기
    for i in range(3):
        print(f"   >>> [Drain] Cycle {i+1}/3", flush=True)
        
        # P1으로 이동
        movel(P1, vel=90, acc=90)
        time.sleep(1.5) # 이동 대기 시간
        
        # 1초 대기
        time.sleep(1.0) 
        
        # P2로 강하게 치기
        movel(P2, vel=700, acc=2500)
        time.sleep(1.0) # 이동 대기 시간

    print(">>> [Module] 기름 털기(Drain) 완료", flush=True)