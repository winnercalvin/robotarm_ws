# tacobot/scoop_tools.py
import time

def scoop_action(p1, p2, p3, p4, p5, p6, p7, p8):
    from DSR_ROBOT2 import movel, posx, set_velx, set_accx, DR_BASE
    
    print("   >>> [Scoop] 스쿱(Scooping) 동작 시작!", flush=True)
    
    # 속도 및 가속도 설정 (직교 공간)
    set_velx(60, 30)
    set_accx(100, 60)
    
    # 리스트로 묶어서 반복문으로 깔끔하게 처리
    waypoints = [p1, p2, p3, p4, p5, p6, p7, p8]
    
    for i, wp in enumerate(waypoints):
        # 배열을 posx 객체로 변환하여 직선 이동(movel)
        movel(posx(wp), ref=DR_BASE)
        # print(f"      - Point {i+1}/8 통과")
        
    print("   >>> [Scoop] 스쿱 동작 완료!", flush=True)