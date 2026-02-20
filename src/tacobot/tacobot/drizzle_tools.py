# tacobot/drizzle_tools.py
import time

def drizzle_action():
    from DSR_ROBOT2 import get_current_posx, movesx, posx, DR_BASE
    
    print("   >>> [Module] 확실한 지그재그(Spline) 소스 뿌리기 시작!", flush=True)
    
    # 1. 현재 위치(Task 좌표) 확인
    try:
        current_pos = get_current_posx(ref=DR_BASE)[0]
    except TypeError:
        # 반환값이 튜플이 아닌 단일 배열일 경우 처리
        current_pos = get_current_posx(ref=DR_BASE)
        
    # 2. 지그재그 경유지 리스트 만들기
    waypoints = []
    
    # 설정값: 총 4번 꺾고, 앞으로 200mm 전진하며, 좌우로 40mm씩 움직임
    steps = 4
    y_step = 200.0 / steps  # 한 번에 전진할 Y축 거리 (50mm)
    x_amp = 40.0            # 좌우 X축 진폭 (40mm)
    
    for i in range(1, steps + 1):
        wp = list(current_pos)
        
        # 앞으로 이동
        wp[1] += y_step * i 
        
        # 짝수/홀수에 따라 좌우(X축)로 번갈아가며 좌표 생성
        if i % 2 == 1:
            wp[0] += x_amp  # 오른쪽으로
        else:
            wp[0] -= x_amp  # 왼쪽으로
            
        # movesx에 넣기 위해 posx 객체로 변환하여 리스트에 추가
        waypoints.append(posx(wp))

    # 3. 계산된 지그재그 좌표들을 멈춤 없이 한 번에 부드럽게(Spline) 이어 그리기
    print(f"   >>> [Drizzle] 총 {steps}개의 지그재그 좌표 논스톱 이동!", flush=True)
    
    # movesx: 여러 개의 Task 좌표를 멈추지 않고 곡선으로 이동
    movesx(waypoints, vel=60, acc=40, ref=DR_BASE)
    
    print("   >>> [Module] 소스 뿌리기 완료!", flush=True)