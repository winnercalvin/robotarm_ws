# tacobot/drizzle_tools.py
import math
import time

def filter_points(path, min_dist, drawing_size):
    """점 간의 거리가 min_dist보다 짧으면 제거하는 함수"""
    if not path:
        return []
    
    filtered = [path[0]] # 첫 번째 점은 포함
    for i in range(1, len(path)):
        # 정규화 좌표 차이에 DRAWING_SIZE를 곱해 실제 mm 단위 거리 계산
        dx = (path[i]['x'] - filtered[-1]['x']) * drawing_size
        dy = (path[i]['y'] - filtered[-1]['y']) * drawing_size
        dist = math.sqrt(dx**2 + dy**2) 

        if dist >= min_dist:
            filtered.append(path[i])
            
    return filtered

def custom_drizzle(flat_data):
    """액션 서버로부터 받은 숫자 배열([x1, y1, x2, y2...])로 소스 뿌리기"""
    from DSR_ROBOT2 import get_current_posx, movel, posx, wait, DR_BASE
    
    print("   >>> [Module] 커스텀 소스 그리기 데이터 파싱 중...", flush=True)

    MIN_DISTANCE = 5.0    
    DRAWING_SIZE = 91.0   
    Z_PRESS_DEPTH = 50.0  
    VELOCITY = 60
    ACC = 80
    
    # 🌟 [추가된 부분] 1차원 리스트를 다시 딕셔너리 리스트로 복원
    raw_path = []
    for i in range(0, len(flat_data), 2):
        # i는 x좌표, i+1은 y좌표
        raw_path.append({'x': flat_data[i], 'y': flat_data[i+1]})
    
    # 3. 필터링 (이후 로직은 기존과 100% 동일)
    filtered_path = filter_points(raw_path, MIN_DISTANCE, DRAWING_SIZE)

    if not filtered_path:
        print("   >>> ❌ 그릴 좌표가 없습니다. 그리기를 취소합니다.", flush=True)
        return

    # 4. 현재 위치 확인 (여기가 캔버스의 (0,0) 원점이 됩니다!)
    try:
        response = get_current_posx(ref=DR_BASE)
        if isinstance(response, tuple): current_pos = response[0]
        else: current_pos = response
        bx, by, bz, ba, bb, bc = current_pos
    except Exception as e:
        print(f"   >>> ❌ [Error] 현재 위치를 읽을 수 없습니다: {e}", flush=True)
        return

    print(f"   >>> [Module] 캔버스 원점 설정 완료 (X:{bx:.2f}, Y:{by:.2f})", flush=True)

    # 5. 경로 따라 그리기
    for i, pt in enumerate(filtered_path):
        tx = bx + (pt['x'] * DRAWING_SIZE)
        ty = by - (pt['y'] * DRAWING_SIZE)

        if i == 0:
            # 첫 번째 점 상공으로 수평 이동 후 하강
            movel(posx([tx, ty, bz, ba, bb, bc]), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            print("   >>> [Module] 소스 뿌리기 깊이로 하강!", flush=True)
            movel(posx([tx, ty, bz - Z_PRESS_DEPTH, ba, bb, bc]), vel=VELOCITY/2, acc=ACC, ref=DR_BASE)
            wait(0.2)
        else:
            # 두 번째 점부터는 종이에 닿은 상태로 연속 이동 (반경 2.0으로 부드럽게)
            target_pos = posx([tx, ty, bz - Z_PRESS_DEPTH, ba, bb, bc])
            if i < len(filtered_path) - 1:
                movel(target_pos, vel=VELOCITY, acc=ACC, radius=2.0, ref=DR_BASE)
            else:
                movel(target_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)

    # 6. 안전 종료: 다 그렸으면 펜(소스통) 들어 올리기
    wait(0.2)
    print("   >>> [Module] 소스 뿌리기 완료! 위로 상승합니다.", flush=True)
    last_pt = filtered_path[-1]
    last_tx = bx + (last_pt['x'] * DRAWING_SIZE)
    last_ty = by - (last_pt['y'] * DRAWING_SIZE)
    
    # 펜을 원래 있던 높이(bz + 50)로 들어 올림
    movel(posx([last_tx, last_ty, bz + 50, ba, bb, bc]), vel=VELOCITY, acc=ACC, ref=DR_BASE)
    time.sleep(1.0)