# pour_tools.py
import time

def pour_action():
    # 함수 안에서 import
    from DSR_ROBOT2 import movej, get_current_posj
    
    VELOCITY, ACC = 30, 20
    print(">>> [Module] 붓기 시작 (Tilting)...", flush=True)
    
    # 1. 현재 위치 저장
    current_joints = list(get_current_posj())
    
    # 2. 쏟는 각도 계산 (J6 -110도)
    target_pour_joints = list(current_joints)
    target_pour_joints[5] = target_pour_joints[5] - 110.0 
    
    # 3. 붓기 (이동)
    movej(target_pour_joints, vel=VELOCITY, acc=ACC)
    
    # 🚨 [수정] wait(0) 대신 time.sleep 사용
    # 붓는 동작 이동 시간(약 3초) + 쏟아지는 시간(2초)
    print("   >>> [Wait] 붓는 중... (5초 대기)", flush=True)
    time.sleep(5.0)
    
    # 4. 원위치
    print(">>> [Module] 원위치 복귀...", flush=True)
    movej(current_joints, vel=VELOCITY, acc=ACC)
    
    # 복귀 이동 시간 대기
    print("   >>> [Wait] 복귀 중... (3초 대기)", flush=True)
    time.sleep(3.0)
    
    print(">>> [Module] 붓기 완료", flush=True)