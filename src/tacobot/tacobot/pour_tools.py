# pour_tools.py
from DSR_ROBOT2 import movej, wait, get_current_posj

VELOCITY, ACC = 30, 20

def pour_action():
    print(">>> [Module] 붓기 시작 (Tilting)...")
    
    # 1. 현재 위치 저장
    current_joints = list(get_current_posj())
    
    # 2. 쏟는 각도 계산 (J6 -110도)
    target_pour_joints = list(current_joints)
    target_pour_joints[5] = target_pour_joints[5] - 110.0 
    
    # 3. 붓기
    movej(target_pour_joints, vel=VELOCITY, acc=ACC)
    wait(0)
    wait(2.0)
    
    # 4. 원위치
    print(">>> [Module] 원위치 복귀...")
    movej(current_joints, vel=VELOCITY, acc=ACC)
    wait(0)