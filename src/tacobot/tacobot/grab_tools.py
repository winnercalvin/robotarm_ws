# grab_tools.py
from DSR_ROBOT2 import set_digital_output, get_digital_input, movej, wait

# 속도 설정 (필요시 수정)
VELOCITY, ACC = 50, 50
ON, OFF = 1, 0

def wait_digital_input(sig_num):
    while not get_digital_input(sig_num):
        wait(0.5)

def set_gripper_signal(s1, s2, s3):
    set_digital_output(1, s1)
    set_digital_output(2, s2)
    set_digital_output(3, s3)
    wait(0.1) 

def release():
    print(">>> [Module] Releasing (0 1 0)...")
    set_gripper_signal(OFF, ON, OFF)
    wait(1.5) 
    print(">>> [Module] Release 완료")

def grip():
    print(">>> [Module] Gripping (1 0 0)...")
    set_gripper_signal(ON, OFF, OFF)
    wait(1.5) 
    print(">>> [Module] Grip 완료")

def sauce_grip():
    """소스통 전용 3비트(111) 그립"""
    # 🚨 [주의] 1, 2, 3은 예시 포트 번호입니다. 실제 로봇에 연결된 포트 번호로 수정하세요!
    set_digital_output(1, 1)
    set_digital_output(2, 1)
    set_digital_output(3, 1)
    print("   >>> [Grip] 소스통 3비트(111) 그립 완료!", flush=True)
