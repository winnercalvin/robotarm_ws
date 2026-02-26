import rclpy
import DR_init
import time
import math

# 로봇 설정 상수 (필요에 따라 수정)
ROBOT_ID = "dsr01"      # ROS2 네트워크에서 식별할 로봇의 이름
ROBOT_MODEL = "m0609"   # 사용 중인 두산 로봇 모델명
ROBOT_TOOL = "Tool Weight"    # 설정된 툴 무게 프로파일 이름
ROBOT_TCP = "GripperDA_v1"    # 도구 중심점(TCP) 설정 이름

# 이동 속도 및 가속도 (필요에 따라 수정)
VELOCITY = 60
ACC = 80
Z_FIXED = 150.0  # 그리기 작업을 수행할 고정 Z 높이 (mm)
MIN_DISTANCE = 5  # [설정] 이 거리(mm)보다 가까운 점은 제거합니다.
DRAWING_SIZE = 91.0  # [수정] (0,0)~(1,1)을 91mm x 91mm 크기로 매핑

# 입력 데이터: 웹/앱에서 전달받은 정규화된(0~1) 좌표 리스트
draw_path = [{'xasDouble': 0.403, 'yasDouble': 0.782, 'x': '0.403', 'y': '0.782', 'timestamp': 1771487516096}, {'xasDouble': 0.347, 'yasDouble': 0.724, 'x': '0.347', 'y': '0.724', 'timestamp': 1771487516142}, {'xasDouble': 0.312, 'yasDouble': 0.686, 'x': '0.312', 'y': '0.686', 'timestamp': 1771487516145}, {'xasDouble': 0.288, 'yasDouble': 0.657, 'x': '0.288', 'y': '0.657', 'timestamp': 1771487516159}, {'xasDouble': 0.27, 'yasDouble': 0.632, 'x': '0.270', 'y': '0.632', 'timestamp': 1771487516167}, {'xasDouble': 0.255, 'yasDouble': 0.609, 'x': '0.255', 'y': '0.609', 'timestamp': 1771487516176}, {'xasDouble': 0.24, 'yasDouble': 0.584, 'x': '0.240', 'y': '0.584', 'timestamp': 1771487516183}, {'xasDouble': 0.227, 'yasDouble': 0.559, 'x': '0.227', 'y': '0.559', 'timestamp': 1771487516192}, {'xasDouble': 0.215, 'yasDouble': 0.531, 'x': '0.215', 'y': '0.531', 'timestamp': 1771487516200}, {'xasDouble': 0.205, 'yasDouble': 0.502, 'x': '0.205', 'y': '0.502', 'timestamp': 1771487516208}, {'xasDouble': 0.198, 'yasDouble': 0.472, 'x': '0.198', 'y': '0.472', 'timestamp': 1771487516217}, {'xasDouble': 0.195, 'yasDouble': 0.444, 'x': '0.195', 'y': '0.444', 'timestamp': 1771487516226}, {'xasDouble': 0.193, 'yasDouble': 0.414, 'x': '0.193', 'y': '0.414', 'timestamp': 1771487516233}, {'xasDouble': 0.193, 'yasDouble': 0.384, 'x': '0.193', 'y': '0.384', 'timestamp': 1771487516241}, {'xasDouble': 0.2, 'yasDouble': 0.352, 'x': '0.200', 'y': '0.352', 'timestamp': 1771487516250}, {'xasDouble': 0.207, 'yasDouble': 0.324, 'x': '0.207', 'y': '0.324', 'timestamp': 1771487516258}, {'xasDouble': 0.215, 'yasDouble': 0.299, 'x': '0.215', 'y': '0.299', 'timestamp': 1771487516266}, {'xasDouble': 0.227, 'yasDouble': 0.278, 'x': '0.227', 'y': '0.278', 'timestamp': 1771487516274}, {'xasDouble': 0.24, 'yasDouble': 0.254, 'x': '0.240', 'y': '0.254', 'timestamp': 1771487516283}, {'xasDouble': 0.257, 'yasDouble': 0.233, 'x': '0.257', 'y': '0.233', 'timestamp': 1771487516292}, {'xasDouble': 0.277, 'yasDouble': 0.212, 'x': '0.277', 'y': '0.212', 'timestamp': 1771487516300}, {'xasDouble': 0.298, 'yasDouble': 0.198, 'x': '0.298', 'y': '0.198', 'timestamp': 1771487516308}, {'xasDouble': 0.32, 'yasDouble': 0.186, 'x': '0.320', 'y': '0.186', 'timestamp': 1771487516316}, {'xasDouble': 0.343, 'yasDouble': 0.177, 'x': '0.343', 'y': '0.177', 'timestamp': 1771487516324}, {'xasDouble': 0.367, 'yasDouble': 0.172, 'x': '0.367', 'y': '0.172', 'timestamp': 1771487516333}, {'xasDouble': 0.387, 'yasDouble': 0.172, 'x': '0.387', 'y': '0.172', 'timestamp': 1771487516341}, {'xasDouble': 0.407, 'yasDouble': 0.177, 'x': '0.407', 'y': '0.177', 'timestamp': 1771487516349}, {'xasDouble': 0.425, 'yasDouble': 0.184, 'x': '0.425', 'y': '0.184', 'timestamp': 1771487516357}, {'xasDouble': 0.442, 'yasDouble': 0.196, 'x': '0.442', 'y': '0.196', 'timestamp': 1771487516366}, {'xasDouble': 0.455, 'yasDouble': 0.209, 'x': '0.455', 'y': '0.209', 'timestamp': 1771487516374}, {'xasDouble': 0.47, 'yasDouble': 0.226, 'x': '0.470', 'y': '0.226', 'timestamp': 1771487516382}, {'xasDouble': 0.483, 'yasDouble': 0.244, 'x': '0.483', 'y': '0.244', 'timestamp': 1771487516391}, {'xasDouble': 0.495, 'yasDouble': 0.263, 'x': '0.495', 'y': '0.263', 'timestamp': 1771487516399}, {'xasDouble': 0.507, 'yasDouble': 0.282, 'x': '0.507', 'y': '0.282', 'timestamp': 1771487516407}, {'xasDouble': 0.517, 'yasDouble': 0.304, 'x': '0.517', 'y': '0.304', 'timestamp': 1771487516415}, {'xasDouble': 0.523, 'yasDouble': 0.326, 'x': '0.523', 'y': '0.326', 'timestamp': 1771487516424}, {'xasDouble': 0.53, 'yasDouble': 0.351, 'x': '0.530', 'y': '0.351', 'timestamp': 1771487516432}, {'xasDouble': 0.532, 'yasDouble': 0.374, 'x': '0.532', 'y': '0.374', 'timestamp': 1771487516440}, {'xasDouble': 0.533, 'yasDouble': 0.401, 'x': '0.533', 'y': '0.401', 'timestamp': 1771487516449}, {'xasDouble': 0.533, 'yasDouble': 0.426, 'x': '0.533', 'y': '0.426', 'timestamp': 1771487516457}, {'xasDouble': 0.533, 'yasDouble': 0.448, 'x': '0.533', 'y': '0.448', 'timestamp': 1771487516465}, {'xasDouble': 0.533, 'yasDouble': 0.468, 'x': '0.533', 'y': '0.468', 'timestamp': 1771487516473}, {'xasDouble': 0.53, 'yasDouble': 0.484, 'x': '0.530', 'y': '0.484', 'timestamp': 1771487516481}, {'xasDouble': 0.53, 'yasDouble': 0.496, 'x': '0.530', 'y': '0.496', 'timestamp': 1771487516490}, {'xasDouble': 0.527, 'yasDouble': 0.501, 'x': '0.527', 'y': '0.501', 'timestamp': 1771487516498}, {'xasDouble': 0.527, 'yasDouble': 0.504, 'x': '0.527', 'y': '0.504', 'timestamp': 1771487516506}, {'xasDouble': 0.527, 'yasDouble': 0.502, 'x': '0.527', 'y': '0.502', 'timestamp': 1771487516517}, {'xasDouble': 0.527, 'yasDouble': 0.501, 'x': '0.527', 'y': '0.501', 'timestamp': 1771487516525}, {'xasDouble': 0.528, 'yasDouble': 0.491, 'x': '0.528', 'y': '0.491', 'timestamp': 1771487516533}, {'xasDouble': 0.535, 'yasDouble': 0.472, 'x': '0.535', 'y': '0.472', 'timestamp': 1771487516541}, {'xasDouble': 0.547, 'yasDouble': 0.454, 'x': '0.547', 'y': '0.454', 'timestamp': 1771487516550}, {'xasDouble': 0.56, 'yasDouble': 0.434, 'x': '0.560', 'y': '0.434', 'timestamp': 1771487516558}, {'xasDouble': 0.575, 'yasDouble': 0.412, 'x': '0.575', 'y': '0.412', 'timestamp': 1771487516566}, {'xasDouble': 0.593, 'yasDouble': 0.393, 'x': '0.593', 'y': '0.393', 'timestamp': 1771487516574}, {'xasDouble': 0.61, 'yasDouble': 0.372, 'x': '0.610', 'y': '0.372', 'timestamp': 1771487516583}, {'xasDouble': 0.628, 'yasDouble': 0.357, 'x': '0.628', 'y': '0.357', 'timestamp': 1771487516591}, {'xasDouble': 0.647, 'yasDouble': 0.343, 'x': '0.647', 'y': '0.343', 'timestamp': 1771487516600}, {'xasDouble': 0.665, 'yasDouble': 0.333, 'x': '0.665', 'y': '0.333', 'timestamp': 1771487516607}, {'xasDouble': 0.683, 'yasDouble': 0.326, 'x': '0.683', 'y': '0.326', 'timestamp': 1771487516616}, {'xasDouble': 0.702, 'yasDouble': 0.324, 'x': '0.702', 'y': '0.324', 'timestamp': 1771487516624}, {'xasDouble': 0.72, 'yasDouble': 0.324, 'x': '0.720', 'y': '0.324', 'timestamp': 1771487516633}, {'xasDouble': 0.737, 'yasDouble': 0.328, 'x': '0.737', 'y': '0.328', 'timestamp': 1771487516641}, {'xasDouble': 0.753, 'yasDouble': 0.338, 'x': '0.753', 'y': '0.338', 'timestamp': 1771487516649}, {'xasDouble': 0.765, 'yasDouble': 0.349, 'x': '0.765', 'y': '0.349', 'timestamp': 1771487516657}, {'xasDouble': 0.773, 'yasDouble': 0.366, 'x': '0.773', 'y': '0.366', 'timestamp': 1771487516666}, {'xasDouble': 0.775, 'yasDouble': 0.384, 'x': '0.775', 'y': '0.384', 'timestamp': 1771487516674}, {'xasDouble': 0.775, 'yasDouble': 0.406, 'x': '0.775', 'y': '0.406', 'timestamp': 1771487516682}, {'xasDouble': 0.773, 'yasDouble': 0.429, 'x': '0.773', 'y': '0.429', 'timestamp': 1771487516691}, {'xasDouble': 0.765, 'yasDouble': 0.453, 'x': '0.765', 'y': '0.453', 'timestamp': 1771487516699}, {'xasDouble': 0.753, 'yasDouble': 0.479, 'x': '0.753', 'y': '0.479', 'timestamp': 1771487516707}, {'xasDouble': 0.738, 'yasDouble': 0.504, 'x': '0.738', 'y': '0.504', 'timestamp': 1771487516715}, {'xasDouble': 0.722, 'yasDouble': 0.529, 'x': '0.722', 'y': '0.529', 'timestamp': 1771487516723}, {'xasDouble': 0.7, 'yasDouble': 0.557, 'x': '0.700', 'y': '0.557', 'timestamp': 1771487516732}, {'xasDouble': 0.678, 'yasDouble': 0.584, 'x': '0.678', 'y': '0.584', 'timestamp': 1771487516740}, {'xasDouble': 0.655, 'yasDouble': 0.614, 'x': '0.655', 'y': '0.614', 'timestamp': 1771487516748}, {'xasDouble': 0.63, 'yasDouble': 0.641, 'x': '0.630', 'y': '0.641', 'timestamp': 1771487516757}, {'xasDouble': 0.605, 'yasDouble': 0.667, 'x': '0.605', 'y': '0.667', 'timestamp': 1771487516765}, {'xasDouble': 0.578, 'yasDouble': 0.693, 'x': '0.578', 'y': '0.693', 'timestamp': 1771487516774}, {'xasDouble': 0.555, 'yasDouble': 0.714, 'x': '0.555', 'y': '0.714', 'timestamp': 1771487516782}, {'xasDouble': 0.532, 'yasDouble': 0.734, 'x': '0.532', 'y': '0.734', 'timestamp': 1771487516789}, {'xasDouble': 0.51, 'yasDouble': 0.749, 'x': '0.510', 'y': '0.749', 'timestamp': 1771487516798}, {'xasDouble': 0.488, 'yasDouble': 0.761, 'x': '0.488', 'y': '0.761', 'timestamp': 1771487516806}, {'xasDouble': 0.467, 'yasDouble': 0.771, 'x': '0.467', 'y': '0.771', 'timestamp': 1771487516815}, {'xasDouble': 0.448, 'yasDouble': 0.776, 'x': '0.448', 'y': '0.776', 'timestamp': 1771487516823}, {'xasDouble': 0.43, 'yasDouble': 0.776, 'x': '0.430', 'y': '0.776', 'timestamp': 1771487516832}, {'xasDouble': 0.422, 'yasDouble': 0.774, 'x': '0.422', 'y': '0.774', 'timestamp': 1771487516839}, {'xasDouble': 0.413, 'yasDouble': 0.774, 'x': '0.413', 'y': '0.774', 'timestamp': 1771487516847}, {'xasDouble': 0.407, 'yasDouble': 0.774, 'x': '0.407', 'y': '0.774', 'timestamp': 1771487516856}, {'xasDouble': 0.402, 'yasDouble': 0.774, 'x': '0.402', 'y': '0.774', 'timestamp': 1771487516864}, {'xasDouble': 0.398, 'yasDouble': 0.774, 'x': '0.398', 'y': '0.774', 'timestamp': 1771487516874}, {'xasDouble': 0.397, 'yasDouble': 0.772, 'x': '0.397', 'y': '0.772', 'timestamp': 1771487516881}, {'xasDouble': 0.395, 'yasDouble': 0.772, 'x': '0.395', 'y': '0.772', 'timestamp': 1771487516889}, {'xasDouble': 0.397, 'yasDouble': 0.772, 'x': '0.397', 'y': '0.772', 'timestamp': 1771487516929}]

# 로봇 제어에 필요한 [x, y] 형태의 리스트로 데이터 가공
drawing_path = [[item['xasDouble'], item['yasDouble']] for item in draw_path]

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# 필터링 함수
def filter_points(path, min_dist):
    """점 간의 거리가 min_dist보다 짧으면 제거하는 함수"""
    if not path:
        return []
    
    filtered = [path[0]] # 첫 번째 점은 포함
    for i in range(1, len(path)):
        # 정규화 좌표 차이에 DRAWING_SIZE를 곱해 실제 mm 단위 거리 계산
        dx = (path[i][0] - filtered[-1][0]) * DRAWING_SIZE
        dy = (path[i][1] - filtered[-1][1]) * DRAWING_SIZE
        dist = math.sqrt(dx**2 + dy**2) # 피타고라스 정리로 유클리드 거리 계산

        # 계산된 거리가 설정된 최소 거리(1.5mm)보다 클 때만 경로에 추가
        if dist >= min_dist:
            filtered.append(path[i])
            
    return filtered

def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp,get_tool,get_tcp,ROBOT_MODE_MANUAL,ROBOT_MODE_AUTONOMOUS  # 필요한 기능만 임포트
    from DSR_ROBOT2 import get_robot_mode,set_robot_mode

    # Tool과 TCP 설정시 매뉴얼 모드로 변경해서 진행
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)    # 툴 하중 설정 적용
    set_tcp(ROBOT_TCP)      # TCP 좌표계 적용
    
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(2)  # 설정 안정화를 위해 잠시 대기
    # 설정된 상수 출력
    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {get_tcp()}") 
    print(f"ROBOT_TOOL: {get_tool()}")
    # print(f"ROBOT_MODE 0:수동, 1:자동 : {get_robot_mode()}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#" * 50)

def perform_task(filtered_path):
    """배열된 좌표를 따라 그리기 수행"""
    from DSR_ROBOT2 import posx, movej, movel, wait, get_current_posx

    if not filtered_path:
        print("No points to draw.")
        return
    
    Z_PRESS_DEPTH = 50.0

    # 0. 안전한 시작 위치로 이동
    print("Moving to Home...")
    # movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACC)
    movej([-82.36, -6.31, 102.97, -59.17, 31.8, -213.59], vel=VELOCITY, acc=ACC)
    # posj(-82.36, -6.31, 102.97, -59.17, 31.8, -213.59)
    # posx(-75.61, -634.5, 409.36, 68.17, -111.54, -99.99)
    wait(0)
    
    # 1. 반환값 ([x,y,z,a,b,c], status) 처리
    # ([2.5166492357404056e-14, 6.25, 1035.0, 0.0, 0.0, 0.0], 0)
    # 형태를 안전하게 풀어서 bx, by, bz 등을 추출합니다.

    response = get_current_posx()
    
    try:
        # response[0]은 [x,y,z,a,b,c] 리스트, response[1]은 상태코드 0
        current_pos_list = response[0] 
        bx, by, bz, ba, bb, bc = current_pos_list
    except (IndexError, TypeError, ValueError):
        print(f"❌ 좌표 데이터를 파싱할 수 없습니다. 반환값: {response}")
        return
    
    print(f"Base coordinate set at: X:{bx:.2f}, Y:{by:.2f}, Z:{bz:.2f}")

    # 현재 위치를 데이터의 (0,0)으로 강제 설정
    origin_x = bx
    origin_y = by

    print(f"Set Current Position as Origin (0,0): X:{origin_x:.2f}, Y:{origin_y:.2f}")
    print(f"Drawing Area: {DRAWING_SIZE}mm x {DRAWING_SIZE}mm")

    # [2단계: 경로 따라 그리기]
    print(f"Drawing {len(filtered_path)} points...")
    for i, pt in enumerate(filtered_path):
        # --- 로그 추가 부분 ---
        current_idx = i + 1
        print(f"[{current_idx}/{len(filtered_path)}] Moving to point: ({pt[0]:.3f}, {pt[1]:.3f})")

        tx = origin_x + (pt[0] * DRAWING_SIZE)
        ty = origin_y - (pt[1] * DRAWING_SIZE)

        if i == 0:
            # 1. 공중에서 첫 번째 점의 XY 좌표로 수평 이동
            movel(posx([tx, ty, bz, ba, bb, bc]), vel=VELOCITY, acc=ACC)
            # 2. 해당 위치에서 수직 하강 (그리기 시작)
            print("Lowering pen to draw...")
            movel(posx([tx, ty, bz - Z_PRESS_DEPTH, ba, bb, bc]), vel=VELOCITY/2, acc=ACC)
            wait(0.2)
        
        else:
            # 두 번째 점부터는 종이에 닿은 상태(bz - Z_PRESS_DEPTH)로 연속 이동
            current_idx = i + 1            
            target_pos = posx([tx, ty, bz - Z_PRESS_DEPTH, ba, bb, bc])
            
            if i < len(filtered_path) - 1:
                movel(target_pos, vel=VELOCITY, acc=ACC, radius=2.0)
            else:
                movel(target_pos, vel=VELOCITY, acc=ACC)

    # [3단계: 안전 종료]
    # 안전 종료: 펜 들어올리기
    wait(0)
    print("Lifting pen...")
    last_x = bx + (filtered_path[-1][0] * DRAWING_SIZE)
    last_y = by - (filtered_path[-1][1] * DRAWING_SIZE)
    movel(posx([last_x, last_y, bz + 50, ba, bb, bc]), vel=VELOCITY, acc=ACC)
    print("Drawing Task Finished.")
    
def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("move_basic", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    try:
        # 초기화는 한 번만 수행
        initialize_robot()

        final_path = filter_points(drawing_path, MIN_DISTANCE)
        print(f"Total points after filtering: {len(final_path)}")

        # 작업 수행 (한 번만 호출)
        perform_task(final_path)

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()