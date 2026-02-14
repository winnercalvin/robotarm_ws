import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from tacobot_interfaces.action import RobotTask # 인터페이스 임포트

class TaskController(Node):

    def __init__(self):
        super().__init__('task_controller')
        
        # 1. 액션 클라이언트 생성 ('scooper_grab' 서버를 찾음)
        self._action_client = ActionClient(self, RobotTask, '/dsr01/scooper_grab')

    def send_goal(self, joints, task_type):
        """
        서버에게 작업을 지시하는 함수
        :param joints: 이동할 관절 각도 리스트 [J1, J2, J3, J4, J5, J6]
        :param task_type: 0(이동만), 1(집기), 2(놓기)
        """
        goal_msg = RobotTask.Goal()
        goal_msg.target_joints = joints
        goal_msg.task_type = task_type

        # 서버가 켜질 때까지 대기
        print("액션 서버(scooper_grab)를 찾는 중...")
        self._action_client.wait_for_server()
        print("서버 연결 완료! 명령을 전송합니다.")

        # 명령 전송 (Feedback이 오면 feedback_callback 함수 실행)
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        
        # 명령이 잘 접수되었는지 확인하는 콜백 연결
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """서버가 명령을 수락했는지 확인"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("❌ 서버가 명령을 거절했습니다.")
            return

        print("✅ 서버가 명령을 수락했습니다. 작업 진행 중...")
        
        # 결과가 나올 때까지 기다림
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """작업 중간중간 서버가 보내주는 상태 메시지 출력"""
        status = feedback_msg.feedback.status
        print(f"📢 [상태 보고] {status}")

    def get_result_callback(self, future):
        """작업이 완전히 끝났을 때 결과 출력"""
        result = future.result().result
        print("=" * 40)
        print(f"🏁 작업 종료!")
        print(f"성공 여부: {result.success}")
        print(f"결과 메시지: {result.message}")
        print("=" * 40)
        
        # 작업이 끝났으니 노드 종료
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    controller = TaskController()

    # ==========================================
    # 👇 [여기만 수정하세요] 목표 좌표 및 행동 입력
    # ==========================================
    
    # 1. 가고 싶은 관절 좌표 (아까 로그에서 본 값)
    target_pos = [-21.679, 31.319, 73.279, 4.451, 61.702, -0.439]
    
    # 2. 가서 할 행동 (1: 집기, 2: 놓기, 0: 이동만)
    action_type = 1 

    # ==========================================

    # 명령 전송
    controller.send_goal(target_pos, action_type)

    # 결과가 올 때까지 대기
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("종료")
    except Exception as e:
        # 이미 종료된 경우(rclpy.shutdown) 예외 처리
        pass

if __name__ == '__main__':
    main()