import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json


class MainDecisionNode(Node):
    def __init__(self):
        super().__init__('main_decision_node')

        # 1. 구독 (수신기): 비전 노드로부터 정다면체 좌표 수신
        self.vision_sub = self.create_subscription(
            String,
            '/detected_objects',
            self.vision_callback,
            10)

        # 2. 발행 (송신기): UGV 컨트롤러로 이동 명령(속도) 송신
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 3. 제어 루프 타이머: 0.1초(10Hz)마다 지속적으로 명령을 내림
        # (ugv_controller의 0.5초 워치독 타임아웃 방지용)
        self.timer = self.create_timer(0.1, self.control_loop)

        # 상태 저장용 변수
        self.target_area = 0  # 객체의 크기(면적)
        self.last_seen_time = self.get_clock().now()  # 마지막으로 객체를 발견한 시간

        # ⚙️ 튜닝 파라미터 (로봇을 실제 굴려보며 숫자를 수정해야 합니다)
        self.AREA_THRESHOLD = 40000  # 객체가 '크다'고 판단할 면적 기준값 (가로 x 세로)
        self.SPEED = 0.2  # 기본 전진 속도

        self.get_logger().info('🧠 대뇌 노드 실행 완료! 시각 정보 대기 중...')

    def vision_callback(self, msg):
        try:
            # yolo_vision_node에서 보낸 JSON 문자열을 딕셔너리로 변환
            data = json.loads(msg.data)

            # 박스의 너비(w)와 높이(h)를 곱해서 객체의 크기(면적)를 계산
            w = data.get('w', 0)
            h = data.get('h', 0)
            self.target_area = w * h

            # 객체를 발견한 현재 시간을 갱신 (생존 신고)
            self.last_seen_time = self.get_clock().now()

        except Exception as e:
            self.get_logger().error(f"데이터 파싱 오류: {e}")

    def control_loop(self):
        twist = Twist()

        # 객체를 본 지 0.5초가 지났는지 확인 (시야에서 사라짐 판별)
        time_since_last_seen = (self.get_clock().now() - self.last_seen_time).nanoseconds / 1e9

        if time_since_last_seen > 0.5:
            # 1. 시야에 아무것도 없으면 무조건 정지
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("👀 정다면체가 보이지 않음. 정지 대기 중...", throttle_duration_sec=2.0)

        else:
            # 2. 시야에 정다면체가 있는 경우 크기(면적) 비교
            if self.target_area < self.AREA_THRESHOLD:
                # [조건 1] 작게 보임 -> 아직 멀리 있음 -> 전진!
                twist.linear.x = self.SPEED
                twist.angular.z = 0.0
                self.get_logger().info(f"🚀 객체 작음(면적: {self.target_area}) -> 전진 중", throttle_duration_sec=1.0)
            else:
                # [조건 2] 크게 보임 -> 목표물 근접 -> 정지!
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info(f"🛑 목표물 도달!(면적: {self.target_area}) -> 정지", throttle_duration_sec=1.0)

        # UGV 노드(/cmd_vel)로 계산된 명령을 퍼블리시
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = MainDecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()