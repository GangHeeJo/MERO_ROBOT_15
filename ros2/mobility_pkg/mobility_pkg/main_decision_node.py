import json
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class State(Enum):
    SEARCHING = 'searching'  # 탐지 + 이동
    GRIPPING  = 'gripping'   # 그리퍼 동작 중, UGV 정지


class MainDecisionNode(Node):
    def __init__(self):
        super().__init__('main_decision_node')

        # 구독
        self.vision_sub = self.create_subscription(
            String, '/detected_objects', self.vision_callback, 10)
        self.gripper_status_sub = self.create_subscription(
            String, '/gripper_status', self.gripper_status_callback, 10)

        # 발행
        self.cmd_pub     = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_pub = self.create_publisher(String, '/gripper_cmd', 10)

        # 제어 루프 (10Hz) — ugv_controller 워치독 타임아웃 방지
        self.timer = self.create_timer(0.1, self.control_loop)

        # 상태 머신
        self.state = State.SEARCHING

        # 비전 데이터
        self.target_area    = 0
        self.target_cls     = ''
        self.last_seen_time = self.get_clock().now()

        # ⚙️ 튜닝 파라미터 (실물 테스트 후 조정)
        self.AREA_THRESHOLD = 40000  # 이 면적 이상이면 '도달'로 판단 (w × h px²)
        self.SPEED          = 0.2    # 전진 속도 (-0.5 ~ 0.5)

        self.get_logger().info('대뇌 노드 실행 완료! 탐지 대기 중...')

    # ──────────────────────────────────────
    # 콜백
    # ──────────────────────────────────────
    def vision_callback(self, msg: String):
        """yolo_vision_node → /detected_objects 수신."""
        try:
            data = json.loads(msg.data)
            w = data.get('w', 0)
            h = data.get('h', 0)
            self.target_area    = w * h
            self.target_cls     = data.get('class', '')
            self.last_seen_time = self.get_clock().now()
        except Exception as e:
            self.get_logger().error(f'비전 데이터 파싱 오류: {e}')

    def gripper_status_callback(self, msg: String):
        """gripper_node → /gripper_status 수신. 'done' 이면 탐색으로 복귀."""
        if msg.data == 'done' and self.state == State.GRIPPING:
            self.state = State.SEARCHING
            self.get_logger().info('그리퍼 완료 신호 수신 → SEARCHING 복귀')

    # ──────────────────────────────────────
    # 제어 루프 (10Hz)
    # ──────────────────────────────────────
    def control_loop(self):
        twist = Twist()
        time_since_last_seen = (
            self.get_clock().now() - self.last_seen_time
        ).nanoseconds / 1e9

        if self.state == State.GRIPPING:
            # 그리퍼 동작 중 → UGV 정지, 완료 신호 대기
            twist.linear.x  = 0.0
            twist.angular.z = 0.0
            self.get_logger().info(
                '그리퍼 동작 중... UGV 정지', throttle_duration_sec=1.0)

        elif time_since_last_seen > 0.5:
            # 탐지 없음 → 정지
            twist.linear.x  = 0.0
            twist.angular.z = 0.0
            self.get_logger().info(
                '탐지 없음. 정지 대기 중...', throttle_duration_sec=2.0)

        elif self.target_area < self.AREA_THRESHOLD:
            # 멀리 있음 → 전진
            twist.linear.x  = self.SPEED
            twist.angular.z = 0.0
            self.get_logger().info(
                f'전진 중 (면적: {self.target_area})', throttle_duration_sec=1.0)

        else:
            # 목표 도달 → UGV 정지 + pick 명령 1회 발행
            twist.linear.x  = 0.0
            twist.angular.z = 0.0
            self._send_pick()  # 내부에서 state → GRIPPING 으로 변경

        self.cmd_pub.publish(twist)

    # ──────────────────────────────────────
    # pick 명령 발행 (SEARCHING 상태일 때만)
    # ──────────────────────────────────────
    def _send_pick(self):
        if self.state != State.SEARCHING:
            return  # 이미 GRIPPING 상태면 중복 발행 방지
        self.state = State.GRIPPING
        cmd = json.dumps({'cmd': 'pick', 'cls': self.target_cls})
        msg = String()
        msg.data = cmd
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'목표 도달! pick 명령 발행 → cls: {self.target_cls}')


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
