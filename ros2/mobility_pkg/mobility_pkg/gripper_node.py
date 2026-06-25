import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

OPENRB_PORT = '/dev/ttyACM1'
BAUD_RATE   = 115200


class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')

        # 구독: main_decision_node → /gripper_cmd
        self.cmd_sub = self.create_subscription(
            String, '/gripper_cmd', self.cmd_callback, 10)

        # 발행: OpenRB 완료 신호 → main_decision_node
        self.status_pub = self.create_publisher(String, '/gripper_status', 10)

        # OpenRB 시리얼 연결
        self.ser = None
        if SERIAL_AVAILABLE:
            try:
                self.ser = serial.Serial(OPENRB_PORT, BAUD_RATE, timeout=1)
                self.get_logger().info(f'✅ OpenRB 연결 성공: {OPENRB_PORT}')
                threading.Thread(target=self._read_loop, daemon=True).start()
            except Exception as e:
                self.get_logger().warn(f'⚠️ OpenRB 연결 실패: {e} — 시리얼 없이 실행합니다.')
        else:
            self.get_logger().warn('⚠️ pyserial 미설치 — pip install pyserial')

        self.get_logger().info('🤖 그리퍼 노드 실행 완료!')

    def cmd_callback(self, msg: String):
        """
        /gripper_cmd 수신 시 OpenRB로 그대로 전달.

        예상 입력 (JSON 문자열):
          {"cmd": "pick", "cls": "d8", "mx": 12.3, "my": -5.1}
          {"cmd": "idle"}
        """
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('OpenRB 미연결 — 명령 무시')
            return

        try:
            # JSON 유효성 검사 후 그대로 전송
            data = json.loads(msg.data)
            payload = json.dumps(data) + '\n'
            self.ser.write(payload.encode())
            self.get_logger().info(f'→ OpenRB 전송: {msg.data}')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'잘못된 JSON: {msg.data} | {e}')
        except Exception as e:
            self.get_logger().error(f'전송 오류: {e}')

    def _read_loop(self):
        """
        OpenRB 응답 수신 스레드.
        OpenRB가 RETURN→IDLE 완료 시 {"status": "done"} 전송 → /gripper_status 발행.
        """
        while True:
            try:
                if self.ser is None or not self.ser.is_open:
                    break
                if self.ser.in_waiting > 0:
                    raw  = self.ser.readline()
                    text = raw.decode('utf-8', errors='ignore').strip()
                    if not text:
                        continue
                    data = json.loads(text)
                    if data.get('status') == 'done':
                        out = String()
                        out.data = 'done'
                        self.status_pub.publish(out)
                        self.get_logger().info('✅ OpenRB 완료 신호 수신 → /gripper_status: done')
            except json.JSONDecodeError:
                pass  # OpenRB 디버그 출력 등 JSON 아닌 라인 무시
            except Exception as e:
                self.get_logger().error(f'수신 오류: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
