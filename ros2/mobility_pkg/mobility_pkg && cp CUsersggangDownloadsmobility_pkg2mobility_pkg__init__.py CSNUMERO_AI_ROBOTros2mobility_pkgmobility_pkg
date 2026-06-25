import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import json

class UGVController(Node):
    def __init__(self):
        super().__init__('ugv_controller')

        # [Issue ② 해결] udev 심볼릭 링크 고정 경로 적용
        self.port = '/dev/ugv_chassis'  
        self.baud = 115200
        self.is_connected = False

        # [Issue ① 해결] 소프트웨어 워치독 타임스탬프 초기화
        self.last_cmd_time = self.get_clock().now()
        self.timeout_threshold = 0.5  # 초 단위 임계치

        self.current_l = 0.0
        self.current_r = 0.0

        # 초기 시리얼 장치 활성화 시도
        self.connect_serial()

        # 구독 및 타이머 바인딩
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def connect_serial(self):
        """[Issue ③ 해결] 직렬 포트 물리적 초기화 및 복구 루틴 분리"""
        try:
            self.ser = serial.Serial(self.port, self.baud, dsrdtr=None, timeout=0.1)
            self.ser.setRTS(False)
            self.ser.setDTR(False)
            self.is_connected = True
            self.get_logger().info(f"✅ ESP32 통신 포트({self.port}) 하드웨어 바인딩 완료.")
        except Exception as e:
            self.is_connected = False
            self.get_logger().error(f"❌ 하드웨어 초기화 실패 (포트 상태 확인 요망): {e}")

    def cmd_vel_callback(self, msg):
        # 메시지 수신 시 타임스탬프 실시간 갱신 (워치독 리셋)
        self.last_cmd_time = self.get_clock().now()

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        left = linear_x - angular_z
        right = linear_x + angular_z

        self.current_l = max(-0.5, min(0.5, left))
        self.current_r = max(-0.5, min(0.5, right))

    def timer_callback(self):
        # [Issue ③ 해결] 단선 상태일 때 시스템 크래시를 방지하고 재연결 프로세스 이행
        if not self.is_connected:
            self.connect_serial()
            return

        # [Issue ① 해결] 상위 토픽 유실 시간 연산 (Fail-Safe)
        elapsed_time = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed_time > self.timeout_threshold:
            self.current_l = 0.0
            self.current_r = 0.0
            self.get_logger().warn("⚠️ [Fail-Safe] /cmd_vel 신호 유실 상태 - 강제 정지 프레임 제어 중", throttle_duration_sec=2.0)

        cmd_data = {"T": 1, "L": self.current_l, "R": self.current_r}
        cmd_str = json.dumps(cmd_data) + "\n"
        
        try:
            self.ser.write(cmd_str.encode('utf-8'))
        except (serial.SerialException, OSError) as e:
            # 주행 중 단선 런타임 예외 처리 및 플래그 다운
            self.get_logger().error(f"❌ 주행 중 직렬 버스 예외 발생 (단선 의심): {e}")
            self.is_connected = False 

    def destroy_node(self):
        self.get_logger().info("🛑 안전 정지 프로토콜 가동")
        if self.is_connected:
            try:
                stop_str = '{"T":1,"L":0.0,"R":0.0}\n'
                self.ser.write(stop_str.encode('utf-8'))
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UGVController()
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