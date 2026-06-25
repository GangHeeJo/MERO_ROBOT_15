import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # OpenCV 이미지 <-> ROS 2 메시지 변환기
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # 1. 영상을 방송할 퍼블리셔 생성 (토픽명: /image_raw)
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        
        # 2. 타이머 설정 (예: 30 FPS로 영상 캡처)
        timer_period = 1.0 / 30.0 
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # 3. OpenCV 카메라 설정 (0번 카메라 장치)
        # 나중에 카메라가 꽂혔을 때 /dev/video0이 아니면 이 숫자를 바꿔야 할 수 있습니다.
        self.cap = cv2.VideoCapture(0) 

        # 4. ArduCAM 최적화 설정 (데이터시트 기준 960x600 적용)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
        
        # 5. 브릿지 초기화
        self.bridge = CvBridge()
        
        self.get_logger().info('📷 카메라 노드가 실행되었습니다. (ArduCAM 영상 퍼블리시 중)')

    def timer_callback(self):
        """지정된 시간(1/30초)마다 반복 실행되며 프레임을 찍어 보냅니다."""
        ret, frame = self.cap.read()
        
        if ret:
            # 카메라가 정상적으로 프레임을 읽었을 경우
            # OpenCV BGR 형식을 ROS 2 Image 메시지로 변환
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(img_msg)
        else:
            self.get_logger().warn('⚠️ 카메라 프레임을 읽을 수 없습니다. (연결 상태 확인 요망)', throttle_duration_sec=2.0)

    def destroy_node(self):
        """노드가 종료될 때 카메라 장치를 안전하게 해제합니다."""
        self.get_logger().info('카메라 장치를 해제합니다.')
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
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