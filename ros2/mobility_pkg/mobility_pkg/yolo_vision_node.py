import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json

# ROS 2 공식 패키지 경로 탐색 도구
from ament_index_python.packages import get_package_share_directory

# 비전 팀이 사용하는 YOLO 패키지
from ultralytics import YOLO


class YoloVisionNode(Node):
    def __init__(self):
        super().__init__('yolo_vision_node')

        # 1. 구독: 카메라 영상 수신
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, 1)

        # 2. 발행: 대뇌 노드로 객체 위치 데이터 송신
        self.publisher_ = self.create_publisher(String, '/detected_objects', 10)
        self.bridge = CvBridge()

        # 3. 모델 경로 설정 (engine 있으면 TensorRT, 없으면 pt 사용)
        base = os.path.expanduser("~/MERO_ROBOT_15/vision/model")
        engine_path = os.path.join(base, "best.engine")
        pt_path     = os.path.join(base, "best.pt")

        if os.path.exists(engine_path):
            model_path = engine_path
            self.get_logger().info(f'⚡ TensorRT 모델 사용: {engine_path}')
        elif os.path.exists(pt_path):
            model_path = pt_path
            self.get_logger().info(f'📦 PyTorch 모델 사용: {pt_path}')
        else:
            self.get_logger().error('❌ 모델 파일 없음 (best.engine / best.pt)')
            model_path = ""

        # 4. 모델 로드
        try:
            self.model = YOLO(model_path, task='detect')
            self.get_logger().info('✅ YOLO 모델 로드 성공!')
        except Exception as e:
            self.get_logger().error(f'❌ 모델 로드 실패: {e}')
            self.model = None

    def image_callback(self, msg):
        if self.model is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            results = self.model(frame, verbose=False)

            if len(results[0].boxes) > 0:
                box = results[0].boxes[0]
                detected_class = self.model.names[int(box.cls[0])]
                center_x, center_y, box_width, box_height = map(int, box.xywh[0])
            else:
                return  # 정다면체가 없으면 무시

            detection_data = {
                "class": detected_class,
                "x": center_x,
                "y": center_y,
                "w": box_width,
                "h": box_height
            }

            msg_out = String()
            msg_out.data = json.dumps(detection_data)
            self.publisher_.publish(msg_out)

        except Exception as e:
            self.get_logger().error(f"비전 데이터 처리 중 오류 발생: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloVisionNode()
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