import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
from ultralytics import YOLO
import time

QUADRANT_GOALS = {
    1: {'x': 0.00464523, 'y': 3.32853, 'name': 'Q1'},
    2: {'x': -2.37742, 'y': 4.07556, 'name': 'Q2'},
    3: {'x': -4.30382, 'y': 1.58606, 'name': 'Q3'},
    4: {'x': -1.51404, 'y': -0.197009, 'name': 'Q4'},
}

class QuadrantPublisher(Node):
    def __init__(self):
        super().__init__('quadrant_publisher_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher_ = self.create_publisher(
            PoseStamped, 
            'customer_pose', 
            qos_profile
        )

        self.model = YOLO('./balloons_2car_params.pt') 
        self.cap = cv2.VideoCapture(2)
        self.target_class_index = 0

        if not self.cap.isOpened():
            self.get_logger().error("[ERROR] 웹캠을 열 수 없습니다. 인덱스를 확인하세요.")
            rclpy.shutdown()
            return
        
        self.get_logger().info('[INFO] Webcam과 YOLO 모델 로드 완료.')
        
        # Turtlebot 속도가 그리 빠르지 않기 때문에 5초마다 새로운 메시지 발행
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.get_logger().info(f'[INFO] 고객 위치 발행 노드가 실행되었습니다. 메시지는 5초마다 발행됩니다.')

        self.last_quadrant = None
        self.last_map_goal = {'x': -0.00831539, 'y': 0.397538}

    def _get_quadrant(self, frame_cx: int, frame_cy: int, xc: int, yc: int) -> int:
        if (xc >= frame_cx) and (yc <= frame_cy):
            return 1
        elif (xc <= frame_cx) and (yc <= frame_cy):
            return 2
        elif (xc <= frame_cx) and (yc >= frame_cy):
            return 3
        else:
            return 4

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("[WARN] 프레임을 읽을 수 없습니다.")
            return
        
        frame_size = frame.shape[:2]
        frame_cx, frame_cy = frame_size[1] // 2, frame_size[0] // 2
        
        results = self.model.predict(frame, classes=[self.target_class_index], conf=0.5, verbose=False)

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = [int(val) for val in box.xyxy[0].tolist()]
                conf = round(box.conf[0].item(), 2)
                cls_id = int(box.cls[0].item())
                cls_name = self.model.names[cls_id]

                xc = int((x1 + x2) / 2)
                yc = int((y1 + y2) / 2)

                current_quadrant = self._get_quadrant(frame_cx, frame_cy, xc, yc)
                self.get_logger().info(f"[ALERT] 고객을 {current_quadrant}사분면에서 발견했습니다.")
                current_goal = QUADRANT_GOALS.get(current_quadrant)

                if current_goal:
                    current_goal_x = current_goal['x']
                    current_goal_y = current_goal['y']

                    if current_goal_x != self.last_map_goal['x'] or current_goal_y != self.last_map_goal['y']:
                        goal_msg = PoseStamped()

                        goal_msg.header.frame_id = 'map'
                        goal_msg.header.stamp = self.get_clock().now().to_msg()
                        
                        goal_msg.pose.position.x = current_goal_x
                        goal_msg.pose.position.y = current_goal_y
                        goal_msg.pose.orientation.w = 1.0

                        self.publisher_.publish(goal_msg)
                        self.last_map_goal = {'x': current_goal_x, 'y': current_goal_y}
                        self.last_quadrant = current_quadrant
                        
                        self.get_logger().info(
                            f'[INFO] 고객의 위치가 x={current_goal_x}, y={current_goal_y}로 변경되었습니다.'
                        )
                    else:
                        self.get_logger().info(f'[INFO] 고객 위치에 변경사항이 없습니다. 메시지를 발행하지 않습니다.')


def main(args=None):
    rclpy.init(args=args)

    try:
        tracker_publisher = QuadrantPublisher()
        rclpy.spin(tracker_publisher)
    except Exception as e:
        tracker_publisher.get_logger().error(f"메인 루프 오류 발생: {e}")
    finally:
        if 'tracker_publisher' in locals() and tracker_publisher.cap.isOpened():
            tracker_publisher.cap.release()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()