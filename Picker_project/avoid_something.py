import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from nav2_simple_commander.robot_navigator import TaskResult
import time
import threading
import cv2
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

# =========================================
# 1. 안전 가드 (백그라운드 감시)
# =========================================
class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.scan_sub = self.create_subscription(LaserScan, '/robot3/scan', self.scan_callback, qos)
        self.input_sub = self.create_subscription(Twist, '/cmd_vel_input', self.input_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/robot3/cmd_vel', 10)
        self.img_sub = self.create_subscription(Image, '/robot3/oakd/rgb/preview/image_raw', self.img_callback, qos)
        
        self.bridge = CvBridge()
        self.latest_cv_image = None
        
        # YOLO 로드
        print("📦 YOLO 모델을 불러오는 중...", flush=True)
        try:
            self.model = YOLO("/home/rokey/rokey_ws/src/final_project/box_yolo8n.pt")
            print("✅ YOLO 모델 로드 완료!", flush=True)
        except Exception as e:
            print(f"❌ YOLO 모델 로드 실패: {e}", flush=True)
            self.model = None

        self.emergency_dist = 0.40 
        self.current_dist = 10.0
        self.is_danger = False
        self.phase2_active = False 
        self.obstacle_dir = 1.0
        self.is_sensor_active = False

    def scan_callback(self, msg):
        self.is_sensor_active = True
        ranges = msg.ranges
        count = len(ranges)
        if count == 0: return

        # [설정] 전방 30도, 최소거리 0.18m (벽타기 최적화)
        CENTER_RATIO = 0.25 
        center_idx = int(count * CENTER_RATIO)
        
        fov_ratio = 30 / 360
        half_width = int(count * fov_ratio / 2)
        
        start_idx = max(0, center_idx - half_width)
        end_idx = min(count, center_idx + half_width)
        
        front_ranges = ranges[start_idx : end_idx]
        valid_ranges = [r for r in front_ranges if 0.18 < r < 1.0]
        
        min_dist = min(valid_ranges) if valid_ranges else 10.0

        self.current_dist = min_dist
        self.is_danger = (min_dist < self.emergency_dist)

        # 방향 결정
        mid = len(front_ranges) // 2
        l_val = min([r for r in front_ranges[:mid] if r > 0.18], default=10.0)
        r_val = min([r for r in front_ranges[mid:] if r > 0.18], default=10.0)
        
        if r_val < l_val: self.obstacle_dir = 1.0 
        else: self.obstacle_dir = -1.0

    def img_callback(self, msg):
        try:
            self.latest_cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError: pass

    def input_callback(self, msg):
        if not self.phase2_active: return
        final_cmd = Twist()
        if self.is_danger:
            final_cmd.linear.x = 0.0
            final_cmd.angular.z = 0.5 * self.obstacle_dir
        else:
            final_cmd = msg
        self.cmd_vel_pub.publish(final_cmd)

    def detect_and_count(self):
        if self.model is None or self.latest_cv_image is None: return -1
        print("📸 이미지 분석 중...", flush=True)
        results = self.model(self.latest_cv_image, verbose=False)[0]
        return len(results.boxes)

# =========================================
# 2. 메인 실행 로직
# =========================================
def main():
    rclpy.init()
    
    safety_node = SafetyMonitor()
    navigator = TurtleBot4Navigator()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(safety_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    if not navigator.getDockedStatus(): navigator.dock()
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()
    navigator.undock()

    print("⏳ 센서 연결 확인 중...", flush=True)
    while not safety_node.is_sensor_active:
        time.sleep(0.1)
    print("✅ 센서 정상 연결됨.", flush=True)

    config_cli = safety_node.create_client(SetParameters, '/robot3/controller_server/set_parameters')
    def set_nav2_params(max_speed, xy_tol, yaw_tol):
        if not config_cli.wait_for_service(timeout_sec=1.0): return
        req = SetParameters.Request()
        req.parameters = [
            Parameter(name='FollowPath.max_vel_x', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=max_speed)),
            Parameter(name='FollowPath.xy_goal_tolerance', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=xy_tol)),
            Parameter(name='FollowPath.yaw_goal_tolerance', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=yaw_tol))
        ]
        config_cli.call_async(req)
        time.sleep(0.5)

    def drive_smart(target_pose, arrival_radius, strict_mode=False):
        mode_str = "정밀" if strict_mode else "고속"
        print(f"🚗 [{mode_str}] 이동 시작! -> {target_pose.pose.position.x:.2f}, {target_pose.pose.position.y:.2f}", flush=True)
        
        navigator.goToPose(target_pose)
        
        # [중요] 경로 계산 대기 시간 (이전 목표의 잔상 제거)
        print("⏳ 경로 계산 대기 중...", flush=True)
        time.sleep(2.0) 

        last_known_dist = float('inf')

        while not navigator.isTaskComplete():
            if safety_node.is_danger:
                print(f"🚨 [장애물] {safety_node.current_dist:.2f}m -> 회피!", flush=True)
                navigator.cancelTask()
                stop_twist = Twist(); stop_twist.linear.x = -0.15
                safety_node.cmd_vel_pub.publish(stop_twist); time.sleep(0.5)
                
                print("🔄 회피 회전 중...", flush=True)
                while safety_node.is_danger:
                    twist = Twist(); twist.linear.x = 0.0
                    twist.angular.z = 0.6 * safety_node.obstacle_dir 
                    safety_node.cmd_vel_pub.publish(twist)
                    time.sleep(0.1)
                
                print("✅ 안전 확보. 재출발.", flush=True)
                safety_node.cmd_vel_pub.publish(Twist()); time.sleep(0.5)
                return "RETRY"

            feedback = navigator.getFeedback()
            if feedback:
                dist = feedback.distance_remaining
                last_known_dist = dist
                if not strict_mode and dist < arrival_radius:
                    print(f"🚩 [도착] 반경 진입 ({dist:.2f}m).", flush=True)
                    navigator.cancelTask(); safety_node.cmd_vel_pub.publish(Twist())
                    return "SUCCESS"
            time.sleep(0.05)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED: return "SUCCESS"
        elif result == TaskResult.CANCELED: return "RETRY"
        limit = arrival_radius + 0.05 if strict_mode else arrival_radius + 0.3
        
        if last_known_dist < limit: return "SUCCESS"
        else:
            print(f"❌ 이동 실패 (남은 거리: {last_known_dist:.2f}m)", flush=True)
            return "FAIL"

    def nudge_robot(distance_m, speed_mps=0.05):
        action = "전진" if distance_m > 0 else "후진"
        print(f"📏 [마무리] {abs(distance_m)}m {action}...", flush=True)
        duration = abs(distance_m) / speed_mps
        twist = Twist(); twist.linear.x = speed_mps if distance_m > 0 else -speed_mps
        start_time = time.time()
        while (time.time() - start_time) < duration:
            safety_node.cmd_vel_pub.publish(twist); time.sleep(0.1)
        safety_node.cmd_vel_pub.publish(Twist())

    # =========================================================
    # Phase 1: 1차 진입
    # =========================================================
    goal_1 = navigator.getPoseStamped([-5.9, 0.4], TurtleBot4Directions.SOUTH)
    set_nav2_params(0.31, 0.5, 3.14)
    
    while True:
        status = drive_smart(goal_1, arrival_radius=1.0, strict_mode=False)
        if status == "SUCCESS": print("✅ 1차 진입 완료.", flush=True); break
        elif status == "RETRY": continue
        else: print("❌ 1차 실패.", flush=True); rclpy.shutdown(); return

    # =========================================================
    # Phase 2: 박스 감지 위치 (근접)
    # =========================================================
    print("📉 [접근] 안전거리 15cm로 축소.", flush=True)
    safety_node.emergency_dist = 0.15 
    
    goal_2 = navigator.getPoseStamped([-6.38, 0.3], TurtleBot4Directions.SOUTH)
    set_nav2_params(0.1, 0.05, 0.1)
    
    while True:
        status = drive_smart(goal_2, arrival_radius=0.05, strict_mode=True)
        if status == "SUCCESS": 
            print("🎉 박스 앞 도착!", flush=True)
            nudge_robot(0.15) 
            break
        elif status == "RETRY": continue
        else: print("❌ 도착 실패.", flush=True); rclpy.shutdown(); return

    # =========================================================
    # Phase 3: YOLO 탐지
    # =========================================================
    print("\n=== [Phase 3] 물체 감지 시작 ===", flush=True)
    time.sleep(2.0)
    box_count = safety_node.detect_and_count()
    print(f"\n📦📦📦 [결과] 감지된 박스 개수: {box_count} 개 📦📦📦\n", flush=True)
    time.sleep(2.0)

    print("🔙 후진하여 거리 확보.", flush=True)
    nudge_robot(-0.25)
    print("📈 [복구] 안전거리 0.5m로 복구.", flush=True)
    safety_node.emergency_dist = 0.50

    # =========================================================
    # Phase 4: 다음 지점으로 이동
    # =========================================================
    print("\n=== [Phase 4] 다음 지점으로 이동 ===", flush=True)
    goal_3 = navigator.getPoseStamped([-0.35, 3.65], TurtleBot4Directions.SOUTH)
    set_nav2_params(0.31, 0.7, 1.0) 

    while True:
        status = drive_smart(goal_3, arrival_radius=0.2, strict_mode=False)
        if status == "SUCCESS": 
            print("✅ 2차 지점 도착 완료!", flush=True)
            nudge_robot(0.2) 
            break
        elif status == "RETRY": continue
        else: print("❌ 이동 실패.", flush=True); rclpy.shutdown(); return
    
    time.sleep(5.0)

    # =========================================================
    # [NEW] Phase 5: 도킹 스테이션 복귀 및 도킹
    # =========================================================
    print("\n=== [Phase 5] 도킹 스테이션 복귀 ===", flush=True)
    
    # 1. 도킹 준비 위치로 이동 (-0.26, -0.3)
    # 도킹 스테이션을 바라보도록 정밀 이동 (여기선 NORTH로 설정)
    dock_pose = navigator.getPoseStamped([-0.26, -0.3], TurtleBot4Directions.NORTH)
    set_nav2_params(0.31, 0.1, 0.1) # 정확하게 가야 도킹 성공률 높음

    while True:
        # 도킹 앞이니 정밀하게(Strict Mode) 이동
        status = drive_smart(dock_pose, arrival_radius=0.10, strict_mode=True)
        if status == "SUCCESS":
            print("✅ 도킹 준비 위치 도착.", flush=True)
            break
        elif status == "RETRY": continue
        else: 
            print("❌ 복귀 실패.", flush=True); rclpy.shutdown(); return

    # 2. 도킹 실행
    print("🔋 도킹 시퀀스 시작...", flush=True)
    
    # 도킹 중에는 SafetyMonitor가 개입하지 않음 (drive_smart 밖이므로)
    navigator.dock()

    if navigator.getDockedStatus():
        print("🎉 도킹 성공! 미션 종료.", flush=True)
    else:
        print("⚠️ 도킹 실패 (수동 확인 필요).", flush=True)
        # 실패 시 재시도 로직을 넣거나 종료

    # 프로그램 종료
    safety_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
