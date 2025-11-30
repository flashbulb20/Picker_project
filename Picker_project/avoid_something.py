import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
import time

class ReflexGuard(Node):
    def __init__(self):
        super().__init__('reflex_guard')
        qos = QoSProfile(depth=10)
        
        # 장애물 감지용 (빠른 반응을 위해 직접 구독)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
        # 긴급 회피 명령용
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        
        # 설정
        self.emergency_dist = 0.6  # Nav2가 반응하기 전에 먼저 반응할 거리
        self.is_danger = False

    def scan_callback(self, msg):
        # 전방 50도(-25 ~ +25) 집중 감시
        ranges = msg.ranges
        front_ranges = ranges[0:25] + ranges[-25:]
        
        min_dist = float('inf')
        for r in front_ranges:
            if 0.1 < r < self.emergency_dist:
                if r < min_dist: min_dist = r
        
        # 감지되면 즉시 플래그 세움
        self.is_danger = (min_dist < self.emergency_dist)

    def execute_evasive_maneuver(self):
        # ⚡ 긴급 회피 동작 (빠르고 짧게!)
        self.get_logger().warn("⚡ 긴급 회피 발동! (Reflex Action)")
        
        twist = Twist()
        
        # 1. 급정지
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        
        # 2. 빠른 후진 (0.5초) -> 쾅 박는거 방지
        twist.linear.x = -0.2 
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.5)
        
        # 3. 빠른 회전 (장애물 없는 곳으로 돌면 좋지만, 일단 왼쪽으로)
        twist.linear.x = 0.0
        twist.angular.z = 1.0 # 꽤 빠른 속도로 회전
        self.cmd_vel_pub.publish(twist)
        time.sleep(1.0) # 1초간 회전
        
        # 4. 정지
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("✅ 회피 완료. Nav2에게 제어권 반환.")

def main():
    rclpy.init()
    
    reflex_node = ReflexGuard()
    navigator = TurtleBot4Navigator()

    # --- Phase 1 준비 ---
    if not navigator.getDockedStatus(): navigator.dock()
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()
    navigator.undock()
    
    # 목표 설정
    goal_pose = navigator.getPoseStamped([-6.83, 0.3], TurtleBot4Directions.EAST)
    
    print("=== Phase 1: 고속 반응 이동 시작 ===")
    
    # 목표에 도착할 때까지 무한 루프
    while True:
        # 1. Nav2 출발
        navigator.startToPose(goal_pose)
        
        # 2. 이동 중 감시 (여기가 핵심)
        while not navigator.isTaskComplete():
            rclpy.spin_once(reflex_node, timeout_sec=0.05) # 아주 짧은 주기로 센서 확인
            
            if reflex_node.is_danger:
                print("🚨 위험 감지! Nav2 강제 중단!")
                navigator.cancelTask() # Nav2 멈춰!!
                
                # 3. 수동 회피 실행
                reflex_node.execute_evasive_maneuver()
                
                # 회피 끝났으니 while 루프 탈출 -> 다시 startToPose 실행됨
                break 
        
        # 3. 상태 확인
        result = navigator.getResult()
        if result == 1: # 도착 성공
            print("🎉 목표 지점 도착 완료!")
            break
        elif result == 6: # 취소됨 (우리가 cancelTask 했으므로 정상)
            print("🔄 경로 재설정 중...")
            continue
        else:
            print("❌ 이동 실패 (갇힘 등)")
            # 필요 시 여기서 종료하거나 재시도 로직 추가

    # --- Phase 2: 추적 모드로 전환 ---
    print("=== Phase 2: 추적 모드 준비 ===")
    # 여기서부터 Phase 2 코드 연결...
    
    reflex_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()