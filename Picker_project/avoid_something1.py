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
        
        # 3. 빠른 회전 (일단 왼쪽으로)
        twist.linear.x = 0.0
        twist.angular.z = 1.0 # 꽤 빠른 속도로 회전
        self.cmd_vel_pub.publish(twist)
        time.sleep(1.0) # 1초간 회전
        
        # 4. 정지
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("회피 완료. Nav2에게 제어권 반환.")

def drive_to_target(navigator, reflex_node, target_pose, target_name="Target"):
    
    print(f"=== {target_name} 이동 시작 ===")
    
    # 목표에 도착할 때까지 무한 루프 (재시도 로직 포함)
    while rclpy.ok():
        navigator.startToPose(target_pose)

        SLOWDOWN_DIST = 0.6     # 목표까지 남은 거리 0.6m 이하면 감속
        SLOW_SPEED = 0.05       # 느려질 속도
        MAX_SLOW_DRIVE_TIME = 3.0
        slowdown_done = False
        
        # 2. 이동 중 감시
        while rclpy.ok() and not navigator.isTaskComplete():
            rclpy.spin_once(reflex_node, timeout_sec=0.05) # 아주 짧은 주기로 센서 확인
            
            if reflex_node.is_danger:
                print("위험 감지! Nav2 강제 중단!")
                navigator.cancelTask() # Nav2 멈춰!!
                
                # 3. 수동 회피 실행
                reflex_node.execute_evasive_maneuver()
                
                # 회피 끝났으니 while 루프 탈출 -> 다시 startToPose 실행됨 (continue 효과)
                break 

            feedback = navigator.getFeedback()
            if feedback is not None:
                dist = getattr(feedback, "distance_remaining", None)
                if dist is not None:
                    # 감속 로
                    if (not slowdown_done) and dist < SLOWDOWN_DIST:
                        slowdown_done = True
                        print("목표 근처 진입 -> 감속 모드로 전환")

                        # 현재 네비게이션 취소 (Nav2에서 /cmd_vel 더 이상 안 보내게)
                        navigator.cancelTask()

                        # 남은 거리만큼 천천히 직진
                        twist = Twist()
                        twist.linear.x = SLOW_SPEED # 느린 속도 (0.05 m/s)
                        
                        start_time = reflex_node.get_clock().now().nanoseconds / 1e9
                        
                        # 지정된 시간(3초) 동안 느리게 직진하며 ReflexGuard를 상시 감시
                        while (reflex_node.get_clock().now().nanoseconds / 1e9) - start_time < MAX_SLOW_DRIVE_TIME:
                            
                            #ReflexGuard 센서 체크
                            rclpy.spin_once(reflex_node, timeout_sec=0.05)
                            
                            if reflex_node.is_danger:
                                reflex_node.get_logger().error("⚡ 감속 중 위험 감지! 즉시 회피.")
                                twist.linear.x = 0.0
                                reflex_node.cmd_vel_pub.publish(twist)
                                reflex_node.execute_evasive_maneuver()
                                break 
                                
                            reflex_node.cmd_vel_pub.publish(twist)
                            time.sleep(0.05) # 짧은 sleep으로 CPU 점유율 낮춤
                        
                        # 수동 제어 정지
                        twist.linear.x = 0.0
                        reflex_node.cmd_vel_pub.publish(twist)
                        reflex_node.get_logger().info("수동 감속 주행 완료. 도착 처리.")
                        break
            time.sleep(0.05)

        # 3. 상태 확인
        # 감속 주행 루프를 통해 break로 나왔거나, Nav2가 정상 도착했을 때
        result = navigator.getResult()
        
        # result 1: Nav2 정상 도착
        # result 6 + slowdown_done: 감속 로직에 의해 Nav2를 취소하고 수동 도착
        if result == 1 or (result == 6 and slowdown_done):
            print(f"{target_name} 도착 완료!")
            return True # 성공 리턴
            
        elif result == 6: # 감속 모드 진입 전 취소됨 (회피 동작 등)
            print("경로 재설정 중...")
            continue # 재시도
        else:
            print("이동 실패 (갇힘 등)")
            return False # 실패 리턴

def main():
    rclpy.init()
    
    reflex_node = ReflexGuard()
    navigator = TurtleBot4Navigator()

    # --- 준비 ---
    if not navigator.getDockedStatus(): navigator.dock()
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()
    navigator.undock()
    
    # 1. 중간 지점
    intermediate_pose = navigator.getPoseStamped([-5.0, 2.15], TurtleBot4Directions.EAST)
    # 2. 최종 지점
    final_pose = navigator.getPoseStamped([-6.38, 0.3], TurtleBot4Directions.EAST)
    
    print("=== Phase 1: 순차 이동 시작 ===")
    
    if drive_to_target(navigator, reflex_node, intermediate_pose, "중간 지점"):
        
        print("중간 지점 도착. 2초간 대기...")
        time.sleep(2.0)
        
        print("최종 지점으로 출발합니다.")
        drive_to_target(navigator, reflex_node, final_pose, "최종 지점")
        
    else:
        print("중간 지점 이동 실패. 프로그램을 종료합니다.")

    # --- Phase 2: 추적 모드로 전환 ---
    print("=== Phase 2: 추적 모드 준비 ===")
    
    reflex_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()