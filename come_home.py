import rclpy
from rclpy.node import Node
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions
from nav2_simple_commander.robot_navigator import TaskResult

def main(args=None):
    rclpy.init(args=args)
    
    navigator = TurtleBot4Navigator()

    # 1. 내비게이션 시스템 활성화 대기
    print("⏳ 내비게이션 시스템 연결 중...")
    navigator.waitUntilNav2Active()
    print("✅ 연결 완료! 복귀 시퀀스를 시작합니다.")

    # 2. 도킹 전 대기 장소 설정
    staging_pose = navigator.getPoseStamped([-0.3, -0.3], TurtleBot4Directions.NORTH)

    print(f"🚀 복귀 시작! {[-0.3, -0.3]} 지점으로 이동합니다.")
    
    # [수정 포인트 1] goToPose 대신 startToPose 사용 (명시적 비동기 시작)
    navigator.startToPose(staging_pose)

    # [수정 포인트 2] 이동이 끝날 때까지 기다리는 루프 추가
    while not navigator.isTaskComplete():
        pass # 작업이 끝날 때까지 계속 대기

    # 3. 이동 결과 확인
    result = navigator.getResult()
    
    if result == TaskResult.SUCCEEDED:
        print("📍 대기 장소 도착 완료. 도킹을 시도합니다...")
        
        # [수정 포인트 3] 이동이 확실히 끝난 후 도킹 시작
        navigator.dock()

        if navigator.getDockedStatus():
             print("🎉 도킹 성공! 충전 시작.")
        else:
             print("⚠️ 도킹 실패. 로봇이 도킹 스테이션을 못 찾았거나 위치가 안 맞습니다.")
             
    elif result == TaskResult.CANCELED:
        print("❌ 이동이 취소되었습니다.")
    elif result == TaskResult.FAILED:
        print("❌ 이동 실패! 경로가 막혀있거나 로봇이 길을 잃었습니다.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()