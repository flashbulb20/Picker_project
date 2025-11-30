import numpy as np
import time
from typing import List, Optional, Tuple, Dict
from math import floor


# =======================================================
# Ultralytics 추적 결과 시뮬레이션 클래스 (재사용)
# =======================================================
class MockBoxes:
    def __init__(self, ids: List[int], xyxy_coords: List[List[float]]):
        self.id = np.array(ids)
        self.xyxy = np.array(xyxy_coords)
        self.cls = np.zeros(len(ids)) 

class MockResults:
    def __init__(self, ids: List[int], xyxy_coords: List[List[float]]):
        if ids:
            self.boxes = MockBoxes(ids, xyxy_coords)
        else:
            self.boxes = MockBoxes([], [])
        self.orig_shape = (480, 640)  # (Height, Width) 480x640 이미지 가정

# =======================================================
# 핵심 로직 클래스: ID 기반 Quadrant 추적기
# =======================================================

class IDBasedQuadrantTracker:
    def __init__(self, width: int = 640, height: int = 480, max_reservation_time: int = 5):
        self.target_rc_id: Optional[int] = None           # 로봇이 찾아야 할 타겟 RC카의 ID
        
        # ID 예약 및 관리 로직
        self.reserved_ids: Dict[int, float] = {}          # {ID: 마지막 감지 타임스탬프}
        self.MAX_RESERVATION_TIME = max_reservation_time  # ID를 예약 상태로 유지할 최대 시간(초)

        # Quadrant 정의
        self.WIDTH = width
        self.HEIGHT = height
        self.Q_W = width / 2                              # Quadrant 경계 너비 (320)
        self.Q_H = height / 2                             # Quadrant 경계 높이 (240)
        
        # Quadrant 인덱스 정의 (매장 상황 맵핑)
        # Q1 | Q2
        # ---+---
        # Q3 | Q4
        self.QUADRANT_MAP = {
            (0, 0): "Q1 (상단 좌측)", 
            (1, 0): "Q2 (상단 우측)",
            (0, 1): "Q3 (하단 좌측)",
            (1, 1): "Q4 (하단 우측)"
        }
        print(f"매장 크기 {self.WIDTH}x{self.HEIGHT}를 4개의 Quadrant로 나누었습니다.")


    # ----------------------------------------------------
    # Quadrant 판단 핵심 함수
    # ----------------------------------------------------
    def _get_quadrant(self, center_x: float, center_y: float) -> str:
        """
        바운딩 박스 중심점 (x, y)가 속하는 Quadrant를 판단합니다.
        """
        # X 인덱스: 0 (좌측), 1 (우측)
        x_index = floor(center_x / self.Q_W)
        # Y 인덱스: 0 (상단), 1 (하단)
        y_index = floor(center_y / self.Q_H)
        
        # 경계값 처리 (최대값 초과 방지)
        x_index = min(x_index, 1)
        y_index = min(y_index, 1)
        
        return self.QUADRANT_MAP.get((x_index, y_index), "판단 불가")


    # ----------------------------------------------------
    # 1. 타겟 고객 ID 설정 (주문 정보 수신 시)
    # ----------------------------------------------------
    def set_target_id(self, target_id: int):
        """
        로봇에게 상품을 요청한 고객의 확정된 Tracking ID를 설정합니다.
        
        Args:
            target_id: YOLO MOT에 의해 부여된 타겟 객체의 고유 ID
        """
        if self.target_rc_id is not None:
            print(f"⚠️ 경고: 이미 타겟 ID {self.target_rc_id}가 설정되어 있습니다. 강제 재설정합니다.")
            self.delivery_completed() # 기존 타겟 정리
        
        self.target_rc_id = target_id
        # 타겟 ID가 확정되는 순간 바로 예약 처리 시작
        self.reserved_ids[target_id] = time.time() 
        print(f"\n🔔 타겟 ID **{target_id}** 확정 및 추적/예약 처리 시작.")


    # ----------------------------------------------------
    # 2. YOLO 추적 결과가 들어올 때마다 호출되는 함수
    # ----------------------------------------------------
    def process_frame(self, results: MockResults):
        current_time = time.time()
        
        if results.boxes.id is None or len(results.boxes.id) == 0:
            current_ids = []
            print("--- 프레임 처리 --- 현재 감지된 객체 없음.")
        else:
            current_ids = results.boxes.id.tolist()
            xyxy = results.boxes.xyxy.tolist()
            print(f"\n--- 프레임 처리 ({time.strftime('%H:%M:%S', time.localtime(current_time))}) ---")
            print(f"현재 감지된 ID: {current_ids}")
            
            # -----------------------------------------------------------------
            # [로직 A] 타겟 ID 추적 및 위치 파악 (Quadrant 판단)
            # -----------------------------------------------------------------
            if self.target_rc_id is not None:
                if self.target_rc_id in current_ids:
                    # 1. 타겟 ID 감지 및 예약 갱신
                    self.reserved_ids[self.target_rc_id] = current_time 
                    
                    target_index = current_ids.index(self.target_rc_id)
                    x1, y1, x2, y2 = xyxy[target_index]
                    
                    # 2. 바운딩 박스의 중심점 계산
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    
                    # 3. Quadrant 판단 및 로봇에게 전달
                    current_quadrant = self._get_quadrant(center_x, center_y)
                    
                    print(f"   🟢 타겟 ID **{self.target_rc_id}** 감지됨.")
                    print(f"   🎯 **로봇에게 전달할 위치:** {current_quadrant} (픽셀 중심: {center_x:.0f}, {center_y:.0f})")
                    
                    # TODO: 이 Quadrant 정보를 ROS 토픽으로 발행하는 로직 추가
                    
                else:
                    print(f"   🟡 타겟 ID **{self.target_rc_id}** 미감지. 예약 상태 유지 중...")

        # -----------------------------------------------------------------
        # [로직 B] ID 예약 만료 및 해제
        # -----------------------------------------------------------------
        ids_to_remove = []
        for id_val, last_seen_time in self.reserved_ids.items():
            time_elapsed = current_time - last_seen_time
            if time_elapsed > self.MAX_RESERVATION_TIME:
                ids_to_remove.append(id_val)
        
        for id_val in ids_to_remove:
            del self.reserved_ids[id_val]
            if id_val == self.target_rc_id:
                self.target_rc_id = None
                print(f"   🔴 ID **{id_val}** 예약 시간 만료. 타겟 추적 종료.")
            else:
                 print(f"   🧹 ID {id_val} 예약 시간 만료로 해제.")


    # ----------------------------------------------------
    # 3. 로봇이 상품 전달을 완료했을 때 호출되는 함수
    # ----------------------------------------------------
    def delivery_completed(self):
        """배송이 완료되면 타겟 ID를 해제하고 다음 요청을 받을 준비를 합니다."""
        if self.target_rc_id is not None:
            delivered_id = self.target_rc_id
            
            # 타겟 ID를 예약 목록에서 제거하여 MOT 알고리즘이 재활용 가능하도록 허용
            if delivered_id in self.reserved_ids:
                del self.reserved_ids[delivered_id]
                
            self.target_rc_id = None
            print(f"\n🎉 배송 완료! 타겟 ID **{delivered_id}**를 해제하고 추적을 종료합니다.")
        else:
            print("❌ 현재 활성화된 배송 타겟이 없습니다.")


# =======================================================
# 시뮬레이션 실행
# =======================================================

if __name__ == '__main__':
    processor = IDBasedQuadrantTracker(width=640, height=480, max_reservation_time=3) 
    
    # 1. 고객 주문 발생: 로봇은 ID 7번 객체가 주문했다는 정보를 받음
    target_rc_id_from_order = 7
    processor.set_target_id(target_rc_id_from_order)

    # --- 프레임 1: 타겟 RC카(ID 7)가 감지됨 ---
    # ID 7의 중심점: (x=500, y=100). Q_W=320, Q_H=240
    # x=500 >= 320 (Index 1), y=100 < 240 (Index 0) -> Q2
    frame1_results = MockResults(
        ids=[1, 5, 7, 10], 
        xyxy_coords=[
            [10, 10, 50, 50],
            [60, 60, 100, 100],
            [480, 80, 520, 120],     # ID 7 (Center: 500, 100)
            [300, 300, 350, 350]
        ]
    )
    processor.process_frame(frame1_results)

    time.sleep(1) 

    # --- 프레임 2: 타겟 RC카(ID 7)가 Q4로 이동 ---
    # ID 7의 중심점: (x=500, y=300). 
    # x=500 >= 320 (Index 1), y=300 >= 240 (Index 1) -> Q4
    frame2_results = MockResults(
        ids=[1, 5, 7, 10], 
        xyxy_coords=[
            [15, 15, 55, 55], 
            [65, 65, 105, 105], 
            [480, 280, 520, 320],    # ID 7 (Center: 500, 300)
            [310, 310, 360, 360]
        ]
    )
    processor.process_frame(frame2_results)

    time.sleep(2) # 총 3초 경과

    # --- 프레임 3: 타겟 RC카(ID 7)가 시야에서 잠시 사라짐 (예약 시간 만료됨) ---
    frame3_results = MockResults(
        ids=[1, 5, 10], # ID 7 누락
        xyxy_coords=[
            [20, 20, 60, 60], 
            [70, 70, 110, 110], 
            [320, 320, 370, 370]
        ]
    )
    processor.process_frame(frame3_results)