from ultralytics import YOLO
import cv2
import time
import numpy as np


class RealTimeCarTracker:
    def __init__(self, model_path='yolov8n.pt', source=0):
        """
        추적기 초기화
        :param model_path: 사용할 YOLO 모델 파일 경로 (예: 'yolov8n.pt')
        :param source: 비디오 소스 (0: 웹캠, 'path/to/video.mp4': 파일 경로)
        """
        self.model = YOLO(model_path)
        self.target_class_index = 2 # 자동차 클래스 index
        
        self.cap = cv2.VideoCapture(source)
        if not self.cap.isOpened():
            raise IOError(f"웹캠 소스 {source}를 열 수 없습니다.")

        print(f"✅ YOLOv8 모델 로드 완료: {model_path}")
        print(f"✅ 'car' 클래스 (ID: {self.target_class_index}) 추적을 시작합니다.")

    def run_tracking(self):
        """웹캠에서 실시간으로 객체 추적을 실행합니다."""
        
        # 추적에 사용할 파라미터 설정
        # tracker='bytetrack.yaml'은 Ultralytics가 권장하는 기본 추적 알고리즘 중 하나입니다.
        tracking_config = 'bytetrack.yaml'
        
        # FPS 계산용 변수
        prev_time = time.time()
        
        while self.cap.isOpened():
            success, frame = self.cap.read()
            if not success:
                print("프레임을 읽을 수 없습니다. 스트림 종료.")
                break

            # ----------------------------------------------------
            # 1. YOLO 모델 추적 실행
            # ----------------------------------------------------
            
            # model.track()을 사용하여 추적 및 감지 수행
            # persist=True: 추적 상태를 프레임 간에 유지
            # classes=[self.target_class_index]: 'car' 클래스만 필터링하여 감지/추적
            results = self.model.track(
                frame, 
                persist=True, 
                tracker=tracking_config,
                classes=[self.target_class_index], 
                verbose=False
            )

            # ----------------------------------------------------
            # 2. 추적 결과 처리 및 시각화
            # ----------------------------------------------------
            
            current_frame = results[0].orig_img # 원본 이미지 가져오기
            
            # 감지된 객체 정보 (바운딩 박스, ID, 클래스 등)
            boxes = results[0].boxes
            
            # 추적 ID가 존재하는 경우에만 처리
            if boxes.id is not None:
                # ID, 바운딩 박스 좌표, 클래스 정보를 numpy 배열에서 리스트로 변환
                track_ids = boxes.id.tolist()
                xyxy = boxes.xyxy.tolist()
                
                # 각 감지된 car 객체에 대해 반복
                for i, track_id in enumerate(track_ids):
                    x1, y1, x2, y2 = map(int, xyxy[i])
                    
                    # 텍스트 오버레이
                    label = f"Car ID: {track_id}"
                    
                    # 바운딩 박스와 ID 표시
                    cv2.rectangle(current_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        current_frame, 
                        label, 
                        (x1, y1 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.9, 
                        (0, 255, 0), 
                        2
                    )
            
            # ----------------------------------------------------
            # 3. FPS 표시 및 화면 출력
            # ----------------------------------------------------
            
            # FPS 계산
            current_time = time.time()
            fps = 1 / (current_time - prev_time)
            prev_time = current_time
            
            cv2.putText(
                current_frame, 
                f"FPS: {fps:.2f}", 
                (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                1, 
                (255, 0, 0), 
                2
            )
            
            cv2.imshow("YOLOv8 Real-Time Car Tracking", current_frame)

            # 'q' 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # 4. 종료 시 자원 해제
        self.cap.release()
        cv2.destroyAllWindows()
        print("프로그램을 종료합니다.")

# =======================================================
# 메인 실행
# =======================================================

if __name__ == '__main__':
    try:
        tracker = RealTimeCarTracker()
        tracker.run_tracking()
    except Exception as e:
        print(f"오류 발생: {e}")