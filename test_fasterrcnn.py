import torch
import torchvision
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
import torchvision.transforms as T
from PIL import Image
import cv2
import numpy as np

# ----------------------------
# 모델 불러오기 함수
# ----------------------------
def get_model(num_classes=2):
    model = torchvision.models.detection.fasterrcnn_resnet50_fpn(weights=None)
    in_features = model.roi_heads.box_predictor.cls_score.in_features
    model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)
    return model


def test_image(model_path, image_path, img_size=640):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # 모델 불러오기
    model = get_model(2)
    model.load_state_dict(torch.load(model_path, map_location=device))
    model.to(device)
    model.eval()

    # -----------------------------
    # 1) 이미지 로드 (원본 크기 저장)
    # -----------------------------
    img = Image.open(image_path).convert("RGB")
    orig_w, orig_h = img.size

    # -----------------------------
    # 2) 테스트 이미지만 resize
    # -----------------------------
    img_resized = img.resize((img_size, img_size))
    img_tensor = T.ToTensor()(img_resized).to(device)

    # -----------------------------
    # 3) 추론
    # -----------------------------
    with torch.no_grad():
        output = model([img_tensor])[0]

    boxes = output["boxes"].cpu().numpy()
    scores = output["scores"].cpu().numpy()

    # -----------------------------
    # 4) 박스 좌표 원본 크기로 복원
    # -----------------------------
    scale_x = orig_w / img_size
    scale_y = orig_h / img_size

    boxes[:, [0,2]] *= scale_x
    boxes[:, [1,3]] *= scale_y

    # OpenCV 변환 (원본 이미지 기준)
    img_cv = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)

    threshold = 0.5
    count = 0

    for box, score in zip(boxes, scores):
        if score < threshold:
            continue

        count += 1
        x1, y1, x2, y2 = box.astype(int)

        cv2.rectangle(img_cv, (x1, y1), (x2, y2), (0,255,0), 2)
        cv2.putText(img_cv, f"{score:.2f}", (x1, y1-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    print(f"🔍 Detected boxes: {count}")

    # 표시
    cv2.imshow("Result", img_cv)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # 저장
    cv2.imwrite("test_result.jpg", img_cv)



# ----------------------------
# 실행
# ----------------------------
if __name__ == "__main__":
    test_image(
        model_path="/home/rokey/box_count/faster_RCNN/epoch_20.pth",  # 네가 학습한 모델
        image_path="/home/rokey/box_count/S_boxes/S_box_img_0002.jpg"
    )
