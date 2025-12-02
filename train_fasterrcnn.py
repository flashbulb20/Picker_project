import numpy as np

def iou(box1, box2):#실제 박스와 예측 박스가 겹치는 정도를 파악하기 위해
    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])
    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])

    inter = max(0, x2-x1) * max(0, y2-y1)
    area1 = (box1[2]-box1[0]) * (box1[3]-box1[1])
    area2 = (box2[2]-box2[0]) * (box2[3]-box2[1])
    union = area1 + area2 - inter

    return inter / union if union > 0 else 0


def evaluate_epoch(model, img_dir, label_dir, device, iou_thresh=0.5):
    tp = fp = fn = 0
    transform = T.ToTensor()

    for fname in sorted(os.listdir(img_dir)):
        if not fname.endswith(".jpg"):
            continue

        img_path = os.path.join(img_dir, fname)
        label_path = os.path.join(label_dir, fname.replace(".jpg", ".txt"))

        # GT load
        img = Image.open(img_path).convert("RGB")
        W, H = img.size
        gt = []
        with open(label_path) as f:
            for line in f:
                cls, cx, cy, bw, bh = map(float, line.split())
                cx *= W; cy *= H; bw *= W; bh *= H
                x1 = cx - bw/2; y1 = cy - bh/2
                x2 = cx + bw/2; y2 = cy + bh/2
                gt.append([x1,y1,x2,y2])

        gt_used = [False] * len(gt)

        img_tensor = transform(img).to(device)

        # inference
        with torch.no_grad():
            out = model([img_tensor])[0]

        pred_boxes = out["boxes"].cpu().numpy()
        pred_scores = out["scores"].cpu().numpy()

        # threshold
        preds = [b for b, s in zip(pred_boxes, pred_scores) if s >= 0.5]

        # match pred ↔ GT
        for pb in preds:
            matched = False
            for i, gb in enumerate(gt):
                if gt_used[i]:
                    continue
                if iou(pb, gb) >= iou_thresh:
                    tp += 1
                    gt_used[i] = True
                    matched = True
                    break
            if not matched:
                fp += 1

        # GT 중 매칭 안 된 것 → FN
        fn += gt_used.count(False)

    # calculate metrics
    precision = tp / (tp + fp + 1e-6)
    recall    = tp / (tp + fn + 1e-6)
    f1        = 2 * precision * recall / (precision + recall + 1e-6)

    return precision, recall, f1, tp, fp, fn

import os
import torch
import torchvision
import torchvision.transforms as T
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torch.utils.data import DataLoader
from PIL import Image
import csv

# ---- YOLO -> x1,y1,x2,y2 Dataset ----
class YoloBoxDataset(torch.utils.data.Dataset):
    def __init__(self, img_dir, label_dir, transforms=None):
        self.img_dir = img_dir
        self.label_dir = label_dir
        self.transforms = transforms
        self.imgs = sorted(os.listdir(img_dir))

    def __getitem__(self, idx):
        filename = self.imgs[idx]
        img_path = os.path.join(self.img_dir, filename)
        label_path = os.path.join(self.label_dir, filename.replace(".jpg", ".txt"))

        img = Image.open(img_path).convert("RGB")
        w, h = img.size

        boxes = []
        labels = []

        with open(label_path) as f:
            for line in f:
                cls, cx, cy, bw, bh = map(float, line.split())
                cx *= w; cy *= h; bw *= w; bh *= h
                x1 = cx - bw/2; y1 = cy - bh/2
                x2 = cx + bw/2; y2 = cy + bh/2
                boxes.append([x1,y1,x2,y2])
                labels.append(1)

        boxes = torch.tensor(boxes, dtype=torch.float32)
        labels = torch.tensor(labels, dtype=torch.int64)

        target = {
            "boxes": boxes,
            "labels": labels,
            "image_id": torch.tensor([idx]),
            "area": (boxes[:,2]-boxes[:,0]) * (boxes[:,3]-boxes[:,1]),
            "iscrowd": torch.zeros((len(boxes),), dtype=torch.int64)
        }

        if self.transforms:
            img = self.transforms(img)

        return img, target

    def __len__(self):
        return len(self.imgs)


# ---- FasterRCNN Model ----
def get_model(num_classes):
    model = torchvision.models.detection.fasterrcnn_resnet50_fpn(weights="DEFAULT")
    in_features = model.roi_heads.box_predictor.cls_score.in_features
    model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)
    return model


def collate_fn(batch):
    return tuple(zip(*batch))


# ---- Training with metric logging ----
def train():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    img_dir = "/home/rokey/box_count/box_training/train/images"
    label_dir = "/home/rokey/box_count/box_training/train/labels"

    dataset = YoloBoxDataset(img_dir, label_dir, transforms=T.ToTensor())
    data_loader = DataLoader(dataset, batch_size=2, shuffle=True,
                             num_workers=4, collate_fn=collate_fn)

    model = get_model(2)
    model.to(device)

    optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)

    num_epochs = 20

    # CSV 파일 생성
    csv_path = "results.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["epoch","loss","precision","recall","f1","tp","fp","fn"])

    for epoch in range(1, num_epochs+1):

        # --------------------------
        #     1) Training Loop
        # --------------------------
        model.train()
        total_loss = 0

        for imgs, targets in data_loader:
            imgs = [i.to(device) for i in imgs]
            tgts = [{k: v.to(device) for k, v in t.items()} for t in targets]

            loss_dict = model(imgs, tgts)
            losses = sum(loss_dict.values())

            optimizer.zero_grad()
            losses.backward()
            optimizer.step()

            total_loss += losses.item()

        # --------------------------
        #     2) Evaluation Loop
        # --------------------------
        model.eval()   # ← ★추가됨★

        precision, recall, f1, tp, fp, fn = evaluate_epoch(
            model, img_dir, label_dir, device
        )

        model.train()  # ← ★다시 학습 모드로★

        # --------------------------
        #     3) CSV 기록
        # --------------------------
        with open(csv_path, "a", newline="") as f:
            csv.writer(f).writerow([
                epoch,
                round(total_loss,4),
                round(precision,4),
                round(recall,4),
                round(f1,4),
                tp, fp, fn
            ])

        # --------------------------
        #     4) 모델 저장
        # --------------------------
        torch.save(model.state_dict(), f"epoch_{epoch}.pth")

        print(f"[Epoch {epoch}] loss={total_loss:.4f} P={precision:.3f} R={recall:.3f} F1={f1:.3f}")

    print("훈련 완료! results.csv 생성됨.")



if __name__ == "__main__":
    train()
