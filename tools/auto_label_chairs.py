#!/usr/bin/env python3
"""Auto-label chair images for YOLO training.

Uses the official COCO-pretrained yolov8n.pt to generate initial labels.
Run this first, then open LabelImg to verify/correct the boxes.

Usage:
    python tools/auto_label_chairs.py \
        --src "D:\李承城大学\采集的图片" \
        --out "D:\李承城大学\chair_dataset"
"""

import argparse
import os
import random
import shutil
from pathlib import Path
from typing import List, Tuple

import yaml
from PIL import Image
from ultralytics import YOLO


def collect_images(src_dir: Path) -> List[Path]:
    """Collect all image files from the source directory."""
    exts = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
    images = []
    for f in sorted(src_dir.iterdir()):
        if f.is_file() and f.suffix.lower() in exts:
            images.append(f)
    return images


def yolo_label_from_pil(image_size: Tuple[int, int], box_xyxy: List[float]) -> str:
    """Convert COCO xyxy box to YOLO normalized xywh format."""
    img_w, img_h = image_size
    x1, y1, x2, y2 = box_xyxy
    x_center = ((x1 + x2) / 2) / img_w
    y_center = ((y1 + y2) / 2) / img_h
    width = (x2 - x1) / img_w
    height = (y2 - y1) / img_h
    # Clamp to [0, 1]
    x_center = max(0.0, min(1.0, x_center))
    y_center = max(0.0, min(1.0, y_center))
    width = max(0.0, min(1.0, width))
    height = max(0.0, min(1.0, height))
    return f"0 {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}"


def auto_label(
    src_dir: Path,
    out_dir: Path,
    model_path: str = "yolov8n.pt",
    conf: float = 0.40,
    val_ratio: float = 0.2,
    seed: int = 42,
) -> None:
    """Build a YOLO dataset with auto-generated chair labels."""
    images = collect_images(src_dir)
    if not images:
        raise RuntimeError(f"No images found in {src_dir}")
    print(f"[info] found {len(images)} images")

    # Create dataset structure
    images_train_dir = out_dir / "images" / "train"
    images_val_dir = out_dir / "images" / "val"
    labels_train_dir = out_dir / "labels" / "train"
    labels_val_dir = out_dir / "labels" / "val"
    for d in (images_train_dir, images_val_dir, labels_train_dir, labels_val_dir):
        d.mkdir(parents=True, exist_ok=True)

    # Split train/val (80/20)
    random.seed(seed)
    random.shuffle(images)
    val_count = int(len(images) * val_ratio)
    val_imgs = images[:val_count]
    train_imgs = images[val_count:]
    print(f"[info] train: {len(train_imgs)}, val: {len(val_imgs)}")

    # Load model
    print(f"[info] loading {model_path} ...")
    model = YOLO(model_path)

    def process_split(split_name: str, split_images: List[Path], img_dir: Path, lbl_dir: Path) -> None:
        labeled_count = 0
        for src_path in split_images:
            # Copy image
            dst_path = img_dir / src_path.name
            shutil.copy2(src_path, dst_path)

            # Detect chairs
            pil_img = Image.open(src_path).convert("RGB")
            results = model.predict(
                source=pil_img,
                imgsz=640,
                conf=conf,
                classes=[56],  # COCO chair class
                verbose=False,
            )
            r = results[0]
            labels = []
            if r.boxes is not None:
                for box in r.boxes:
                    xyxy = [float(v) for v in box.xyxy[0]]
                    label = yolo_label_from_pil(pil_img.size, xyxy)
                    labels.append(label)

            # Save label file
            label_path = lbl_dir / (src_path.stem + ".txt")
            if labels:
                label_path.write_text("\n".join(labels), encoding="utf-8")
                labeled_count += 1
            else:
                label_path.write_text("", encoding="utf-8")

        print(f"[info] {split_name}: {labeled_count}/{len(split_images)} images have at least one chair label")

    process_split("train", train_imgs, images_train_dir, labels_train_dir)
    process_split("val", val_imgs, images_val_dir, labels_val_dir)

    # Write data.yaml
    data_yaml = {
        "path": str(out_dir.resolve()),
        "train": "images/train",
        "val": "images/val",
        "names": {0: "chair"},
    }
    yaml_path = out_dir / "data.yaml"
    with open(yaml_path, "w", encoding="utf-8") as f:
        yaml.dump(data_yaml, f, sort_keys=False, allow_unicode=True)

    print(f"[info] dataset saved to: {out_dir}")
    print(f"[info] data.yaml: {yaml_path}")
    print("[info] next step: open LabelImg and verify/correct the labels")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Auto-label chair images for YOLO training")
    parser.add_argument("--src", type=str, required=True, help="source folder with chair images")
    parser.add_argument("--out", type=str, default=r"D:\李承城大学\chair_dataset", help="output YOLO dataset folder")
    parser.add_argument("--model", type=str, default="yolov8n.pt", help="COCO pretrained YOLO model")
    parser.add_argument("--conf", type=float, default=0.40, help="confidence threshold")
    parser.add_argument("--val-ratio", type=float, default=0.2, help="validation split ratio")
    args = parser.parse_args()

    auto_label(
        src_dir=Path(args.src),
        out_dir=Path(args.out),
        model_path=args.model,
        conf=args.conf,
        val_ratio=args.val_ratio,
    )
