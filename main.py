#!/usr/bin/env python3

import shutil
import time
from pathlib import Path

import cv2
import numpy as np
from pypylon import pylon
from ultralytics import YOLO

# ============================================================
# CONFIGURATION
# ============================================================

EXPOSURE_TIME = 25000.0

BASE_DIR = Path(__file__).resolve().parent
MODEL_PATH = BASE_DIR / "models" / "best.pt"
OUTPUT_DIR = BASE_DIR / "output"

A3_WIDTH_MM = 420.0
A3_HEIGHT_MM = 297.0
PX_PER_MM = 4.0

MIN_OBJECT_MM = 35.0
MAX_OBJECT_MM = 70.0
MIN_CONTOUR_AREA_PX = 1500
WORK_MARGIN_MM = 5.0

CONFIDENCE_THRESHOLD = 0.9
REJECT_CLASS = "other"


# ============================================================
# BASLER CAMERA
# ============================================================

def clear_genicam_cache():
    cache_dir = Path.home() / "genicam_xml_cache"
    shutil.rmtree(cache_dir, ignore_errors=True)
    cache_dir.mkdir(parents=True, exist_ok=True)


def find_devices():
    factory = pylon.TlFactory.GetInstance()
    devices = factory.EnumerateDevices()

    if devices:
        return devices

    print("No camera found. Clearing GenICam cache and retrying once...")
    clear_genicam_cache()
    return factory.EnumerateDevices()


# ============================================================
# A3 DETECTION & RECTIFICATION
# ============================================================

def order_points(points):
    pts = np.asarray(points, dtype=np.float32).reshape(4, 2)
    ordered = np.zeros((4, 2), dtype=np.float32)

    point_sum = pts.sum(axis=1)
    point_diff = np.diff(pts, axis=1).reshape(-1)

    ordered[0] = pts[np.argmin(point_sum)]
    ordered[2] = pts[np.argmax(point_sum)]
    ordered[1] = pts[np.argmin(point_diff)]
    ordered[3] = pts[np.argmax(point_diff)]

    return ordered


def create_white_mask(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    _, saturation, value = cv2.split(hsv)

    mask = np.zeros(img.shape[:2], dtype=np.uint8)
    mask[(saturation < 70) & (value > 170)] = 255

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 21))
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)


def find_a3_corners(img):
    white_mask = create_white_mask(img)
    contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None

    image_h, image_w = img.shape[:2]
    image_area = float(image_h * image_w)
    candidates = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 0.10 * image_area:
            continue

        x, y, w, h = cv2.boundingRect(cnt)
        spans_full_width = (x <= 2 and (x + w) >= image_w - 2)
        spans_full_height = (y <= 2 and (y + h) >= image_h - 2)

        if spans_full_width or spans_full_height:
            continue

        candidates.append(cnt)

    if not candidates:
        return None

    paper_contour = max(candidates, key=cv2.contourArea)
    perimeter = cv2.arcLength(paper_contour, True)
    paper_quad = None

    for epsilon_factor in (0.020, 0.025, 0.030, 0.035, 0.040, 0.050):
        approx = cv2.approxPolyDP(paper_contour, epsilon_factor * perimeter, True)
        if len(approx) == 4:
            paper_quad = approx.reshape(4, 2).astype(np.float32)
            break

    if paper_quad is None:
        rect = cv2.minAreaRect(paper_contour)
        paper_quad = cv2.boxPoints(rect).astype(np.float32)

    return order_points(paper_quad)


def rectify_a3(img, corners):
    width_px = int(round(A3_WIDTH_MM * PX_PER_MM))
    height_px = int(round(A3_HEIGHT_MM * PX_PER_MM))

    destination = np.array(
        [[0, 0], [width_px - 1, 0], [width_px - 1, height_px - 1], [0, height_px - 1]],
        dtype=np.float32,
    )

    matrix = cv2.getPerspectiveTransform(corners.astype(np.float32), destination)
    return cv2.warpPerspective(img, matrix, (width_px, height_px), flags=cv2.INTER_LINEAR)


# ============================================================
# OBJECT DETECTION & CLASSIFICATION
# ============================================================

def create_object_mask(rectified_img):
    h, w = rectified_img.shape[:2]
    margin_px = int(round(WORK_MARGIN_MM * PX_PER_MM))

    work_mask = np.zeros((h, w), dtype=np.uint8)
    cv2.rectangle(
        work_mask,
        (margin_px, margin_px),
        (w - margin_px - 1, h - margin_px - 1),
        255,
        thickness=-1
    )

    hsv = cv2.cvtColor(rectified_img, cv2.COLOR_BGR2HSV)
    _, saturation, value = cv2.split(hsv)

    object_mask = np.zeros(rectified_img.shape[:2], dtype=np.uint8)
    object_pixels = ((saturation > 55) | (value < 180)) & (work_mask > 0)
    object_mask[object_pixels] = 255

    object_mask = cv2.morphologyEx(
        object_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1
    )
    return cv2.morphologyEx(
        object_mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8), iterations=1
    )


def normalize_box_angle(rect):
    (_, _), (w, h), angle = rect
    if w < h:
        angle += 90.0
    return angle % 180.0


def find_object_candidates(rectified_img):
    object_mask = create_object_mask(rectified_img)
    contours, _ = cv2.findContours(object_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_CONTOUR_AREA_PX:
            continue

        rect = cv2.minAreaRect(cnt)
        (cx, cy), (w_px, h_px), _ = rect
        if w_px <= 0 or h_px <= 0:
            continue

        w_mm = w_px / PX_PER_MM
        h_mm = h_px / PX_PER_MM

        short_mm = min(w_mm, h_mm)
        long_mm = max(w_mm, h_mm)

        if not (
            MIN_OBJECT_MM <= short_mm <= MAX_OBJECT_MM
            and MIN_OBJECT_MM <= long_mm <= MAX_OBJECT_MM
        ):
            continue

        raw_x_mm = cx / PX_PER_MM
        raw_y_mm = cy / PX_PER_MM

        # Swapped coordinates
        output_x_mm = raw_y_mm
        output_y_mm = raw_x_mm

        candidates.append(
            {
                "contour": cnt,
                "rect": rect,
                "center_px": (cx, cy),
                "center_mm_raw": (raw_x_mm, raw_y_mm),
                "center_mm": (output_x_mm, output_y_mm),
                "size_mm": (w_mm, h_mm),
                "angle_deg": normalize_box_angle(rect),
            }
        )

    candidates.sort(key=lambda obj: (obj["center_mm"][0], obj["center_mm"][1]))
    return candidates, object_mask


def crop_candidate(img, contour, padding_px=20):
    x, y, w, h = cv2.boundingRect(contour)
    x1 = max(0, x - padding_px)
    y1 = max(0, y - padding_px)
    x2 = min(img.shape[1], x + w + padding_px)
    y2 = min(img.shape[0], y + h + padding_px)
    return img[y1:y2, x1:x2]


def classify_crop(model, crop):
    start = time.perf_counter()
    results = model.predict(source=crop, verbose=False)
    elapsed = time.perf_counter() - start

    result = results[0]
    if result.probs is None:
        return None, 0.0, elapsed

    top1_id = int(result.probs.top1)
    confidence = float(result.probs.top1conf.cpu().item())
    class_name = result.names[top1_id]

    return class_name, confidence, elapsed


def classify_objects(model, rectified_img, candidates):
    accepted_objects = []
    total_ai_time = 0.0

    print(f"\n--- Found {len(candidates)} candidate shapes in mask ---")

    for idx, obj in enumerate(candidates, start=1):
        crop = crop_candidate(rectified_img, obj["contour"], padding_px=20)
        
        # Save crops to inspect what the model sees
        cv2.imwrite(str(OUTPUT_DIR / f"crop_{idx}.png"), crop)

        class_name, confidence, ai_time = classify_crop(model, crop)
        total_ai_time += ai_time

        print(f"Candidate {idx}: Predicted='{class_name}', Conf={confidence:.3f}, Size={obj['size_mm'][0]:.1f}x{obj['size_mm'][1]:.1f}mm")

        if class_name is None or class_name.strip().lower() == REJECT_CLASS.lower():
            continue

        if confidence < CONFIDENCE_THRESHOLD:
            print(f" -> Rejected due to low confidence ({confidence:.3f} < {CONFIDENCE_THRESHOLD})")
            continue

        obj["class_name"] = class_name
        obj["confidence"] = confidence
        obj["ai_time"] = ai_time
        accepted_objects.append(obj)

    return accepted_objects, total_ai_time


# ============================================================
# DRAWING & ANNOTATION
# ============================================================

def draw_results(rectified_img, accepted_objects, total_ai_time):
    output = rectified_img.copy()

    for obj in accepted_objects:
        box = cv2.boxPoints(obj["rect"])
        box = np.round(box).astype(np.int32)

        cx, cy = obj["center_px"]
        x_mm, y_mm = obj["center_mm"]

        cv2.drawContours(output, [box], 0, (0, 255, 0), 4)
        cv2.circle(output, (int(round(cx)), int(round(cy))), 7, (0, 0, 255), -1)

        label1 = f"{obj['class_name']} {obj['confidence']:.2f}"
        label2 = f"X={x_mm:.1f} mm Y={y_mm:.1f} mm A={obj['angle_deg']:.1f} deg"

        text_x = max(5, int(round(cx)) - 110)
        text_y = max(25, int(round(cy)) - 40)

        cv2.putText(
            output, label1, (text_x, text_y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 140, 0), 2
        )
        cv2.putText(
            output, label2, (text_x, text_y + 28),
            cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 140, 0), 2
        )

    status = f"Objects: {len(accepted_objects)} | AI: {total_ai_time * 1000:.0f} ms"
    cv2.putText(
        output, status, (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2
    )

    return output


def print_objects(accepted_objects, total_ai_time):
    print("\n" + "=" * 70)
    print(f"Accepted target objects: {len(accepted_objects)}")

    for index, obj in enumerate(accepted_objects, start=1):
        x_mm, y_mm = obj["center_mm"]
        print(
            f"Object {index}: {obj['class_name']} | "
            f"confidence={obj['confidence']:.3f} | "
            f"X={x_mm:.1f} mm | Y={y_mm:.1f} mm | "
            f"angle={obj['angle_deg']:.1f} deg | "
            f"AI={obj['ai_time'] * 1000:.1f} ms"
        )

    print(f"Total AI inference time: {total_ai_time:.3f} s")


# ============================================================
# MAIN SINGLE-SHOT CAPTURE
# ============================================================

def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    if not MODEL_PATH.exists():
        print(f"Model not found: {MODEL_PATH.resolve()}")
        return 1

    print(f"Loading model: {MODEL_PATH.resolve()}")
    model = YOLO(str(MODEL_PATH))

    factory = pylon.TlFactory.GetInstance()
    devices = find_devices()

    if not devices:
        print("No Basler camera found.")
        return 1

    try:
        camera = pylon.InstantCamera(factory.CreateFirstDevice())
        camera.Open()
    except Exception as exc:
        print(f"Camera error: {exc}")
        return 1

    try:
        camera.ExposureTime.SetValue(EXPOSURE_TIME)
    except Exception as exc:
        print(f"Could not set exposure time: {exc}")

    converter = pylon.ImageFormatConverter()
    converter.OutputPixelFormat = pylon.PixelType_BGR8packed
    converter.OutputBitAlignment.Value = "MsbAligned"

    print("Capturing frame from Basler camera...")

    try:
        # Capture exactly ONE frame
        grab_result = camera.GrabOne(5000, pylon.TimeoutHandling_ThrowException)

        if not grab_result.GrabSucceeded():
            print("Failed to grab frame from camera.")
            return 1

        image = converter.Convert(grab_result)
        frame_bgr = image.GetArray()
        grab_result.Release()

        # Save raw captured image
        raw_path = OUTPUT_DIR / "raw_capture.png"
        cv2.imwrite(str(raw_path), frame_bgr)
        print(f"Saved raw image to: {raw_path}")

        # 1. Find A3 paper
        corners = find_a3_corners(frame_bgr)

        if corners is None:
            print("A3 paper not found in capture.")
            return 0

        # 2. Rectify A3 paper
        rectified = rectify_a3(frame_bgr, corners)

        # 3. Find candidate shapes
        candidates, object_mask = find_object_candidates(rectified)
        cv2.imwrite(str(OUTPUT_DIR / "object_mask.png"), object_mask)

        # 4. Classify objects
        accepted_objects, total_ai_time = classify_objects(model, rectified, candidates)

        # 5. Draw and save annotated output
        output_img = draw_results(rectified, accepted_objects, total_ai_time)
        result_path = OUTPUT_DIR / "detection_result.png"
        cv2.imwrite(str(result_path), output_img)

        # 6. Print terminal output
        print_objects(accepted_objects, total_ai_time)
        print(f"\nDetection image saved successfully to: {result_path}")

        return 0

    except Exception as exc:
        print(f"An error occurred during detection: {exc}")
        return 1

    finally:
        if camera.IsOpen():
            camera.Close()


if __name__ == "__main__":
    raise SystemExit(main())