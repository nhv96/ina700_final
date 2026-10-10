#!/usr/bin/env python3

import argparse
import shutil
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from pypylon import pylon
from ultralytics import YOLO

from http_server import ROBOT_HOST, ROBOT_PORT, RobotLink, parse_position

# ============================================================
# CONFIGURATION
# ============================================================

EXPOSURE_TIME = 25000.0

# Upper limit on processed frames per second, to keep low-resource devices cool.
MAX_FPS = 1.0
WINDOW_NAME = "Detection (ESC / q to quit)"

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


A3_ASPECT = A3_WIDTH_MM / A3_HEIGHT_MM
MIN_PAPER_AREA_FRAC = 0.08
MAX_PAPER_AREA_FRAC = 0.90
MIN_RECTANGULARITY = 0.85
MAX_ASPECT_ERROR = 0.30


def create_white_masks(img):
    """Yield several 'paper-like' masks so one lighting change cannot break detection.

    Thresholds are derived from the image itself (Otsu / percentiles) instead of
    being fixed, and a few variants are tried; the best-scoring one wins later.
    """
    blurred = cv2.GaussianBlur(img, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    _, saturation, value = cv2.split(hsv)

    otsu_value, _ = cv2.threshold(value, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    bright_ref = float(np.percentile(value, 98))
    max_sat = max(60.0, float(np.percentile(saturation[value > otsu_value], 75)) + 25.0) \
        if np.any(value > otsu_value) else 70.0

    value_thresholds = {
        otsu_value,
        0.85 * otsu_value,
        0.60 * bright_ref,
        0.75 * bright_ref,
        170,
    }

    open_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
    close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 21))

    for threshold in sorted(value_thresholds):
        mask = ((value > threshold) & (saturation < max_sat)).astype(np.uint8) * 255
        # Opening cuts thin bridges (e.g. paper touching a bright rail),
        # closing fills the shapes sitting on the paper.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
        yield cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel, iterations=2)


def fit_quad(contour):
    hull = cv2.convexHull(contour)
    perimeter = cv2.arcLength(hull, True)

    for epsilon_factor in (0.010, 0.015, 0.020, 0.025, 0.030, 0.040, 0.050):
        approx = cv2.approxPolyDP(hull, epsilon_factor * perimeter, True)
        if len(approx) == 4:
            return approx.reshape(4, 2).astype(np.float32)

    return cv2.boxPoints(cv2.minAreaRect(hull)).astype(np.float32)


def score_paper_quad(quad, contour, image_shape):
    """Return a 0..1 score of how much a contour looks like an A3 sheet, or 0."""
    image_h, image_w = image_shape[:2]
    image_area = float(image_h * image_w)

    quad_area = cv2.contourArea(quad)
    if quad_area <= 0:
        return 0.0

    area_frac = quad_area / image_area
    if not (MIN_PAPER_AREA_FRAC <= area_frac <= MAX_PAPER_AREA_FRAC):
        return 0.0

    if not cv2.isContourConvex(quad.astype(np.int32)):
        return 0.0

    rectangularity = min(cv2.contourArea(contour) / quad_area, 1.0)
    if rectangularity < MIN_RECTANGULARITY:
        return 0.0

    ordered = order_points(quad)
    side_top = np.linalg.norm(ordered[1] - ordered[0])
    side_bottom = np.linalg.norm(ordered[2] - ordered[3])
    side_left = np.linalg.norm(ordered[3] - ordered[0])
    side_right = np.linalg.norm(ordered[2] - ordered[1])
    long_side = (side_top + side_bottom) / 2.0
    short_side = (side_left + side_right) / 2.0
    if min(long_side, short_side) <= 0:
        return 0.0

    # Orientation independent; tolerant to perspective.
    aspect = max(long_side, short_side) / min(long_side, short_side)
    aspect_error = abs(aspect - A3_ASPECT) / A3_ASPECT
    if aspect_error > MAX_ASPECT_ERROR:
        return 0.0

    # Paper that is cut off by the image border cannot be rectified correctly.
    margin = 3
    if (quad[:, 0].min() < -margin or quad[:, 1].min() < -margin
            or quad[:, 0].max() > image_w + margin or quad[:, 1].max() > image_h + margin):
        return 0.0

    return rectangularity * (1.0 - aspect_error) * (0.5 + 0.5 * area_frac)


def find_a3_corners(img, debug=False):
    best_score = 0.0
    best_quad = None

    for white_mask in create_white_masks(img):
        contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            if cv2.contourArea(cnt) < MIN_PAPER_AREA_FRAC * img.shape[0] * img.shape[1]:
                continue

            quad = fit_quad(cnt)
            score = score_paper_quad(quad, cnt, img.shape)
            if score > best_score:
                best_score, best_quad = score, quad

    if best_quad is None:
        return (None, 0.0) if debug else None

    corners = order_points(best_quad)
    return (corners, best_score) if debug else corners


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

    for obj in candidates:
        crop = crop_candidate(rectified_img, obj["contour"], padding_px=20)

        class_name, confidence, ai_time = classify_crop(model, crop)
        total_ai_time += ai_time

        if class_name is None or class_name.strip().lower() == REJECT_CLASS.lower():
            continue

        if confidence < CONFIDENCE_THRESHOLD:
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


def print_objects(frame_idx, accepted_objects, total_ai_time):
    print(f"[frame {frame_idx}] objects={len(accepted_objects)} AI={total_ai_time * 1000:.0f} ms")

    for index, obj in enumerate(accepted_objects, start=1):
        x_mm, y_mm = obj["center_mm"]
        print(
            f"  {index}: {obj['class_name']} | "
            f"confidence={obj['confidence']:.3f} | "
            f"X={x_mm:.1f} mm | Y={y_mm:.1f} mm | "
            f"angle={obj['angle_deg']:.1f} deg"
        )


# ============================================================
# ROBOT COMMUNICATION
# ============================================================

class DoneAwareRobotLink(RobotLink):
    """RobotLink that also remembers when the robot reports "DONE"."""

    def __init__(self, host, port):
        super().__init__(host, port)
        self.done = threading.Event()

    def _read_forever(self, client):
        while True:
            try:
                data = client.recv(1024)
            except OSError:
                break
            if not data:
                break
            text = data.decode("utf-8", errors="replace")
            print(f"Received: {text}")
            if "DONE" in text:
                self.done.set()
        with self._lock:
            if self._client is client:
                print("Robot closed the connection.")
                self._close_client()


def send_first_object(robot, objects):
    """Send the first object of the list the robot accepts. Returns the sent object or None."""
    for obj in objects:
        x_mm, y_mm = obj["center_mm"]
        try:
            message = parse_position(
                {"x": x_mm, "y": y_mm, "shape": obj["class_name"], "angle": obj["angle_deg"]}
            )
        except ValueError as exc:
            print(f"Skipping {obj['class_name']}: {exc}")
            continue

        robot.done.clear()
        if robot.send(message):
            return obj
        return None
    return None


def draw_banner(img, text, color):
    cv2.putText(
        img, text, (20, 85),
        cv2.FONT_HERSHEY_SIMPLEX, 1.1, color, 3
    )


# ============================================================
# MAIN STREAMING LOOP
# ============================================================

def parse_args():
    parser = argparse.ArgumentParser(description="Real-time A3 object detection from a Basler camera.")
    parser.add_argument(
        "--fps", type=float, default=MAX_FPS,
        help=f"maximum frames processed per second, keeps the CPU cool (default: {MAX_FPS:g})",
    )
    parser.add_argument(
        "--show", action="store_true",
        help="show a live window (needs a display); ESC or q quits. Without it, quit with Ctrl+C. "
             "Required to send objects to the robot (SPACE confirms the start)",
    )
    parser.add_argument(
        "--robot-host", default=ROBOT_HOST,
        help=f"IP to listen on for the robot (default: {ROBOT_HOST})",
    )
    parser.add_argument(
        "--robot-port", type=int, default=ROBOT_PORT,
        help=f"TCP port for the robot (default: {ROBOT_PORT})",
    )
    args = parser.parse_args()
    if args.fps <= 0:
        parser.error("--fps must be greater than 0")
    return args


def open_camera(fps):
    devices = find_devices()
    if not devices:
        raise RuntimeError("No Basler camera found.")

    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    camera.Open()

    try:
        camera.ExposureTime.SetValue(EXPOSURE_TIME)
    except Exception as exc:
        print(f"Could not set exposure time: {exc}")

    # Ask the camera itself to produce no more than `fps` frames per second,
    # so the USB/GigE link and the CPU are not busy with frames we throw away.
    try:
        camera.AcquisitionFrameRateEnable.SetValue(True)
        camera.AcquisitionFrameRate.SetValue(fps)
    except Exception as exc:
        print(f"Could not limit camera frame rate (software limit still applies): {exc}")

    return camera


def process_frame(model, frame_bgr):
    """Run the full pipeline on one frame. Returns (annotated_image, objects, ai_time)."""
    corners = find_a3_corners(frame_bgr)
    if corners is None:
        annotated = frame_bgr.copy()
        cv2.putText(
            annotated, "A3 paper not found", (20, 45),
            cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 255), 3
        )
        return annotated, None, 0.0

    rectified = rectify_a3(frame_bgr, corners)
    candidates, _ = find_object_candidates(rectified)
    accepted_objects, total_ai_time = classify_objects(model, rectified, candidates)
    return draw_results(rectified, accepted_objects, total_ai_time), accepted_objects, total_ai_time


def wait_until(deadline, show):
    """Sleep until `deadline` (perf_counter time).

    Returns "quit" if the user asked to quit, "space" if SPACE was pressed, else None.
    """
    while True:
        remaining = deadline - time.perf_counter()
        if show:
            key = cv2.waitKey(max(1, min(int(remaining * 1000), 50))) & 0xFF
            if key in (27, ord("q")):
                return "quit"
            if key == ord(" "):
                return "space"
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                return "quit"
        elif remaining > 0:
            time.sleep(remaining)
        if remaining <= 0:
            return None


def wait_for_done(robot):
    """Block until the robot reports DONE, keeping the window alive.

    Returns "done", "quit" or "disconnected".
    """
    while not robot.done.wait(0.05):
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")) or cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
            return "quit"
        if not robot.connected:
            return "disconnected"
    return "done"


def main():
    args = parse_args()
    frame_period = 1.0 / args.fps

    if not MODEL_PATH.exists():
        print(f"Model not found: {MODEL_PATH.resolve()}")
        return 1

    print(f"Loading model: {MODEL_PATH.resolve()}")
    model = YOLO(str(MODEL_PATH))

    robot = None
    if args.show:
        try:
            robot = DoneAwareRobotLink(args.robot_host, args.robot_port)
        except OSError as exc:
            print(f"Cannot open robot socket on {args.robot_host}:{args.robot_port}: {exc}")
            return 1
        threading.Thread(target=robot.accept_forever, daemon=True).start()
    else:
        print("Running without --show: objects are only detected, nothing is sent to the robot.")

    try:
        camera = open_camera(args.fps)
    except Exception as exc:
        print(f"Camera error: {exc}")
        if robot is not None:
            robot.close()
        return 1

    converter = pylon.ImageFormatConverter()
    converter.OutputPixelFormat = pylon.PixelType_BGR8packed
    converter.OutputBitAlignment.Value = "MsbAligned"

    if args.show:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    print(f"Streaming at max {args.fps:g} fps. {'Press ESC or q' if args.show else 'Press Ctrl+C'} to quit.")

    # LatestImageOnly drops old frames while we are busy, so we never fall behind.
    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    frame_idx = 0
    next_tick = time.perf_counter()
    armed = False  # True after the operator pressed SPACE: objects are being sent to the robot

    try:
        while camera.IsGrabbing():
            try:
                grab_result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            except pylon.TimeoutException:
                print("Timed out waiting for a camera frame, retrying...")
                continue

            try:
                if not grab_result.GrabSucceeded():
                    print(f"Grab failed: {grab_result.ErrorDescription}")
                    continue
                frame_bgr = converter.Convert(grab_result).GetArray()
            finally:
                grab_result.Release()

            frame_idx += 1
            annotated, objects, ai_time = process_frame(model, frame_bgr)

            if objects is None:
                print(f"[frame {frame_idx}] A3 paper not found")
            else:
                print_objects(frame_idx, objects, ai_time)

            found = bool(objects)

            if armed:
                if not found:
                    print("No more objects found. Sending stopped.")
                    armed = False
                elif (sent := send_first_object(robot, objects)) is None:
                    print("Could not send to the robot (not connected?). Sending stopped.")
                    armed = False
                else:
                    draw_banner(annotated, f"Sent {sent['class_name']}, waiting for DONE...", (0, 140, 255))
                    cv2.imshow(WINDOW_NAME, annotated)
                    cv2.waitKey(1)

                    # Pause the camera while the robot works: no stale frames afterwards.
                    camera.StopGrabbing()
                    result = wait_for_done(robot)
                    if result == "quit":
                        break
                    if result == "disconnected":
                        print("Robot disconnected while waiting for DONE. Sending stopped.")
                        armed = False
                    else:
                        print("Robot reported DONE.")
                    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
                    next_tick = time.perf_counter()
                    continue

            if robot is not None and not armed and found:
                if robot.connected:
                    draw_banner(
                        annotated, f"{len(objects)} object(s) found. Press SPACE to send to robot", (0, 160, 0)
                    )
                else:
                    draw_banner(annotated, "Objects found, but robot is not connected", (0, 0, 255))

            if args.show:
                cv2.imshow(WINDOW_NAME, annotated)

            # Pace the loop: never start the next frame before the period has elapsed.
            # If a frame took longer than the period, restart the clock instead of
            # trying to catch up (catching up is what makes small devices overheat).
            next_tick = max(next_tick + frame_period, time.perf_counter())
            event = wait_until(next_tick, args.show)
            if event == "quit":
                break
            if event == "space" and not armed and found and robot.connected:
                armed = True
                print("Sending to robot started.")

        return 0

    except KeyboardInterrupt:
        print("\nStopped.")
        return 0

    except Exception as exc:
        print(f"An error occurred: {exc}")
        return 1

    finally:
        if camera.IsGrabbing():
            camera.StopGrabbing()
        if camera.IsOpen():
            camera.Close()
        if args.show:
            cv2.destroyAllWindows()
        if robot is not None:
            robot.close()
            print("Sockets closed.")


if __name__ == "__main__":
    raise SystemExit(main())
