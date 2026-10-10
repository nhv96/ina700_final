"""Live preview of the Basler camera, for checking the camera's field of view.

Press ESC or q to quit.
"""
import shutil
from pathlib import Path

import cv2
from pypylon import pylon

WINDOW_NAME = "Basler preview (ESC / q to quit)"


def find_device():
    factory = pylon.TlFactory.GetInstance()
    devices = factory.EnumerateDevices()

    if not devices:
        print("No camera found. Clearing GenICam cache and retrying once...")
        cache_dir = Path.home() / "genicam_xml_cache"
        shutil.rmtree(cache_dir, ignore_errors=True)
        cache_dir.mkdir(parents=True, exist_ok=True)
        devices = factory.EnumerateDevices()

    if not devices:
        raise SystemExit("No Basler camera found.")
    return factory.CreateDevice(devices[0])


def main():
    camera = pylon.InstantCamera(find_device())
    converter = pylon.ImageFormatConverter()
    converter.OutputPixelFormat = pylon.PixelType_BGR8packed
    converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

    camera.Open()
    print(f"Using camera: {camera.GetDeviceInfo().GetModelName()}")
    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    try:
        while camera.IsGrabbing():
            result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            try:
                if result.GrabSucceeded():
                    frame = converter.Convert(result).GetArray()
                    cv2.imshow(WINDOW_NAME, frame)
            finally:
                result.Release()

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        camera.StopGrabbing()
        camera.Close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
