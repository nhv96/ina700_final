# Run with uv (recommended on the Raspberry Pi)

uv installs the exact versions from `uv.lock`, including the CPU-only PyTorch, so the
environment is about 1.5 GB (the default CUDA build is about 6 GB). It runs directly on the
desktop, so the live window works.

```bash
# 1. Install uv (once)
curl -LsSf https://astral.sh/uv/install.sh | sh

# 2. In the project folder: create .venv with Python 3.12 and install the locked packages
uv sync --frozen

# 3. Run with the live window (SPACE starts sending objects to the robot, ESC / q quits)
uv run --frozen main.py --show
```

The Python version comes from `.python-version` (3.12). uv downloads it by itself if it is
not installed, so the Pi's system Python 3.13 is left untouched.

Without `--show` the program only detects objects and never sends anything to the robot.

# Run with Docker (older way, no live window)

Step to build:

## 1. Build ARM64 image on your PC
docker buildx build --platform linux/arm64 -t yolo-pi:latest --load .

## 2. Save & compress to a tar archive (~800MB)
docker save yolo-pi:latest | gzip > yolo-pi.tar.gz

# 3. Transfer yolo-pi.tar.gz to the Pi (via SCP or USB)

## 4. On the Raspberry Pi terminal:
docker load < yolo-pi.tar.gz

rm yolo-pi.tar.gz # Delete immediately after loading to free space

## 5. Run the container (pass webcam device if main.py uses a video feed)
docker run --rm --device /dev/video0:/dev/video0 yolo-pi:latest


## If using a USB Basler Camera:

docker run --rm --privileged -v /dev/bus/usb:/dev/bus/usb yolo-pi:latest

## If using a GigE (Ethernet) Basler Camera:

docker run --rm --net=host yolo-pi:latest


# 1. Grant local Docker containers permission to draw windows on your screen
xhost +local:docker

# 2. Run the container with GUI and camera access
docker run --rm -it \
  --net=host \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --privileged -v /dev/bus/usb:/dev/bus/usb \
  yolo-pi:latest


# 2.1. Run container and map /app/output to current directory's output folder
docker run --rm \
  --privileged -v /dev/bus/usb:/dev/bus/usb \
  -v $(pwd)/output:/app/output \
  yolo-pi:latest