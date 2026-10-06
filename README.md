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