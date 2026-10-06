FROM python:3.12.15-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    YOLO_CONFIG_DIR=/tmp/Ultralytics

WORKDIR /app

# Install minimal system libraries for OpenCV and USB Basler camera
RUN apt-get update && apt-get install -y --no-install-recommends \
    libglib2.0-0 \
    libgomp1 \
    libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/*

# 1. Install CPU-only PyTorch
RUN pip install --no-cache-dir \
    torch torchvision --index-url https://download.pytorch.org/whl/cpu

# 2. Install headless OpenCV & Ultralytics
RUN pip install --no-cache-dir \
    opencv-python-headless \
    pypylon \
    pyyaml \
    requests \
    pillow \
    scipy \
    pandas

RUN pip install --no-cache-dir ultralytics --no-deps


# Copy project files (main.py, models/ directory, pyproject.toml/requirements.txt)
COPY . /app

# Run main.py on startup
CMD ["python", "main.py"]