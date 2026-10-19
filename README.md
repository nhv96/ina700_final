# Project2: shape picking with a Basler camera, YOLO and an ABB robot

A Raspberry Pi finds objects on an A3 sheet (camera + YOLO), then sends each object's
`x,y,shape,angle` over TCP to an ABB robot, which picks it up.

```
Basler camera --> Raspberry Pi (main.py) --TCP port 5000--> ABB robot (RAPID code)
```

## 1. Raspberry Pi setup

### Files to copy to the Pi

Copy these into one folder on the Pi (for example `~/project2`):

| File | Why |
|---|---|
| `main.py` | the program |
| `http_server.py` | TCP link to the robot (imported by `main.py`) |
| `models/best.pt` | the trained YOLO model |
| `pyproject.toml`, `uv.lock`, `.python-version` | exact package versions and Python version |
| `Makefile` | optional, short command to run the program |

`stream.py` (a camera preview for checking the field of view) is optional.
Do not copy `.venv`, `output` or `rapid`.

Example from your PC:

```bash
ssh <user>@<pi-ip> mkdir -p ~/project2/models
scp main.py http_server.py pyproject.toml uv.lock .python-version Makefile <user>@<pi-ip>:~/project2/
scp models/best.pt <user>@<pi-ip>:~/project2/models/
```

### Install (once, on the Pi)

uv installs the exact versions from `uv.lock`, including the CPU-only PyTorch, so the
environment is about 1.5 GB (the default CUDA build is about 6 GB). The Python version comes
from `.python-version` (3.12). uv downloads it by itself, so the Pi's system Python is left alone.

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
cd ~/project2
uv sync --frozen
```

### Network

The Pi listens for the robot on `192.168.125.203:5000` (`ROBOT_HOST` / `ROBOT_PORT` in
`http_server.py`). Give the Pi this IP on the robot network, or pass `--robot-host` and
`--robot-port` to `main.py`. The robot's RAPID code must connect to the same IP and port.

## 2. ABB robot setup

Copy `rapid/lab2.mod` to the robot controller (or load it with RobotStudio) and load it as a
program module. Its `main` procedure waits for the start button, then reads
`x,y,shape,angle` messages until it receives `stop`.

`lab2.mod` calls `RobotAsClientConnect` and `RobotClientReciveMessage`, which are not in this
repository. They must already exist on the robot (your socket module) and connect to the Pi's
IP and port above.

## 3. Run

Start the Pi first, then the robot. The Pi must be listening before the RAPID program connects.

1. On the Pi, in the project folder (on its desktop, since the live window needs a display):

   ```bash
   make run
   # same as: uv run --frozen main.py --show
   ```

   The program prints `Waiting for ABB Robot to connect...`.

2. On the robot, start the RAPID program. The Pi prints `Robot connected from IP: ...`.
3. Press the robot's OK button (`diOkButton`) so the program starts reading messages.
4. In the live window, press **SPACE** to start sending objects to the robot.
   Press **ESC** or **q** to quit; the Pi then sends `stop` to the robot.

Without `--show` the program only detects objects and never sends anything to the robot.
