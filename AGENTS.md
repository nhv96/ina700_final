# Project overview

This is a computer vision project which leverage a pre-trained AI model to detect object shapes.

This project mainly use "uv" to manage python packages. The `requirements.txt` file is a leftover from the previous source code
when migrating to "uv.

This project is intended to run on a Raspberry Pi device that running Ubuntu OS and have around 5 GB of space left.

So the goal is the project must be small and lightweight.

The Raspberry will run the program to detect objects and their positions (x,y,shape,angle), and send this information over
TCP socket connection to ABB robot for it to move to that position and pick up the object.

The RAPID code of ABB robot is stored in the folder `rapid`.

# Techstacks

This project uses YOLO model and Ultralytics libraries.

The pre-trained model is in `models` folder.

# How to run

The bash commands to run this project is in `README.md` file.