<div align="center">

# ROS 2 Media Publisher Pipeline

**A complete, Dockerized ROS 2 pipeline for capturing, visualizing, and recording audio and video streams.**

[![ROS Distro](https://img.shields.io/badge/ROS-Humble-blueviolet)](https://docs.ros.org/en/humble/index.html)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)](https://github.com/)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

</div>

This project provides a robust, out-of-the-box solution for handling common media tasks in ROS 2. It's designed for rapid prototyping and development, with all dependencies managed inside a Docker container.

---

## Table of Contents

- [Features](#features)
- [Prerequisites](#prerequisites)
- [Quick Start & Setup](#quick-start--setup)
- [Usage](#usage)
    - [Launching the System](#launching-the-system)
    - [Recording a ROS Bag](#recording-a-ros-bag)
- [Troubleshooting](#troubleshooting)
- [Project Structure](#project-structure)

---

## Features

- **Audio Pipeline**: Captures from a microphone (`mic_publisher`) and provides a terminal-based FFT visualizer (`audio_visualizer`).
- **Video Pipeline**: Captures from a webcam (`camera_publisher`) and displays the feed in a GUI (`rqt_image_view`).
- **Dockerized Environment**: All ROS 2 and system dependencies are pre-installed. No need to install libraries on your host system.
- **Modular & Reusable**: Launch files are split by functionality (`audio`, `video`), allowing you to run only the components you need.
- **One-Command Recording**: A simple launch argument (`record_bag:=true`) records all active topics to a ROS 2 bag file.
- **Configurable**: Easily select a specific microphone or camera via ROS parameters.

## Prerequisites

- **Docker** and **Docker Compose**
- **Linux Host OS** (tested on Ubuntu 22.04)
- An **X11 display server** (standard on most Linux desktops)
- A working **microphone** and **webcam** connected to your host machine.

## Quick Start & Setup

Follow these steps on your host machine to get the environment running.

#### 1. Clone the Repository

```bash
git clone git@github.com:AndBerra/media_publisher_ros2.git
cd media_publisher_ros2
```

#### 2. Configure Host Permissions

The Docker container needs permission to access your host's GUI and audio hardware.

**a) Grant GUI Access**
This command allows the Docker container to connect to your host's display server to show graphical windows like `rqt_image_view`.

```bash
xhost +local:docker
```

<details>
<summary>Why is this necessary?</summary>
By default, Docker containers are isolated from the host's graphical environment. This command securely tells your X11 display server (which manages all GUI windows) to accept connections from local containers, allowing `rqt_image_view` to open a window on your desktop. This setting is temporary and will reset on reboot.
</details>

**b) Grant Audio Device Access**
This command creates a `.env` file that tells Docker Compose which user group owns the audio devices on your host machine.

```bash
echo "AUDIO_GID=$(getent group audio | cut -d: -f3)" > .env
```
This step only needs to be done **once**.

#### 3. Build and Run

Use Docker Compose to build the image (which runs `colcon build` internally) and start the container.

```bash
docker compose up --build
```
This will start the container and attach your terminal to the logs. To run in the background, use the `-d` flag: `docker compose up --build -d`.

You are now set up. Proceed to the [Usage](#usage) section to launch the ROS nodes.

## Usage

All ROS commands must be run from inside the container. First, open a new terminal and get an interactive shell:

```bash
docker exec -it media_publisher bash
```

> **Note:** Your container name may vary. Use `docker ps` to find the exact name (e.g., `media_publisher-runner-1`).

### Launching the System

Inside the container's shell, you have several options:

#### Option A: Launch Everything

Run the audio and video pipelines simultaneously. The `rqt_image_view` window will appear on your desktop.

```bash
ros2 launch media_publisher media.launch.py
```

#### **Option B: Launch Audio Only**

```bash
ros2 launch media_publisher audio.launch.py
```

#### **Option C: Launch Video Only**

```bash
ros2 launch media_publisher video.launch.py
```

### Recording a ROS Bag

You can record all published topics by adding the `record_bag:=true` argument to your launch command.

```bash
# Example: Launch everything and record a bag file
ros2 launch media_publisher media.launch.py record_bag:=true
```

- The bag file will be saved in a new, timestamped folder (e.g., `rosbag2_2023_10_27_...`) inside the **`ros2_bags/`** directory on your **host machine**. This directory will be created automatically in your project root.
- Press `Ctrl+C` in the launch terminal to stop the nodes and finalize the bag recording.

## Troubleshooting

#### GUI (rqt_image_view) Fails to Start

If you see an error like `could not connect to display :0` or `Authorization required`:

1. Confirm you ran `xhost +local:docker` on your host machine **before** running `docker compose up`.

2. Verify your `docker-compose.yml` correctly forwards the `DISPLAY` environment variable and mounts the `/tmp/.X11-unix` socket.

#### Selecting a Specific Microphone

If the wrong microphone is being used, first find its index by launching the audio nodes. The available devices will be printed in the terminal:

```bash
[INFO] [mic_publisher]: --- Available Audio Input Devices ---
[INFO] [mic_publisher]:   Index 0: HDA Intel PCH: ALC289 Analog (hw:1,0)
[INFO] [mic_publisher]:   Index 7: My USB Headset (hw:2,0)
[INFO] [mic_publisher]: ------------------------------------
```

Then, re-launch with the `device` parameter. For example, to use `Index 7`:

```bash
ros2 launch media_publisher audio.launch.py device:=7
```

#### **"No suitable microphone found" or No Video**

1.  Ensure your microphone/webcam is plugged in and functional on the host.
2.  Check if the device is visible inside the container:
    -   Audio: `docker exec -it <container_name> arecord -l`
    -   Video: `docker exec -it <container_name> v4l2-ctl --list-devices`
3.  If no devices appear, it is likely a Docker permission issue. Double-check that your `docker-compose.yml` correctly mounts the `/dev/snd` and `/dev/video*` devices and that the `.env` file was created correctly.

## Project Structure

```bash
.
├── docker/                # Docker related files
│    ├── docker-compose.yml     # Defines the Docker services and permissions
│    ├── Dockerfile             # Defines the container image and build steps
├── .env                   # Stores host-specific group ID for audio (auto-generated)
├── media_publisher/
│   ├── launch/            # ROS 2 launch files
│   │   ├── all_media.launch.py
│   │   ├── audio.launch.py
│   │   └── video.launch.py
│   ├── src/               # nodes source code
│   ├── package.xml
│   └── CMakeLists.txt
└── README.md              
```