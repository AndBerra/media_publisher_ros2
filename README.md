# ROS 2 Microphone Publisher

This project contains a robust ROS 2 node that captures audio from a microphone and publishes it to a topic. It is designed to be run inside Docker and intelligently handles different audio hardware setups.

## Prerequisites

- Docker and Docker Compose
- A working microphone connected to your host machine.

## Quick Start

1.  **Clone the repository:**
    ```bash
    git clone <your-repo-url>
    cd <your-repo-name>
    ```

2.  **Configure Audio Permissions:**
    This project needs permission to access your host's audio hardware. Run the following command **once** to create a `.env` file that grants this permission.
    ```bash
    echo "AUDIO_GID=$(getent group audio | cut -d: -f3)" > .env
    ```

3.  **Build and Run the Container:**
    ```bash
    docker compose up --build -d
    ```
    The `-d` flag runs the container in the background.

## Usage

1.  **Launch the Microphone Publisher:**
    Open a new terminal and get a shell inside the running container:
    ```bash
    docker exec -it media_publisher_runner bash
    ```
    Inside the container, launch the node:
    ```bash
    ros2 launch media_publisher mic_publisher.launch.py
    ```

2.  **Listen to the Audio Topic:**
    In another terminal, get another shell inside the container:
    ```bash
    docker exec -it media_publisher_runner bash
    ```
    Inside this second shell, listen to the published audio data:
    ```bash
    ros2 topic echo /audio
    ```
    You should see a stream of data if it's working.

## Troubleshooting

### Selecting a Specific Microphone

If the node automatically selects the wrong microphone, you can specify one manually.

First, find the index of your desired microphone by launching the node. It will print a list of all available input devices.
```
[INFO] [mic_publisher]: --- Available Audio Input Devices ---
[INFO] [mic_publisher]:   Index 0: HDA Intel PCH: ALC289 Analog (hw:1,0)
[INFO] [mic_publisher]:   Index 7: My USB Headset (hw:2,0)
[INFO] [mic_publisher]: ------------------------------------
```
Let's say you want to use your USB Headset at `Index 7`. You can launch the node with a ROS parameter:
```bash
ros2 launch media_publisher mic_publisher.launch.py device:=7
```

### "No suitable microphone found" Error

- Ensure your microphone is plugged in and working on the host machine.
- Run `arecord -l` inside the container (`docker exec -it media_publisher_runner arecord -l`). If this shows no devices, there is a problem with your Docker setup. Make sure the `devices` and `group_add` sections in `docker-compose.yml` are correct and that you ran the `.env` setup command.

### ALSA Errors on Startup

Messages like `ALSA lib pcm_dsnoop.c:601:(snd_pcm_dsnoop_open) unable to open slave` are generally harmless. They are printed when the audio system first initializes. If the node proceeds to find your microphone and start publishing, you can safely ignore them.