FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

# Install system and python dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    portaudio19-dev \
    python3-tk \
    ros-humble-cv-bridge \
    ros-humble-rqt-image-view \
    python3-opencv \         
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir PyAudio "numpy<2" matplotlib

# Set up workspace
WORKDIR /ros2_ws

# add to bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Create an empty src directory as a placeholder for the volume mount
RUN mkdir src

# Copy the entrypoint script and make it executable
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh
ENTRYPOINT ["entrypoint.sh"]
CMD ["bash"]