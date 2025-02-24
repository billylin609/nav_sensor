FROM ros:noetic-ros-core

# Use bash as the default shell
SHELL ["/bin/bash", "-c"]

# Install build tools and required ROS packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        make \
        g++ \
        cmake \
        ros-noetic-tf2 \
        ros-noetic-tf2-geometry-msgs \
        python3-pip && \
    rm -rf /var/lib/apt/lists/*

# Copy the entire repository into the catkin workspace
COPY . /catkin_ws/src

# Change to the workspace directory
WORKDIR /catkin_ws

# Install python package 'lcm', build the workspace, and (optionally) set LCM_DEFAULT_URL for the build step
RUN pip install lcm && \
    source /opt/ros/noetic/setup.bash && \
    catkin_make

# Create an entrypoint script that sources the workspace, sets the LCM environment variable,
# and then executes any command passed to the container.
RUN printf '#!/bin/bash\n' > /entrypoint.bash && \
    printf 'export LCM_DEFAULT_URL=udpm://239.255.76.67:7667?ttl=1\n' >> /entrypoint.bash && \
    printf 'source /catkin_ws/devel/setup.bash\n' >> /entrypoint.bash && \
    printf 'exec "$@"\n' >> /entrypoint.bash && \
    chmod +x /entrypoint.bash && \
    printf 'source /entrypoint.bash' >> .bashrc
