FROM ros:humble-ros-base

ARG USER
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Taipei
ENV TERM=xterm-256color

RUN apt-get update && \
    apt-get install -y \
    git \
    vim \
    cmake  \
    build-essential \
    libusb-1.0-0-dev \
    libjpeg-dev \
    libopencv-dev \
    pkg-config \
    tzdata \
    usbutils \
    v4l-utils \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/libuvc/libuvc.git && \
    cd libuvc && mkdir build && cd build && \
    cmake .. && make && make install && ldconfig

RUN groupadd --gid $USER_GID $USER && \
  useradd --uid $USER_UID --gid $USER_GID -ms /bin/bash $USER && \
  echo "$USER ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USER && \
  chmod 0440 /etc/sudoers.d/$USER && \
  echo "source /opt/ros/humble/setup.bash" >> /home/$USER/.bashrc

USER $USER

WORKDIR /home/$USER/infra-ws
COPY ./packages /home/$USER/infra-ws/src
RUN bash -c ". /opt/ros/humble/setup.bash && \
    colcon build --packages-select libuvc_infra" && \
    echo "source /home/$USER/infra-ws/install/setup.bash" >> /home/$USER/.bashrc

CMD ["/bin/bash"]
