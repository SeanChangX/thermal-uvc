FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Taipei

RUN apt-get update && \
    apt-get install -y \
    git cmake build-essential \
    libusb-1.0-0-dev \
    libjpeg-dev \
    libopencv-dev \
    pkg-config \
    tzdata \
    && rm -rf /var/lib/apt/lists/*

# Install libuvc
RUN git clone https://github.com/libuvc/libuvc.git && \
    cd libuvc && mkdir build && cd build && \
    cmake .. && make && make install && ldconfig

WORKDIR /app
COPY . .

RUN mkdir build
WORKDIR /app/build

RUN cmake .. && make

CMD ["./thermal_uvc"]
