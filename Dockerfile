FROM ubuntu:22.04

# Install ohmyzsh for decent terminal
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.2.1/zsh-in-docker.sh)"

RUN apt-get update && apt-get install -y \
    wget \
    build-essential \
    clang-format \
    cppcheck \
    git \
    libusb-1.0-0-dev \
    libtool \
    pkg-config \
    cmake \
    python3 \
    python3-pip \
    libncurses5-dev \
    zlib1g-dev \
    libglib2.0-dev \
    libpixman-1-dev \
    gcc \
    g++ \
    libnewlib-arm-none-eabi \
    stlink-tools \
    && apt-get clean

WORKDIR /opt/toolchain
RUN wget -O toolchain.tar.xz "https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/arm-gnu-toolchain-13.2.rel1-x86_64-arm-none-eabi.tar.xz?rev=e434b9ea4afc4ed7998329566b764309&hash=688C370BF08399033CA9DE3C1CC8CF8E31D8C441" \
    && tar -xf toolchain.tar.xz \
    && rm toolchain.tar.xz \
    && mv arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi /opt/arm-toolchain
ENV PATH="/opt/arm-toolchain/bin:${PATH}"

RUN wget -O renode.tar.xy "https://github.com/renode/renode/releases/download/v1.15.3/renode-1.15.3.linux-portable.tar.gz" \
    && tar -xf renode.tar.xy \
    && rm renode.tar.xy \
    && mv renode_1.15.3_portable /opt/renode
ENV PATH="/opt/renode:${PATH}"

RUN wget -O openocd.tar.xy "https://github.com/xpack-dev-tools/openocd-xpack/releases/download/v0.12.0-4/xpack-openocd-0.12.0-4-linux-arm64.tar.gz" \
    && tar -xf openocd.tar.xy \
    && rm openocd.tar.xy \
    && mv xpack-openocd-0.12.0-4 /opt/openocd
ENV PATH="/opt/openocd:${PATH}"

# RUN git clone https://github.com/ntfreak/openocd.git /opt/openocd && \
#     cd /opt/openocd && \
#     ./bootstrap && \
#     ./configure && \
#     make && \
#     make install

WORKDIR /project

COPY . .

CMD ["/bin/bash"]
