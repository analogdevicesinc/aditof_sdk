ARG DEBIAN_FRONTEND=noninteractive
FROM arm64v8/ubuntu:bionic AS base

RUN apt-get update && \
      apt-get -y install sudo
RUN useradd -m docker && echo "docker:docker" | chpasswd && adduser docker sudo


RUN DEBIAN_FRONTEND=noninteractive apt-get update
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y git
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y cmake --fix-missing
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y g++

# Installing glog for sdk

RUN git clone --branch v0.3.5 --depth 1 https://github.com/google/glog
RUN mkdir build_glog
RUN cd build_glog && cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=/opt/glog ../glog/
RUN cd build_glog && make -j4 && make install

# Install libwebsocket for sdk

RUN apt-get install libssl-dev
RUN git clone --branch v3.2.3 --depth 1 https://github.com/warmcat/libwebsockets
RUN mkdir build_libwebsockets
RUN cd build_libwebsockets && cmake -DLWS_STATIC_PIC=ON -DCMAKE_INSTALL_PREFIX=/opt/websockets ../libwebsockets/
RUN cd build_libwebsockets && make -j4 && make install

# Install protobuf for the sdk

RUN git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf
RUN mkdir build_protobuf
RUN cd build_protobuf && cmake -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=/opt/protobuf ../protobuf/cmake
RUN cd build_protobuf && make -j4 && make install

# Build the sdk
RUN git clone https://github.com/analogdevicesinc/aditof_sdk.git
RUN cd aditof_sdk && mkdir build && cd build && cmake -DWITH_EXAMPLES=off -DUSE_3D_SMART=on -DWITH_NETWORK=on -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" .. && make -j4

# Start server
USER docker
EXPOSE 5000
#CMD ./aditof_sdk/build/apps/server/aditof-server
