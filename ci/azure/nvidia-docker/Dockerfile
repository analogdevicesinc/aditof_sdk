FROM nvcr.io/nvidia/l4t-base:r32.4.3
# LABEL io.balena.architecture="aarch64"
 
# LABEL io.balena.qemu.version="5.2.0+balena-aarch64"
# COPY qemu-aarch64-static /usr/bin 

RUN mkdir -p aditof-deps
WORKDIR aditof-deps

COPY ci/azure/lib.sh /aditof-deps
COPY ci/azure/setup_docker.sh /aditof-deps
ADD temp_deps/ /aditof-deps

RUN apt update
RUN apt install -y sudo

RUN sudo apt install -y build-essential cmake python-dev python3-dev \
        libssl-dev git

RUN chmod +x ./setup_docker.sh
RUN ./setup_docker.sh
