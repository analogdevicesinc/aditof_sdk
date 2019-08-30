FROM rycus86/arm64v8-debian-qemu

RUN mkdir -p aditof-deps
WORKDIR aditof-deps

COPY ci/travis/lib.sh /aditof-deps
COPY ci/travis/setup_docker.sh /aditof-deps
ADD temp_deps/ /aditof-deps

RUN apt update
RUN apt install -y sudo

RUN sudo apt install -y build-essential cmake python-dev python3-dev \
        libssl-dev git

RUN chmod +x ./setup_docker.sh
RUN ./setup_docker.sh
