FROM --platform=linux/amd64 ubuntu:22.04

RUN apt-get update --fix-missing && apt-get install -y --no-install-recommends \
    gcc \
    gcc-multilib \
    g++-multilib \
    make \
    vim \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /malloclab

COPY . .

RUN make clean

CMD ["/bin/bash"]
