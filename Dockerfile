ARG BASE_IMAGE=ubuntu:22.04
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV COMPETITION_LOCAL_DIR=/var/tmp/aeromind-sim

# Run the same installer as a fresh desktop user, without host dependencies.
RUN apt-get update && apt-get install -y --no-install-recommends sudo ca-certificates \
    && useradd --create-home --shell /bin/bash simulator \
    && echo 'simulator ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/simulator \
    && chmod 0440 /etc/sudoers.d/simulator

ARG UBUNTU_MIRROR
RUN if [ -n "${UBUNTU_MIRROR}" ]; then \
      sed -i "s|http://archive.ubuntu.com/ubuntu|${UBUNTU_MIRROR}|g; s|http://security.ubuntu.com/ubuntu|${UBUNTU_MIRROR}|g" /etc/apt/sources.list; \
    fi

USER simulator
WORKDIR /workspace/aeromind-sim
COPY --chown=simulator:simulator . .
ARG BUILD_JOBS=2
ARG ROS_APT_URL
RUN bash tools/setup_ubuntu_22_04.sh \
    && sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

ENV LIBGL_ALWAYS_SOFTWARE=1
ENTRYPOINT ["bash", "/workspace/aeromind-sim/start.sh"]
CMD ["--headless"]
