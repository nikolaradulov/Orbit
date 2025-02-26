# Copyright (c) 2022-2024, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# Nvidia Dockerfiles: https://github.com/NVIDIA-Omniverse/IsaacSim-dockerfiles
# Please check above link for license information.

# Base image
ARG ISAACSIM_VERSION
FROM nvcr.io/nvidia/isaac-sim:${ISAACSIM_VERSION}

# Set default RUN shell to bash
SHELL ["/bin/bash", "-c"]

# Adds labels to the Dockerfile
LABEL version="1.0"
LABEL description="Dockerfile for building and running the Orbit framework inside Isaac Sim container image."

# Arguments
# Path to Isaac Sim root folder
ARG ISAACSIM_PATH
# Path to the Docker User Home
ARG DOCKER_USER_HOME

# Set environment variables
ENV LANG=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ENV ORBIT_PATH=/workspace/orbit

# Install dependencies and remove cache
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libglib2.0-0 \
    ncurses-term && \
    apt -y autoremove && apt clean autoclean && \
    rm -rf /var/lib/apt/lists/*

# Copy the orbit directory (files to exclude are defined in .dockerignore)
COPY ../ ${ORBIT_PATH}

# Set up a symbolic link between the installed Isaac Sim root folder and _isaac_sim in the orbit directory
RUN ln -sf ${ISAACSIM_PATH} ${ORBIT_PATH}/_isaac_sim

# for singularity usage, have to create the directories that will binded
RUN mkdir -p ${ISAACSIM_PATH}/kit/cache && \
    mkdir -p ${DOCKER_USER_HOME}/.cache/ov && \
    mkdir -p ${DOCKER_USER_HOME}/.cache/pip && \
    mkdir -p ${DOCKER_USER_HOME}/.cache/nvidia/GLCache &&  \
    mkdir -p ${DOCKER_USER_HOME}/.nv/ComputeCache && \
    mkdir -p ${DOCKER_USER_HOME}/.nvidia-omniverse/logs && \
    mkdir -p ${DOCKER_USER_HOME}/.local/share/ov/data && \
    mkdir -p ${DOCKER_USER_HOME}/Documents

# for singularity usage, create NVIDIA binary placeholders
RUN touch /bin/nvidia-smi && \
    touch /bin/nvidia-debugdump && \
    touch /bin/nvidia-persistenced && \
    touch /bin/nvidia-cuda-mps-control && \
    touch /bin/nvidia-cuda-mps-server && \
    touch /etc/localtime && \
    mkdir -p /var/run/nvidia-persistenced && \
    touch /var/run/nvidia-persistenced/socket

# installing Orbit dependencies
RUN ${ORBIT_PATH}/orbit.sh --install --extra
# aliasing orbit.sh and python for convenience
RUN echo "alias orbit=${ORBIT_PATH}/orbit.sh" >> ${HOME}/.bashrc && \
    echo "alias python=${ISAACSIM_PATH}/python.sh" >> ${HOME}/.bashrc && \
    echo "alias python3=${ISAACSIM_PATH}/python.sh" >> ${HOME}/.bashrc && \
    echo "alias pip='${ISAACSIM_PATH}/python.sh -m pip'" >> ${HOME}/.bashrc && \
    echo "alias pip3='${ISAACSIM_PATH}/python.sh -m pip'" >> ${HOME}/.bashrc && \
    echo "alias tensorboard='${ISAACSIM_PATH}/python.sh ${ISAACSIM_PATH}/tensorboard'" >> ${HOME}/.bashrc && \
    echo "export TZ=$(date +%Z)" >> ${HOME}/.bashrc

# make working directory as the orbit directory
# this is the default directory when the container is run
WORKDIR ${ORBIT_PATH}
