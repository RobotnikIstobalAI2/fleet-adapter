# Description:   fleet-adapter docker image
#
# Company:       Instituto Universitario de Automática e Informática Industrial (AI2)
# Creation Year: 2023
# Author:        Emima Jiva <emji@ai2.upv.es>
#
#
# Copyright (c) 2023, Instituto Universitario de Automática e Informática Industrial (AI2)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the AI2 nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL AI2 BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

ARG base_image="robotnik/ros"
ARG ros_distro="humble"
ARG image_base_version="use-args"
ARG version="use-args"

FROM ${base_image}:${ros_distro}-builder-${image_base_version} as builder

#Copy fleet_adapter_mqtt
COPY --chown=$USER_NAME \
  fleet_adapter_mqtt/  \
  $USER_WORKSPACE/src/fleet_adapter_mqtt

RUN generate_debs.sh

FROM  ${base_image}:${ros_distro}-base-${image_base_version} as base

LABEL maintainer="Guillem Gari <ggari@robotnik.es>" \
      org.opencontainers.image.title="fleet-adapter" \
      org.opencontainers.image.version="${version}" \
      org.opencontainers.image.description="Open-RMF Mqtt fleet adapter" \
      org.opencontainers.image.source="https://github.com/RobotnikIstobalAI2/fleet-adapter" \
      org.opencontainers.image.vendor="Robotnik Automation S.L." \
      org.opencontainers.image.licenses="BSD-3-Clause" \
      org.opencontainers.image.authors="Emima Jiva <emji@ai2.upv.es>, Guillem Gari <ggari@robotnik.es>"
USER root

RUN --mount=type=bind,source=./container/builder/requirements/fleet-adapter.txt,target=/tmp/requirements.txt true \
  && apt-get update \
  && apt-get install -q -y \
  --no-install-recommends \
    $(eval "echo $(cat /tmp/requirements.txt | xargs)") \
  && apt-get clean -q -y \
  && apt-get autoremove -q -y \
  && rm -rf /var/lib/apt/lists/*

RUN --mount=type=bind,source=./container/builder/requirements/python.txt,target=/tmp/requirements.txt true \
	&& pip install -r /tmp/requirements.txt

USER root
RUN --mount=type=bind,from=builder,source=${USER_WORKSPACE}/debs,target=/tmp/debs \
    apt update && \
    apt-get install -y --no-install-recommends \
        /tmp/debs/*.deb \
    && rm -rf /var/lib/apt/lists/*

CMD ["bash", "-c", "/usr/local/bin/ros_launcher.sh"]

USER $USER_NAME

# Set non root user
ARG config_dir=$USER_HOME/config
ARG graph_dir=$USER_HOME/graph

#Set enviorment variables
ENV CONFIG_DIR 	$config_dir
ENV GRAPH_DIR   $graph_dir

#Copy config folder
RUN mkdir -p $CONFIG_DIR
WORKDIR $CONFIG_DIR
COPY --chown=$user_name \
	config/ \
	$CONFIG_DIR/

#Copy graph folder
RUN mkdir -p $GRAPH_DIR
WORKDIR $GRAPH_DIR
COPY --chown=$user_name \
	graph/ \
	$GRAPH_DIR/

ENV FLEET_CONFIG_FILE "${CONFIG_DIR}/config_rbkairos.yaml"
ENV GRAPH_CONFIG_FILE "${GRAPH_DIR}/0.yaml"

ENV STARTUP_TYPE "run"
# package to launch
ENV ROS_BU_PKG "fleet_adapter_mqtt"
# script of program to launch with all its arguments
ENV ROS_BU_LAUNCH "fleet_adapter -c \${FLEET_CONFIG_FILE} -n \${GRAPH_CONFIG_FILE}"
