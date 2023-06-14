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

FROM rmf-core-base:humble-0.1.1-rc01

LABEL authors="Emima Jiva <emji@ai2.upv.es>"
LABEL maintainers="Emima Jiva <emji@ai2.upv.es>"

# Non Root user
ARG user_name=ros
ARG user_uid=1000
ARG user_home=/home/$user_name
ARG user_shell=/bin/bash
ARG ros2_dir=$user_home/ros2_ws
ARG ros2_src_dir=$ros2_dir/src
ARG config_dir=$user_home/config
ARG graph_dir=$user_home/graph

ENV DEBIAN_FRONTEND noninteractive

USER $root

RUN true \
	&& pip install paho-mqtt \
	&& true

USER $user_name

#Copy config folder
RUN mkdir -p $config_dir
WORKDIR $config_dir
COPY --chown=$user_name \
	config/ \
	$config_dir/

#Copy graph folder
RUN mkdir -p $graph_dir
WORKDIR $graph_dir
COPY --chown=$user_name \
	graph/ \
	$graph_dir/

#Copy fleet_adapter_mqtt
WORKDIR $ros2_src_dir
COPY --chown=$user_name \
	fleet_adapter_mqtt/fleet_adapter_mqtt \
	$ros2_src_dir/fleet_adapter_mqtt

ENV ROS_BU_PKG "fleet_adapter_mqtt"
ENV ROS_BU_PRG "fleet_adapter"
ENV FLEET_CONFIG_FILE "/home/ros/config/config_rbkairos.yaml"
ENV GRAPH_CONFIG_FILE "/home/ros/graph/0.yaml"

WORKDIR $ros2_dir
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash; colcon build'
ENV ROS_SETUP_FILES "${ros2_dir}/install/setup.bash"

CMD ros2 run ${ROS_BU_PKG} ${ROS_BU_PRG} \
 	-c ${FLEET_CONFIG_FILE} \
 	-n ${GRAPH_CONFIG_FILE}











