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

FROM rmf-core-humble-base:latest

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

RUN apt-get update \
	&& apt-get install -q -y \
	 	python3-pip \
	&& pip install paho-mqtt \
	&& apt-get install -q -y \
	&& apt-get clean -q -y \
	&& apt-get autoremove -q -y \
	&& rm -rf /var/lib/apt/lists/* \
	&& echo '%ros ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER $user_name

#Delete rmf_demos because it has other fleet_adapter_template
WORKDIR $ros2_src_dir/
RUN rm -r rmf_demos

#Copy fleet_adapter_template
WORKDIR $ros2_src_dir/fleet_adapter_template
RUN rm -r fleet_adapter_template/
COPY --chown=$user_name \
	fleet_adapter_template/ \
	$ros2_src_dir/fleet_adapter_template

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
	
WORKDIR $ros2_dir
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash; sudo colcon build --packages-select fleet_adapter_template'

#CMD bash -c "/ros_entrypoint.sh ros2 run fleet_adapter_template fleet_adapter -c /home/ros/config/config_rbkairos.yaml  -n /home/ros/graph/0.yaml"
#CMD bash -c "while true; do sleep 600; done;"











