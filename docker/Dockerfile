FROM rmf-core-humble-base:latest

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











