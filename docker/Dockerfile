FROM osrf/ros:galactic-desktop

# Non Root user
ARG user_name=ros
ARG user_uid=1000
ARG user_home=/home/$user_name
ARG user_shell=/bin/bash
ARG ros1_dir=$user_home/ros1_ws
ARG ros1_src_dir=$ros1_dir/src
ARG ros2_dir=$user_home/ros2_ws
ARG ros2_src_dir=$ros2_dir/src
ARG config_dir=$user_home/config
ARG graph_dir=$user_home/graph

RUN useradd -m -d $user_home -s $user_shell -u $user_uid $user_name \
	&& echo "PS1='\[\e]0;\u@\h: \w\a\]${debian_chroot:+($debian_chroot)}\[\033[01;33m\]\u\[\033[00m\]@\[\033[01;31m\]\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> ~/.bashrc

#Key for ROS1 NOETIC
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update \
	&& apt-get install -q -y \
		wget \
		apt-utils \
		sudo \
		git wget qt5-default \
		python3-rosdep \
		python3-vcstool \
		python3-colcon-common-extensions \
		curl \
		lsb-release \
		ros-noetic-desktop-full \
		python3-catkin-tools \
		ros-galactic-rmf-demos \
		python3-pip \
		ros-galactic-rmf-fleet-adapter-python \
	&& pip install flask-socketio \
	&& pip install nudged \
	&& apt-get clean -q -y \
	&& apt-get autoremove -q -y \
	&& rm -rf /var/lib/apt/lists/* \
	&& echo '%ros ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

RUN apt-get update \
	&& apt-get install -q -y \
		ros-noetic-move-base-msgs \
	&& apt-get clean -q -y \
	&& apt-get autoremove -q -y \
	&& rm -rf /var/lib/apt/lists/* \
	&& echo '%ros ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
				
USER $user_name

#Source ros2
RUN /bin/bash -c 'source /opt/ros/galactic/setup.bash'
#Copy adapter_template 
RUN mkdir -p $ros2_src_dir
WORKDIR $ros2_src_dir
RUN git clone https://github.com/open-rmf/fleet_adapter_template
#ROS2 catkin install and build
WORKDIR $ros2_dir
RUN  sudo apt-get update \ 
	&& sudo rm /etc/ros/rosdep/sources.list.d/20-default.list \
	&& sudo rosdep init -yr \
	&& rosdep update -y\
	&& rosdep install --from-paths src --ignore-src --rosdistro galactic -yr 

#Copy fleet_adapter_template
WORKDIR $ros2_src_dir/fleet_adapter_template/fleet_adapter_template
RUN rm -r fleet_adapter_template
COPY --chown=$user_name \
	fleet_adapter_template/ \
	$ros2_src_dir/fleet_adapter_template/fleet_adapter_template/fleet_adapter_template/

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
RUN /bin/bash -c 'source /opt/ros/galactic/setup.bash; colcon build'	

USER root
COPY docker/ros_entrypoint.sh /

WORKDIR /
RUN sudo chmod +x ros_entrypoint.sh
CMD bash -c "/ros_entrypoint.sh ros2 run fleet_adapter_template fleet_adapter -c /home/ros/config/config_rbvogui.yaml  -n /home/ros/graph/0.yaml"

#CMD bash -c "while true; do sleep 600; done;"











