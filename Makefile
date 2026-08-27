SHELL := /bin/bash

BUILD_FILES := log build install
WORKSPACE_SETUP := install/setup.bash

.PHONY: \
	clean \
	build \
	clean_build \
	slam \
	navigation \
	gui \
	follow \
	save_map \
	teleop \

clean:
	rm -rf $(BUILD_FILES)


build:
	source /opt/ros/jazzy/setup.bash && \
	colcon build --symlink-install


clean_build: clean build

slam:
	source install/setup.bash && \
	ros2 launch avf_15 slam.launch.py  

navigation:
	source install/setup.bash && \
	ros2 launch avf_15 navigation.launch.py  

gui: 
	source install/setup.bash && \
	python3 avf_15/launch/fork_gui_status.py

follow:
	source install/setup.bash && \
	python3 avf_15/launch/follow_route_graph.py

save_map:
	source install/setup.bash && \
	ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'avf_15/maps/my_map11'}}"

teleop:
	source install/setup.bash && \
	ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true --remap cmd_vel:=/tricycle_controller/cmd_vel