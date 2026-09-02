SHELL := /bin/bash

BUILD_FILES := log build install
WORKSPACE_SETUP := install/setup.bash

# Значения по умолчанию. Можно переопределять из командной строки.
ROBOT_NAME ?= robot01
SPAWN_X ?= 1.0
SPAWN_Y ?= 1.0
USE_RVIZ ?= false

# Полный ROS namespace
ROBOT_NS := /$(ROBOT_NAME)

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
	world \
	kill

kill:
	pkill -INT -f "ros2 launch" && \
	pkill -INT -f "gz sim" && \
	pkill -INT -f "ekf_node" && \
	pkill -INT -f "rviz2" && \
	ros2 daemon stop && \
	fastdds shm clean

clean:
	rm -rf $(BUILD_FILES)

build:
	source /opt/ros/jazzy/setup.bash && \
	colcon build --symlink-install

clean_build: clean build

world:
	source $(WORKSPACE_SETUP) && \
	ros2 launch avf_15 world.launch.py use_rviz:=false

slam:
	source $(WORKSPACE_SETUP) && \
	ros2 launch avf_15 slam.launch.py \
		robot_name:=$(ROBOT_NAME) \
		spawn_x:=$(SPAWN_X) \
		spawn_y:=$(SPAWN_Y) \
		use_rviz:=$(USE_RVIZ)

navigation:
	source $(WORKSPACE_SETUP) && \
	ros2 launch avf_15 display.launch.py \
		robot_name:=$(ROBOT_NAME) \
		spawn_x:=$(SPAWN_X) \
		spawn_y:=$(SPAWN_Y) \
		use_rviz:=$(USE_RVIZ)



gui:
	source $(WORKSPACE_SETUP) && \
	python3 avf_15/launch/fork_gui_status.py \
		--ros-args \
		-r __ns:=$(ROBOT_NS)

follow:
	source $(WORKSPACE_SETUP) && \
	python3 avf_15/launch/follow_route_graph.py \
		--ros-args \
		-r __ns:=$(ROBOT_NS)

save_map:
	source $(WORKSPACE_SETUP) && \
	ros2 service call \
		$(ROBOT_NS)/slam_toolbox/save_map \
		slam_toolbox/srv/SaveMap \
		"{name: {data: 'avf_15/maps/$(ROBOT_NAME)_map'}}"

teleop:
	source $(WORKSPACE_SETUP) && \
	ros2 run teleop_twist_keyboard teleop_twist_keyboard \
		--ros-args \
		-p stamped:=true \
		-r __ns:=$(ROBOT_NS) \
		-r cmd_vel:=tricycle_controller/cmd_vel