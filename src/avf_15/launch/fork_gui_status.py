import sys

from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QLabel,
    QSlider,
    QPushButton
)

from PyQt5.QtCore import Qt, QTimer

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState


class ForkControl(Node):

    def __init__(self):
        super().__init__('fork_gui')

        self.target_positions = {
            'J_main_villas': 0.0,
            'J_second_villas': 0.0,
        }

        self.actual_positions = {
            'J_main_villas': None,
            'J_second_villas': None,
        }

        self.actual_velocities = {
            'J_main_villas': None,
            'J_second_villas': None,
        }

        self.pub = self.create_publisher(
            Float64MultiArray,
            '/fork_controller/commands',
            10
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/tricycle_controller/cmd_vel',
            10
        )


        # self.joint_state_sub = self.create_subscription(
        #     JointState,
        #     '/joint_states',
        #     self.joint_state_callback,
        #     10
        # )

        self.sensor_pub = self.create_publisher(
            JointState,
            '/fork_sensor_sim',
            10
        )

        self.sensor_state_sub = self.create_subscription(
            JointState,
            '/fork_sensor_sim',
            self.sensor_state_callback,
            10
        )

    def send(self, forks, mast):
        self.target_positions['J_main_villas'] = forks
        self.target_positions['J_second_villas'] = mast

        msg = Float64MultiArray()
        msg.data = [forks, mast]
        self.pub.publish(msg)

    def joint_state_callback(self, msg):
        for index, joint_name in enumerate(msg.name):

            if joint_name not in self.actual_positions:
                continue

            if index < len(msg.position):
                self.actual_positions[joint_name] = msg.position[index]

            if index < len(msg.velocity):
                self.actual_velocities[joint_name] = msg.velocity[index]

    def send_vel(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.cmd_pub.publish(msg)

    def send_sensor_feedback(self, main_position, second_position):
        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = [
            'J_main_villas',
            'J_second_villas'
        ]

        msg.position = [
            float(main_position),
            float(second_position)
        ]

        msg.velocity = [0.0, 0.0]

        self.sensor_pub.publish(msg)

    def sensor_state_callback(self, msg):
        for index, joint_name in enumerate(msg.name):
            if joint_name not in self.actual_positions:
                continue

            if index < len(msg.position):
                self.actual_positions[joint_name] = msg.position[index]

            if index < len(msg.velocity):
                self.actual_velocities[joint_name] = msg.velocity[index]

class Window(QWidget):

    def __init__(self, ros_node):
        super().__init__()

        self.ros_node = ros_node
        self.pressed_keys = set()
        layout = QVBoxLayout()

        layout.addWidget(QLabel("Main Villas"))

        self.slider1 = QSlider(Qt.Horizontal)
        self.slider1.setRange(-3, 100)
        self.slider1.valueChanged.connect(self.publish)

        self.btn_up = QPushButton("Up")
        self.btn_up.clicked.connect(self.move_up)

        self.btn_down = QPushButton("Down")
        self.btn_down.clicked.connect(self.move_down)

        self.target_label = QLabel("Target: 0.000 m")
        self.actual_main_label = QLabel("J_main_villas: --")
        self.actual_second_label = QLabel("J_second_villas: --")
        self.error_main_label = QLabel("Main error: --")
        self.error_second_label = QLabel("Second error: --")

        layout.addWidget(self.target_label)
        layout.addWidget(self.actual_main_label)
        layout.addWidget(self.actual_second_label)
        layout.addWidget(self.error_main_label)
        layout.addWidget(self.error_second_label)


        layout.addWidget(self.slider1)
        layout.addWidget(self.btn_up)
        layout.addWidget(self.btn_down)   

        self.feedback_timer = QTimer(self)
        self.feedback_timer.timeout.connect(self.update_feedback)
        self.feedback_timer.start(100)

        layout.addWidget(QLabel("Имитация датчика J_main_villas"))

        self.main_sensor_slider = QSlider(Qt.Horizontal)
        self.main_sensor_slider.setRange(-3, 100)
        self.main_sensor_slider.setValue(0)
        self.main_sensor_slider.setSingleStep(1)

        layout.addWidget(self.main_sensor_slider)

        self.main_sensor_label = QLabel("Sensor main: 0.00 m")
        layout.addWidget(self.main_sensor_label)


        layout.addWidget(QLabel("Имитация датчика J_second_villas"))

        self.second_sensor_slider = QSlider(Qt.Horizontal)
        self.second_sensor_slider.setRange(-3, 100)
        self.second_sensor_slider.setValue(0)
        self.second_sensor_slider.setSingleStep(1)

        layout.addWidget(self.second_sensor_slider)

        self.second_sensor_label = QLabel("Sensor second: 0.00 m")
        layout.addWidget(self.second_sensor_label)

        self.main_sensor_slider.valueChanged.connect(
            self.publish_sensor_feedback
        )

        self.second_sensor_slider.valueChanged.connect(
            self.publish_sensor_feedback
        )

        self.sensor_timer = QTimer(self)
        self.sensor_timer.timeout.connect(self.publish_sensor_feedback)
        self.sensor_timer.start(100)  # 10 Гц
        
        self.setLayout(layout)
        self.setFocusPolicy(Qt.StrongFocus)

    def update_feedback(self):
        target_main = self.ros_node.target_positions['J_main_villas']
        target_second = self.ros_node.target_positions['J_second_villas']

        actual_main = self.ros_node.actual_positions['J_main_villas']
        actual_second = self.ros_node.actual_positions['J_second_villas']

        self.target_label.setText(
            f"Target: {target_main:.3f} m"
        )

        if actual_main is not None:
            main_error = target_main - actual_main

            self.actual_main_label.setText(
                f"J_main_villas: {actual_main:.3f} m"
            )
            self.error_main_label.setText(
                f"Main error: {main_error:+.3f} m"
            )
        else:
            self.actual_main_label.setText("J_main_villas: no feedback")
            self.error_main_label.setText("Main error: --")

        if actual_second is not None:
            second_error = target_second - actual_second

            self.actual_second_label.setText(
                f"J_second_villas: {actual_second:.3f} m"
            )
            self.error_second_label.setText(
                f"Second error: {second_error:+.3f} m"
            )
        else:
            self.actual_second_label.setText(
                "J_second_villas: no feedback"
            )
            self.error_second_label.setText("Second error: --")

    def publish_sensor_feedback(self):
        main_position = self.main_sensor_slider.value() / 100.0
        second_position = self.second_sensor_slider.value() / 100.0

        self.main_sensor_label.setText(
            f"Sensor main: {main_position:.2f} m"
        )

        self.second_sensor_label.setText(
            f"Sensor second: {second_position:.2f} m"
        )

        self.ros_node.send_sensor_feedback(
            main_position,
            second_position
        )
        
    def publish(self):
        forks = self.slider1.value() / 100
        mast = forks

        self.ros_node.send(forks, mast)
        self.target_label.setText(f"Target: {forks:.3f} m")

    def move_up(self):
        self.slider1.setValue(100)

    def move_down(self):
        self.slider1.setValue(-3)

   
    def keyPressEvent(self, event):

        key = event.text().lower()

        if key:
            self.pressed_keys.add(key)

        self.update_motion()


    def keyReleaseEvent(self, event):

        key = event.text().lower()

        if key in self.pressed_keys:
            self.pressed_keys.remove(key)

        self.update_motion()

    def update_motion(self):

        max_line_vel = 0.5
        min_line_vel = -0.5

        max_line_ang = 0.5
        min_line_ang = -0.5

        if "w" in self.pressed_keys:
            self.ros_node.send_vel(max_line_vel, 0.0)
        elif "x" in self.pressed_keys:
            self.ros_node.send_vel(min_line_vel, 0.0)
        elif "a" in self.pressed_keys:
            self.ros_node.send_vel(max_line_vel/2, max_line_ang)
        elif "d" in self.pressed_keys:
            self.ros_node.send_vel(max_line_vel/2, min_line_ang)
        elif "q" in self.pressed_keys:
            self.ros_node.send_vel(max_line_vel, max_line_ang/2)
        elif "e" in self.pressed_keys:
            self.ros_node.send_vel(max_line_vel, min_line_ang/2)
        elif "z" in self.pressed_keys:
            self.ros_node.send_vel(min_line_vel/2, max_line_ang/2)
        elif "c" in self.pressed_keys:
            self.ros_node.send_vel(min_line_vel/2, min_line_ang)
        elif "s" in self.pressed_keys:
            self.ros_node.send_vel(0.0, 0.0)

def main():
    rclpy.init()

    node = ForkControl()

    app = QApplication(sys.argv)

    window = Window(node)
    window.show()

    timer = QTimer()
    timer.timeout.connect(
        lambda: rclpy.spin_once(node, timeout_sec=0)
    )
    timer.start(50)

    app.exec()
    rclpy.shutdown()


if __name__ == "__main__":
    main()