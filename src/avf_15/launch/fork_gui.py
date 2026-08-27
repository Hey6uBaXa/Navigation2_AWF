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

class ForkControl(Node):

    def __init__(self):
        super().__init__('fork_gui')

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

    def send(self, forks, mast):
        msg = Float64MultiArray()
        msg.data = [forks, mast]
        self.pub.publish(msg)

    def send_vel(self, linear, angular):
        msg = TwistStamped()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.cmd_pub.publish(msg)   

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

        layout.addWidget(self.slider1)
        layout.addWidget(self.btn_up)
        layout.addWidget(self.btn_down)   

        self.setLayout(layout)
        self.setFocusPolicy(Qt.StrongFocus)

    def publish(self):
        forks = self.slider1.value() / 100
        mast = forks

        self.ros_node.send(forks, mast)

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