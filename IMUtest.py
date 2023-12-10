import motion
import rclpy
import numpy as np
from rclpy.node import Node
import threading
import time

rclpy.init(args=None)
robot = motion.Motion()



try:
    while True:
        print(robot.robot.global_deg)
        rclpy.spin_once(robot.robot)
        time.sleep(0.01)

except KeyboardInterrupt:
    rclpy.shutdown()