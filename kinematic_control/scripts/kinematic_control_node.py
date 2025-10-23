#!/usr/bin/env python3
import rospkg
import json
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from kinematic_control.msg import EulerPose, Jacobian, TransformationMatrix
import time
import numpy as np

from inverse_kinematics import RobotKinematics, UnitQuaternion
# DO NOT ADD ANY OTHER IMPORT!


class InverseKinematicControl:

    def __init__(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path("kinematic_control")
        with open(path + '/config/dh_params.json') as json_file:
            dh_params = json.load(json_file)
        # TODO: start the rospy node
        # TODO: instantiate all the subscribers and publisher necessary
        # TODO: read the server parameters
        # TODO: instantiate RobotKinematics
        # TODO: create all the method necessary for reading and publishing topics

    # TODO: define your own methods

    def control(self):
        while True:
            # TODO: perform inverse dynamics and
            # TODO: Publish the joint velocity and all other topics required by the assignment
            time.sleep(0.05)  # TODO: do not change this value


ikc = InverseKinematicControl()
ikc.control()
