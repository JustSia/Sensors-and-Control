#!/usr/bin/env python3

import os
import sys
#from urllib import response
from time import sleep
from python_qt_binding import loadUi
import rospy
import rospkg

import time
from PyQt5.QtWidgets import (
    QApplication,
    QDesktopWidget,
    QMainWindow,
    QWidget,
    QFileDialog,
)
from PyQt5.QtCore import QTimer
from nav_msgs.msg import Odometry
#from gui.teleop import TeleOp
from gui.camera_viz import Cameras
from gui.fetch import Robot

from fetch_robot_sim.msg import RGB_Image_Info
from fetch_robot_sim.msg import Location_3D
from std_msgs.msg import String
from iksolver.srv import calcTraj
from moveit_msgs.msg import RobotTrajectory
from std_srvs.srv import Empty
class GUI(QWidget):
    """
    Class GUI is the main qwidget for user interaction
    Loads required ui files and creates objects of the files listed in the src/gui folder

    GUI buttons are linked the methods in the class to execute the command through the files listed in the src/gui folder
    Also subscribes to topics from iksolver and object_detect to either run the service (iksolver) or display information (object_detect)
    """
    def __init__(self, parent=None):
        super().__init__(parent)

        rospy.init_node("gui", anonymous=True)
        rp = rospkg.RosPack()
        package_path = rp.get_path('fetch_robot_sim')
        ui_file = os.path.join(package_path, "ui", "control.ui")
        loadUi(ui_file, self)

        # Subs
        self.detect_object_request = rospy.Publisher("/object_detect_request", String, queue_size=10)
        self.detect_object_feedback =rospy.Subscriber("/object_info", RGB_Image_Info, self.obj_info_callback)
        self.distance_to_object =rospy.Subscriber("/obj_distance", Location_3D, self.obj_distance)
        self.distance =rospy.Subscriber("/distance", Location_3D, self.distance)
        self.odom_sub = rospy.Subscriber("/odom",Odometry,self.odom)
        self.obj_distance = Location_3D()

        self.fetch = Robot("Fetchy")

        self.cameras = Cameras()
        self.teleop = TeleOp(self, self.fetch)

        self.layout.addWidget(self.cameras)
        self.layout.addWidget(self.teleop)

        self.Grasp_Prep_Button.clicked.connect(self.grasp_prep)
        self.Arm_Reset_Button.clicked.connect(self.reset_robo_arm)
        self.Kinematics_Button.clicked.connect(self.kinematics_finder)
        self.Grasp_Button.clicked.connect(self.grasp_obj)
        self.bin_button.clicked.connect(self.bin_obj)
        self.pick_button.clicked.connect(self.pick_obj)
        self.reset_button.clicked.connect(self.reset)

        self.show()

        self.timer = QTimer()
        self.timer.setInterval(1000) # 1 second
        self.timer.timeout.connect(self.check_obj_request)
        self.timer.start()
    def reset(self):

        rospy.wait_for_service('/gazebo/reset_world')
        reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)

        reset_world()

    def odom(self,data):
        self.robot_pose_label.setText("x: " + str("{:.3f}".format(data.pose.pose.position.x)) + "  y: " + str("{:.3f}".format(data.pose.pose.position.x)) + "  theta: " + str("{:.3f}".format(data.pose.pose.orientation.w)))
    def distance(self,data):
            self.obj_distance = data
            self.obj_detect_info_2_label_2.setText(str("{:.2f}".format(data.x)) + ",  " + str("{:.2f}".format(data.y)) + ",  "+ str("{:.2f}".format(data.z)))

    def obj_distance(self,data):
        self.obj_detect_info_2_label_4.setText(str("{:.2f}".format(data.x)) + ",  " + str("{:.2f}".format(data.y)) + ",  "+ str("{:.2f}".format(data.z)))

    def check_obj_request(self):
        self.detect_object_request.publish(self.comboBox.currentText())

    def grasp_prep(self):
        self.status_label.setText("Preparing Arm for Grasp")
        self.fetch.update_torso(50)
        self.fetch.preGrasp()
        self.fetch.update_gripper(100)
        self.fetch.update_torso(0)
        self.teleop.update_sliders()
        self.status_label.setText("Status: Ready")

    def reset_robo_arm(self):
        self.fetch.update_torso(50)
        self.fetch.preGrasp()
        self.fetch.reset_arm()
        self.fetch.update_torso(0)
        self.teleop.update_sliders()
    def pick_obj(self):
        self.status_label.setText("Status: Picking and Placing Object")
        self.fetch.pick_obj()
        self.teleop.update_sliders()
        self.status_label.setText("Status: Ready")
    def bin_obj(self):
        self.status_label.setText("Throwing Object")
        self.fetch.binObj()
        self.teleop.update_sliders()
        self.status_label.setText("Status: Ready")

    def kinematics_finder(self):
        
        send_coor = Location_3D()
        if (self.checkbox.isChecked()):
            # commit to over ride
            self.status_label.setText("Status: Over riding desired enf effector pose")
            send_coor.x = float(self.x_label.text())
            send_coor.y = float(self.y_label.text())
            send_coor.z = float(self.z_label.text())
        else:
            self.status_label.setText("Loading x,y,z of object")
            send_coor.x = self.obj_distance.x - 0.17
            send_coor.y = self.obj_distance.y
            send_coor.z = self.obj_distance.z
        self.status_label.setText("Setting pre IK Solver Pose")
        self.fetch.preGrasp()
        self.fetch.update_gripper(100)
        self.status_label.setText("Status: Calculating IK")
        self.gripper_pose_info.setText("searching ...")
        print("IK service")
        sleep(2)
        rospy.wait_for_service('calc_traj')
        
        try:
            trajectory = rospy.ServiceProxy('calc_traj', calcTraj)

            response = trajectory(send_coor)
            
            print("success")
            # joints = RobotTrajectory()
            # joints = response.joint_trajectory.points
            print(str(response.traj))
            print(str(response.traj.joint_trajectory.points[-1].positions))
            self.gripper_pose_info.setText("Found, iksolver successful")
            self.status_label.setText("Status: Ready to Grasp")
            # self.gripper_pose_info.setText(str(response.traj.joint_trajectory.points[-1].positions))
            self.fetch.updateJoints(response.traj.joint_trajectory.points[-1].positions)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            self.gripper_pose_info.setText("Not found, iksolver unsuccessful")


    def grasp_obj(self):
        self.status_label.setText("Status: Grasping Object")
        self.fetch.update_gripper(100)
        self.fetch.update_torso(50)
        self.fetch.grasp_seq()
        self.fetch.update_torso(0)
        sleep(2)
        self.fetch.update_gripper(0)
        self.teleop.update_sliders()
        self.status_label.setText("Status: Finished")

    def obj_info_callback(self, data):
        self.obj_detect_label.setText("Obj Detected:  " + str(data.objectName))
        self.obj_detect_info_label.setText(
            str("Image Frame (x,y): ") +
            str("{:.1f}".format(data.x))
            + ", "
            + str("{:.1f}".format(data.y))
        )


def main(args=None):

    app = QApplication(sys.argv)
    window = GUI()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
