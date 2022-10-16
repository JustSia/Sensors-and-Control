def pick(self, obj, close_gripper_to=0.02, retry=1, tolerance=0.01, x_diff_pick=-0.01, z_diff_pick=0.1, x_diff_grasp=-0.01, z_diff_grasp=0.01):
       self.gripper_client.fully_open_gripper()
       angle_tmp = self.angle
       input_retry = retry
       success = False
       while angle_tmp <= self.angle_max and not success:
           radien = (angle_tmp / 2.0) * (pi / 180.0)
           rot_orientation = Quaternion(0.0, sin(radien), 0.0, cos(radien))
           obj_ori = obj.primitive_poses[0].orientation
           obj_quat = [obj_ori.x, obj_ori.y, obj_ori.z, obj_ori.w]
           roll, pitch, yaw = transformations.euler_from_quaternion(obj_quat)
           yaw_quat = transformations.quaternion_from_euler(0.0, 0.0, yaw)
           yaw_orientation = Quaternion(yaw_quat[0], yaw_quat[1], yaw_quat[2], yaw_quat[3])

           gripper_orientation = self.quaternion_multiply(yaw_orientation, rot_orientation)
           first_poseStamped = self.make_poseStamped("base_link", obj.primitive_poses[0], gripper_orientation)
           first_poseStamped.pose.position.x += x_diff_pick
           first_poseStamped.pose.position.z += z_diff_pick
           while retry > 0:
              	move_pose_result = self.move_group.moveToPose(first_poseStamped, "gripper_link", tolerance=tolerance, PLAN_ONLY=True)
                rospy.sleep(1.0)
           	if move_pose_result.error_code.val == MoveItErrorCodes.SUCCESS:
                   success = True
                   break
                else:
                   if move_pose_result.error_code.val == MoveItErrorCodes.NO_IK_SOLUTION:
                       rospy.loginfo("no valid IK found")               
                   rospy.loginfo(move_pose_result.error_code.val)
                retry -= 1
           angle_tmp += self.angle_step
           retry = input_retry
       if retry == 0:
           return False

       success = False
       curr_retry = retry
       while angle_tmp  <= 90 and not success:
           radien = (angle_tmp  / 2) * (pi / 180)
           rot_orientation = Quaternion(0.0, sin(radien), 0.0, cos(radien))
           gripper_orientation = self.quaternion_multiply(yaw_orientation, rot_orientation)
           gripper_pose_stamped = self.make_poseStamped("base_link", obj.primitive_poses[0], gripper_orientation)
           gripper_pose_stamped.pose.position.z += z_diff_grasp
           gripper_pose_stamped.pose.position.x += x_diff_grasp
           while curr_retry > 0:
               move_pose_result = self.move_group.moveToPose(gripper_pose_stamped, "gripper_link", tolerance=tolerance)
               rospy.sleep(1.0)
               if move_pose_result.error_code.val == MoveItErrorCodes.SUCCESS:
                   success = True
                   break
               else:
                   if move_pose_result.error_code.val == MoveItErrorCodes.NO_IK_SOLUTION:
                       rospy.loginfo("no valid IK found")      
                   rospy.loginfo(move_pose_result.error_code.val)
               curr_retry -= 1
           angle_tmp  += self.angle_step
           curr_retry = retry
           rospy.loginfo("closing the gripper")
           self.makeAttach(obj)
           self.gripper_client.close_gripper_to(close_gripper_to)
       if curr_retry == 0:
           return False
       rospy.loginfo("done picking")
       return True
