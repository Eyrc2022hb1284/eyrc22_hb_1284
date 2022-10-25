#!/usr/bin/env python3

'''
Author: [Debrup, Sachin]
'''

import rospy
from geometry_msgs.msg import PoseArray, Pose2D, Wrench
import math
from tf.transformations import euler_from_quaternion

class goToPose:
    def __init__(self):
        # initalise node
        rospy.init_node('go_to_pose')

        # pose params
        self.x=0
        self.y=0
        self.theta=0

        # bot params
        self.d=0.1
        self.r=0.03

        # threshold params
        self.linear_thresh=0.04
        self.ang_thresh=float(math.pi)/181

        # goals
        self.x_goals=[50,350,50,250,250]
        self.y_goals=[350,50,50,350,50]
        self.theta_goals=[0, 0, 0, 0, 0]

        self.rate=rospy.Rate(10)

        # subscriber/publisher
        # self.test_sub=rospy.Subscriber('task2_goals', PoseArray, self.task2_goals_Cb)
        self.odom_sub=rospy.Subscriber('/detected_aruco', Pose2D, self.aruco_feedback_Cb)
        
        self.right_wheel_pub = rospy.Publisher('/right_wheel_force', Wrench, queue_size=10)
        self.front_wheel_pub = rospy.Publisher('/front_wheel_force', Wrench, queue_size=10)
        self.left_wheel_pub = rospy.Publisher('/left_wheel_force', Wrench, queue_size=10)

        # pid params
        self.params_linear={'Kp':1, 'Ki':0, 'Kd':0}
        self.params_ang={'Kp':0.5, 'Ki':0, 'Kd':0}
        self.intg=0
        self.last_error=0

        # ROS msg
        self.rw_msg=Wrench()
        self.lw_msg=Wrench()
        self.fw_msg=Wrench()

        # control loop
        while not rospy.is_shutdown():
            # if self.prev==self.x_goals: continue
            
            for i in range(len(self.x_goals)):
                goal_x=self.x_goals[i]
                goal_y=self.y_goals[i]
                goal_theta=self.theta_goals[i]

                while True:
                    # error calculation
                    angle_error=goal_theta-self.theta
                    error_x=(goal_x-self.x)*math.cos(self.theta)+(goal_y-self.y)*math.sin(self.theta)
                    error_y=-(goal_x-self.x)*math.sin(self.theta)+(goal_y-self.y)*math.cos(self.theta)

                    # velocity calculation
                    v_x, v_y=self.getLinearVel(error_x,  error_y, self.params_linear)
                    ang_vel=self.getAngVel(angle_error, self.params_ang)

                    v1, v2, v3=self.inverse_kinematics(v_x, v_y, ang_vel)
                    
                    print(v1, v2, v3)

                    self.fw_msg.torque.z=v1
                    self.rw_msg.torque.z=v2
                    self.lw_msg.torque.z=v3
                    
                    self.front_wheel_pub.publish(self.fw_msg)
                    self.right_wheel_pub.publish(self.rw_msg)
                    self.left_wheel_pub.publish(self.lw_msg)

                    self.rate.sleep()

                    # move to next pose when reached target pose
                    if abs(angle_error)<=self.ang_thresh and abs(error_x)<=self.linear_thresh and abs(error_y)<=self.linear_thresh:
                        break
                    
                rospy.sleep(1)

            # self.prev=self.x_goals

    # def task2_goals_Cb(self, msg):
    #     self.x_goals.clear()
    #     self.y_goals.clear()
    #     self.theta_goals.clear()

    #     for waypoint_pose in msg.poses:
    #         self.x_goals.append(waypoint_pose.position.x)
    #         self.y_goals.append(waypoint_pose.position.y)

    #         orientation_q = waypoint_pose.orientation
    #         orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    #         theta_goal = euler_from_quaternion (orientation_list)[2]
    #         self.theta_goals.append(theta_goal)

    def aruco_feedback_Cb(self, msg):
        self.x=msg.x
        self.y=msg.y
        self.theta=msg.theta

    def inverse_kinematics(self, vx, vy, w):
        u1=(-self.d*w+vx)/self.r
        u2=(-self.d*w-vx/2-math.sqrt(3)*vy/2)/self.r
        u3=(-self.d*w-vx/2+math.sqrt(3)*vy/2)/self.r

        return u1, u2, u3

    def pid(self, error, const):
        prop = error
        self.intg = error + self.intg
        diff = error - self.last_error
        balance = const['Kp'] * prop + const['Ki'] * self.intg + const['Kd'] * diff
        self.last_error = error
        return balance

    # angular pid function
    def getAngVel(self, error, const):
        ang_vel=0

        if abs(error) > self.ang_thresh:
            if error > 3.14:
                ang_vel = self.pid((error-6.28), const)
            elif error < -3.14:
                ang_vel = self.pid((error+6.28), const)
            else:
                ang_vel = self.pid(error, const)

            if ang_vel<0: ang_vel=-1
            else: ang_vel=1

        else:
            self.stop()

        return ang_vel

    # linear pid function
    def getLinearVel(self, error_x,  error_y, const, x=True):
        v_x=0
        v_y=0
        
        if abs(error_x)>self.linear_thresh or abs(error_y)>self.linear_thresh:
            v_x=self.pid(error_x, const)
            v_y=self.pid(error_y, const)
        else:
            self.stop()

        return v_x, v_y

    # bot halt function
    def stop(self):
        self.fw_msg.force.x=0
        self.rw_msg.force.x=0
        self.lw_msg.force.x=0
        
        self.front_wheel_pub.publish(self.fw_msg)
        self.right_wheel_pub.publish(self.rw_msg)
        self.left_wheel_pub.publish(self.lw_msg)

if __name__=='__main__':
    gt=goToPose()

    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)