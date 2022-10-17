#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion

class goToPose:
    def __init__(self):

        rospy.init_node('go_to_pose')

        self.x=0
        self.y=0
        self.theta=0

        self.linear_thresh=0.01
        self.ang_thresh=0.04

        # goals
        self.x_goals=[1, -1, -1, 1, 0]
        self.y_goals=[1, 1, -1, -1, 0]
        self.theta_goals=[math.pi/4, 3*math.pi/4, -3*math.pi/4, -math.pi/4, 0]

        self.odom_sub=rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # self.test_sub=rospy.Subscriber('task1_goals', PoseArray, self.task1_goals_Cb)
        self.cmd_pub=rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # pid params
        self.params_linear={'Kp':2, 'Ki':0, 'Kd':0}
        self.params_ang={'Kp':0.5, 'Ki':0, 'Kd':0}

        self.intg=0
        self.last_error=0

        self.msg=Twist()
        self.rate=rospy.Rate(10)

        for i in range(len(self.x_goals)):
            goal_x=self.x_goals[i]
            goal_y=self.y_goals[i]
            goal_theta=self.theta_goals[i]

            while(True):
                angle_error=goal_theta-self.theta

                error_x=(goal_x-self.x)*math.cos(self.theta)+(goal_y-self.y)*math.sin(self.theta)
                error_y=-(goal_x-self.x)*math.sin(self.theta)+(goal_y-self.y)*math.cos(self.theta)

                v_x, v_y=self.getLinearVel(error_x,  error_y, self.params_linear)
                ang_vel=self.getAngVel(angle_error, self.params_ang)

                self.msg.linear.x=v_x
                self.msg.linear.y=v_y
                self.msg.angular.z=ang_vel

                self.cmd_pub.publish(self.msg)
                self.rate.sleep()

                # if bot reached goal, move to next goal
                if abs(angle_error)<=self.ang_thresh and abs(error_x)<=self.linear_thresh and abs(error_y)<=self.linear_thresh :
                    self.stop(x=True, y=True, z=True)
                    rospy.sleep(1)
                    break

        rospy.loginfo("Task finished")
        rospy.signal_shutdown("Task finished")

    def odom_callback(self, data):
        x  = data.pose.pose.orientation.x
        y  = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w

        self.x = data.pose.pose.position.x # x coordinate of bot
        self.y = data.pose.pose.position.y # y coordinate of bot
        _, _, self.theta = euler_from_quaternion([x,y,z,w]) #real time orientation of bot

    def task1_goals_Cb(self, msg):
        self.x_goals.clear()
        self.y_goals.clear()
        self.theta_goals.clear()

        for waypoint_pose in msg.poses:
            self.x_goals.append(waypoint_pose.position.x)
            self.y_goals.append(waypoint_pose.position.y)

            orientation_q = waypoint_pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            theta_goal = euler_from_quaternion (orientation_list)[2]
            self.theta_goals.append(theta_goal)

    def pid(self, error, const):
        prop = error
        self.intg = error + self.intg
        diff = error - self.last_error
        balance = const['Kp'] * prop + const['Ki'] * self.intg + const['Kd'] * diff
        self.last_error = error
        return balance

    def getAngVel(self, error, const):
        ang_vel=0

        if abs(error) > self.ang_thresh:
            if error > 3.14:
                ang_vel = self.pid((error-6.28), const)
            elif error < -3.14:
                ang_vel = self.pid((error+6.28), const)
            else:
                ang_vel = self.pid(error, const)

            if ang_vel<0: ang_vel=-2
            else: ang_vel=2

            print("Aligning: ", ang_vel)

        else:
            self.stop(z=True)
            print("Stopped")

        print("Orientation: ", self.theta)

        return ang_vel

    def getLinearVel(self, error_x,  error_y, const, x=True):
        v_x=0
        v_y=0
        
        if abs(error_x)>self.linear_thresh or abs(error_y)>self.linear_thresh:
            v_x=self.pid(error_x, const)
            v_y=self.pid(error_y, const)
            print("vel_x: {} vel_y: {}".format(v_x, v_y))
        else:
            self.stop(x=True, y=True)
            print("Stopped")

        return v_x, v_y

    def stop(self, x=False, y=False, z=False):
        #Function to halt the bot wherever its called
        if x: self.msg.linear.x = 0
        if y: self.msg.linear.y=0
        if z: self.msg.angular.z = 0

        self.cmd_pub.publish(self.msg)


if __name__=='__main__':
    gt=goToPose()

    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)