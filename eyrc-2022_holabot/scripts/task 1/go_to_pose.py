#!/usr/bin/env python3

from tkinter.tix import Tree
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion

class goToPose:
    def __init__(self):

        rospy.init_node('go_to_pose')

        self.odom_sub=rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_pub=rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.x=0
        self.y=0
        self.theta=0

        # goals
        self.x_goals=[1, -1, -1, 1, 0]
        self.y_goals=[1, 1, -1, -1, 0]
        self.theta_goals=[math.pi/4, 3*math.pi/4, -3*math.pi/4, -math.pi/4, 0]

        # pid params
        self.params_linear={'Kp':1.0, 'Ki':0, 'Kd':0}
        self.params_ang={'Kp':0.5, 'Ki':0, 'Kd':0}

        self.msg=Twist()

        while not rospy.is_shutdown():
            for i in range(len(self.x_goals)):
                goal_x=self.x_goals[i]
                goal_y=self.y_goals[i]
                goal_theta=self.theta_goals[i]

                v_x=self.getLinearVel(goal_x,  self.params_linear)
                v_y=self.geLinearVel(goal_y, self.params_linear, x=False)
                ang_vel=self.getAngVel(goal_theta, self.params_ang)

                self.msg.linear.x=v_x
                self.msg.linear.y=v_y
                self.msg.angular.z=ang_vel

                self.cmd_pub.publish(self.msg)

    def odom_callback(self, data):
        x  = data.pose.pose.orientation.x
        y  = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w,

        self.x = data.pose.pose.position.x # x coordinate of bot
        self.y = data.pose.pose.position.y # y coordinate of bot
        self.theta = euler_from_quaternion([x,y,z,w])[2] #real time orientation of bot

    def pid(self, error, const):
        prop = error
        self.intg = error + self.intg
        diff = error - self.last_error
        balance = const['KP'] * prop + const['KI'] * self.intg + const['KD'] * diff
        self.last_error = error
        return balance

    def getAngVel(self, final_orientation, const):
        error = final_orientation - self.theta
        abs_angle_diff = abs(abs(final_orientation)-abs(self.theta))
        ang_vel=0

        if abs_angle_diff > 0.1:
            if error > 3.14:
                ang_vel = self.pid((error-6.28), const)
            elif error < -3.14:
                ang_vel = self.pid((error+6.28), const)
            else:
                ang_vel = self.pid(error, const)

            # if ang_vel > 0:
            #     ang_vel = angular_vel
            # else:
                # ang_vel = -angular_vel
        else:
            self.stop()

        return ang_vel

    def getLinearVel(self, target_point, const, x=True):
        error=0
        if x:
            error=target_point-self.x
        else:
            error=target_point-self.y

        linear_vel=self.pid(error, const)

        return linear_vel

        

    def stop(self):
        #Function to halt the bot wherever its called
        self.msg.linear.x = 0
        self.msg.linear.y=0
        self.msg.angular.z = 0

        self.cmd_pub.publish(self.msg)


if __name__=='__main__':
    gt=goToPose()

    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)