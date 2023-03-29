#!/usr/bin/env python3

'''
Author: Debrup
Purpose: This script takes input from User and accordingly initiates or ends task.
0-Initiate Task
1-End task
Any other button pressed-Log warning and continue to publish previous data
'''
import rospy
from std_msgs.msg import Int32
from pynput.keyboard import Key, Listener

class contolHola:
    def __init__(self):
        rospy.init_node('control_hola')
        print('''
        Press 'up' to start task
        Press 'down' to end task''')

        self.task_status=1
        self.pub=rospy.Publisher('/taskStatus', Int32, queue_size=10)

        with Listener(on_press = self.publishTaskStatus) as listener:  
            listener.join()

        rospy.signal_shutdown("User command")

    def publishTaskStatus(self, key):
        
        if(key==Key.up): 
            self.task_status=0
        elif(key==Key.down): 
            self.task_status=1
        else:
            print('Invalid input, only allowed buttons--UP and DOWN')
    
        if key == Key.delete:
            # end task
            self.task_status=1
            # Stop listener
            return False
        
        self.pub.publish(self.task_status)
        rospy.loginfo('Task status: {}'.format(self.task_status))
    
if __name__=='__main__':
    ch=contolHola()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
