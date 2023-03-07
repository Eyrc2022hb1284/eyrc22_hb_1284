'''
Team Id : HB1284
Author List : Debrup
Filename: communication_utils.py
Theme: HoLA Bot
Functions: getWheelVel(), Vel2RPM()
Global Variables: None
'''

import math

'''
Function Name: getWheelVel
Input: vx(Chassis velocity along x axis), vy(chassis velocity along y axis), w(chassis angular velocity), d(distance from center of robot to wheel), r(radius of wheel)
Output: fw(front wheel velocity), lw(left wheel velocity), rw(right wheel velocity)
Logic: 
    This function performs inverse kinematics(calculates wheel velocities from chassis velocity)
Example call: fw, lw, rw = getwheelVel(vx, vy, w, d, r)
'''
def getWheelVel(vx, vy, w, d, r):
    
        fw=(-d*w+vx)/r
        rw=(-d*w-vx/2-math.sqrt(3)*vy/2)/r
        lw=(-d*w-vx/2+math.sqrt(3)*vy/2)/r

        return fw, lw, rw

'''
Function Name: Vel2RPM
Input: fw_vel(front wheel velocity), lw_vel(left wheel velocity), rw_vel(right wheel velocity)
Output: fw_rpm(front wheel RPM), lw_rpm(left wheel RPM), rw_rpm(right wheel RPM)
Logic: 
    This function converts wheel velocities into RPM and clips it incase any of the wheel velocities get maxed out.
Example call: fw_rpm, lw_rpm, rw_rpm = Vel2RPM(fw_vel, lw_vel, rw_vel)
'''
def Vel2RPM(fw_vel, lw_vel, rw_vel):
    fw_rpm=fw_vel*30/math.pi
    lw_rpm=lw_vel*30/math.pi
    rw_rpm=rw_vel*30/math.pi

    # clip velocities incase it exceeds 196RPM (Hardware limit in our case)
    if abs(fw_rpm)>196 or abs(lw_rpm)>196 or abs(rw_rpm)>196:
        max_rpm_magnitude=max(abs(fw_rpm), abs(lw_rpm), abs(rw_rpm))

        fw_rpm=float(fw_rpm)/max_rpm_magnitude*196
        lw_rpm=float(lw_rpm)/max_rpm_magnitude*196
        rw_rpm=float(rw_rpm)/max_rpm_magnitude*196
      


    return int(fw_rpm), int(lw_rpm), int(rw_rpm)

