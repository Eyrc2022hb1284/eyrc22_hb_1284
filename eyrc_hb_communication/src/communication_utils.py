'''
Author: Debrup
Utility file for the transmitter script
'''

import math



def getWheelVel(vx, vy, w, d, r):
    
        fw=(-d*w+vx)/r
        rw=(-d*w-vx/2-math.sqrt(3)*vy/2)/r
        lw=(-d*w-vx/2+math.sqrt(3)*vy/2)/r

        return fw, lw, rw

# converts wheel velocities into rpm and clips it incase it exceeds 196 RPM
def Vel2RPM(fw_vel, lw_vel, rw_vel):
    fw_rpm=fw_vel*30/math.pi
    lw_rpm=lw_vel*30/math.pi
    rw_rpm=rw_vel*30/math.pi

    if abs(fw_rpm)>196 or abs(lw_rpm)>196 or abs(rw_rpm)>196:
        max_rpm_magnitude=max(abs(fw_rpm), abs(lw_rpm), abs(rw_rpm))

        fw_rpm=float(fw_rpm)/max_rpm_magnitude*196
        lw_rpm=float(lw_rpm)/max_rpm_magnitude*196
        rw_rpm=float(rw_rpm)/max_rpm_magnitude*196
      


    return int(fw_rpm), int(lw_rpm), int(rw_rpm)

#     if(rpm>=0): return min(int(rpm), 200)
#     else: return max(int(rpm), -200)

