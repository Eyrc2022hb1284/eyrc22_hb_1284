import math

def getWheelVel(vx, vy, w, d, r):
    
        fw=(-d*w+vx)/r
        rw=(-d*w-vx/2-math.sqrt(3)*vy/2)/r
        lw=(-d*w-vx/2+math.sqrt(3)*vy/2)/r

        return fw, lw, rw

def Vel2RPM(fw_vel, lw_vel, rw_vel):
    fw_rpm=fw_vel*30/math.pi
    lw_rpm=lw_vel*30/math.pi
    rw_rpm=rw_vel*30/math.pi

    return int(fw_rpm), int(lw_rpm), int(rw_rpm)

#     if(rpm>=0): return min(int(rpm), 200)
#     else: return max(int(rpm), -200)