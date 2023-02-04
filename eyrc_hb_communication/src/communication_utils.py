import math

def getWheelVel(vx, vy, w, d, r):
        fw=(-d*w+vx)/r
        rw=(-d*w-vx/2-math.sqrt(3)*vy/2)/r
        lw=(-d*w-vx/2+math.sqrt(3)*vy/2)/r

        return fw, lw, rw

def Vel2RPM(v):
    return v*30/math.pi