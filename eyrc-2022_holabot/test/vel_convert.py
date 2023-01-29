import math

print("Enter Chassis velocity: ")

vx=float(input("Enter vel along x axis(m/s): "))
vy=float(input("Enter vel along y axis(m/s): "))
w=float(input("Enter angular velocity(m/s): "))

# distance between center and wheel(m)
d=0.105
# radius of wheel(m)
r=0.029
# steps to make 360 deg
steps=200

fw_vel=(-d*w+vx)/r
rw_vel=(-d*w-(vx/2)-(math.sqrt(3)*vy/2))/r
lw_vel=(-d*w-(vx/2)+(math.sqrt(3)*vy/2))/r

print("Front Wheel velocity(m/s): {}".format(fw_vel))
print("Right Wheel velocity(m/s): {}".format(lw_vel))
print("Left Wheel velocity(m/s): {}".format(rw_vel))

def getStepsPersecond(wheel_vel):
    return (wheel_vel/(2*3.14*r))*steps

def getRPM(wheel_vel):
    return wheel_vel*60/(2*3.14*r)

fw_rpm=getRPM(fw_vel)
lw_rpm=getRPM(lw_vel)
rw_rpm=getRPM(rw_vel)

print("\nFront Wheel RPM: {}".format(fw_rpm))
print("Right Wheel RPM: {}".format(rw_rpm))
print("Left Wheel RPM: {}".format(lw_rpm))

fw_ss=getStepsPersecond(fw_vel)
rw_ss=getStepsPersecond(rw_vel)
lw_ss=getStepsPersecond(lw_vel)

print("\nFront Wheel steps per second: {}".format(fw_ss))
print("Right Wheel steps per second: {}".format(lw_ss))
print("Left Wheel steps per second: {}".format(rw_ss))
