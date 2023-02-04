# angular pid function
def getAngVel(error, const, ang_thresh):
    ang_vel=0

    if abs(error) > ang_thresh:
        if error > 3.14:
            ang_vel = pid((error-6.28), const)
        elif error < -3.14:
            ang_vel = pid((error+6.28), const)
        else:
            ang_vel = pid(error, const)

        if ang_vel<0: ang_vel=-1.5
        else: ang_vel=1.5

    # else:
    #     self.stop()

    return ang_vel

# linear pid function
def getLinearVel(error_x,  error_y, const, linear_thresh):
    v_x=0
    v_y=0
    
    if abs(error_x)>linear_thresh or abs(error_y)>linear_thresh:
        v_x=pid(error_x, const)
        v_y=pid(error_y, const)
    # else:
    #     self.stop()

    return v_x, v_y

def pid(error, const, intg, last_error):
        prop = error
        intg = error + intg
        diff = error - last_error
        balance = const['Kp'] * prop + const['Ki'] * intg + const['Kd'] * diff
        last_error = error

        return balance