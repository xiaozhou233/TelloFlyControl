
def P_Control(Error, Target, Kp):
    Kp_Control = Error*Kp

    ReturnData = Kp_Control - Target

    return ReturnData
