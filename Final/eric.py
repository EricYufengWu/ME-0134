from hex import *

#Eric's Code
lower_pos = np.asarray([120,30,30,    #FL
                        90,30,30,    #ML
                        60,30,30,    #FR
                        90,30,30,    #MR
                        60,30,30,    #BL
                        120,30,30])    #BR
retr_pos = np.asarray([120,0,0,    #FL
                        90,0,0,    #ML
                        60,0,0,    #FR
                        90,0,0,    #MR
                        60,0,0,    #BL
                        120,0,0])    #BR
level_pos = np.asarray([120,90,90,    #FL
                        90,90,90,    #ML
                        60,90,90,    #FR
                        90,90,90,    #MR
                        60,90,90,    #BL
                        120,90,90])    #BR
level_L_pos = np.asarray([120,80,80,    #FL
                        90,80,80,    #ML
                        60,80,80,    #FR
                        90,80,80,    #MR
                        60,80,80,    #BL
                        120,80,80])    #BR


def create_ranges(start, stop, N, endpoint=True):
    if endpoint==1:
        divisor = N-1
    else:
        divisor = N
    steps = (1.0/divisor) * (stop - start)
    return steps[:,None]*np.arange(N) + start[:,None]

def go_pos(pos_list):
    count = 0
    for leg in LEGS:
        for joint in leg.joints:
            drivers[joint.driver].servo[joint.channel].angle = pos_list[count]
            joint.currAng = pos_list[count]
            count += 1
    sleep(1)

def ramp_pos(pos_list,rez = 50):
    count = 0
    prev_list = np.zeros(18)
    for leg in LEGS:
        for joint in leg.joints:
            prev_list[count] = joint.currAng
            count += 1

    vals = create_ranges(prev_list, pos_list, rez)
    # print(vals)
    ramp = np.zeros(18*rez).reshape(rez,18)

    for j in range(len(vals[0])):
        for i in range(len(vals)):
            ramp[j][i] = int(vals[i][j])

    for step in ramp:
        print(step)
        go_pos(step)
        sleep(0.02)

def jump():
    go_pos(level_pos)
    sleep(1)
    for i in range(20):
        go_pos(level_L_pos)
        sleep(0.1)
        go_pos(level_pos)
        sleep(0.1)
