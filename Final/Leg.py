import math

class Leg:
    def __init__(self, rotate, knee, ankle, rotate_offset = 0, width = 41.9, length = 22, coxia = 19.1, femur = 120.6, tibia = 202, x0 = 50, y0 = 100, z0 = 50):
        self.W = width   # Half the distance between the left and right base servo axis in X direction
        self.L = length
        self.C = coxia   # Coxia - Rotate
        self.F = femur   # Femur - Knee
        self.T = tibia   # Tibia - Ankle
        self.x0 = x0
        self.y0 = y0
        self.z0 = z0
        self.curr_x = -20
        self.curr_y = 0
        self.curr_z = -20


        self.rotate_offset = rotate_offset

        self.rotate = rotate
        self.knee = knee		
        self.ankle = ankle		
        self.joints = [rotate, knee, ankle]

    # Transfer coordinates from base coordinates to individual leg coordinate system
    def trans_coord(self, x_offs,y_offs,z_offs):# Does more stuff here
        return self.x0 + x_offs, self.y0 + y_offs, self.z0 + z_offs



    def inverse_k(self, x_offs=0, y_offs=0, z_offs=0):
        x, y, z = self.trans_coord(x_offs,y_offs,z_offs)
        # print("solving IK for: ", x, y, z)

        # Update joint variable for interpolation between points
        self.joints[0].currAng = self.joints[0].goAng
        self.joints[1].currAng = self.joints[1].goAng
        self.joints[2].currAng = self.joints[2].goAng

        try:
            theta_c = math.atan(z/x)
            r = x / math.cos(theta_c)
            theta_f = math.pi/2 - math.atan((r-self.C)/y) - math.acos((pow(self.F,2) + y*y + pow((r - self.C),2) - pow(self.T,2)) / (2 * self.F * math.sqrt(y*y + pow((r - self.C),2))))
            theta_t = math.pi - math.acos((pow(self.F,2) + pow(self.T,2) - y*y - pow((r - self.C),2)) / (2 * self.F * self.T)) # Write stuff here
            # print(math.degrees(theta_c), math.degrees(theta_f), math.degrees(theta_t))
            self.joints[0].goAng = math.degrees(theta_c)

            self.joints[1].goAng = math.degrees(theta_f) 

            self.joints[2].goAng = math.degrees(theta_t)

        except ValueError:
            print('No Solutions Found')






