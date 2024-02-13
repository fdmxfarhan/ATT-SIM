import math
import utils
import time
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP

def distannce(x1, y1, x2, y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)

class MyRobot3(RCJSoccerRobot):
    def readSensors(self):
        self.robot_angle = math.degrees(self.get_compass_heading())
        self.robot_pos = self.get_gps_coordinates()
        if self.name[0] == 'B':
            self.robot_pos[0] *= -1
            self.robot_pos[1] *= -1
        if self.is_new_ball_data():
            self.is_ball = True
            ball_data = self.get_new_ball_data()
            self.ball_angle = math.atan2(ball_data['direction'][1], ball_data['direction'][0])*180/math.pi
            self.ball_distance = abs(0.016666666/(abs(ball_data['direction'][2])/math.sqrt(1 - ball_data['direction'][2]**2)))
            self.ball_x = -math.sin((self.ball_angle + self.robot_angle)*math.pi/180) * self.ball_distance + self.robot_pos[0]
            self.ball_y =  math.cos((self.ball_angle + self.robot_angle)*math.pi/180) * self.ball_distance + self.robot_pos[1]
            # print(self.ball_x, self.ball_y)
        else:
            self.is_ball = False
        Xb = self.ball_x
        Yb = self.ball_y
        if Xb == 0: Xb = 0.0000000000000001
        # Line Equation
        m = (Yb - 0.7)/Xb
        b = 0.7
        # Circle Equation
        r = 0.15

        # Intersection of Line and Circle
        y = Yb
        while y > Yb-1:
            x = (y-b)/m
            if(distannce(x, y, Xb, Yb) >= r):
                self.target_x = x
                self.target_y = y
                self.target_distance = distannce(self.robot_pos[0], self.robot_pos[1], self.target_x, self.target_y)
                break
            y -= 0.01
    def move(self, target_x, target_y):
        target_angle = math.degrees(math.atan2(self.robot_pos[0] - target_x, target_y - self.robot_pos[1]))
        target_distance = math.sqrt((self.robot_pos[0] - target_x)**2 + (self.robot_pos[1] - target_y)**2)
        diff = self.robot_angle - target_angle
        if diff > 180: diff -= 360
        if diff <-180: diff += 360
        
        VR = 0
        VL = 0
        if diff > -90 and diff < 90:
            if target_distance < 0.05:
                VR = 0
                VL = 0
            elif diff > 30:
                VL = -10
                VR = 10
            elif diff < -30:
                VL = 10
                VR = -10
            else:
                VL = 10 - diff*0.3
                VR = 10 + diff*0.3
        else:
            if diff > 0: diff = diff - 180
            else: diff = -180 - diff
            if target_distance < 0.05:
                VR = 0
                VL = 0
            elif diff > 30:
                VL = -10
                VR = 10
            elif diff < -30:
                VL = 10
                VR = -10
            else:
                VL = -10 - diff*0.3
                VR = -10 + diff*0.3

        if VR > 10: VR = 10
        if VR < -10: VR = -10
        if VL > 10: VL = 10
        if VL < -10: VL = -10
        self.left_motor.setVelocity(VL)
        self.right_motor.setVelocity(VR)
    def run(self):
        startTime = time.time()
        self.robot_pos = [0, 0]
        self.robot_angle = 0
        self.target_x = 0
        self.target_y = 0
        self.target_distance = 0
        self.arrived_to_target = False
        self.ball_distance = 0
        self.name = self.robot.getName()
        self.robot_index = int(self.name[1])
        self.is_ball = False
        self.ball_x = 0
        self.ball_y = 0
        self.form_positions = [[-0.2, -0.3], [0, -0.7], [0.2, -0.3]]
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():  
                self.readSensors()
                if self.is_ball:
                    if self.target_distance > 0.05 and not self.arrived_to_target:
                        self.move(self.target_x, self.target_y)
                    else:
                        self.arrived_to_target = True
                        self.move(self.ball_x, self.ball_y)
                        if self.ball_distance > 0.2:
                            self.arrived_to_target = False
                else:
                    self.move(
                        self.form_positions[self.robot_index - 1][0], 
                        self.form_positions[self.robot_index - 1][1]
                    )



# Tmrin 12:
# BBK, agar toop tavasote robot dide shod robot top ra goal konad va dar gheir in soorat be position
# haye zir beravad
# robot1 -> (-0.2, -0.3)
# robot2 -> (   0, -0.7)
# robot3 -> ( 0.2, -0.3)

# tamrin 13:
# BBK, robot az har 2 jahat betavanad be samte target beravad