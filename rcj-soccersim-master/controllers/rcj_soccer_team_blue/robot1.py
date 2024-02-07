import math
import utils
import time
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP

def distannce(x1, y1, x2, y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)

class MyRobot1(RCJSoccerRobot):
    def readSensors(self):
        self.robot_angle = math.degrees(self.get_compass_heading())
        self.robot_pos = self.get_gps_coordinates()
        if self.name[0] == 'B':
            self.robot_pos[0] *= -1
            self.robot_pos[1] *= -1
        if self.is_new_ball_data():
            ball_data = self.get_new_ball_data()
            self.ball_angle = math.atan2(ball_data['direction'][1], ball_data['direction'][0])*180/math.pi
            self.ball_distance = abs(0.016666666/(abs(ball_data['direction'][2])/math.sqrt(1 - ball_data['direction'][2]**2)))
            self.ball_x = -math.sin((self.ball_angle + self.robot_angle)*math.pi/180) * self.ball_distance + self.robot_pos[0]
            self.ball_y =  math.cos((self.ball_angle + self.robot_angle)*math.pi/180) * self.ball_distance + self.robot_pos[1]
            # print(self.ball_x, self.ball_y)

        Xb = self.ball_x
        Yb = self.ball_y
        # if Xb == 0: Xb = 0.0000000000000001
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
        if target_distance < 0.05:
            self.left_motor.setVelocity(0)
            self.right_motor.setVelocity(0)
        elif self.robot_angle > target_angle + 15:
            self.left_motor.setVelocity(-10)
            self.right_motor.setVelocity(10)
        elif self.robot_angle < target_angle - 15:
            self.left_motor.setVelocity(10)
            self.right_motor.setVelocity(-10)
        else:
            self.left_motor.setVelocity(10)
            self.right_motor.setVelocity(10)
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
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():  
                self.readSensors()
                if self.target_distance > 0.05 and not self.arrived_to_target:
                    self.move(self.target_x, self.target_y)
                else:
                    self.arrived_to_target = True
                    self.move(self.ball_x, self.ball_y)
                    if self.ball_distance > 0.2:
                        self.arrived_to_target = False
