import math
import utils
import time
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP

def distannce(x1, y1, x2, y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)

class MyRobot2(RCJSoccerRobot):
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
        


        self.send_data_to_team({
            'id': self.robot_index, 
            'robot_pos': self.robot_pos, 
            'ball_x': self.ball_x, 
            'ball_y': self.ball_y,
            'is_ball': self.is_ball,
            'ball_distance': self.ball_distance
            })
    def readTeamData(self):
        while self.is_new_team_data():
            team_data = self.get_new_team_data()['robot_id']
            if not self.is_ball and team_data['is_ball']:
                self.ball_x = team_data['ball_x']
                self.ball_y = team_data['ball_y']
                self.is_ball = True
                self.ball_distance = distannce(self.robot_pos[0], self.robot_pos[1], self.ball_x, self.ball_y)
            self.robots_poses[team_data['id']-1] = [
                team_data['robot_pos'][0],
                team_data['robot_pos'][1],
                team_data['ball_distance']
            ]
        self.robots_poses[self.robot_index-1] = [
            self.robot_pos[0],
            self.robot_pos[1],
            self.ball_distance,
        ]
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
    def forward_AI(self):
        if self.is_ball_moving and not self.arrived_to_target:
            self.move(self.ball_x_pred, self.ball_y_pred)
            if self.ball_distance < 0.2:
                self.arrived_to_target = True
        elif self.target_distance > 0.05 and not self.arrived_to_target:
            self.move(self.target_x, self.target_y)
        else:
            self.arrived_to_target = True
            self.move(self.ball_x, self.ball_y)
            if self.ball_distance > 0.2:
                self.arrived_to_target = False
    def goalKeeper_AI(self):
        self.goalkeeper_x = self.ball_x
        if self.goalkeeper_x > 0.3:
            self.goalkeeper_x = 0.3
        if self.goalkeeper_x <-0.3:
            self.goalkeeper_x = -0.3
        self.move(self.goalkeeper_x, -0.7)
    def defineRoll(self):
        minDist = self.robots_poses[0][2]
        maxDist = self.robots_poses[0][2]
        minIndex = 0
        maxIndex = 0
        for i in range(3):
            if self.robots_poses[i][2] < minDist:
                minDist = self.robots_poses[i][2]
                minIndex = i
            if self.robots_poses[i][2] > maxDist:
                maxDist = self.robots_poses[i][2]
                maxIndex = i
        if maxIndex == self.robot_index-1:
            self.roll = 'goalkeeper'
        else:
            self.roll = 'forward'
    def predictBallFuturePos(self):
        delta_time = time.time() - self.last_ball_update_time
        if delta_time > 0.1: 
            delta_x = self.ball_x - self.last_ball_x
            delta_y = self.ball_y - self.last_ball_y

            ball_vx = delta_x/delta_time
            ball_vy = delta_y/delta_time
            ball_v = math.sqrt(ball_vx**2 + ball_vy**2)
            print(f'vx: {round(ball_vx,2)},\t vy: {round(ball_vy,2)},\t v: {round(ball_v,2)}')
        
            if time.time() - self.predict_time > 3.0:
                self.ball_x_pred = self.ball_x + ball_vx*3.0
                self.ball_y_pred = self.ball_y + ball_vy*3.0
                self.predict_time = time.time()

            if ball_v > 0.01:
                self.is_ball_moving = True
            else:
                self.is_ball_moving = False

            self.last_ball_x = self.ball_x
            self.last_ball_y = self.ball_y
            self.last_ball_update_time = time.time()
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
        self.form_positions = [[-0.4, -0.2], [0, -0.7], [0.4, -0.2]]
        self.robots_poses = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.roll = 'forward'
        self.last_ball_x  = 0
        self.last_ball_y  = 0
        self.is_ball_moving = False
        self.last_ball_update_time = time.time()
        self.predict_time = time.time()
        self.ball_x_pred = 0
        self.ball_y_pred = 0
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():  
                self.readSensors()
                self.readTeamData()
                self.defineRoll()
                self.predictBallFuturePos()
                if self.is_ball:
                    if self.roll == 'forward':
                        self.forward_AI()
                    if self.roll == 'goalkeeper':
                        self.goalKeeper_AI()
                else:
                    self.move(
                        self.form_positions[self.robot_index - 1][0], 
                        self.form_positions[self.robot_index - 1][1]
                    )

