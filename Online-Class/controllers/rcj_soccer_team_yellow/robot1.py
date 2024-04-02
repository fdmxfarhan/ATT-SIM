import math
import utils
import time
# import pygame
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP

def distance(x1, y1, x2, y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)

class MyRobot1(RCJSoccerRobot):
    def readSensors(self):
        self.robot_angle = math.degrees(self.get_compass_heading())
        self.robot_pos = self.get_gps_coordinates()
        if self.name[0] == 'B':
            self.robot_pos[0] *= -1    # Robot_x = -Robot_x
            self.robot_pos[1] *= -1    # Robot_y = -Robot_y
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
            if(distance(x, y, Xb, Yb) >= r):
                self.target_x = x
                self.target_y = y
                self.target_distance = distance(self.robot_pos[0], self.robot_pos[1], self.target_x, self.target_y)
                break
            y -= 0.01
        # Tamame data hayi ke robot haye dige beheshon niaz daran ersal beshe
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
                self.ball_distance = distance(self.robot_pos[0], self.robot_pos[1], self.ball_x, self.ball_y)
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
                VL = 10 - diff*0.5
                VR = 10 + diff*0.5
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
                VL = -10 - diff*0.5
                VR = -10 + diff*0.5

        if VR > 10: VR = 10
        if VR < -10: VR = -10
        if VL > 10: VL = 10
        if VL < -10: VL = -10
        self.left_motor.setVelocity(VL)
        self.right_motor.setVelocity(VR)
    def moveAndLookAt(self, target_x, target_y, look_x, look_y):
        target_angle = math.degrees(math.atan2(self.robot_pos[0] - target_x, target_y - self.robot_pos[1]))
        look_angle = math.degrees(math.atan2(self.robot_pos[0] - look_x, look_y - self.robot_pos[1]))
        target_distance = math.sqrt((self.robot_pos[0] - target_x)**2 + (self.robot_pos[1] - target_y)**2)
        diff = self.robot_angle - target_angle
        if diff > 180: diff -= 360
        if diff <-180: diff += 360
        look_diff = self.robot_angle - look_angle
        if look_diff > 180: look_diff -= 360
        if look_diff <-180: look_diff += 360
        
        VR = 0
        VL = 0
        if target_distance < 0.05: 
            VR =  look_diff * 0.3
            VL = -look_diff * 0.3
        elif diff > -90 and diff < 90:
            if diff > 30:
                VL = -10
                VR = 10
            elif diff < -30:
                VL = 10
                VR = -10
            else:
                VL = 10 - diff*0.5
                VR = 10 + diff*0.5
        else:
            if diff > 0: diff = diff - 180
            else: diff = -180 - diff
            if diff > 30:
                VL = -10
                VR = 10
            elif diff < -30:
                VL = 10
                VR = -10
            else:
                VL = -10 - diff*0.5
                VR = -10 + diff*0.5

        if VR > 10: VR = 10
        if VR < -10: VR = -10
        if VL > 10: VL = 10
        if VL < -10: VL = -10
        self.left_motor.setVelocity(VL)
        self.right_motor.setVelocity(VR)
    def LookAt(self, look_x, look_y):
        look_angle = math.degrees(math.atan2(self.robot_pos[0] - look_x, look_y - self.robot_pos[1]))
        look_diff = self.robot_angle - look_angle
        if look_diff > 180: look_diff -= 360
        if look_diff <-180: look_diff += 360
        
        VR = 0
        VL = 0
        VR =  look_diff * 0.3
        VL = -look_diff * 0.3
        
        if VR > 10: VR = 10
        if VR < -10: VR = -10
        if VL > 10: VL = 10
        if VL < -10: VL = -10
        self.left_motor.setVelocity(VL)
        self.right_motor.setVelocity(VR)
    def forward_AI(self):
        # baraye zamani ke toop mire gooshe haye zamin kenare darvaze khodemoon.
        if self.ball_y < -0.65 and (self.ball_x > 0.3 or self.ball_x < -0.3):
            if self.ball_x > 0.3:
                self.move(self.ball_x-0.02, self.ball_y)
            elif self.ball_x < -0.3:
                self.move(self.ball_x+0.02, self.ball_y)
        # zamani ke robot jelo tar az toop bashe (jelo giri az goal be khodi)
        elif self.robot_pos[1] > self.ball_y and self.robot_pos[0] > self.ball_x-0.2 and self.robot_pos[0] < self.ball_x+0.2:
            if self.robot_pos[0] > self.ball_x:
                self.move(self.ball_x+0.2, self.ball_y-0.1)
            else:
                self.move(self.ball_x-0.2, self.ball_y-0.1)
        # zamani ke toop dar hale harekat bashe robot position nahayi toop ro pish bini mikone
        # elif self.is_ball_moving and not self.arrived_to_target:
        #     target = self.findTargetTwardGoal(self.ball_x_pred, self.ball_y_pred)
        #     self.moveAndLookAt(target[0], target[1], 0, 0.7)
        #     if self.ball_distance < 0.2:
        #         self.arrived_to_target = True

        # agar be noghte poshte toop narisidi aval boro oonja
        elif self.target_distance > 0.05 and not self.arrived_to_target:
            self.move(self.target_x, self.target_y)
        # agar be noghte poshte toop residi boro be toop zarbe bezan va Goalesh kon!!
        else:
            self.arrived_to_target = True
            self.move(self.ball_x, self.ball_y)
            if self.ball_distance > 0.2:
                self.arrived_to_target = False
    def forward_AI2(self):
        # baraye zamani ke toop mire gooshe haye zamin kenare darvaze khodemoon.
        if self.ball_y < -0.65 and (self.ball_x > 0.3 or self.ball_x < -0.3):
            if self.ball_x > 0.3:
                self.move(self.ball_x-0.02, self.ball_y-0.02)
            elif self.ball_x < -0.3:
                self.move(self.ball_x+0.02, self.ball_y-0.02)
        # zamani ke robot jelo tar az toop bashe (jelo giri az goal be khodi)
        elif self.robot_pos[1] > self.ball_y and self.robot_pos[0] > self.ball_x-0.2 and self.robot_pos[0] < self.ball_x+0.2:
            if self.robot_pos[0] > self.ball_x:
                self.move(self.ball_x+0.2, self.ball_y-0.1)
            else:
                self.move(self.ball_x-0.2, self.ball_y-0.1)
        elif self.ball_distance > 0.1:
            if self.ball_x > self.robot_pos[0]:
                self.move(self.ball_x - 0.02, self.ball_y - 0.02)
            else:
                self.move(self.ball_x + 0.02, self.ball_y - 0.02)
        else: 
            self.move(self.ball_x, self.ball_y)
    def goalKeeper_AI(self):
        self.goalkeeper_x = self.ball_x
        if (self.last_3sec_ball_x - self.ball_x) != 0 and self.last_ball_y > self.ball_y:
            m = (self.last_ball_y - self.ball_y)/(self.last_3sec_ball_x - self.ball_x)
            b = self.ball_y - m * self.ball_x
            if(m != 0):
                self.goalkeeper_x = (-0.7 - b) / m
        if self.goalkeeper_x > 0.3:
            self.goalkeeper_x = 0.3
        if self.goalkeeper_x <-0.3:
            self.goalkeeper_x = -0.3


        if self.ball_y < -0.65 and (self.ball_x > 0.3 or self.ball_x < -0.3):
            if self.ball_x > 0.3:
                self.move(self.ball_x-0.02, self.ball_y)
            elif self.ball_x < -0.3:
                self.move(self.ball_x+0.02, self.ball_y)
        else:
            self.move(self.goalkeeper_x, -0.7)
    def goalkeeper_AI2(self):
        if time.time() - self.goalkeeper_lop_time > 13 and self.ball_y > 0:
            self.move(self.ball_x, -0.5)
        elif self.robot_pos[1] > -0.5:
            self.moveAndLookAt(self.ball_x, -0.6, 1, -0.6)
        else:
            if self.robot_angle < -100 or self.robot_angle > -80:
                self.LookAt(self.robot_pos[0]+1, self.robot_pos[1])
            else:
                if self.ball_x > self.robot_pos[0] and self.robot_pos[0] < 0.3:
                    self.left_motor.setVelocity(10)
                    self.right_motor.setVelocity(10)
                elif self.ball_x < self.robot_pos[0] and self.robot_pos[0] > -0.3:
                    self.left_motor.setVelocity(-10)
                    self.right_motor.setVelocity(-10)
                else:
                    self.stop()
        # Reset lack of progress time
        if time.time() - self.goalkeeper_lop_time > 15:
            self.goalkeeper_lop_time = time.time()
    def LackOfProgress_AI(self):
        nutralSpot = self.findNutralSpot()[0]
        if self.roll == 'goalkeeper':
            # self.goalkeeper_AI2()
            if self.ball_y > 0:
                self.moveAndLookAt(0, -0.05, 0, 0.7)
            else:
                self.moveAndLookAt(0, -0.45, 0, 0.7)
        elif self.is_closest_robot_to_ball:
            self.move(self.ball_x, self.ball_y)
        else:
            target = self.findTargetTwardGoal(nutralSpot[0], nutralSpot[1])
            self.moveAndLookAt(target[0], target[1], 0, 0.7)
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
        if minIndex == self.robot_index-1:
            self.is_closest_robot_to_ball = True
        else:
            self.is_closest_robot_to_ball = False
    def predictBallFuturePos(self):
        LBX = self.last_ball_x
        LBY = self.last_ball_y
        BX = self.ball_x
        BY = self.ball_y
        
        ########### Mohasebe zavie harekat toop
        if self.is_ball and self.is_ball_moving:
            self.ball_move_angle = math.degrees(math.atan2(LBX - BX, LBY - BY))
            if self.ball_move_angle < 0: self.ball_move_angle += 360
        
        
        if self.ball_move_dir_changed == True and self.is_ball_moving:
            x = BX
            while True:
                if BX > LBX:
                    x += 0.05
                else:
                    x -= 0.05
                y = self.ball_move_m * x + self.ball_move_b
                if distance(x, y, BX, BY) > 0.4: #self.ball_v:
                    self.ball_x_pred = x
                    self.ball_y_pred = y
                    break
            self.ball_move_dir_changed = False

        if time.time() - self.predict_time > 0.9:
            self.ball_v = distance(LBX, LBY, BX, BY)/0.3

            if math.fabs(self.last_ball_move_angle - self.ball_move_angle) > 20:
                if LBX - BX != 0: 
                    self.ball_move_m = (LBY - BY)/(LBX - BX)
                self.ball_move_b = BY - self.ball_move_m * BX
                self.ball_move_dir_changed = True

            if self.ball_v > 0.1:
                self.is_ball_moving = True
            else:
                self.is_ball_moving = False
            self.last_ball_x = self.ball_x
            self.last_ball_y = self.ball_y
            self.last_ball_move_angle = self.ball_move_angle
            self.predict_time = time.time()

        # delta_time = time.time() - self.last_ball_update_time
        # if delta_time > 0.1: 
        #     delta_x = self.ball_x - self.last_ball_x
        #     delta_y = self.ball_y - self.last_ball_y

        #     ball_vx = delta_x/delta_time
        #     ball_vy = delta_y/delta_time
        #     ball_v = math.sqrt(ball_vx**2 + ball_vy**2)
        #     # print(f'vx: {round(ball_vx,2)},\t vy: {round(ball_vy,2)},\t v: {round(ball_v,2)}')
        
        #     if time.time() - self.predict_time > 3.0:
        #         self.ball_x_pred = self.ball_x + ball_vx*3.0
        #         self.ball_y_pred = self.ball_y + ball_vy*3.0
        #         self.predict_time = time.time()

        #     if ball_v > 0.01:
        #         self.is_ball_moving = True
        #     else:
        #         self.is_ball_moving = False

        #     self.last_ball_x = self.ball_x
        #     self.last_ball_y = self.ball_y
        #     self.last_ball_update_time = time.time()
    def findNutralSpot(self):
        nutral_poses = [
            # [0, 0, 0],
            # [0, -0.2, 0],
            # [0, 0.2, 0],
            [-0.3, -0.3, 0],
            [0.3, -0.3, 0],
            [0.3, 0.3, 0],
            [-0.3, 0.3, 0],
        ]
        
        # 1. ye for minevisism ke faseleye har yek az nutral_poses[i] ha ra az toop hesab konad
        for i in range(len(nutral_poses)):
            nutral_poses[i][2] = distance(nutral_poses[i][0], nutral_poses[i][1], self.ball_x, self.ball_y)
        
        # 2. ye for dige ham minevisim ke nutral_poses ra bar asas fasele az toop az kochik be bozorg moratab konad
        sorted_nutral_poses = sorted(nutral_poses, key=lambda x: x[2])
        unoccupied_sorted_nutral_poses = []
        
        # 3. ye for dige ham minevisim ke nutral_poses[i] hayi ke eshghal shodeand hazf shavand
        for i in range(len(sorted_nutral_poses)):
            is_occupied = False
            for j in range(len(self.robots_poses)):
                if distance(self.robots_poses[j][0], self.robots_poses[j][1], sorted_nutral_poses[i][0], sorted_nutral_poses[i][1]) < 0.08:
                    is_occupied = True
            if distance(sorted_nutral_poses[i][0], sorted_nutral_poses[i][1], self.ball_x, self.ball_y) < 0.08:
                is_occupied = True
            if not is_occupied:
                unoccupied_sorted_nutral_poses.append(sorted_nutral_poses[i])
        
        
        # 4. nazdik tarin noghte khonsaye eshghal nashode return shavad
        return unoccupied_sorted_nutral_poses
    def stop(self):
        self.right_motor.setVelocity(0)
        self.left_motor.setVelocity(0)
    def checkLackOfProgress(self):
        if time.time() - self.lack_of_time > 3:
            if(distance(self.ball_x, self.ball_y, self.last_3sec_ball_x, self.last_3sec_ball_y) < 0.12):
                self.lack_of_progress = True
            else:
                self.lack_of_progress = False

            self.lack_of_time = time.time()
            self.last_3sec_ball_x = self.ball_x
            self.last_3sec_ball_y = self.ball_y
    def findTargetTwardGoal(self, Xb, Yb):
        if Xb == 0: Xb = 0.0000000000000001
        # Line Equation
        m = (Yb - 0.7)/Xb
        b = 0.7
        # Circle Equation
        r = 0.3

        # Intersection of Line and Circle
        y = Yb
        while y > Yb-1:
            x = (y-b)/m
            if(distance(x, y, Xb, Yb) >= r):
                target_x = x
                target_y = y
                target_distance = distance(self.robot_pos[0], self.robot_pos[1], self.target_x, self.target_y)
                return [target_x, target_y, target_distance]
            y -= 0.01
    def run(self):
        startTime = time.time()
        self.robot_pos = [0, 0]
        self.robot_angle = 0
        self.target_x = 0
        self.target_y = 0
        self.target_distance = 0
        self.arrived_to_target = False
        self.ball_distance = 0
        self.name = self.robot.getName()     # Y1,Y2,Y3  -  B1,B2,B3
        self.robot_index = int(self.name[1])
        self.is_ball = False
        self.ball_x = 0
        self.ball_y = 0
        self.form_positions = [[-0.4, -0.2], [0, -0.7], [0.4, -0.2]]
        self.robots_poses = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]           # [ [x1, y1, d1], [x2, y2, d2], [x3, y3, d3] ]
        self.roll = 'forward'
        self.last_ball_x  = 0
        self.last_ball_y  = 0
        self.is_ball_moving = False
        self.last_ball_update_time = time.time()
        self.predict_time = time.time()
        self.ball_x_pred = 0
        self.ball_y_pred = 0
        self.lack_of_progress = False
        self.lack_of_time = time.time()
        self.last_3sec_ball_x = 0
        self.last_3sec_ball_y = 0
        self.is_closest_robot_to_ball = False
        self.ball_move_angle = 0
        self.last_ball_move_angle = 0
        self.ball_move_dir_changed = False
        self.ball_move_m = 0
        self.ball_move_b = 0
        self.ball_v = 0
        self.goalkeeper_lop_time = time.time()
        # pygame.init()
        # display = pygame.display.set_mode((300, 300))
        if self.robot_index == 2: self.roll = 'goalkeeper'
        while self.robot.step(TIME_STEP) != -1:
            # for event in pygame.event.get():
            #     if event.type == pygame.QUIT:
            #         pygame.quit()
            # keys=pygame.key.get_pressed()
            # if keys[pygame.K_w]:
            #     self.left_motor.setVelocity(10)
            #     self.right_motor.setVelocity(10)
            # if keys[pygame.K_s]:
            #     self.left_motor.setVelocity(-10)
            #     self.right_motor.setVelocity(-10)
            # if keys[pygame.K_d]:
            #     self.left_motor.setVelocity(-10)
            #     self.right_motor.setVelocity(10)
            # if keys[pygame.K_a]:
            #     self.left_motor.setVelocity(10)
            #     self.right_motor.setVelocity(-10)
            # if keys[pygame.K_SPACE]:
            #     self.left_motor.setVelocity(0)
            #     self.right_motor.setVelocity(0)
            
            if self.is_new_data():  
                self.readSensors()
                self.readTeamData()
                self.checkLackOfProgress()
                self.defineRoll()
                self.predictBallFuturePos()
                
                if self.is_ball:
                    if self.lack_of_progress:
                        self.LackOfProgress_AI()
                    elif self.roll == 'forward':
                        self.forward_AI2()
                    elif self.roll == 'goalkeeper':
                        self.goalkeeper_AI2()
                else:
                    self.moveAndLookAt(
                        self.form_positions[self.robot_index - 1][0], 
                        self.form_positions[self.robot_index - 1][1],
                        self.form_positions[self.robot_index - 1][0], 0.7
                    )

# taklif: Barname ei benevisid ke agar toop be modate 3 sanye sabet bood yek robot be nazdik tarin noghte khonsa va ba hefze
# fasele (0.08) beravad va montazere lack of progress shavad.