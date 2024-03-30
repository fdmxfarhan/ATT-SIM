import math
import utils
import time
import random
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP

class MyRobot1(RCJSoccerRobot):
    
    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x1-x2)**2 + (y1-y2)**2)
    
    def readSensors(self):
        # khandan zaviye va mogheiyate robat
        self.robot_angle = math.degrees(self.get_compass_heading())
        self.robot_pos = self.get_gps_coordinates()
        
        # qarine kardane mogheiyate robat haye aabi 
        if self.name[0] == 'B':
            self.robot_pos[0] *= -1
            self.robot_pos[1] *= -1
                
        # mohasebe mogheiyate toop
        if self.is_new_ball_data():
            self.is_ball = True
            
            # khandane etelaate jadid toop
            ball_data = self.get_new_ball_data()
            
            # mohasebe zaviye va faseleye toop az robat 
            self.ball_angle = math.atan2(ball_data['direction'][1], ball_data['direction'][0])*180/math.pi
            self.ball_distance = abs(1/60.0/(abs(ball_data['direction'][2])/math.sqrt(1 - ball_data['direction'][2]**2)))
            
            # mogheiyate vagheie toop
            self.ball_x = -math.sin((self.ball_angle + self.robot_angle)*math.pi/180) * self.ball_distance + self.robot_pos[0]
            self.ball_y =  math.cos((self.ball_angle + self.robot_angle)*math.pi/180) * self.ball_distance + self.robot_pos[1]
        else:
            self.is_ball = False
                
        # mohasebe sorate tup
        delta_ball_x = self.ball_x - self.ball_x_old       
        delta_ball_y = self.ball_y - self.ball_y_old       
        delta_time_ball_velocity = time.time() - self.start_time_ball_velocity
        
        if  delta_time_ball_velocity > 0.1:  
            self.ball_vx = delta_ball_x / delta_time_ball_velocity
            self.ball_vy = delta_ball_y / delta_time_ball_velocity
            self.ball_velocity_magnitude = math.sqrt(self.ball_vx**2 + self.ball_vy**2)
        
            self.start_time_ball_velocity = time.time()
        
            self.ball_x_old = self.ball_x
            self.ball_y_old = self.ball_y
            
        if self.ball_velocity_magnitude > 0.05 :
            self.is_ball_moving = True
        else:
            self.is_ball_moving = False
                       
        # mohasebe noghte pishbini harekete tup
        if self.ball_velocity_magnitude > 0.14:
            tgo = self.ball_distance / 0.2 / 2 * 0
        else:
            tgo = self.ball_distance / 0.2 * 0
        
        self.ball_x_pred = self.ball_x + self.ball_vx * tgo        
        self.ball_y_pred = self.ball_y + self.ball_vy * tgo
        self.ball_distance_pred = self.distance(self.robot_pos[0], self.robot_pos[1], self.ball_x_pred, self.ball_y_pred)
        
        # mohasebe mokhtasat poshte noghte pred nesbat be darvaze harif
        r = 0.16
        dis_norm = self.distance(self.ball_x_pred, self.ball_y_pred, 0, 0.7)
        Xb = (self.ball_x_pred - 0) * r / dis_norm
        Yb = (self.ball_y_pred - 0.7) * r / dis_norm
                
        self.ball_x_back = self.ball_x_pred + Xb
        self.ball_y_back = self.ball_y_pred + Yb
        self.ball_distance_back = self.distance(self.robot_pos[0], self.robot_pos[1], self.ball_x_back, self.ball_y_back)  
        
        # agar tup az robat be darvaze khodi nazdiktar bud aval bayad be kenar tup beravim
        if self.robot_pos[1] > self.ball_y :
            self.ball_x_back = self.ball_x_pred - Yb
            self.ball_y_back = self.ball_y_pred + Xb 
        
        # maghadir bayad dar mahode zamin bazi bashad
        self.ball_x_back = min(max(self.ball_x_back, -0.6), 0.6)    
        self.ball_y_back = min(max(self.ball_y_back, -0.7), 0.7)    
        
        self.goal_distance = self.distance(self.robot_pos[0], self.robot_pos[1], 0, -0.7)
        
        # print('RNo.',self.robot_index,self.is_ball,' bal dis=',round(self.ball_distance,2),
        #       'dx=',round(self.robot_pos[0]-self.ball_x,2),'dy=',round(self.robot_pos[1]-self.ball_y,2))
        
        # ersal etelaat be ham timi ha
        self.send_data_to_team({
            'id': self.robot_index, 
            'robot_pos': self.robot_pos, 
            'ball_x': self.ball_x, 
            'ball_y': self.ball_y,
            'is_ball': self.is_ball,
            'ball_distance': self.ball_distance,
            'goal_distance': self.goal_distance
            })
    
    def readTeamData(self):
        while self.is_new_team_data():
            team_data = self.get_new_team_data()['robot_id']
            
            if not self.is_ball and team_data['is_ball']:
                self.ball_x = team_data['ball_x']
                self.ball_y = team_data['ball_y']
                self.is_ball = True
                self.ball_distance = self.distance(self.robot_pos[0], self.robot_pos[1], self.ball_x, self.ball_y)
            self.robots_poses[team_data['id']-1] = [
                team_data['robot_pos'][0],
                team_data['robot_pos'][1],
                team_data['ball_distance'],
                team_data['goal_distance']
            ]
            
        self.robots_poses[self.robot_index-1] = [
            self.robot_pos[0],
            self.robot_pos[1],
            self.ball_distance,
            self.goal_distance
        ]
        
    def move(self, target_x, target_y):
        
        target_angle = math.degrees(math.atan2(self.robot_pos[0] - target_x, target_y - self.robot_pos[1]))
        angle_diff = target_angle - self.robot_angle
        
        if angle_diff > 180:
           angle_diff -= 360
        elif angle_diff < -180:
           angle_diff += 360
        
        if abs(angle_diff) > 90 :
            angle_diff += 180
            if angle_diff > 180:
               angle_diff -= 360
            elif angle_diff < -180:
                angle_diff += 360

            Vmax = -10
        else:
            Vmax = 10
            
        VR = Vmax - angle_diff * 0.5
        VL = Vmax + angle_diff * 0.5        
        
        # محدود کردن سرعت موتورها
        VR = min(max(VR, -10), 10)
        VL = min(max(VL, -10), 10)
                        
        self.left_motor.setVelocity(VL)
        self.right_motor.setVelocity(VR)
     
    def forward_AI(self):
        m = (self.robot_pos[1]-self.ball_y) / (self.robot_pos[0]-self.ball_x)
        x = (0.7 - self.robot_pos[1]) / m
        
        if  (self.ball_distance_back > 0.16) and not self.arrived_to_target:
            self.move(self.ball_x_back, self.ball_y_back)
        else:
            self.arrived_to_target = True
            if self.ball_distance <0.08:
                self.move(0, 0.7)
            else:
                self.move(self.ball_x, self.ball_y)
            if (self.ball_distance > 0.2) or (self.robot_pos[1] > self.ball_y ) :
                self.arrived_to_target = False
 
    def goalKeeper_AI(self):
        self.goalkeeper_x = self.ball_x
        
        if abs(self.ball_x) > 0.3 or self.ball_y > -0.4:
            self.goalkeeper_y = -0.7 
        else:
            self.goalkeeper_y = self.ball_y
        
        self.goalkeeper_x = min(max(self.goalkeeper_x, -0.3), 0.3)    
        
        self.move(self.goalkeeper_x, self.goalkeeper_y)
    
    def defineRoll(self):
        #peyda kardan shomare robati ke hadeaghal va hadeaksar fasele az tup ra daarand
        minDist_ball = self.robots_poses[0][2]
        minDist_goal = self.robots_poses[0][3]
        minIndex_ball = 0
        minIndex_goal = 0
                
        for i in range(3):
            if self.robots_poses[i][2] < minDist_ball:
                minDist_ball = self.robots_poses[i][2]
                minIndex_ball = i
            if self.robots_poses[i][3] < minDist_goal:
                minDist_goal = self.robots_poses[i][3]
                minIndex_goal = i
        
        if minIndex_ball == (self.robot_index-1):
            self.roll = 'forward'
        elif minIndex_goal == (self.robot_index-1):
            self.roll = 'goalkeeper'
        else:
            self.roll = 'none'    
                
    def run(self):
        self.startTime = time.time()
        self.start_time_ball_velocity = 0
        self.ball_x = 0
        self.ball_y = 0
        self.is_ball_moving = False
        self.ball_x_old = 0
        self.ball_y_old = 0
        self.arrived_to_target = False
        self.ball_distance = 0
        self.name = self.robot.getName()
        self.robot_index = int(self.name[1])
        self.is_ball = False
        self.form_positions = [[-0.2, -0.3], [0, 0.1], [0, -0.7]]
        self.robots_poses = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
        self.roll = 'forward'
              
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():  
                self.readSensors()
                self.readTeamData()
                self.defineRoll()
                
                if self.is_ball:
                    if self.roll == 'forward':
                        self.forward_AI()

                    elif self.roll == 'goalkeeper':
                        self.goalKeeper_AI()
                        
                    else:
                        self.move(0, -0.3)
                   
                else:
                    if self.roll != 'goalkeeper' :
                        self.move(self.form_positions[self.robot_index - 1][0],
                                  self.form_positions[self.robot_index - 1][1]) 
                
                #self.fix_pos = [[-0.3, -0.7], [0.3, -0.7], [0, -0.7]]
                #self.move(self.fix_pos[self.robot_index - 1][0],self.fix_pos[self.robot_index - 1][1])
