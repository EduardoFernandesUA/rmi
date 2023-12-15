
import sys
from croblink import *
from math import cos, sin, pi, radians, sqrt, atan2

CELLROWS=7
CELLCOLS=14

class KalmanFilter:
    def __init__(self, initial_state, initial_variance):
        self.state = initial_state
        self.variance = initial_variance
        self.measurement_variance = 0.02
        self.process_variance = 0.02

    def measurement_update(self, measured_value):
        kalman_gain = self.variance / (self.variance + self.measurement_variance)
        self.state = self.state + kalman_gain * (measured_value - self.state)
        self.variance = (1 - kalman_gain) * self.variance
        return self.state, self.variance

    def prediction(self, dynamic_model):
        self.variance = (dynamic_model-self.state)**2 * self.variance + self.process_variance
        self.state = dynamic_model
        return self.state, self.variance

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.map = Map()
        self.gps = GPS(0, 0, 0)
        self.nextPos = (0, 0)

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def printMap(self):
        for l in reversed(self.map.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        self.locs = [(20,0), (20, -10), (-4, -10), (-4,-2), (0, -2), (0,0)]*100

        self.pid = PID(0.05, 0 ,0.03)
        self.karman = KalmanFilter(0, 0.02)

        self.outl, self.outr = 0, 0

        while True:
            print("\n\nRUNNING")
            print("STATE:", state)
            self.printMap()
            self.readSensors()

            if self.measures.endLed:
                print(self.robName + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                # if self.measures.visitingLed==True:
                #     state='wait'
                # if self.measures.ground==0:
                #     self.setVisitingLed(True);
                self.drive()
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)

    def drive(self):
        print("Drive:")
        print("\t",self.measures.lineSensor)
        print("\t",radians(self.measures.compass))

        # get current x and y approximations
        # inputs are based on previous frame calculations plus current compass
        x, y, a = self.gps.update(self.outl, self.outr, radians(self.measures.compass))
        print("Position:", x, y, a)

        # use line sensors to center coords
        if self.measures.lineSensor[2]=='1' and self.measures.lineSensor[3]=='1' and self.measures.lineSensor[4]=='1':
            if abs(a)<0.1 or abs(a)-pi<0.1: # moving horizontally (center Y)
                y += (round(y)-y)*0.01
                self.gps.y = y
            elif abs(a)-pi/2<0.1: # moving vertically (center X)
                x += (round(x)-x)*0.01
                self.gps.x = x
        

        # self.closePaths(x, y, a)
            

        # get next movement action
        if len(self.locs)==0: 
            ml = 0
            mr = 0
        else:
            ml, mr, self.gps.x, self.gps.y = self.driveTo(x, y, a, self.locs[0][0], self.locs[0][1])

        # add PID adjustment
        # pidL, pidR = self.pid.follow_line(self.measures.lineSensor)
        # ml += pidL*0.1
        # mr += pidR*0.1
        
        # print("[driveTo]", ml, mr)

        self.driveMotors(ml, mr)
        self.outl = (self.outl + ml) / 2 
        self.outr = (self.outr + mr) / 2 

        self.fillMap(x, y, a)

        return

    def closePaths(self, x, y, a):
        for s in range(7): # for each sensor
            if self.measures.lineSensor[s]=='0': continue
            sx = x+0.438*cos(a)+cos(a-pi/2)*0.08*(s-3)
            sy = y+0.438*sin(a)+sin(a-pi/2)*0.08*(s-3)
            print("sensor", s, (sx, sy))
            ix = round(sx)
            iy = round(sy)
            if ix%2==0 and iy%2==0:
                # print("on intersection", (ix, iy))
                if  (abs(1-cos(a))<0.1 and sx > ix+0.15) or \
                    (abs(1+sin(a))<0.1 and sy < iy-0.15) or \
                    (abs(1+cos(a))<0.1 and sx < ix-0.15) or \
                    (abs(1-sin(a))<0.1 and sy > iy+0.15):
                    print(abs(1-cos(a)), abs(1+sin(a)), abs(1+cos(a)), abs(1-sin(a)))
                    if self.map.get(round(ix+cos(a)), round(iy+sin(a))) == ' ':
                        self.map.put('?', round(ix+cos(a)), round(iy+sin(a)))

    def driveTo(self, x, y, a, dx, dy): # dx/dy -> destination x/y
        distance = sqrt(pow(dx-x, 2) + pow(dy-y,2))
        if distance<0.15:# or self.measures.lineSensor[3]=='0': 
            self.locs = self.locs[1::]
            # inFront = self.map.get(round(x+cos(a)), round(y+sin(a)))
            # if inFront=='?':
            #     self.nextPos = (round(x+round(cos(a))*2), round(y+round(sin(a))*2))
            #     self.driveTo(x, y, a, self.nextPos[0], self.nextPos[1])
            # else:
            #     return -0.1, 0.1
            
        speed = min(0.15, max(0.05, distance/2))

        da = atan2(dy-y, dx-x)
        dif = da - a
        dif += -pi*2 if (dif>pi) else 2*pi if (dif<-pi) else 0

        print("[drive To]", dif)
        pidPercentage = 0.5
        pixL, pidR = self.pid.follow_line(self.measures.lineSensor)
        turn = -pixL*pidPercentage + dif*(1-pidPercentage)
        

        return min(0.15, max(-0.15, speed-turn)), min(0.15, max(-0.15, speed+turn)), x, y

    def fillMap(self, x, y, a):
        x = round(x)
        y = round(y) 
        if x%2!=0 and y%2==0:
            if abs(a)<0.2 or abs(a/pi)<0.2:
                self.map.put('-', x, y)
        elif x%2==0 and y%2!=0:
            if abs(a+pi/2)<0.2 or abs(a-pi/2)<0.2:
                self.map.put('|', x, y)
        elif x%2!=0 and y%2!=0:
            if abs(a+pi/4)<0.2 or abs(a-3*pi/4)<0.2:
                self.map.put('\\', x, y)
            elif abs(a-pi/4)<0.2 or abs(a+3*pi/4)<0.2:
                self.map.put('/', x, y)


class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def calculate_error(self, sensors):
        weighted_sum = sum(i * sensor for i, sensor in enumerate(sensors))
        total_sum = sum(sensors)
        if total_sum == 0:
            return 0  # Avoid division by zero
        position = weighted_sum / total_sum

        desired_position = len(sensors) // 2
        error = desired_position - position

        return error


    def follow_line(self, lineSensor):
        sensors = [int(i) for i in lineSensor]
        error = self.calculate_error(sensors)

        proportional = self.kp * error
        self.integral += error
        integral = self.ki * self.integral
        derivative = self.kd * (error - self.prev_error)
        pid_output = proportional + integral + derivative
        self.prev_error = error

        print("lineSensor Error & PID:", error, pid_output)

        speed = 0.15
        return max(-speed, min(speed, -pid_output)), max(-speed, min(speed, +pid_output))

class GPS:
    def __init__(self, x, y, a):
        self.karman = KalmanFilter(0, 0.02)
        self.x = x
        self.y = y
        self.a = a

    def update(self, outl, outr, compass):
        lin = (outl + outr) / 2
        rot = outr - outl
        self.karman.prediction(rot)

        diff = compass - self.a
        while diff<-pi: diff+=2*pi
        while diff>pi: diff-=2*pi
        state, variance = self.karman.measurement_update(diff)
        self.a = self.a + state
        self.x = self.x + lin * cos(self.a)
        self.y = self.y + lin * sin(self.a)

        return self.x, self.y, self.a 

class Map():
    def __init__(self):
        self.labMap = [[' '] * (CELLCOLS*5-1) for _ in range(CELLROWS*5-1) ]

    def put(self, char, x, y):
        print("INSERT",char,"at",y+CELLROWS, x+CELLCOLS)
        # if self.labMap[y+CELLROWS][x+CELLCOLS]==' ':
        if (char == '?' and self.labMap[y+CELLROWS][x+CELLCOLS]==' ') or (char!='?'):
            self.labMap[y+CELLROWS][x+CELLCOLS] = char

    def get(self, x, y):
        return self.labMap[y+CELLROWS][x+CELLCOLS]
    # def __init__(self, filename):
    #     tree = ET.parse(filename)
    #     root = tree.getroot()
    #     
    #     self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
    #     i=1
    #     for child in root.iter('Row'):
    #        line=child.attrib['Pattern']
    #        row =int(child.attrib['Pos'])
    #        if row % 2 == 0:  # this line defines vertical lines
    #            for c in range(len(line)):
    #                if (c+1) % 3 == 0:
    #                    if line[c] == '|':
    #                        self.labMap[row][(c+1)//3*2-1]='|'
    #                    else:
    #                        None
    #        else:  # this line defines horizontal lines
    #            for c in range(len(line)):
    #                if c % 3 == 0:
    #                    if line[c] == '-':
    #                        self.labMap[row][c//3*2]='-'
    #                    else:
    #                        None
    #            
    #        i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        # mapc = Map(sys.argv[i + 1])
        pass
    else:
        print("Unknown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    # if mapc != None:
        # rob.setMap(mapc.labMap)
        # rob.printMap()
    
    rob.run()
