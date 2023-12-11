import sys
from croblink import *
from math import *
# import xml.etree.ElementTree as ET
# import keyboard

CELLROWS = 7
CELLCOLS = 14


def is_point_inside_polygon(point, polygon):
    x, y = point
    n = len(polygon)
    inside = False

    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]

        # Check if the point is on a horizontal boundary of the polygon
        if y1 == y2 and y1 == y and x >= min(x1, x2) and x <= max(x1, x2):
            return True

        if (y1 < y and y2 >= y) or (y2 < y and y1 >= y):
            if x1 + (y - y1) / (y2 - y1) * (x2 - x1) < x:
                inside = not inside

    return inside

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.map = Map()

    def run(self):
        self.readSensors()
        self.dx = self.measures.x
        self.dy = self.measures.y
        self.previousError = 0

        # self.unexplored = [(0,0), (5,-5), (5,0), (0,0)]
        self.go = (0,0)
        self.unexplored = []
        self.historic = []

        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.robName + " exiting")
                quit()

            # print(state, self.measures.dir, (self.x(), self.y()), self.measures.compass)
            self.map.print()
            print(self.unexplored)
            print(self.historic)

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'
                print("\nSTOPPED")

            if state == 'run':
                print("\nRUNNING")
                print("POSITION", "x:", self.x(), "y", self.y(), "a", self.a())
                print("SENSORS", "".join(self.measures.lineSensor))
                self.line = [0 if i=='0' else 1 for i in self.measures.lineSensor]

                xx = round(self.x())
                yy = round(self.y())
                found = False

                def EMPTY(): pass

                self.fillMap()
                self.detectPaths()
                print("GO:", self.go)
                if self.go:
                    self.driveTo(*self.go)
                if not self.go:
                    # for i in range(0,360,45):
                    a = round(self.a()/45)
                    # a = 0
                    for i in range(8):
                        # print("BAH", a+i*45)
                        xa = round(cos(radians(a+i*45)))
                        ya = round(sin(radians(a+i*45)))
                        piece = self.map.get(xx+xa,yy+ya)
                        # print("BAH", (xa, ya), piece)
                        if piece in ["?","a","b"]:
                            if piece == "a" and abs(self.getAngle(xx,yy,xx+xa*2,yy+ya*2)%180-45)>10: continue
                            if piece == "b" and abs(self.getAngle(xx,yy,xx+xa*2,yy+ya*2)%180-135)>10: continue
                            self.go = (xx+xa*2, yy+ya*2)
                            self.driveTo(*self.go)
                            found = True
                            break
                if not self.go and not found and len(self.historic)>=2 and self.map.unexploredQueue!=0:
                    self.go = self.historic[-2]
                    self.historic = self.historic[:-2]
                    self.driveTo(*self.go)
                if not self.go and not found and (len(self.historic)<2 or self.map.unexploredQueue==0):
                    self.map.cleanprint()
                    self.driveMotors(0,0)
                    self.finish()

                    # print map to file
                    file = open("map.out", "w")
                    self.map.cpiece("I", 0, 0)
                    file.write("\n".join([''.join(i).replace('x', ' ').replace('?', ' ').replace('.', ' ') for i in self.map.labMap[1:-1]]))
                    file.close()
                    # exit()


            elif state == 'wait':
                self.setReturningLed(True)
                if self.measures.visitingLed is True:
                    self.setVisitingLed(False)
                if self.measures.returningLed is True:
                    state = 'return'
                self.driveMotors(0.0, 0.0)
            elif state == 'return':
                if self.measures.visitingLed is True:
                    self.setVisitingLed(False)
                if self.measures.returningLed is True:
                    self.setReturningLed(False)

    def detectPaths(self):
        x = self.x()
        y = self.y()
        a = self.a()

        s3x = x+cos(radians(a))*0.438
        s3y = y+sin(radians(a))*0.438
        xx = round(s3x)
        yy = round(s3y)
        if xx%2!=0 or yy%2!=0: return


        # get intersection angle
        inta = round((self.getAngle(x, y, round(s3x), round(s3y)))/45)*45
        # if abs(a-inta)>4: return
        print("INTA", inta)
            
        def getDiagonal(a):
            if abs(a%180-45)<10: return "a"
            elif abs(a%180-135)<10: return "b"
            else: return None

        def calculatePoints(xx,yy,inta):
            basepoints = [
                [(0.26,0.1), (0.20,0), (0.26,-0.1), (0.5,-0.1), (0.5,0.1)], # right
                [(0.11,0.11), (0.25,0.11), (0.5,0.36), (0.36, 0.5), (0.11, 0.25)], # right-up
                [(0,0.20), (0.1,0.26), (0.1,0.5), (-0.1,0.5), (-0.1,0.26)], # up
                [(-0.11,0.11), (-0.11,0.25), (-0.36,0.5), (-0.25,0.11), (-0.5, 0.36)], # left-up
                [(-0.5,-0.1), (-0.5,-0.1),(-0.26,-0.1),(-0.2,0),(-0.26,-0.1)], #left
                [(-0.11,-0.11), (-0.24,-0.11), (-0.5, -0.36), (-0.36,-0.5), (-0.11, -0.24)], # left-bottom
                [(0,-0.20), (-0.1,-0.25), (-0.1,-0.5), (0.1,-0.5), (0.1,-0.25)], # bottom
                [(0.11,-0.11), (0.11,-0.24), (0.36,-0.5), (0.5,-0.36), (0.24,-0.11)] # bottom-right
            ]
            output = []
            for j in basepoints:
                points = []
                for k in j:
                    aa = self.getAngle(0,0,k[0],k[1])
                    dd = sqrt(pow(k[0],2)+pow(k[1],2))
                    points.append( (xx+cos(radians(aa+inta))*dd, yy+sin(radians(aa+inta))*dd) )
                output.append(points)
            return output



        points = calculatePoints(xx,yy,inta)
        # print(points)

        for i in range(7):
            if self.line[i]!=1: continue
            # get sensors coords
            sx = s3x - cos(radians(a+90))*0.08*(i-3)
            sy = s3y - sin(radians(a+90))*0.08*(i-3)
            print(i-3, (sx, sy))

    
            for k, j in enumerate(points):
                if k == 4: continue
                if is_point_inside_polygon((sx, sy), j):
                    print("NEW PATH", "Sensor",i, "detected at", (sx,sy), "to the", k,"of the intersection")
                    # print("intersection block sensored", j)
                    self.addUnx(xx+round(cos(radians(inta+k*45)))*2, yy+round(sin(radians(inta+k*45)))*2, getDiagonal(inta+k*45))


    def fillMap(self):
        x = round(self.x())
        y = round(self.y())
        xx = x%2
        yy = y%2
        piece = ''
        if abs(self.x()-x)<0.2 and abs(self.y()-y)<0.2:
            if xx == 1 and yy == 0:
                piece = '-'
            elif xx == 0 and yy == 1:
                piece = '|'
            elif xx == 1 and yy == 1:
                if abs(self.a()-45)<5 or abs(self.a()-225)<5:
                    piece = '/'
                elif abs(self.a()-135)<5 or abs(self.a()-315)<5:
                    piece = '\\'
            if piece!='' and any(self.line[2:5]): self.map.cpiece(piece, x, y)
            else: self.map.cpiece('.', x, y)


    def driveTo(self, x, y, garbage=0):
        print("DRIVING TO", (x, y), "CURRENTLY", (self.x(), self.y()), "DIFFERENCE", (abs(self.x()-x), abs(self.y()-y) ) )
        distance = sqrt(pow(x-self.x(), 2) + pow(y-self.y(),2))
        if distance<0.1: # reached target
            self.historic += [self.go]
            self.go = None
            return True

        speed = min(0.15, max(0.08, distance/5))

        a = degrees(atan2(y-self.y(), x-self.x()))
        dif = a - self.measures.compass
        dif += -360 if (dif>180) else 360 if (dif<-180) else 0

        # print("not on track", distance>1, not any(self.line), len(self.unexplored[0])==1, abs(dif)<2)
        if distance>1 and not (self.line[2]==1 or self.line[3]==1 or self.line[4]==1) and abs(dif)<=3:
            self.map.cpiece("x", (round(self.x())+self.go[0])//2, (round(self.y())+self.go[1])//2)
            self.go = False
            return False

        if abs(dif) > 5:
            self.driveMotors((0.15 if dif<0 else -0.15)*abs(dif/45), (-0.15 if dif<0 else 0.15)*abs(dif/45))
        else:
            self.driveMotors(speed-dif/50, speed+dif/50)

        return False
                
    def followLine(self, line):
        posOverLine = 0
        nActiveSensors = 0
        for i, elem in enumerate(line):
            if elem=='1':
                posOverLine += (i-3)
                nActiveSensors += 1
        error = 0
        if nActiveSensors == 0:
            for k in range(5):
                self.driveMotors(-0.15,-0.15)
                self.readSensors()
                return
        elif line[0] == '1':
            error = error-5
        elif line[6] == '1':
            error = error+5
        else: 
            error = posOverLine+1 / nActiveSensors

        kp = 0.05
        kd = 0.01

        derivative = error - self.previousError
        control_signal = kp * error + kd * derivative
        self.previousError = error

        newOutl = -0.10 if control_signal<-0.15 else 0.15 if control_signal > 0.15 else 0.15+control_signal/5
        newOutr = 0.15 if control_signal<-0.15 else -0.10 if control_signal > 0.15 else 0.15-control_signal/5

        # print(int(control_signal*100)/100,int(error*100)/100,"Drive:",newOutl, newOutr)
        self.driveMotors(newOutl, newOutr);

    def x(self):
        return self.measures.x - self.dx
    def y(self):
        return self.measures.y - self.dy
    def a(self):
        return (180*2+self.measures.compass)%360
    def addUnx(self, x, y, diagonal=None):
        middleX = round((x+self.x())/2)
        middleY = round((y+self.y())/2)
        piece = self.map.get(middleX, middleY)
        # print("addUnx:", (,y),(self.x()),(middleX, middleY), piece)
        if piece in [" "]:
            p = "?"
            if diagonal: p = diagonal
            self.map.cpiece(p, middleX, middleY)

    def getAngle(self, x1, y1, x2, y2):
        return (degrees(atan2(y2-y1, x2-x1))+360)%360

class Map():
    # create empty map
    def __init__(self):
        width = 49
        height = 23
        self.startX = width//2
        self.startY = height//2
        self.labMap = [[' ' for j in range(width)] for i in range(height)]
        self.cpiece('I', 0, 0)
        self.unexploredQueue = 0

    def print(self):
        print(*[''.join(i) for i in self.labMap], sep="\n")

    def cleanprint(self):
        print(*[''.join(i).replace('x', ' ').replace('?', ' ').replace('.', ' ') for i in self.labMap], sep="\n")

    def cpiece(self, piece, x, y):
        y = -y
        yy = self.startY+y
        xx = self.startX+x

        if piece in ["?","/","\\"]: self.unexploredQueue += 1
        elif self.labMap[yy][xx] in ["?","/","\\"]: self.unexploredQueue -= 1

        self.labMap[yy][xx] = piece

    def get(self, x, y):
        return self.labMap[self.startY+floor(-y)][self.startX+floor(x)]

class Line:
    def __init__(self, start, end):
        self.start = start # (x, y)
        self.end = end # (x, y)


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv), 2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob = MyRob(rob_name, pos, [0.0, 60.0, -60.0, 180.0], host)

    rob.run()
 
