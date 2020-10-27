import requests
import serial
import pygame

#s = serial.Serial("COM7")
#print(s.name)
ip_address="127.0.0.1"
port="8000"
r = requests.get("http://" + ip_address + ":" + port)
response=r.json()
color_goal="RED"
color_goal=color_goal.encode()
color_car_inf="CYAN"
color_car_inf=color_car_inf.encode()
color_car_sup="BLUE"
color_car_sup=color_car_sup.encode()
x_ref = 0.1298114662300744
y_ref = 0.40645885327777387
vec = pygame.math.Vector2
kp = 5
pwmmax = 500


def getData():
    for x in range(0,len(response)):
        coord= response[x][0]
        x_goal=coord[0]
        y_goal= coord[1]

        #Radio_de_pelota

        radio=response[x][1]

        #Color

        color_obt=response[x][2]
        color_obt=color_obt.encode()
        if color_obt==color_car_inf:
            x_car_inf = x_goal
            y_car_inf = y_goal
        if color_obt==color_car_sup:
            x_car_sup = x_goal
            y_car_sup = y_goal
        else:
            pass
    
    return x_car_inf,y_car_inf,x_car_sup,y_car_sup


def getAngle(x_car_inf,y_car_inf,x_car_sup,y_car_sup):
    Vup = vec(x_car_sup,y_car_sup)
    Vdown = vec(x_car_inf,y_car_inf)
    Vref = vec(x_ref,y_ref)
    Vc = Vup - Vdown
    Vcar = Vup - Vc/2
    Vdp = Vref - Vcar 
    angle = Vc.angle_to(Vdp)
    return angle

def pController(angle,kp):
    if angle < 0:
        angle = angle*(-1)
        pwm = angle*kp
        if pwm > pwmmax:
            pwm = pwmmax
        leftMotor = 500 - pwm
        rightMotor = 'd'
    else:
        pwm = angle*kp
        if pwm > pwmmax:
            pwm = pwmmax
        leftMotor = 'd'
        rightMotor = 500 - pwm
    
    return leftMotor,rightMotor


def sendSerial(leftMotor,rightMotor):
    #s.write('L'.encode())
    #s.write(str(leftMotor).encode())
    #s.write('R'.encode())
    #s.write(str(rightMotor).encode())
    #s.write('\r'.encode())
    


def main():
    x_car_inf,y_car_inf,x_car_sup,y_car_sup = getData()
    angle = getAngle(x_car_inf,y_car_inf,x_car_sup,y_car_sup)
    print(angle)
    if angle > -5 and angle < 5:
        leftMotor = 300
        rightMotor = 300
    else:
        leftMotor,rightMotor = pController(angle,kp)
    print(leftMotor)
    sendSerial(leftMotor,rightMotor)

main()

