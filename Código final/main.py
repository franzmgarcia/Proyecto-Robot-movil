"""
A* grid based planning
author: Atsushi Sakai(@Atsushi_twi)
"""

import matplotlib.pyplot as plt
import math
import requests
import pygame
import serial

grid_size = 5.0  # [m]
robot_size = 5.0  # [m]   
s = serial.Serial("COM6")
ip_address="127.0.0.1"
port="8000"
color_goal="RED"
color_goal=color_goal.encode()
color_car_inf="CYAN"
color_car_inf=color_car_inf.encode()
color_car_sup="BLUE"
color_car_sup=color_car_sup.encode()
vec = pygame.math.Vector2
kp = 0.5
pwmmax = 500


"""
Clase que define cada uno de los nodos de la trayectoria.
x: Posición horizontal del nodo
y: Posición vertical del nodo
cost: Costo asignado al nodo
pind: Identificador del nodo
"""
class Node:

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind
        " Agregar atributo del potencial "

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)

"""
Función que determina el camino final a partir del conjunto cerrado de elementos.
ngoal: Nodo de destino
closedset: Conjunto de todos los nodos ya evaluados
reso: Resolución del mapa
rx, ry: Definen el conjunto de pares coordenados que identifican los nodos del recorrido final
"""
def calc_fianl_path(ngoal, closedset, reso):
    # generate final course

    # La construcción del camino inicia en el nodo final
    rx, ry = [ngoal.x * reso], [ngoal.y * reso]

    # Indicador que denota los nodos de inicio y llegada
    pind = ngoal.pind

    # Hasta que no se anexe el nodo de origen, se extraen elementos del conjunto
    # El indicador del nodo de origen es siempre -1.
    while pind != -1:

        # Extrae un nodo del conjunto de nodos evaluados
        n = closedset[pind]

        # Anexa el nuevo nodo a la lista de nodos del camino
        rx.append(n.x * reso)
        ry.append(n.y * reso)
        pind = n.pind

    return rx, ry

"""
Planificación de trayectoria con el algoritmo A*

NOTAS: - Lista abierta: Todos los nodos sin evaluar
       - Lista cerrada: Nodos evaluados
       - Nodo padre: Nodo por el cual se llega a tener en cuenta un nodo 
                     dado como posible candidato
       - G: Coste de ir desde el nodo padre al actual
       - H: Distancia mínima y optimista (ignorando los obstáculos) al destino
       - F: G + H 

0. La celda de origen se añade a la lista abierta

1. Se extrae el primer elemento de la lista abierta y se inserta en la lista cerrada 

2. Se escogen los nodos adyacentes al nodo escogido

3. Para cada nodo adyacente:

   a) Si el nodo es el objetivo, se ha construido exitosamente la trayectoria. Se recorre
      la cadena de padres en sentido inverso hasta llegar al origen
   b) Si el nodo es un obstáculo, éste se ignora
   c) Si el nodo está en la lista cerrada (ya fue evaluado), éste se ignora
   d) Si el nodo ya está en la lista abierta: 
      - Si su costo G es mejor que el actual se recalculan F, G y H, 
        y se coloca como padre el nodo extraído.
      - Si su costo G no es mejor que el actual, éste se ignora
   e) Para el resto de los nodos adyacentes, se establece como padre el nodo extraído y se
      recalculan F, G, H. Después se añaden a la lista abierta

4. Se ordena la lista abierta. Ésta es una lista ordenada de manera ascendente en función
   del valor F de las celdas

5. Volver al paso 1
"""
def a_star_planning(sx, sy, gx, gy, ox, oy, reso, rr):
    """
    sx: Posición x del nodo de inicio
    sy: Posición y del nodo de inicio
    gx: Posición x del nodo objetivo
    gy: Posición y del nodo objetivo
    ox: Lista de posiciones x de los obstáculos
    oy: Lista de posiciones x de los obstáculos
    reso: Resolución de la grilla
    rr: Radio del robot
    """

    # Inicialización del nodo de partida
    nstart = Node(round(sx / reso), round(sy / reso), 0.0, -1)

    # Inicialización del nodo de llegada
    ngoal = Node(round(gx / reso), round(gy / reso), 0.0, -1)

    # Se ajustan las posiciones de los obstáculos a la resolución del mapa
    ox = [iox / reso for iox in ox]
    oy = [ioy / reso for ioy in oy]

    # Se define la región del mapa sobre la cual existen obstáculos
    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, rr)

    # Modelo base para G (ver descripción de la función)
    motion = get_motion_model()

    # Inicialización de las listas abierta y cerrada (diccionario vacío)
    openset, closedset = dict(), dict()
    openset[calc_index(nstart, xw, minx, miny)] = nstart # El nodo de partida se anexa a la lista abierta (Paso 0)

 
    while 1:
        # Se extrae el primer nodo de la lista abierta y se define como el nodo actual(Paso 1)
        c_id = min(
            openset, key=lambda o: openset[o].cost + calc_h(ngoal, openset[o].x, openset[o].y))
        current = openset[c_id]
        
        #  print("current", current)

        # Mostrar animación (NO ES PARTE DEL ALGORITMO)
         
        """
        De aquí en adelante se siguen los pasos 2 y 3 del algoritmo
        """

        # Paso 3, caso a (Se ha llegado a la meta, finaliza la construcción de la trayectoria)
        if current.x == ngoal.x and current.y == ngoal.y:
            print("Find goal")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Se extrae el primer elemento de la lista abierta y se inserta en la lista cerrada (Paso 2)
        del openset[c_id]
        closedset[c_id] = current

        # Se asigna al nodo el modelo de movimiento
        for i in range(len(motion)):
            node = Node(current.x + motion[i][0], current.y + motion[i][1],
                        current.cost + motion[i][2], c_id)
            n_id = calc_index(node, xw, minx, miny)

            # Paso 3, caso b (el nodo es un obstáculo)
            if not verify_node(node, obmap, minx, miny, maxx, maxy):
                continue

            # Paso 3, caso c (el nodo ya fue evaluado)
            if n_id in closedset:
                continue

            # Paso 3, caso d (el nodo ya está en la lisa abierta y se recalculan F,G y H)
            if n_id in openset:
                if openset[n_id].cost > node.cost:
                    openset[n_id].cost = node.cost
                    openset[n_id].pind = c_id
            else:
                openset[n_id] = node

    # Se reconstruye la trayectoria final una vez hallado el nodo de meta
    rx, ry = calc_fianl_path(ngoal, closedset, reso)

    return rx, ry


"""
Función que, dada la coordenada (x,y) de un nodo, calcula la distancia optimista
desde el mismo hasta el nodo de llegada. 

NOTA: La métrica utilizada es la distancia euclideana, no distancia de Manhattan
"""
def calc_h(ngoal, x, y):
    w = 10.0  # weight of heuristic
    d = w * math.sqrt((ngoal.x - x)**2 + (ngoal.y - y)**2)
    return d


"""
Función que verifica si un nodo dado constituye un obstáculo, o si este pertenece a la
región con obstáculos
"""
def verify_node(node, obmap, minx, miny, maxx, maxy):

    if node.x < minx:
        return False
    elif node.y < miny:
        return False
    elif node.x >= maxx:
        return False
    elif node.y >= maxy:
        return False

    if obmap[node.x][node.y]:
        return False

    return True


"""
ox: Lista de posiciones en x de los obstáculos
oy: Lista de posiciones en y de los obstáculos
reso: Resolución del mapa
vr: Radio del robot
"""
def calc_obstacle_map(ox, oy, reso, vr):

    """
    Ubica los menores puntos en el eje x y el eje y para 
    delimitar la región del mapa en la cual hay obstáculos
    """
    
    minx = round(min(ox))
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))
    #  print("minx:", minx)
    #  print("miny:", miny)
    #  print("maxx:", maxx)
    #  print("maxy:", maxy)

    xwidth = round(maxx - minx)
    ywidth = round(maxy - miny)
    #  print("xwidth:", xwidth)
    #  print("ywidth:", ywidth)

    # obstacle map generation

    
    # Genera un mapa de xwidth x ywidth de ceros
    obmap = [[False for i in range(xwidth)] for i in range(ywidth)]

    
    for ix in range(xwidth):
        x = ix + minx
        for iy in range(ywidth):
            y = iy + miny
            #  print(x, y)
            for iox, ioy in zip(ox, oy):
                # Condición para evitar colisión
                d = math.sqrt((iox - x)**2 + (ioy - y)**2)
                if d <= vr / reso:
                    obmap[ix][iy] = True
                    break

    return obmap, minx, miny, maxx, maxy, xwidth, ywidth

"""
???
"""
def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)


"""
Permite asignar la ponderación de cada una de las casillas
adyacentes a una posición dada. Cada fila de la lista motion
representa las ocho celdas adyacentes a la posición actual.
1: dx (derecha), dy (arriba)
-1: dx (izquierda), dy (abajo)

"""
def get_motion_model():
    # dx, dy, cost
    motion = [[1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, math.sqrt(2)],
              [-1, 1, math.sqrt(2)],
              [1, -1, math.sqrt(2)],
              [1, 1, math.sqrt(2)]]

    return motion

def draw_circle(radius, xcoords, ycoords):

    ox, oy = [], []

    for i in range (0, len(radius)):

        for j in range (0,10):

            ox.append(xcoords[i] + radius[i]*math.cos(math.pi*j/5))
            oy.append(ycoords[i] + radius[i]*math.sin(math.pi*j/5))

    return ox, oy

def getData(response,gx,gy,cx,cy,ox,oy):
    gx = []
    gy = []
    for x in range(0,len(response)):
        coord= response[x][0]
        x_goal= coord[0]
        y_goal= coord[1]

        #Radio_de_pelota

        radio=response[x][1]

        #Color

        color_obt=response[x][2]
        color_obt=color_obt.encode()
        if color_obt==color_car_inf:
            x_car_inf = x_goal
            y_car_inf = y_goal
        elif color_obt==color_car_sup:
            x_car_sup = x_goal
            y_car_sup = y_goal
        elif color_obt == color_goal:
            gx.append(round(coord[0]*100,2))
            gy.append(round(coord[1]*100,2))
            """distancia.append(round(math.sqrt(coord[0]**2+coord[1]**2),2))
            pelotas.append(x)"""
        else:
            cx.append(round(coord[0]*100,2))
            cy.append(round(coord[1]*100,2))
            radius.append(round(response[x][1]*100,2))
    
    pelotasx,pelotasy=draw_circle(radius,cx,cy)

    ox=ox+pelotasx
    oy=oy+pelotasy
    
    return x_car_inf,y_car_inf,x_car_sup,y_car_sup,gx,gy,ox,oy


def getVectors(x_car_inf,y_car_inf,x_car_sup,y_car_sup):
    Vup = vec(x_car_sup,y_car_sup)
    Vdown = vec(x_car_inf,y_car_inf)
    Vc = Vup - Vdown
    Vcar = Vup - Vc/2
    sx = round((Vcar.x)*100,2)
    sy = round((Vcar.y)*100,2)

    return sx,sy

def getAngle(x_car_inf,y_car_inf,x_car_sup,y_car_sup,x_ref,y_ref):
    Vup = vec(x_car_sup,y_car_sup)
    Vdown = vec(x_car_inf,y_car_inf)
    Vc = Vup - Vdown
    Vcar = Vup - Vc/2
    Vref = vec(x_ref,y_ref)
    Vdp = Vref - Vcar 
    angle = Vc.angle_to(Vdp)
    return angle

def pController(angle,kp):


    if angle < 0:
        angle = angle*(-1)
        pwm = angle*kp

        if pwm > pwmmax:
            pwm = pwmmax
        
        leftMotor = 1000 - pwm
        leftMotor=leftMotor-300
        rightMotor = 'd'
        

        """
        if angle > -10 and angle < 10:
        leftMotor = 700
        rightMotor = 700    
        """
    else:
        pwm = angle*kp

        if pwm > pwmmax:
            pwm = pwmmax
        leftMotor = 'd'
        rightMotor = 1000 - pwm
        rightMotor = rightMotor-300
    
    return leftMotor,rightMotor


def sendSerial(leftMotor,rightMotor):
    s.write('L'.encode())
    s.write(str(leftMotor).encode())
    s.write('R'.encode())
    s.write(str(rightMotor).encode())
    s.write('\r'.encode())

def path(sx, sy, gx, gy, ox, oy, grid_size, robot_size):

    rx, ry = a_star_planning(sx, sy, gx[0], gy[0], ox, oy, grid_size, robot_size)

    return rx[0],ry[0]

def main(response,gx,gy,cx,cy,ox,oy):
    x_car_inf,y_car_inf,x_car_sup,y_car_sup,gx,gy,ox,oy = getData(response,gx,gy,cx,cy,ox,oy)

    print("gx",gx)
    print("gy",gy)

    sx,sy = getVectors(x_car_inf,y_car_inf,x_car_sup,y_car_sup)

    print("sx",sx)
    print("sy",sy)

    x_ref,y_ref = path(sx, sy, gx, gy, ox, oy, grid_size, robot_size)

    print("x_ref", x_ref)
    print("y_ref",y_ref)


    print("goal_x",x_ref)
    print("goal_y",y_ref)

    angle = getAngle(x_car_inf,y_car_inf,x_car_sup,y_car_sup,x_ref/100,y_ref/100)
    print(angle)
    

    """
    if angle > -20 and angle < 20:
        leftMotor = 700
        rightMotor = 700
    
    else:
    """  
    if angle > -20 and angle < 20: 
        leftMotor = 700
        rightMotor = 700 
        
    elif angle > 20:
        leftMotor = 700
        rightMotor = 'd'
    
    elif angle < -20:
        leftMotor = 'd'
        rightMotor = 700
              
    #else:
     #   leftMotor,rightMotor = pController(angle,kp)  
    if leftMotor != 'd':
        leftMotor = round(leftMotor)
    if rightMotor != 'd':
        rightMotor = round(rightMotor)
    print(leftMotor)
    print(rightMotor)
    sendSerial(leftMotor,rightMotor)

#if __name__ == '__main__':
"""
grid_size = 1.0  # [m]
robot_size = 1.0  # [m]
"""
ox, oy = [], []
pelotasx, pelotasy = [], []
cx, cy = [], []

# Lista de objetivos
gx, gy = [], []

radius = []

for i in range(80):
        ox.append(i)
        oy.append(0.0)
for i in range(100):
        ox.append(10.0)
        oy.append(i)
for i in range(101):
        ox.append(i)
        oy.append(100.0)
for i in range(101):
        ox.append(0.0)
        oy.append(i)

while True:
    r = requests.get("http://" + ip_address + ":" + port)
    response=r.json()
    main(response,gx,gy,cx,cy,ox,oy)