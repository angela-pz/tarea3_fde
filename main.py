import random
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import cv2 as cv
import numpy as np
client = RemoteAPIClient()
sim = client.require('sim')

sim.setStepping(True)
sim.startSimulation()  

# Definimos el robot que utilizamos, y los elementos necesarios
My_robot = sim.getObject('/PioneerP3DX') 
leftMotor = sim.getObject('/PioneerP3DX/leftMotor')  
rightMotor = sim.getObject('/PioneerP3DX/rightMotor')
camera = sim.getObject('/PioneerP3DX/Camera')
distanceSensorLeft = sim.getObject('/PioneerP3DX/ultrasonicSensorL') 
distanceSensorFrontLeft = sim.getObject('/PioneerP3DX/ultrasonicSensorFL') 
distanceSensorFrontRight = sim.getObject('/PioneerP3DX/ultrasonicSensorFR') 
distanceSensorRight = sim.getObject('/PioneerP3DX/ultrasonicSensorR') 
detectarRojo = False

# Configuramos la velocidad inicial de nuestro robot
sim.setJointTargetVelocity(leftMotor, 0.5)  
sim.setJointTargetVelocity(rightMotor, 0.5)

# Distancia a la que detecta la pared
threshold = 0.2  

while True: 
# Capturamos la imagen de la cámara
    packed_image, resolution = sim.getVisionSensorImg(camera)
    raw_image = sim.unpackUInt8Table(packed_image, 0, 0)
    image = np.array(raw_image, dtype = np.uint8)
    image = image.reshape([resolution[1], resolution[0], 3])
    image = cv.cvtColor(image, cv.COLOR_RGB2BGR)
    image = np.rot90(image, 2)
    image = np.fliplr(image)
#Aqui detectamos si el color es rojo (rgb) mediante los sensores
    red_low = np.array([110, 50, 50])
    red_high= np.array([130, 255, 255])
    red_image = cv.cvtColor(image, cv.COLOR_RGB2HSV)
    red_mask = cv.inRange(red_image, red_low, red_high)

    red_moment = cv.moments(red_mask)

    # Detectamos la distancia con los sensores ultrasónicos
    frontState, frontDistance, *_ = sim.readProximitySensor(distanceSensorLeft)
    leftState, leftDistance, *_ = sim.readProximitySensor(distanceSensorFrontLeft)
    rightState, rightDistance, *_ = sim.readProximitySensor(distanceSensorFrontRight)
    right2State, right2Distance, *_ = sim.readProximitySensor(distanceSensorRight)
    #si detecta el color rojo:
    if red_moment["m00"] > 0:
        centro_x = int(red_moment["m10"] / red_moment["m00"])
        centro_y = int(red_moment["m01"] / red_moment["m00"])
        print(f"x: {centro_x}, y: {centro_y}")

        centro_pantalla_x = resolution[0] / 2
        margen = 20  # Margen  centrado
        # Gira hacia la izquierda si el objeto está a la izquierda
        if centro_x < centro_pantalla_x - margen:
            sim.setJointTargetVelocity(leftMotor, -0.1)  
            sim.setJointTargetVelocity(rightMotor, 0.1)

        # Gira hacia la derecha si el objeto está a la derecha
        elif centro_x > centro_pantalla_x + margen:
            sim.setJointTargetVelocity(leftMotor, 0.1)  
            sim.setJointTargetVelocity(rightMotor, -0.1)

        # Avanza recto si el objeto está centrado  
        else:
            sim.setJointTargetVelocity(leftMotor, 0.2)  
            sim.setJointTargetVelocity(rightMotor, 0.2)
            detectarRojo = True
    
        # if centro_y > resolution[1] * 0.9:  #  0.9 según proximidad
        #     sim.setJointTargetVelocity(leftMotor, 0)  
        #     sim.setJointTargetVelocity(rightMotor, 0)
        #     print("Objeto rojo cerca. Detenemos robot feo.")
        #     break  # Finaliza el programa al alcanzar el objeto rojo
 
    else:
        detectarRojo=False
    # Calculamos la velocidad del robot
    linearVelocity, _ = sim.getVelocity(My_robot)
    velocity = (linearVelocity[0]**2 + linearVelocity[1]**2 + linearVelocity[2]**2)**0.5
    print(f"Velocidad del robot: {velocity}")

    # Verificamos si alguno de los sensores detecta un obstáculo dentro de los 0.2 metros
    if (frontState == 1 and frontDistance < threshold) or \
         (leftState == 1 and leftDistance < threshold) or \
         (rightState == 1 and rightDistance < threshold) or \
         (right2State == 1 and right2Distance < threshold):
        if(detectarRojo==False):


            print(f"Obstáculo detectado. Distancias a las que se ha detectado: - Frente: {frontDistance}, Izquierda: {leftDistance}, Derecha: {rightDistance}, Derecha2: {right2Distance}")

            # Cambiamos la dirección del robot para que no choque con la pared
            sim.setJointTargetVelocity(leftMotor, -0.5)  
            sim.setJointTargetVelocity(rightMotor, 0.5)

            # Avanza  unos pasos de simulación para hacer el giro
            for _ in range(20):
                sim.step()

            # Vuelve a avanzar en línea recta después del giro
            sim.setJointTargetVelocity(leftMotor, 0.5)  
            sim.setJointTargetVelocity(rightMotor, 0.5)
        else:
            sim.pauseSimulation()
            
    #En caso de que no funcionen/no tenga sensores, al dtenerse con la pared, también girará:
    # elif velocity < 0.01:
    #     print("Colisión detectada debido a falta de movimiento. Iniciando maniobra de giro.")

    #     # Cambiamos la dirección del robot para intentar liberarlo del choque
    #     sim.setJointTargetVelocity(leftMotor, -0.5)  
    #     sim.setJointTargetVelocity(rightMotor, 0.5)

    #     # Ejecuta unos pasos de simulación para realizar el giro
    #     for _ in range(20):
    #         sim.step()

    #     # Vuelve a avanzar en línea recta después del giro
    #     sim.setJointTargetVelocity(leftMotor, 0.5)  
    #     sim.setJointTargetVelocity(rightMotor, 0.5)
    
    
#cámara:
    cv.namedWindow("Camera", cv.WINDOW_NORMAL)
    cv.resizeWindow("Camera", resolution[0], resolution[1])
    cv.imshow("Camera", red_mask)

    #pulsamos ESC para salir
    key = cv.waitKey(5)
    if key == 27:
        break

    
    # Imprimimos el estado y sigue la simulación
    print(f"Estado del sensor frontal: {frontState}, Distancia al obstáculo: {frontDistance}")
    #avanza más steps de simulación
    
    sim.step()
    

sim.stopSimulation()