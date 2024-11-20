from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import cv2 as cv
import numpy as np
client = RemoteAPIClient()
sim = client.require('sim')
sim.setStepping(True)

myRobot = sim.getObject('/PioneerP3DX')  

sim.startSimulation()

leftMotor = sim.getObject('/PioneerP3DX/leftMotor')  
rightMotor = sim.getObject('/PioneerP3DX/rightMotor') 
camera = sim.getObject('/PioneerP3Dx/Camera')

sim.setJointTargetVelocity(leftMotor, -0.2)
sim.setJointTargetVelocity(rightMotor, 0.2)

while True:
    packed_image, resolution = sim.getVisionSensorImg(camera)
    raw_image = sim.unpackUint8Table(packed_image, 0, 0)
    image = np.array(raw_image, dtype = np.uint8)
    image = image.reshape([resolution[1], resolution[0], 3])
    image = cv.cvtColor(image, cv.COLOR_RGB2BGR)
    image = np.rot90(image, 2)
    image = np.fliplr(image)

    red_low = np.array([110, 50, 50])
    red_high= np.array([130, 255, 255])
    red_image = cv.cvtColor(image, cv.COLOR_RGB2HSV)
    red_mask = cv.inRange(red_image, red_low, red_high)

    red_moment = cv.moments(red_mask)
    if red_moment["m00"] > 0:
        centro_x = int(red_moment["m10"] / red_moment["m00"])
        centro_y = int(red_moment["m01"] / red_moment["m00"])
        print(f"x: {centro_x}, y: {centro_y}")

        #stop when red is in center
        if centro_x < resolution[0] / 2:
            sim.setJointTargetVelocity(leftMotor, 0)
            sim.setJointTargetVelocity(rightMotor, 0)
        else:
            sim.setJointTargetVelocity(leftMotor, -0.2)
        sim.setJointTargetVelocity(rightMotor, 0.2)

    cv.namedWindow("Camera", cv.WINDOW_NORMAL)
    cv.resizeWindow("Camera", resolution[0], resolution[1])
    cv.imshow("Camera", red_mask)

    #pulsamos ESC para salir
    key = cv.waitKey(5)
    if key == 27:
        break

    sim.step()

sim.stopSimulation()