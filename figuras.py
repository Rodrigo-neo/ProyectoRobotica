import cv2
import numpy as np
import matplotlib.pyplot as plt
import sim
import sys
import time


#Terminar todas las conexiones con el simulador:
sim.simxFinish(-1)
 
#Iniciar una nueva conexión en el puerto 19999 (dirección por defecto)
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5)
 
#Comprobamos que se haya podido conectar con CoppeliaSim
if clientID!=-1:
    #Si clientID es diferente de -1, la conexión es exitosa.
    print ('Conexion establecida')
else:
    #En caso contrario, ha habido algún error. Printeamos un msj de error y salimos.
    sys.exit("Error: no se puede conectar")
     
#Guardamos los motores del robot:
error_izq, motor_izquierdo = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_oneshot_wait)
error_der, motor_derecho = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_oneshot_wait)
 
if error_izq or error_der:
    #Si no conseguimos conectarnos con los motores, emitimos un mensaje de error y salimos:
    sys.exit("Error: no se puede conectar con los motores")
     
#Guardamos el handle de la cámara del robot (el 'Vision_sensor')
error_cam, camara = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_oneshot_wait)
 
if error_cam:
    #Si no conseguimos conectarnos con el sensor de visión, emitimos un msj de error y salimos:
    sys.exit("Error: no se puede conectar con el sensor de visión")
while(1): 
	#Capturamos un frame para activar la cámara y esperamos 1 segundo:
	_, resolution, image = sim.simxGetVisionSensorImage(clientID, camara, 0, sim.simx_opmode_streaming)
	time.sleep(1)
	# read image through command line
	#Capturamos un frame de la cámara del robot. Guardamos la imagen y su resolución:
	_, resolution, image = sim.simxGetVisionSensorImage(clientID, camara, 0, sim.simx_opmode_buffer)
	 
	#Modificaremos esta imagen para que OpenCV pueda tratarla:
	img = np.array(image, dtype = np.uint8) #La convertimos a un array de numpy
	img.resize([resolution[0], resolution[1], 3]) #Cambiamos sus dimensiones
	img = np.rot90(img,2) #La rotamos 90 grados para enderezarla
	img = np.fliplr(img) #La invertimos

	# convert the image to grayscale
	gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

	# convert the grayscale image to binary image
	ret,thresh = cv2.threshold(gray_image,127,255,0)

	# find contours in the binary image
	contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	for c in contours:
	   # calculate moments for each contour
	   area = cv2.contourArea(c)
    #Mostramos la imagen:
	print(area)
	cv2.imshow('Image', img)
     
    #Se sale con ESC.
	tecla = cv2.waitKey(5) & 0xFF
	if tecla == 27:
		break
         

 
#Cerramos la conexión:
sim.simxFinish(-1)