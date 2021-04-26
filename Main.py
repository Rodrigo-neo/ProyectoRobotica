import sim
import sys
import cv2
import numpy as np
import time
#Umbrales
umbral_bajo_A = np.array([68, 0, 255])
umbral_alto_A = np.array([180, 200, 255])
umbral_bajo_R = np.array([0, 0, 255])
umbral_alto_R = np.array([56, 200, 255])
umbral_bajo_V = np.array([10, 0, 255])
umbral_alto_V = np.array([75, 200, 255])

k=0

sim.simxFinish(-1) #Terminar todas las conexiones
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) #Iniciar una nueva conexion en el puerto 19999 (direccion por defecto)
 
if clientID!=-1:
    print ('Conexion establecida')
 
else:
    sys.exit("Error: no se puede conectar") #Terminar este script
 
#Guardar la referencia de los motores
_, joint_a=sim.simxGetObjectHandle(clientID, 'joint1', sim.simx_opmode_blocking)
_, joint_b=sim.simxGetObjectHandle(clientID, 'joint3', sim.simx_opmode_blocking)
_,Proximity_sensor=sim.simxGetObjectHandle(clientID,'Proximity_sensor',sim.simx_opmode_blocking)
_, pescado_R=sim.simxGetObjectHandle(clientID, 'C_R', sim.simx_opmode_oneshot_wait)
_, pescado_V=sim.simxGetObjectHandle(clientID, 'C_V', sim.simx_opmode_oneshot_wait)
_, pescado_A=sim.simxGetObjectHandle(clientID, 'C_A', sim.simx_opmode_oneshot_wait)
_, pacman_R=sim.simxGetObjectHandle(clientID, 'P_R', sim.simx_opmode_oneshot_wait)
_, pacman_V=sim.simxGetObjectHandle(clientID, 'P_V', sim.simx_opmode_oneshot_wait)
_, pacman_A=sim.simxGetObjectHandle(clientID, 'P_A', sim.simx_opmode_oneshot_wait)
_, mickey_R=sim.simxGetObjectHandle(clientID, 'M_R', sim.simx_opmode_oneshot_wait)
_, mickey_V=sim.simxGetObjectHandle(clientID, 'M_V', sim.simx_opmode_oneshot_wait)
_, mickey_A=sim.simxGetObjectHandle(clientID, 'M_A', sim.simx_opmode_oneshot_wait)
_, parent=sim.simxGetObjectHandle(clientID, 'ROBOTIQ_85_attachPoint', sim.simx_opmode_oneshot_wait)

error_cam, camara = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_oneshot_wait)
if error_cam:
    sys.exit("Error: no se puede conectar con el sensor de visión")
_, resolution, image = sim.simxGetVisionSensorImage(clientID, camara, 0, sim.simx_opmode_streaming)
time.sleep(1)


while True:
	area_A = 0
	area_R = 0
	area_V = 0
	_, resolution, image = sim.simxGetVisionSensorImage(clientID, camara, 0, sim.simx_opmode_buffer)
	img = np.array(image, dtype = np.uint8) 
	img.resize([resolution[0], resolution[1], 3])
	img = np.rot90(img,2)
	img = np.fliplr(img)       
	img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
	mask_A = cv2.inRange(img_hsv, umbral_bajo_A, umbral_alto_A)
	#res_A = cv2.bitwise_and(img, img, mask=mask_A)
	mask_R = cv2.inRange(img_hsv, umbral_bajo_R, umbral_alto_R)
	#res_R = cv2.bitwise_and(img, img, mask=mask_R)
	mask_V = cv2.inRange(img_hsv, umbral_bajo_V, umbral_alto_V)
	#res_V = cv2.bitwise_and(img, img, mask=mask_V)
	contours_A, hierarchy_A = cv2.findContours(mask_A,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	for c in contours_A:
	    area_A = cv2.contourArea(c)   
	contours_R, hierarchy_R = cv2.findContours(mask_R,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	for a in contours_R:
	    area_R = cv2.contourArea(a) 
	contours_V, hierarchy_V = cv2.findContours(mask_V,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	for b in contours_V:
	    area_V = cv2.contourArea(b)        
	#Mostramos la imagen:
	#cv2.imshow('Azul', mask_A)
	#cv2.imshow('Rojo', mask_R)
    #cv2.imshow('Verde', mask_V)
	Psensor_distance = sim.simxReadProximitySensor(clientID,Proximity_sensor,sim.simx_opmode_blocking)
	if(Psensor_distance[1]==True and k==0):
		for i in np.arange(0,1.31,0.10):
			sim.simxSetJointTargetPosition(clientID, joint_a,-i, sim.simx_opmode_oneshot)
			time.sleep(0.1)
		k = 1
		print('Area Rojo: ',area_R)
		print('Area Verde: ',area_V)
		print('Area Azul: ',area_A)
	#Pescado Rojo
		if 5370 <= area_R <= 6500 and k == 1:
			print('Figura Detectada: Pescado Rojo')
			time.sleep(2)
			sim.simxSetObjectParent(clientID, pescado_R, parent, True, sim.simx_opmode_oneshot)
			#Movimiento superior
			for j in np.arange(0,1,0.10):
				sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
				time.sleep(0.1)
			#Traslacion
			for i in np.arange(1.31,3.20,0.10):
				sim.simxSetJointTargetPosition(clientID, joint_a,-i, sim.simx_opmode_oneshot)
				time.sleep(0.1)		
			time.sleep(2)
			#movimiento superior
			for j in np.arange(1,0,-0.10):
				sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
				time.sleep(0.1)
			time.sleep(2)
			sim.simxSetObjectParent(clientID, pescado_R, -1, False, sim.simx_opmode_oneshot)
			k = 2
			if k == 2:
				#Movimiento superior
				for j in np.arange(0,1,0.10):
					sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				for i in np.arange(3.20,0,-0.10):
					sim.simxSetJointTargetPosition(clientID, joint_a,-i, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				for j in np.arange(1,0,-0.10):
					sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				k = 0
				print('---------------------------')
	#Pescado Verde
		if 5370 <= area_V <= 6500 and k == 1:
			print('Figura Detectada: Pescado Verde')
			time.sleep(2)
			sim.simxSetObjectParent(clientID, pescado_V, parent, True, sim.simx_opmode_oneshot)
			#Movimiento superior
			for j in np.arange(0,1,0.10):
				sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
				time.sleep(0.1)
			#Traslacion
			for i in np.arange(1.31,4.50,0.10):
				sim.simxSetJointTargetPosition(clientID, joint_a,-i, sim.simx_opmode_oneshot)
				time.sleep(0.1)		
			time.sleep(2)
			#movimiento superior
			for j in np.arange(1,0,-0.10):
				sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
				time.sleep(0.1)
			time.sleep(2)
			sim.simxSetObjectParent(clientID, pescado_V, -1, False, sim.simx_opmode_oneshot)
			k = 2
			if k == 2:
				#Movimiento superior
				for j in np.arange(0,1,0.10):
					sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				for i in np.arange(4.50,0,-0.10):
					sim.simxSetJointTargetPosition(clientID, joint_a,-i, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				for j in np.arange(1,0,-0.10):
					sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				k = 0	
				print('---------------------------')
	#Pescado Azul
		if 5370 <= area_A <= 6500 and k == 1:
			print('Figura Detectada: Pescado Azul')
			time.sleep(2)
			sim.simxSetObjectParent(clientID, pescado_A, parent, True, sim.simx_opmode_oneshot)
			#Movimiento superior
			for j in np.arange(0,1,0.10):
				sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
				time.sleep(0.1)
			#Traslacion
			for i in np.arange(1.31,6.18,0.10):
				sim.simxSetJointTargetPosition(clientID, joint_a,-i, sim.simx_opmode_oneshot)
				time.sleep(0.1)		
			time.sleep(2)
			#movimiento superior
			for j in np.arange(1,0,-0.10):
				sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
				time.sleep(0.1)
			time.sleep(2)
			sim.simxSetObjectParent(clientID, pescado_A, -1, False, sim.simx_opmode_oneshot)
			k = 2
			if k == 2:
				#Movimiento superior
				for j in np.arange(0,1,0.10):
					sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				for i in np.arange(6.18,0,-0.10):
					sim.simxSetJointTargetPosition(clientID, joint_a,-i, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				for j in np.arange(1,0,-0.10):
					sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				k = 0
				print('---------------------------')	  
	#Pacman Verde
		if 11910 <= area_V <= 15000 and k == 1:
			print('Figura Detectada: Pacman Verde')
			time.sleep(2)
			sim.simxSetObjectParent(clientID, pacman_V, parent, True, sim.simx_opmode_oneshot)
			#Movimiento superior
			for j in np.arange(0,1,0.10):
				sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
				time.sleep(0.1)
			#Traslacion
			for i in np.arange(1.31,5.00,0.10):
				sim.simxSetJointTargetPosition(clientID, joint_a,-i, sim.simx_opmode_oneshot)
				time.sleep(0.1)		
			time.sleep(2)
			#movimiento superior
			for j in np.arange(1,0,-0.10):
				sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
				time.sleep(0.1)
			time.sleep(2)
			sim.simxSetObjectParent(clientID, pacman_V, -1, False, sim.simx_opmode_oneshot)
			k = 2
			if k == 2:
				#Movimiento superior
				for j in np.arange(0,1,0.10):
					sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				for i in np.arange(5.00,0,-0.10):
					sim.simxSetJointTargetPosition(clientID, joint_a,-i, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				for j in np.arange(1,0,-0.10):
					sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				k = 0
				print('---------------------------')
	#Pacman Rojo
		if 11910 <= area_R <= 15000 and k == 1:
			print('Figura Detectada: Pacman Rojo')
			time.sleep(2)
			sim.simxSetObjectParent(clientID, pacman_R, parent, True, sim.simx_opmode_oneshot)
			#Movimiento superior
			for j in np.arange(0,1,0.10):
				sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
				time.sleep(0.1)
			#Traslacion
			for i in np.arange(1.31,3.50,0.10):
				sim.simxSetJointTargetPosition(clientID, joint_a,-i, sim.simx_opmode_oneshot)
				time.sleep(0.1)		
			time.sleep(2)
			#movimiento superior
			for j in np.arange(1,0,-0.10):
				sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
				time.sleep(0.1)
			time.sleep(2)
			sim.simxSetObjectParent(clientID, pacman_R, -1, False, sim.simx_opmode_oneshot)
			k = 2
			if k == 2:
				#Movimiento superior
				for j in np.arange(0,1,0.10):
					sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				for i in np.arange(3.20,0,-0.10):
					sim.simxSetJointTargetPosition(clientID, joint_a,-i, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				for j in np.arange(1,0,-0.10):
					sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				k = 0
				print('---------------------------')
	#Pacman Azul
		if 11910 <= area_A <= 15000 and k == 1:
			print('Figura Detectada: Pacman Azul')
			time.sleep(2)
			sim.simxSetObjectParent(clientID, pacman_A, parent, True, sim.simx_opmode_oneshot)
			#Movimiento superior
			for j in np.arange(0,1,0.10):
				sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
				time.sleep(0.1)
			#Traslacion
			for i in np.arange(1.31,6.50,0.10):
				sim.simxSetJointTargetPosition(clientID, joint_a,-i, sim.simx_opmode_oneshot)
				time.sleep(0.1)		
			time.sleep(2)
			#movimiento superior
			for j in np.arange(1,0,-0.10):
				sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
				time.sleep(0.1)
			time.sleep(2)
			sim.simxSetObjectParent(clientID, pacman_A, -1, False, sim.simx_opmode_oneshot)
			k = 2
			if k == 2:
				#Movimiento superior
				for j in np.arange(0,1,0.10):
					sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				for i in np.arange(6.50,0,-0.10):
					sim.simxSetJointTargetPosition(clientID, joint_a,-i, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				for j in np.arange(1,0,-0.10):
					sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				k = 0
				print('---------------------------')
	#Mickey Azul
		if 1 <= area_A <= 30 and k == 1:
			print('Figura Detectada: Mickey Azul')
			time.sleep(2)
			sim.simxSetObjectParent(clientID, mickey_A, parent, True, sim.simx_opmode_oneshot)
			#Movimiento superior
			for j in np.arange(0,1,0.10):
				sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
				time.sleep(0.1)
			#Traslacion
			for i in np.arange(1.31,5.80,0.10):
				sim.simxSetJointTargetPosition(clientID, joint_a,-i, sim.simx_opmode_oneshot)
				time.sleep(0.1)		
			time.sleep(2)
			#movimiento superior
			for j in np.arange(1,0,-0.10):
				sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
				time.sleep(0.1)
			time.sleep(2)
			sim.simxSetObjectParent(clientID, mickey_A, -1, False, sim.simx_opmode_oneshot)
			k = 2
			if k == 2:
				#Movimiento superior
				for j in np.arange(0,1,0.10):
					sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				for i in np.arange(5.80,0,-0.10):
					sim.simxSetJointTargetPosition(clientID, joint_a,-i, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				for j in np.arange(1,0,-0.10):
					sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				k = 0
				print('---------------------------')
	#Mickey Rojo
		if 1 <= area_R <= 30 and k == 1:
			print('Figura Detectada: Mickey Rojo')
			time.sleep(2)
			sim.simxSetObjectParent(clientID, mickey_R, parent, True, sim.simx_opmode_oneshot)
			#Movimiento superior
			for j in np.arange(0,1,0.10):
				sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
				time.sleep(0.1)
			#Traslacion
			for i in np.arange(1.31,2.80,0.10):
				sim.simxSetJointTargetPosition(clientID, joint_a,-i, sim.simx_opmode_oneshot)
				time.sleep(0.1)		
			time.sleep(2)
			#movimiento superior
			for j in np.arange(1,0,-0.10):
				sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
				time.sleep(0.1)
			time.sleep(2)
			sim.simxSetObjectParent(clientID, mickey_R, -1, False, sim.simx_opmode_oneshot)
			k = 2
			if k == 2:
				#Movimiento superior
				for j in np.arange(0,1,0.10):
					sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				for i in np.arange(2.40,0,-0.10):
					sim.simxSetJointTargetPosition(clientID, joint_a,-i, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				for j in np.arange(1,0,-0.10):
					sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				k = 0
				print('---------------------------')
	#Mickey Verde
		if 1 <= area_V <= 30 and k == 1 :
			print('Figura Detectada: Mickey Verde')
			time.sleep(2)
			sim.simxSetObjectParent(clientID, mickey_V, parent, True, sim.simx_opmode_oneshot)
			#Movimiento superior
			for j in np.arange(0,1,0.10):
				sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
				time.sleep(0.1)
			#Traslacion
			for i in np.arange(1.31,4.00,0.10):
				sim.simxSetJointTargetPosition(clientID, joint_a,-i, sim.simx_opmode_oneshot)
				time.sleep(0.1)		
			time.sleep(2)
			#movimiento superior
			for j in np.arange(1,0,-0.10):
				sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
				time.sleep(0.1)
			time.sleep(2)
			sim.simxSetObjectParent(clientID, mickey_V, -1, False, sim.simx_opmode_oneshot)
			k = 2
			if k == 2:
				#Movimiento superior
				for j in np.arange(0,1,0.10):
					sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				for i in np.arange(4.00,0,-0.10):
					sim.simxSetJointTargetPosition(clientID, joint_a,-i, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				for j in np.arange(1,0,-0.10):
					sim.simxSetJointTargetPosition(clientID, joint_b,-j, sim.simx_opmode_oneshot)
					time.sleep(0.1)
				k = 0
				print('---------------------------')