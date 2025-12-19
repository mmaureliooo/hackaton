# Robot seguidor de linea con deteccion de obstaculos
# Sigue una linea roja y se para si detecta algo verde

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import cv2 as cv
import numpy as np

# Configuracion
BASE_SPEED = 2.0
TURN_SPEED = 1.0

# Estados del robot
class State:
	FOLLOWING_LINE = 0
	SEARCHING_LINE = 1
	WAITING_OBSTACLE = 2

# Detecta la linea roja y devuelve su posicion (-1 izq, 0 centro, 1 der)
def detect_line(image, resolution):
	if image is None:
		return None
	
	# Convertir a HSV
	hsv = cv.cvtColor(image, cv.COLOR_RGB2HSV)
	
	# Mascara para rojo (el rojo tiene dos rangos en HSV)
	lower_red_1 = np.array([0, 100, 100])
	upper_red_1 = np.array([10, 255, 255])
	lower_red_2 = np.array([160, 100, 100])
	upper_red_2 = np.array([180, 255, 255])
	
	mask_1 = cv.inRange(hsv, lower_red_1, upper_red_1)
	mask_2 = cv.inRange(hsv, lower_red_2, upper_red_2)
	mask = cv.bitwise_or(mask_1, mask_2)
	
	# Solo miramos la parte de abajo de la imagen
	height = resolution[1]
	bottom_section = mask[height//2:, :]
	
	# Calcular el centro de la linea
	mask_moment = cv.moments(bottom_section)
	
	if mask_moment["m00"] > 0:
		center_x = int(mask_moment["m10"] / mask_moment["m00"])
		# Convertir a posicion relativa
		relative_pos = (center_x - resolution[0] / 2) / (resolution[0] / 2)
		return relative_pos
	
	return None

# Detecta si hay un objeto verde (obstaculo)
def detect_obstacle(image, resolution):
	if image is None:
		return False
	
	hsv = cv.cvtColor(image, cv.COLOR_RGB2HSV)
	
	# Mascara para verde
	lower_green = np.array([35, 100, 100])
	upper_green = np.array([85, 255, 255])
	mask = cv.inRange(hsv, lower_green, upper_green)
	
	# Si hay muchos pixeles verdes, hay obstaculo
	green_pixels = cv.countNonZero(mask)
	return green_pixels > 1000

def main():
	# Conectar con CoppeliaSim
	client = RemoteAPIClient()
	sim = client.require('sim')
	
	sim.setStepping(True)
	sim.startSimulation()
	
	# Handles del robot
	handles = dict(
		leftMotor=sim.getObject("/LineTracer/leftMotor"),
		rightMotor=sim.getObject("/LineTracer/rightMotor"),
		camera=sim.getObject("/LineTracer/camera"),
	)
	
	state = State.SEARCHING_LINE
	
	# Empezar parado
	sim.setJointTargetVelocity(handles['leftMotor'], 0)
	sim.setJointTargetVelocity(handles['rightMotor'], 0)
	
	while True:
		# Leer camara
		raw_image, resolution = sim.getVisionSensorImg(handles['camera'])
		image = np.frombuffer(raw_image, dtype=np.uint8)
		image = image.reshape([resolution[1], resolution[0], 3])
		image = np.rot90(image, 2)
		image = np.fliplr(image)
		
		# Detectar obstaculo y linea
		obstacle_detected = detect_obstacle(image, resolution)
		line_position = detect_line(image, resolution)
		line_detected = line_position is not None
		
		# Mostrar mascara (para debug)
		hsv = cv.cvtColor(image, cv.COLOR_RGB2HSV)
		lower_black = np.array([0, 0, 0])
		upper_black = np.array([180, 255, 50])
		mask_line = cv.inRange(hsv, lower_black, upper_black)
		
		cv.namedWindow("Line Mask", cv.WINDOW_NORMAL)
		cv.resizeWindow("Line Mask", resolution[0], resolution[1])
		cv.imshow("Line Mask", mask_line)
		
		key = cv.waitKey(5)
		if key == 27:  # ESC para salir
			break
		
		# Maquina de estados
		match state:
			case State.FOLLOWING_LINE:
				if obstacle_detected:
					# Parar si hay obstaculo
					sim.setJointTargetVelocity(handles['leftMotor'], 0)
					sim.setJointTargetVelocity(handles['rightMotor'], 0)
					state = State.WAITING_OBSTACLE
				elif line_detected:
					# Seguir la linea
					if abs(line_position) < 0.2:
						# Linea centrada, ir recto
						sim.setJointTargetVelocity(handles['leftMotor'], BASE_SPEED)
						sim.setJointTargetVelocity(handles['rightMotor'], BASE_SPEED)
					elif line_position < 0:
						# Linea a la izquierda, girar izq
						correction = abs(line_position) * BASE_SPEED
						sim.setJointTargetVelocity(handles['leftMotor'], BASE_SPEED - correction)
						sim.setJointTargetVelocity(handles['rightMotor'], BASE_SPEED)
					else:
						# Linea a la derecha, girar der
						correction = abs(line_position) * BASE_SPEED
						sim.setJointTargetVelocity(handles['leftMotor'], BASE_SPEED)
						sim.setJointTargetVelocity(handles['rightMotor'], BASE_SPEED - correction)
				else:
					state = State.SEARCHING_LINE
			
			case State.SEARCHING_LINE:
				if obstacle_detected:
					sim.setJointTargetVelocity(handles['leftMotor'], 0)
					sim.setJointTargetVelocity(handles['rightMotor'], 0)
					state = State.WAITING_OBSTACLE
				elif line_detected:
					state = State.FOLLOWING_LINE
				else:
					# Girar para buscar la linea
					sim.setJointTargetVelocity(handles['leftMotor'], TURN_SPEED)
					sim.setJointTargetVelocity(handles['rightMotor'], -TURN_SPEED)
			
			case State.WAITING_OBSTACLE:
				if not obstacle_detected and line_detected:
					state = State.FOLLOWING_LINE
				elif not obstacle_detected:
					state = State.SEARCHING_LINE
				else:
					# Seguir parado
					sim.setJointTargetVelocity(handles['leftMotor'], 0)
					sim.setJointTargetVelocity(handles['rightMotor'], 0)
			
			case _:
				pass
		
		sim.step()
	
	sim.stopSimulation()

if _name_ == "_main_":
	main()