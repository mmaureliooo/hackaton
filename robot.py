from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import cv2 as cv
import numpy as np
import math

OBSTACLE_DISTANCE = 0.3
BASE_SPEED = 2.0
TURN_SPEED = 1.0

class State:
	FOLLOWING_LINE = 0
	SEARCHING_LINE = 1
	WAITING_OBSTACLE = 2

class Sensor:
	def __init__(self, sim, handle):
		self.sim = sim
		self.handle = handle
	
	def read(self):
		result = self.sim.readProximitySensor(self.handle)
		if result[0]:
			point = result[1]
			return math.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
		return None

def detect_line(image, resolution):
    if image is None:
        return None

    # Convertir a HSV
    hsv = cv.cvtColor(image, cv.COLOR_RGB2HSV)

    # Máscara para línea roja (considerando los dos rangos de rojo en HSV)
    lower_red_1 = np.array([0, 100, 100])
    upper_red_1 = np.array([10, 255, 255])
    lower_red_2 = np.array([160, 100, 100])
    upper_red_2 = np.array([180, 255, 255])

    mask1 = cv.inRange(hsv, lower_red_1, upper_red_1)
    mask2 = cv.inRange(hsv, lower_red_2, upper_red_2)
    mask = cv.bitwise_or(mask1, mask2)

    # Analizar solo la parte inferior de la imagen
    height = resolution[1]
    bottom_section = mask[height//2:, :]

    mask_moment = cv.moments(bottom_section)
    if mask_moment["m00"] > 0:
        center_x = int(mask_moment["m10"] / mask_moment["m00"])
        # Posición relativa (-1 a 1)
        relative_pos = (center_x - resolution[0] / 2) / (resolution[0] / 2)
        return relative_pos

    return None


def detect_obstacle(image, resolution):
	if image is None:
		return False
	
	# Convertir a HSV
	hsv = cv.cvtColor(image, cv.COLOR_RGB2HSV)
	
	# Máscara para objeto rojo
	lower_red_1 = np.array([0, 100, 100])
	upper_red_1 = np.array([10, 255, 255])
	lower_red_2 = np.array([160, 100, 100])
	upper_red_2 = np.array([180, 255, 255])
	
	mask_1 = cv.inRange(hsv, lower_red_1, upper_red_1)
	mask_2 = cv.inRange(hsv, lower_red_2, upper_red_2)
	mask = cv.bitwise_or(mask_1, mask_2)
	
	# Si hay suficientes píxeles rojos, hay obstáculo
	red_pixels = cv.countNonZero(mask)
	return red_pixels > 1000

def main():
	client = RemoteAPIClient()
	sim = client.require('sim')
	
	sim.setStepping(True)
	sim.startSimulation()
	
	# Diccionario que almacena los manejadores de los motores y sensores del robot
	handles = dict(
		leftMotor=sim.getObject("/PioneerP3DX/leftMotor"),
		rightMotor=sim.getObject("/PioneerP3DX/rightMotor"),
		camera=sim.getObject("/PioneerP3DX/camera"),
		frontLeftSensor=sim.getObject("/PioneerP3DX/ultrasonicSensor[3]"),
		frontRightSensor=sim.getObject("/PioneerP3DX/ultrasonicSensor[6]"),
	)
	
	state = State.SEARCHING_LINE
	
	sim.setJointTargetVelocity(handles['leftMotor'], 0)
	sim.setJointTargetVelocity(handles['rightMotor'], 0)
	
	while True:
		# Leer cámara
		raw_image, resolution = sim.getVisionSensorImg(handles['camera'])
		image = np.frombuffer(raw_image, dtype=np.uint8)
		image = image.reshape([resolution[1], resolution[0], 3])
		image = np.rot90(image, 2)
		image = np.fliplr(image)
		
		# Detectar obstáculo rojo
		obstacle_detected = detect_obstacle(image, resolution)
		
		# Detectar línea negra
		line_position = detect_line(image, resolution)
		line_detected = line_position is not None
		
		# Mostrar imagen (opcional)
		hsv = cv.cvtColor(image, cv.COLOR_RGB2HSV)
		lower_black = np.array([0, 0, 0])
		upper_black = np.array([180, 255, 50])
		mask_line = cv.inRange(hsv, lower_black, upper_black)
		
		cv.namedWindow("Line Mask", cv.WINDOW_NORMAL)
		cv.resizeWindow("Line Mask", resolution[0], resolution[1])
		cv.imshow("Line Mask", mask_line)
		key = cv.waitKey(5)
		if key == 27:
			break
		
		# Máquina de estados:
		match state:

			# Estado en el que el robot sigue la línea
			case State.FOLLOWING_LINE:

				# Si se detecta un obstáculo, detener el robot y esperar
				if obstacle_detected:
					sim.setJointTargetVelocity(handles['leftMotor'], 0)
					sim.setJointTargetVelocity(handles['rightMotor'], 0)
					state = State.WAITING_OBSTACLE

				# Si se detecta la línea, ajustar el movimiento
				elif line_detected:

					# Línea centrada: avanzar recto
					if abs(line_position) < 0.2:
						sim.setJointTargetVelocity(handles['leftMotor'], BASE_SPEED)
						sim.setJointTargetVelocity(handles['rightMotor'], BASE_SPEED)

					# Línea a la izquierda: corregir hacia la izquierda
					elif line_position < 0:
						correction = abs(line_position) * BASE_SPEED
						sim.setJointTargetVelocity(handles['leftMotor'], BASE_SPEED - correction)
						sim.setJointTargetVelocity(handles['rightMotor'], BASE_SPEED)

					# Línea a la derecha: corregir hacia la derecha
					else:
						correction = abs(line_position) * BASE_SPEED
						sim.setJointTargetVelocity(handles['leftMotor'], BASE_SPEED)
						sim.setJointTargetVelocity(handles['rightMotor'], BASE_SPEED - correction)

				# Si no se detecta la línea, pasar a buscarla
				else:
					state = State.SEARCHING_LINE
					
			# Estado en el que el robot busca la línea
			case State.SEARCHING_LINE:

				# Si se detecta un obstáculo, detener el robot
				if obstacle_detected:
					sim.setJointTargetVelocity(handles['leftMotor'], 0)
					sim.setJointTargetVelocity(handles['rightMotor'], 0)
					state = State.WAITING_OBSTACLE

				# Si se encuentra la línea, volver a seguirla
				elif line_detected:
					state = State.FOLLOWING_LINE

				# Si no hay línea ni obstáculo, girar para buscarla
				else:
					sim.setJointTargetVelocity(handles['leftMotor'], TURN_SPEED)
					sim.setJointTargetVelocity(handles['rightMotor'], -TURN_SPEED)
					
			# Estado en el que el robot espera a que desaparezca el obstáculo
			case State.WAITING_OBSTACLE:

				# Si ya no hay obstáculo y hay línea, seguirla
				if not obstacle_detected and line_detected:
					state = State.FOLLOWING_LINE

				# Si no hay obstáculo pero no hay línea, buscarla
				elif not obstacle_detected:
					state = State.SEARCHING_LINE

				# Si el obstáculo sigue presente, permanecer detenido
				else:
					sim.setJointTargetVelocity(handles['leftMotor'], 0)
					sim.setJointTargetVelocity(handles['rightMotor'], 0)
			
			# Caso por defecto: no hacer nada
			case _:
				pass

		
		sim.step()
	
	sim.stopSimulation()

if __name__ == "__main__":
	main()