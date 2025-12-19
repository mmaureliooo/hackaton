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
	FOLLOWING_LINE = 0    # Estado: Robot siguiendo la línea negra
	SEARCHING_LINE = 1    # Estado: Robot buscando la línea (girando)
	WAITING_OBSTACLE = 2  # Estado: Robot detenido esperando que se retire el obstáculo

# Detecta la linea roja y devuelve su posicion (-1 izq, 0 centro, 1 der)
def detect_line(image, resolution):
	"""
	Detecta una línea negra en la imagen usando máscaras de color HSV.
	
	El algoritmo:
	1. Convierte la imagen a espacio de color HSV
	2. Aplica una máscara para detectar píxeles negros (la línea)
	3. Analiza solo la parte inferior de la imagen (donde está la línea)
	4. Calcula el centroide de los píxeles detectados
	5. Devuelve la posición relativa de la línea (-1 a 1)
	
	Args:
		image: Imagen capturada por la cámara (formato RGB)
		resolution: Resolución de la imagen [ancho, alto]
	
	Returns:
		float: Posición relativa de la línea (-1 = izquierda, 0 = centro, 1 = derecha)
		None: Si no se detecta la línea
	"""
	if image is None:
		return None
	
	# Convertir imagen de RGB a HSV (Hue, Saturation, Value)
	# HSV facilita la detección de colores independientemente de la iluminación
	hsv = cv.cvtColor(image, cv.COLOR_RGB2HSV)
	
	# Mascara para rojo (el rojo tiene dos rangos en HSV)
	lower_red_1 = np.array([0, 100, 100])
	upper_red_1 = np.array([10, 255, 255])
	lower_red_2 = np.array([160, 100, 100])
	upper_red_2 = np.array([180, 255, 255])
	
	# Rango 2: Rojo con H alto (160-180)
	lower_red_2 = np.array([160, 100, 100])  # H=160, S=100, V=100
	upper_red_2 = np.array([180, 255, 255])  # H=180, S=255, V=255
	
	# Crear máscaras para cada rango de rojo
	mask_1 = cv.inRange(hsv, lower_red_1, upper_red_1)
	mask_2 = cv.inRange(hsv, lower_red_2, upper_red_2)
	
	# Combinar ambas máscaras con OR lógico
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

# -----------------------------------------------------------------------------
# FUNCIÓN MAIN - Bucle principal del programa
# -----------------------------------------------------------------------------
def main():
	# Conectar con CoppeliaSim
	client = RemoteAPIClient()
	sim = client.require('sim')
	
	Flujo del programa:
	1. Conectar con CoppeliaSim
	2. Obtener handles de los componentes del robot
	3. Bucle principal:
	   a. Capturar imagen de la cámara
	   b. Detectar obstáculos y línea
	   c. Ejecutar máquina de estados
	   d. Controlar motores según el estado
	"""
	
	# Handles del robot
	handles = dict(
		leftMotor=sim.getObject("/LineTracer/leftMotor"),
		rightMotor=sim.getObject("/LineTracer/rightMotor"),
		camera=sim.getObject("/LineTracer/camera"),
	)
	
	# -------------------------------------------------------------------------
	# INICIALIZACIÓN
	# -------------------------------------------------------------------------
	state = State.SEARCHING_LINE  # Estado inicial: buscar la línea
	
	# Empezar parado
	sim.setJointTargetVelocity(handles['leftMotor'], 0)
	sim.setJointTargetVelocity(handles['rightMotor'], 0)
	
	# -------------------------------------------------------------------------
	# BUCLE PRINCIPAL
	# -------------------------------------------------------------------------
	while True:
		# Leer camara
		raw_image, resolution = sim.getVisionSensorImg(handles['camera'])
		
		# Convertir datos raw a array de NumPy
		image = np.frombuffer(raw_image, dtype=np.uint8)
		
		# Redimensionar al tamaño correcto [alto, ancho, 3 canales RGB]
		image = image.reshape([resolution[1], resolution[0], 3])
		
		# Rotar 180° porque la imagen viene invertida
		image = np.rot90(image, 2)
		
		# Voltear horizontalmente para corregir efecto espejo
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
		
		# Mostrar la máscara en una ventana
		cv.namedWindow("Line Mask", cv.WINDOW_NORMAL)
		cv.resizeWindow("Line Mask", resolution[0], resolution[1])
		cv.imshow("Line Mask", mask_line)
		
		key = cv.waitKey(5)
		if key == 27:  # ESC para salir
			break
		
		# Maquina de estados
		match state:
			# -----------------------------------------------------------------
			# ESTADO: SIGUIENDO LA LÍNEA
			# -----------------------------------------------------------------
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
<<<<<<< HEAD
						# Línea a la derecha -> Girar a la derecha
						# Reducir velocidad del motor derecho proporcionalmente
=======
						# Linea a la derecha, girar der
>>>>>>> 22179d3a0ff361a9a51d3535226abb4400bb09c8
						correction = abs(line_position) * BASE_SPEED
						sim.setJointTargetVelocity(handles['leftMotor'], BASE_SPEED)
						sim.setJointTargetVelocity(handles['rightMotor'], BASE_SPEED - correction)
				else:
					# Línea perdida -> Cambiar a estado de búsqueda
					state = State.SEARCHING_LINE
			
<<<<<<< HEAD
			# -----------------------------------------------------------------
			# ESTADO: BUSCANDO LA LÍNEA
			# -----------------------------------------------------------------
=======
>>>>>>> 22179d3a0ff361a9a51d3535226abb4400bb09c8
			case State.SEARCHING_LINE:
				if obstacle_detected:
					# Obstáculo detectado -> Detenerse y esperar
					sim.setJointTargetVelocity(handles['leftMotor'], 0)
					sim.setJointTargetVelocity(handles['rightMotor'], 0)
					state = State.WAITING_OBSTACLE
					
				elif line_detected:
					# Línea encontrada -> Cambiar a seguimiento
					state = State.FOLLOWING_LINE
					
				else:
<<<<<<< HEAD
					# Seguir buscando -> Girar sobre sí mismo
					# Motor izquierdo adelante, motor derecho atrás = giro en el sitio
					sim.setJointTargetVelocity(handles['leftMotor'], TURN_SPEED)
					sim.setJointTargetVelocity(handles['rightMotor'], -TURN_SPEED)
			
			# -----------------------------------------------------------------
			# ESTADO: ESPERANDO QUE SE RETIRE EL OBSTÁCULO
			# -----------------------------------------------------------------
=======
					# Girar para buscar la linea
					sim.setJointTargetVelocity(handles['leftMotor'], TURN_SPEED)
					sim.setJointTargetVelocity(handles['rightMotor'], -TURN_SPEED)
			
>>>>>>> 22179d3a0ff361a9a51d3535226abb4400bb09c8
			case State.WAITING_OBSTACLE:
				if not obstacle_detected and line_detected:
					# Obstáculo retirado y línea visible -> Continuar siguiendo
					state = State.FOLLOWING_LINE
					
				elif not obstacle_detected:
					# Obstáculo retirado pero línea no visible -> Buscar línea
					state = State.SEARCHING_LINE
					
				else:
					# Seguir parado
					sim.setJointTargetVelocity(handles['leftMotor'], 0)
					sim.setJointTargetVelocity(handles['rightMotor'], 0)
			
			# -----------------------------------------------------------------
			# ESTADO POR DEFECTO (no debería ocurrir)
			# -----------------------------------------------------------------
			case _:
				pass
		
		# Avanzar un paso en la simulación
		sim.step()
	
	# -------------------------------------------------------------------------
	# FINALIZACIÓN
	# -------------------------------------------------------------------------
	sim.stopSimulation()  # Detener la simulación al salir

if _name_ == "_main_":
	main()
