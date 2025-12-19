# =============================================================================
# ROBOT SEGUIDOR DE LÍNEA CON DETECCIÓN DE OBSTÁCULOS
# =============================================================================
# Este programa controla un robot Pioneer P3DX en CoppeliaSim para que:
# 1. Siga una línea negra en el suelo usando visión por computadora
# 2. Se detenga cuando detecte un obstáculo rojo en su camino
# 3. Continúe siguiendo la línea cuando el obstáculo desaparezca
# =============================================================================

# -----------------------------------------------------------------------------
# IMPORTACIÓN DE LIBRERÍAS
# -----------------------------------------------------------------------------
from coppeliasim_zmqremoteapi_client import RemoteAPIClient  # API para conectar con CoppeliaSim
import cv2 as cv      # OpenCV para procesamiento de imágenes
import numpy as np    # NumPy para operaciones con arrays
import math           # Operaciones matemáticas básicas

# -----------------------------------------------------------------------------
# CONSTANTES DE CONFIGURACIÓN
# -----------------------------------------------------------------------------
OBSTACLE_DISTANCE = 0.3  # Distancia mínima para detectar obstáculo (metros)
BASE_SPEED = 2.0         # Velocidad base de los motores (rad/s)
TURN_SPEED = 1.0         # Velocidad de giro cuando busca la línea (rad/s)

# -----------------------------------------------------------------------------
# CLASE STATE - Define los estados de la máquina de estados del robot
# -----------------------------------------------------------------------------
class State:
	FOLLOWING_LINE = 0    # Estado: Robot siguiendo la línea negra
	SEARCHING_LINE = 1    # Estado: Robot buscando la línea (girando)
	WAITING_OBSTACLE = 2  # Estado: Robot detenido esperando que se retire el obstáculo

# -----------------------------------------------------------------------------
# CLASE SENSOR - Wrapper para leer sensores de proximidad ultrasónicos
# -----------------------------------------------------------------------------
class Sensor:
	def __init__(self, sim, handle):
		"""
		Constructor del sensor de proximidad.
		
		Args:
			sim: Objeto de simulación de CoppeliaSim
			handle: Handle del sensor en la simulación
		"""
		self.sim = sim
		self.handle = handle
	
	def read(self):
		"""
		Lee la distancia detectada por el sensor ultrasónico.
		
		Returns:
			float: Distancia al objeto detectado en metros, o None si no detecta nada
		"""
		result = self.sim.readProximitySensor(self.handle)
		if result[0]:  # Si detectó algo
			point = result[1]  # Punto de detección [x, y, z]
			# Calcular distancia euclidiana al punto detectado
			return math.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
		return None

# -----------------------------------------------------------------------------
# FUNCIÓN DETECT_LINE - Detecta la posición de la línea negra en la imagen
# -----------------------------------------------------------------------------
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
	
	# Definir rango de color para detectar negro
	# H: 0-180 (cualquier tono), S: 0-255 (cualquier saturación), V: 0-50 (brillo bajo = negro)
	lower_black = np.array([0, 0, 0])
	upper_black = np.array([180, 255, 50])
	
	# Crear máscara binaria: píxeles negros = 255 (blanco), resto = 0
	mask = cv.inRange(hsv, lower_black, upper_black)
	
	# Analizar solo la mitad inferior de la imagen
	# La línea está en el suelo, así que solo nos interesa la parte de abajo
	height = resolution[1]
	bottom_section = mask[height//2:, :]
	
	# Calcular momentos de la máscara para encontrar el centroide
	# Los momentos son propiedades estadísticas de la forma
	mask_moment = cv.moments(bottom_section)
	
	if mask_moment["m00"] > 0:  # m00 = área total de píxeles blancos
		# Calcular coordenada X del centroide
		# m10 = momento de primer orden en X, m00 = área
		center_x = int(mask_moment["m10"] / mask_moment["m00"])
		
		# Convertir a posición relativa (-1 a 1)
		# -1 = línea completamente a la izquierda
		#  0 = línea en el centro
		# +1 = línea completamente a la derecha
		relative_pos = (center_x - resolution[0] / 2) / (resolution[0] / 2)
		return relative_pos
	
	return None  # No se detectó la línea

# -----------------------------------------------------------------------------
# FUNCIÓN DETECT_OBSTACLE - Detecta obstáculos rojos en la imagen
# -----------------------------------------------------------------------------
def detect_obstacle(image, resolution):
	"""
	Detecta si hay un obstáculo de color rojo en la imagen.
	
	El algoritmo:
	1. Convierte la imagen a HSV
	2. Aplica dos máscaras para detectar rojo (el rojo está en dos rangos de H)
	3. Combina las máscaras
	4. Cuenta los píxeles rojos detectados
	5. Si hay suficientes píxeles, se considera que hay obstáculo
	
	Args:
		image: Imagen capturada por la cámara (formato RGB)
		resolution: Resolución de la imagen [ancho, alto]
	
	Returns:
		bool: True si se detecta un obstáculo rojo, False en caso contrario
	"""
	if image is None:
		return False
	
	# Convertir a HSV para mejor detección de color
	hsv = cv.cvtColor(image, cv.COLOR_RGB2HSV)
	
	# El color rojo en HSV está en dos rangos porque el Hue es circular
	# Rango 1: Rojo con H bajo (0-10)
	lower_red_1 = np.array([0, 100, 100])    # H=0, S=100, V=100
	upper_red_1 = np.array([10, 255, 255])   # H=10, S=255, V=255
	
	# Rango 2: Rojo con H alto (160-180)
	lower_red_2 = np.array([160, 100, 100])  # H=160, S=100, V=100
	upper_red_2 = np.array([180, 255, 255])  # H=180, S=255, V=255
	
	# Crear máscaras para cada rango de rojo
	mask_1 = cv.inRange(hsv, lower_red_1, upper_red_1)
	mask_2 = cv.inRange(hsv, lower_red_2, upper_red_2)
	
	# Combinar ambas máscaras con OR lógico
	mask = cv.bitwise_or(mask_1, mask_2)
	
	# Contar píxeles rojos (píxeles blancos en la máscara)
	red_pixels = cv.countNonZero(mask)
	
	# Si hay más de 1000 píxeles rojos, hay un obstáculo
	# Este umbral puede ajustarse según el tamaño del obstáculo
	return red_pixels > 1000

# -----------------------------------------------------------------------------
# FUNCIÓN MAIN - Bucle principal del programa
# -----------------------------------------------------------------------------
def main():
	"""
	Función principal que ejecuta el control del robot.
	
	Flujo del programa:
	1. Conectar con CoppeliaSim
	2. Obtener handles de los componentes del robot
	3. Bucle principal:
	   a. Capturar imagen de la cámara
	   b. Detectar obstáculos y línea
	   c. Ejecutar máquina de estados
	   d. Controlar motores según el estado
	"""
	
	# -------------------------------------------------------------------------
	# CONEXIÓN CON COPPELIASIM
	# -------------------------------------------------------------------------
	client = RemoteAPIClient()       # Crear cliente de conexión
	sim = client.require('sim')      # Obtener objeto de simulación
	
	sim.setStepping(True)            # Modo stepping: control manual del tiempo
	sim.startSimulation()            # Iniciar la simulación
	
	# -------------------------------------------------------------------------
	# OBTENER HANDLES DE LOS COMPONENTES DEL ROBOT
	# -------------------------------------------------------------------------
	handles = dict(
		# Motores de las ruedas
		leftMotor=sim.getObject("/PioneerP3DX/leftMotor"),    # Motor izquierdo
		rightMotor=sim.getObject("/PioneerP3DX/rightMotor"),  # Motor derecho
		
		# Cámara para visión por computadora
		camera=sim.getObject("/PioneerP3DX/camera"),
		
		# Sensores ultrasónicos frontales (opcionales, para respaldo)
		frontLeftSensor=sim.getObject("/PioneerP3DX/ultrasonicSensor[3]"),
		frontRightSensor=sim.getObject("/PioneerP3DX/ultrasonicSensor[6]"),
	)
	
	# -------------------------------------------------------------------------
	# INICIALIZACIÓN
	# -------------------------------------------------------------------------
	state = State.SEARCHING_LINE  # Estado inicial: buscar la línea
	
	# Asegurar que el robot comience detenido
	sim.setJointTargetVelocity(handles['leftMotor'], 0)
	sim.setJointTargetVelocity(handles['rightMotor'], 0)
	
	# -------------------------------------------------------------------------
	# BUCLE PRINCIPAL
	# -------------------------------------------------------------------------
	while True:
		# ---------------------------------------------------------------------
		# CAPTURA Y PROCESAMIENTO DE IMAGEN
		# ---------------------------------------------------------------------
		# Obtener imagen raw de la cámara
		raw_image, resolution = sim.getVisionSensorImg(handles['camera'])
		
		# Convertir datos raw a array de NumPy
		image = np.frombuffer(raw_image, dtype=np.uint8)
		
		# Redimensionar al tamaño correcto [alto, ancho, 3 canales RGB]
		image = image.reshape([resolution[1], resolution[0], 3])
		
		# Rotar 180° porque la imagen viene invertida
		image = np.rot90(image, 2)
		
		# Voltear horizontalmente para corregir efecto espejo
		image = np.fliplr(image)
		
		# ---------------------------------------------------------------------
		# DETECCIÓN DE OBSTÁCULOS Y LÍNEA
		# ---------------------------------------------------------------------
		# Detectar si hay un obstáculo rojo en la imagen
		obstacle_detected = detect_obstacle(image, resolution)
		
		# Detectar la posición de la línea negra
		line_position = detect_line(image, resolution)
		line_detected = line_position is not None
		
		# ---------------------------------------------------------------------
		# VISUALIZACIÓN (OPCIONAL - para debugging)
		# ---------------------------------------------------------------------
		# Crear máscara de la línea para mostrar en pantalla
		hsv = cv.cvtColor(image, cv.COLOR_RGB2HSV)
		lower_black = np.array([0, 0, 0])
		upper_black = np.array([180, 255, 50])
		mask_line = cv.inRange(hsv, lower_black, upper_black)
		
		# Mostrar la máscara en una ventana
		cv.namedWindow("Line Mask", cv.WINDOW_NORMAL)
		cv.resizeWindow("Line Mask", resolution[0], resolution[1])
		cv.imshow("Line Mask", mask_line)
		
		# Salir si se presiona ESC
		key = cv.waitKey(5)
		if key == 27:
			break
		
		# ---------------------------------------------------------------------
		# MÁQUINA DE ESTADOS - Control del comportamiento del robot
		# ---------------------------------------------------------------------
		match state:
			# -----------------------------------------------------------------
			# ESTADO: SIGUIENDO LA LÍNEA
			# -----------------------------------------------------------------
			case State.FOLLOWING_LINE:
				if obstacle_detected:
					# Obstáculo detectado -> Detenerse y esperar
					sim.setJointTargetVelocity(handles['leftMotor'], 0)
					sim.setJointTargetVelocity(handles['rightMotor'], 0)
					state = State.WAITING_OBSTACLE
					
				elif line_detected:
					# Línea detectada -> Seguirla con control proporcional
					if abs(line_position) < 0.2:
						# Línea centrada -> Avanzar recto
						sim.setJointTargetVelocity(handles['leftMotor'], BASE_SPEED)
						sim.setJointTargetVelocity(handles['rightMotor'], BASE_SPEED)
					elif line_position < 0:
						# Línea a la izquierda -> Girar a la izquierda
						# Reducir velocidad del motor izquierdo proporcionalmente
						correction = abs(line_position) * BASE_SPEED
						sim.setJointTargetVelocity(handles['leftMotor'], BASE_SPEED - correction)
						sim.setJointTargetVelocity(handles['rightMotor'], BASE_SPEED)
					else:
						# Línea a la derecha -> Girar a la derecha
						# Reducir velocidad del motor derecho proporcionalmente
						correction = abs(line_position) * BASE_SPEED
						sim.setJointTargetVelocity(handles['leftMotor'], BASE_SPEED)
						sim.setJointTargetVelocity(handles['rightMotor'], BASE_SPEED - correction)
				else:
					# Línea perdida -> Cambiar a estado de búsqueda
					state = State.SEARCHING_LINE
			
			# -----------------------------------------------------------------
			# ESTADO: BUSCANDO LA LÍNEA
			# -----------------------------------------------------------------
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
					# Seguir buscando -> Girar sobre sí mismo
					# Motor izquierdo adelante, motor derecho atrás = giro en el sitio
					sim.setJointTargetVelocity(handles['leftMotor'], TURN_SPEED)
					sim.setJointTargetVelocity(handles['rightMotor'], -TURN_SPEED)
			
			# -----------------------------------------------------------------
			# ESTADO: ESPERANDO QUE SE RETIRE EL OBSTÁCULO
			# -----------------------------------------------------------------
			case State.WAITING_OBSTACLE:
				if not obstacle_detected and line_detected:
					# Obstáculo retirado y línea visible -> Continuar siguiendo
					state = State.FOLLOWING_LINE
					
				elif not obstacle_detected:
					# Obstáculo retirado pero línea no visible -> Buscar línea
					state = State.SEARCHING_LINE
					
				else:
					# Obstáculo todavía presente -> Permanecer detenido
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

# =============================================================================
# PUNTO DE ENTRADA DEL PROGRAMA
# =============================================================================
if __name__ == "__main__":
	main()
	
#comentario para commit