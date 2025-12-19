# Robot seguidor de linea roja con deteccion de obstaculos
# Detecta bloqueo si el robot no se mueve aunque los motores esten activos

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Estados del robot
SIGUIENDO_LINEA = 0
BUSCANDO_LINEA = 1
OBSTACULO_DETECTADO = 2

# Configuracion de velocidades
BASE_SPEED = 2.0
TURN_SPEED = 1.5

# Deteccion de bloqueo
BLOCK_THRESHOLD = 0.001
BLOCK_CYCLES = 10

# Umbrales de color
RED_THRESHOLD = 0.5

def is_red(color):
	if color is None:
		return False
	r, g, b = color[0], color[1], color[2]
	return r > RED_THRESHOLD and g < 0.3 and b < 0.3

def main():
	client = RemoteAPIClient()
	sim = client.require('sim')
	
	sim.setStepping(True)
	sim.startSimulation()
	
	# Handles del robot LineTracer
	handles = dict(
		leftMotor=sim.getObject("/LineTracer/DynamicLeftJoint"),
		rightMotor=sim.getObject("/LineTracer/DynamicRightJoint"),
		leftSensor=sim.getObject("/LineTracer/LeftSensor"),
		rightSensor=sim.getObject("/LineTracer/RightSensor"),
		middleSensor=sim.getObject("/LineTracer/MiddleSensor"),
		robot=sim.getObject("/LineTracer"),
	)
	
	state = BUSCANDO_LINEA
	last_position = sim.getObjectPosition(handles['robot'], -1)
	block_counter = 0
	
	while True:
		# Obtener posicion actual
		current_position = sim.getObjectPosition(handles['robot'], -1)
		
		# Calcular movimiento
		dx = current_position[0] - last_position[0]
		dy = current_position[1] - last_position[1]
		movement = (dx*dx + dy*dy) ** 0.5
		
		# Leer sensores
		left_result = sim.readProximitySensor(handles['leftSensor'])
		right_result = sim.readProximitySensor(handles['rightSensor'])
		middle_result = sim.readProximitySensor(handles['middleSensor'])
		
		# Obtener colores
		left_color = left_result[5] if left_result[0] > 0 else None
		right_color = right_result[5] if right_result[0] > 0 else None
		middle_color = middle_result[5] if middle_result[0] > 0 else None
		
		# Detectar linea roja
		left_red = is_red(left_color)
		right_red = is_red(right_color)
		middle_red = is_red(middle_color)
		line_detected = left_red or right_red or middle_red
		
		# Detectar bloqueo
		motors_active = state == SIGUIENDO_LINEA or state == BUSCANDO_LINEA
		if motors_active and movement < BLOCK_THRESHOLD:
			block_counter += 1
		else:
			block_counter = 0
		blocked = block_counter > BLOCK_CYCLES
		
		# Maquina de estados
		match state:
			case 0:  # SIGUIENDO_LINEA
				if blocked:
					print("Obstaculo detectado!")
					state = OBSTACULO_DETECTADO
					block_counter = 0
				elif middle_red and not left_red and not right_red:
					sim.setJointTargetVelocity(handles['leftMotor'], BASE_SPEED)
					sim.setJointTargetVelocity(handles['rightMotor'], BASE_SPEED)
				elif left_red and not right_red:
					sim.setJointTargetVelocity(handles['leftMotor'], BASE_SPEED * 0.5)
					sim.setJointTargetVelocity(handles['rightMotor'], BASE_SPEED)
				elif right_red and not left_red:
					sim.setJointTargetVelocity(handles['leftMotor'], BASE_SPEED)
					sim.setJointTargetVelocity(handles['rightMotor'], BASE_SPEED * 0.5)
				elif middle_red:
					sim.setJointTargetVelocity(handles['leftMotor'], BASE_SPEED)
					sim.setJointTargetVelocity(handles['rightMotor'], BASE_SPEED)
				else:
					state = BUSCANDO_LINEA
			
			case 1:  # BUSCANDO_LINEA
				if blocked:
					print("Obstaculo detectado!")
					state = OBSTACULO_DETECTADO
					block_counter = 0
				elif line_detected:
					print("Linea roja encontrada!")
					state = SIGUIENDO_LINEA
				else:
					sim.setJointTargetVelocity(handles['leftMotor'], TURN_SPEED)
					sim.setJointTargetVelocity(handles['rightMotor'], -TURN_SPEED)
			
			case 2:  # OBSTACULO_DETECTADO
				# TODO: Decidir que hacer aqui
				sim.setJointTargetVelocity(handles['leftMotor'], 0)
				sim.setJointTargetVelocity(handles['rightMotor'], 0)
				pass
			
			case _:
				state = BUSCANDO_LINEA
		
		last_position = current_position
		sim.step()
	
	sim.stopSimulation()

if __name__ == "__main__":
	main()