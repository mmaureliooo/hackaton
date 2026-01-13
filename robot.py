# Robot seguidor de linea roja con deteccion de obstaculos
# Detecta bloqueo si el robot no se mueve aunque los motores esten activos

from enum import IntEnum
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Estados del robot
class State(IntEnum):
	SIGUIENDO_LINEA = 0
	BUSCANDO_LINEA = 1
	OBSTACULO_DETECTADO = 2

# Configuracion de velocidades
BASE_SPEED = 2.0
TURN_SPEED = 1.5

# Deteccion de bloqueo: si motores comandan y no avanza
BLOCK_THRESHOLD = 0.001  # movimiento minimo por paso
BLOCK_CYCLES = 30        # pasos consecutivos sin moverse
SPIN_STEPS = 45          # pasos para girar ~180° (ajusta segun giro)

# Deteccion por intensidad (linea = mas oscuro)
LINE_THRESHOLD = 0.55  # ajusta segun escena

def get_intensity(sim, sensor_handle):
	"""Promedio de intensidad en una ROI inferior del sensor (mas oscuro = linea).
	Si falla la lectura, devolvemos 1.0 para forzar modo busqueda.
	"""
	try:
		img_bytes, res = sim.getVisionSensorImg(sensor_handle)
		if img_bytes is None or res[0] == 0 or res[1] == 0:
			return 1.0
		expected_size = res[0] * res[1] * 3
		img = np.frombuffer(img_bytes, dtype=np.uint8)
		if img.size != expected_size:
			return 1.0
		img = img.reshape(res[1], res[0], 3)
		# Tomar la parte baja (60%) para ver el suelo
		start_row = int(res[1] * 0.4)
		roi = img[start_row:, :, :]
		return float(roi.mean() / 255.0)
	except Exception:
		return 1.0

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
	
	state = State.BUSCANDO_LINEA
	last_position = sim.getObjectPosition(handles['robot'], -1)
	block_counter = 0
	spin_counter = 0
	cmd_left = 0.0
	cmd_right = 0.0
	step = 0
	
	while True:
		# Obtener posicion actual
		current_position = sim.getObjectPosition(handles['robot'], -1)
		
		# Calcular movimiento
		dx = current_position[0] - last_position[0]
		dy = current_position[1] - last_position[1]
		movement = (dx*dx + dy*dy) ** 0.5
		
		# Leer intensidades (promedio R+G+B)/3; linea = mas oscuro
		left_int = get_intensity(sim, handles['leftSensor'])
		right_int = get_intensity(sim, handles['rightSensor'])
		middle_int = get_intensity(sim, handles['middleSensor'])
		
		left_line = left_int < LINE_THRESHOLD
		right_line = right_int < LINE_THRESHOLD
		middle_line = middle_int < LINE_THRESHOLD
		line_detected = left_line or right_line or middle_line

		# Log rapido cada 20 ciclos para verificar deteccion
		if step % 20 == 0:
			print(f"L:{left_int:.3f} R:{right_int:.3f} M:{middle_int:.3f} | LL:{left_line} RL:{right_line} ML:{middle_line}")
		
		# Detectar bloqueo: motores mandan pero no hay desplazamiento
		cmd_active = abs(cmd_left) + abs(cmd_right) > 0.2
		if movement < BLOCK_THRESHOLD and cmd_active:
			block_counter += 1
		else:
			block_counter = 0
		blocked = block_counter >= BLOCK_CYCLES
		
		# Maquina de estados
		match state:
			case State.SIGUIENDO_LINEA:
				if blocked:
					print("Obstaculo detectado!")
					state = State.OBSTACULO_DETECTADO
					block_counter = 0
					spin_counter = SPIN_STEPS
				elif (middle_line and not left_line and not right_line) or (left_line and right_line):
					cmd_left = BASE_SPEED
					cmd_right = BASE_SPEED
					sim.setJointTargetVelocity(handles['leftMotor'], cmd_left)
					sim.setJointTargetVelocity(handles['rightMotor'], cmd_right)
				elif left_line and not right_line:
					cmd_left = 0.0
					cmd_right = BASE_SPEED * 1.5
					sim.setJointTargetVelocity(handles['leftMotor'], cmd_left)
					sim.setJointTargetVelocity(handles['rightMotor'], cmd_right)
				elif right_line and not left_line:
					cmd_left = BASE_SPEED * 1.5
					cmd_right = 0.0
					sim.setJointTargetVelocity(handles['leftMotor'], cmd_left)
					sim.setJointTargetVelocity(handles['rightMotor'], cmd_right)
				elif middle_line:
					cmd_left = BASE_SPEED
					cmd_right = BASE_SPEED
					sim.setJointTargetVelocity(handles['leftMotor'], cmd_left)
					sim.setJointTargetVelocity(handles['rightMotor'], cmd_right)
				else:
					state = State.BUSCANDO_LINEA
			
			case State.BUSCANDO_LINEA:
				if blocked:
					print("Obstaculo detectado!")
					state = State.OBSTACULO_DETECTADO
					block_counter = 0
					spin_counter = SPIN_STEPS
				elif line_detected:
					print("Linea encontrada!")
					state = State.SIGUIENDO_LINEA
				else:
					cmd_left = TURN_SPEED
					cmd_right = -TURN_SPEED
					sim.setJointTargetVelocity(handles['leftMotor'], cmd_left)
					sim.setJointTargetVelocity(handles['rightMotor'], cmd_right)
			
			case State.OBSTACULO_DETECTADO:
				# Girar ~180° y volver a seguir linea
				cmd_left = TURN_SPEED
				cmd_right = -TURN_SPEED
				sim.setJointTargetVelocity(handles['leftMotor'], cmd_left)
				sim.setJointTargetVelocity(handles['rightMotor'], cmd_right)
				spin_counter -= 1
				if spin_counter <= 0:
					state = State.SIGUIENDO_LINEA
			
			case _:
				state = State.BUSCANDO_LINEA
		
		last_position = current_position
		step += 1
		sim.step()
	
	sim.stopSimulation()

if __name__ == "__main__":
	main()
#este codigo funciona