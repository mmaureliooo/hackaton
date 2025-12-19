from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Estados del robot
SIGUIENDO_LINEA = 0
BUSCANDO_LINEA = 1
ESQUIVANDO_OBSTACULO = 2
RODEANDO_OBSTACULO = 3

# Configuracion de velocidades
#siempre va a tardar un poco mas en girar para que pille bien los sensores
BASE_SPEED = 2.0
TURN_SPEED = 1.5

def main():
    client = RemoteAPIClient()
    sim = client.require('sim')

    sim.setStepping(True)
    sim.startSimulation()

    # Obtener handles del robot - motores
    leftMotor = sim.getObject('/LineTracer/DynamicLeftJoint')
    rightMotor = sim.getObject('/LineTracer/DynamicRightJoint')
    
    # Sensores de proximidad para detectar la linea
    leftSensor = sim.getObject('/LineTracer/LeftSensor')
    rightSensor = sim.getObject('/LineTracer/RightSensor')
    middleSensor = sim.getObject('/LineTracer/MiddleSensor')
    
    state = BUSCANDO_LINEA
    esquive_counter = 0
    rodeo_counter = 0

    sim.addLog(sim.verbosity_scriptinfos, "Iniciando robot seguidor de linea con esquive de obstaculos")

    while True:
        # Leer sensores de proximidad
        left_result = sim.readProximitySensor(leftSensor)
        right_result = sim.readProximitySensor(rightSensor)
        middle_result = sim.readProximitySensor(middleSensor)
        
        # Detectar linea (sensor detecta algo = linea debajo)
        left_line = left_result[0] > 0
        right_line = right_result[0] > 0
        middle_line = middle_result[0] > 0
        
        # Distancia del sensor central (si detecta algo)
        middle_distance = middle_result[1] if middle_result[0] > 0 else float('inf')
        
        # Detectar obstaculo (objeto muy cerca en el sensor central)
        obstacle_detected = middle_result[0] > 0 and middle_distance < 0.05 #por 0.05 por la distancia al objeto
        
        line_detected = left_line or right_line or (middle_line and not obstacle_detected)
        
        # Maquina de estados
        match state:
            case 0:  # SIGUIENDO_LINEA
                if obstacle_detected:
                    sim.addLog(sim.verbosity_scriptinfos, "Obstaculo detectado! Iniciando esquive")
                    state = ESQUIVANDO_OBSTACULO
                    esquive_counter = 0
                elif middle_line and not left_line and not right_line:
                    # Linea centrada, ir recto
                    sim.setJointTargetVelocity(leftMotor, BASE_SPEED)
                    sim.setJointTargetVelocity(rightMotor, BASE_SPEED)
                elif left_line and not right_line:
                    # Linea a la izquierda, girar izquierda
                    sim.setJointTargetVelocity(leftMotor, BASE_SPEED * 0.5) #el 0.5 es para que el giro no sea ni muy abierto ni muy cerrado, cuanto mas pequeño el valo, mas cerrado el angulo de giro
                    sim.setJointTargetVelocity(rightMotor, BASE_SPEED)
                elif right_line and not left_line:
                    # Linea a la derecha, girar derecha
                    sim.setJointTargetVelocity(leftMotor, BASE_SPEED)
                    sim.setJointTargetVelocity(rightMotor, BASE_SPEED * 0.5) #el 0.5 es para que el giro no sea ni muy abierto ni muy cerrado, cuanto mas pequeño el valo, mas cerrado el angulo de giro
                elif middle_line:
                    # Linea detectada en el centro
                    sim.setJointTargetVelocity(leftMotor, BASE_SPEED)
                    sim.setJointTargetVelocity(rightMotor, BASE_SPEED)
                else:
                    # Perdimos la linea
                    state = BUSCANDO_LINEA
            
            case 1:  # BUSCANDO_LINEA
                if obstacle_detected:
                    state = ESQUIVANDO_OBSTACULO
                    esquive_counter = 0
                elif line_detected:
                    sim.addLog(sim.verbosity_scriptinfos, "Linea encontrada!")
                    state = SIGUIENDO_LINEA
                else:
                    # Girar para buscar la linea
                    sim.setJointTargetVelocity(leftMotor, TURN_SPEED)
                    sim.setJointTargetVelocity(rightMotor, -TURN_SPEED)
            
            case 2:  # ESQUIVANDO_OBSTACULO
                # Girar a la derecha para esquivar
                sim.setJointTargetVelocity(leftMotor, TURN_SPEED)
                sim.setJointTargetVelocity(rightMotor, -TURN_SPEED * 0.5) #el 0.5 es para que el giro no sea ni muy abierto ni muy cerrado, cuanto mas pequeño el valo, mas cerrado el angulo de giro
                esquive_counter += 1
                
                if esquive_counter > 30:
                    state = RODEANDO_OBSTACULO
                    rodeo_counter = 0
            
            case 3:  # RODEANDO_OBSTACULO
                # Avanzar rodeando el obstaculo
                sim.setJointTargetVelocity(leftMotor, BASE_SPEED)
                sim.setJointTargetVelocity(rightMotor, BASE_SPEED * 0.5) #el 0.5 es para que el giro no sea ni muy abierto ni muy cerrado, cuanto mas pequeño el valo, mas cerrado el angulo de giro
                rodeo_counter += 1
                
                if line_detected and rodeo_counter > 20: #20 porque es valor sacado de pruebas y errores del robot
                    sim.addLog(sim.verbosity_scriptinfos, "Linea recuperada despues del esquive!")
                    state = SIGUIENDO_LINEA
                elif rodeo_counter > 100: #100 porque es valor sacado de pruebas y errores del robot, y tiene que ser mayor que el 20 porque hace mas código y necesita más tiempo para hacer las cosas
                    state = BUSCANDO_LINEA
            
            case _:
                state = BUSCANDO_LINEA
        
        sim.step()

if __name__ == "__main__":
    main()