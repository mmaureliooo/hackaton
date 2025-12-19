from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import cv2 as cv
import numpy as np

# Estados del robot
SIGUIENDO_LINEA = 0
BUSCANDO_LINEA = 1
ESQUIVANDO_OBSTACULO = 2
RODEANDO_OBSTACULO = 3

# Configuracion de velocidades
BASE_SPEED = 2.0
TURN_SPEED = 1.5

def detectar_linea_roja(image, resolution):
    """Detecta la linea roja y devuelve su posicion (-1 izq, 0 centro, 1 der)"""
    if image is None:
        return None
    
    hsv = cv.cvtColor(image, cv.COLOR_RGB2HSV)
    
    # Mascara para rojo (dos rangos en HSV)
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
    
    mask_moment = cv.moments(bottom_section)
    
    if mask_moment["m00"] > 0:
        center_x = int(mask_moment["m10"] / mask_moment["m00"])
        relative_pos = (center_x - resolution[0] / 2) / (resolution[0] / 2)
        return relative_pos
    
    return None

def detectar_obstaculo(image, resolution):
    """Detecta si hay un obstaculo (objeto no rojo) en el camino"""
    if image is None:
        return False
    
    hsv = cv.cvtColor(image, cv.COLOR_RGB2HSV)
    
    # Detectar objetos oscuros/negros como obstaculos
    lower_obstacle = np.array([0, 0, 0])
    upper_obstacle = np.array([180, 255, 80])
    mask = cv.inRange(hsv, lower_obstacle, upper_obstacle)
    
    # Solo miramos la parte central-superior (donde estaria el obstaculo)
    height = resolution[1]
    width = resolution[0]
    center_section = mask[0:height//2, width//4:3*width//4]
    
    obstacle_pixels = cv.countNonZero(center_section)
    return obstacle_pixels > 500

def main():
    client = RemoteAPIClient()
    sim = client.require('sim')

    sim.setStepping(True)
    sim.startSimulation()

    # Obtener handles del robot
    leftMotor = sim.getObject('/LineTracer/DynamicLeftJoint')
    rightMotor = sim.getObject('/LineTracer/DynamicRightJoint')
    
    # Intentar obtener camara
    try:
        camera = sim.getObject('/LineTracer/camera')
    except:
        camera = sim.getObject('/LineTracer/Vision_sensor')
    
    state = BUSCANDO_LINEA
    esquive_counter = 0
    rodeo_counter = 0

    sim.addLog(sim.verbosity_scriptinfos, "Iniciando robot seguidor de linea con esquive de obstaculos")

    while True:
        # Leer imagen de la camara
        try:
            raw_image, resolution = sim.getVisionSensorImg(camera)
            image = np.frombuffer(raw_image, dtype=np.uint8)
            image = image.reshape([resolution[1], resolution[0], 3])
            image = np.rot90(image, 2)
            image = np.fliplr(image)
        except:
            sim.step()
            continue
        
        # Detectar linea y obstaculo
        line_position = detectar_linea_roja(image, resolution)
        line_detected = line_position is not None
        obstacle_detected = detectar_obstaculo(image, resolution)
        
        # Maquina de estados
        match state:
            case 0:  # SIGUIENDO_LINEA
                if obstacle_detected:
                    sim.addLog(sim.verbosity_scriptinfos, "Obstaculo detectado! Iniciando esquive")
                    state = ESQUIVANDO_OBSTACULO
                    esquive_counter = 0
                elif line_detected:
                    # Seguir la linea
                    if abs(line_position) < 0.2:
                        # Linea centrada, ir recto
                        sim.setJointTargetVelocity(leftMotor, BASE_SPEED)
                        sim.setJointTargetVelocity(rightMotor, BASE_SPEED)
                    elif line_position < 0:
                        # Linea a la izquierda
                        correction = abs(line_position) * BASE_SPEED
                        sim.setJointTargetVelocity(leftMotor, BASE_SPEED - correction)
                        sim.setJointTargetVelocity(rightMotor, BASE_SPEED)
                    else:
                        # Linea a la derecha
                        correction = abs(line_position) * BASE_SPEED
                        sim.setJointTargetVelocity(leftMotor, BASE_SPEED)
                        sim.setJointTargetVelocity(rightMotor, BASE_SPEED - correction)
                else:
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
                sim.setJointTargetVelocity(rightMotor, -TURN_SPEED * 0.5)
                esquive_counter += 1
                
                if esquive_counter > 30:  # Girar por un tiempo
                    state = RODEANDO_OBSTACULO
                    rodeo_counter = 0
            
            case 3:  # RODEANDO_OBSTACULO
                # Avanzar rodeando el obstaculo
                sim.setJointTargetVelocity(leftMotor, BASE_SPEED)
                sim.setJointTargetVelocity(rightMotor, BASE_SPEED * 0.7)
                rodeo_counter += 1
                
                if line_detected and rodeo_counter > 20:
                    sim.addLog(sim.verbosity_scriptinfos, "Linea recuperada despues del esquive!")
                    state = SIGUIENDO_LINEA
                elif rodeo_counter > 100:
                    # Si no encontramos la linea, buscarla
                    state = BUSCANDO_LINEA
            
            case _:
                state = BUSCANDO_LINEA
        
        sim.step()

    sim.stopSimulation()

if __name__ == "__main__":
    main()