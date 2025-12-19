from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.require('sim')

# Umbral: linea ~0.75, suelo ~0.83
LINE_THRESHOLD = 0.80

def get_intensity(response):
    """Extrae la intensidad promedio (R+G+B)/3"""
    if isinstance(response, tuple) and response[0] == 1:
        data = response[1]
        r = data[11]
        g = data[12]
        b = data[13]
        return (r + g + b) / 3.0
    return 0.85  # Valor por defecto (suelo)

def main():
    sim.setStepping(True)
    sim.startSimulation()
    
    # Obtener handles
    leftJoint = sim.getObject('/LineTracer/DynamicLeftJoint')
    rightJoint = sim.getObject('/LineTracer/DynamicRightJoint')
    leftSensor = sim.getObject('/LineTracer/LeftSensor')
    rightSensor = sim.getObject('/LineTracer/RightSensor')
    
    print("=== ROBOT SEGUIDOR DE LINEA ===")
    print(f"Umbral: {LINE_THRESHOLD}")
    
    v = 3
    
    counter = 0
    last_turn = "right"
    
    while True:
        # Leer sensores
        response_left = sim.readVisionSensor(leftSensor)
        response_right = sim.readVisionSensor(rightSensor)
        
        # Extraer intensidades
        intensity_left = get_intensity(response_left)
        intensity_right = get_intensity(response_right)
        
        # Detectar linea (intensidad baja = linea)
        left_on_line = intensity_left < LINE_THRESHOLD
        right_on_line = intensity_right < LINE_THRESHOLD
        
        # Debug
        counter += 1
        if counter >= 15:
            print(f"L:{intensity_left:.3f}({left_on_line}) R:{intensity_right:.3f}({right_on_line})")
            counter = 0
        
        # Logica de seguimiento
        if left_on_line and right_on_line:
            # Ambos en linea -> recto
            sim.setJointTargetVelocity(leftJoint, v)
            sim.setJointTargetVelocity(rightJoint, v)
        elif left_on_line:
            # Linea a la izquierda -> girar izquierda (frenar izquierda, acelerar derecha)
            sim.setJointTargetVelocity(leftJoint, v * 0.2)
            sim.setJointTargetVelocity(rightJoint, v * 2)
            last_turn = "left"
        elif right_on_line:
            # Linea a la derecha -> girar derecha (acelerar izquierda, frenar derecha)
            sim.setJointTargetVelocity(leftJoint, v * 2)
            sim.setJointTargetVelocity(rightJoint, v * 0.2)
            last_turn = "right"
        else:
            # Perdio la linea -> buscar en la ultima direccion
            if last_turn == "left":
                sim.setJointTargetVelocity(leftJoint, -v)
                sim.setJointTargetVelocity(rightJoint, v)
            else:
                sim.setJointTargetVelocity(leftJoint, v)
                sim.setJointTargetVelocity(rightJoint, -v)
        
        sim.step()

if __name__ == "__main__":
    main()