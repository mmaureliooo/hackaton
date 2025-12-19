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
    
    v = 5
    dv = 8
    
    counter = 0
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
        
        # Velocidades base
        sim.setJointTargetVelocity(leftJoint, v)
        sim.setJointTargetVelocity(rightJoint, v)
        
        # Logica de seguimiento (igual que el Lua original)
        if right_on_line and not left_on_line:
            sim.setJointTargetVelocity(rightJoint, v + dv)
        if left_on_line and not right_on_line:
            sim.setJointTargetVelocity(leftJoint, v + dv)
        
        sim.step()

if __name__ == "__main__":
    main()