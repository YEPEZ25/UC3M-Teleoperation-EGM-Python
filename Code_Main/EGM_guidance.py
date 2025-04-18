import egm_pb2 as egm
import socket

# IP de la computadora (localhost para simulación)
computer_ip = "127.0.0.1"
robot_port = 6510
num = 0

# Crear socket UDP
robot_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
robot_socket.bind((computer_ip, robot_port))

def CreateSensorMessage(seq_number, pos, quat):
    """
    Función para crear el mensaje de corrección de posición
    """
    egmSensor = egm.EgmSensor()

    # Cabecera del mensaje
    header = egmSensor.header
    header.seqno = seq_number
    header.mtype = egm.EgmHeader.MessageType.MSGTYPE_CORRECTION

    # Datos de corrección de pose
    planned = egmSensor.planned
    pose = planned.cartesian
    position = pose.pos
    orientation = pose.orient

    position.x, position.y, position.z = pos
    orientation.u0, orientation.u1, orientation.u2, orientation.u3 = quat

    return egmSensor


print(f"[INFO] EGM Guidance activo en {computer_ip}:{robot_port}")

while True:
    data, addr = robot_socket.recvfrom(1024)

    # Leer mensaje recibido desde el robot
    message = egm.EgmRobot()
    message.ParseFromString(data)

    seq = message.header.seqno
    time = message.header.tm
    curX = message.feedBack.cartesian.pos.x
    curY = message.feedBack.cartesian.pos.y
    curZ = message.feedBack.cartesian.pos.z

    print(f"SeqNum={seq}, Time={time}, X={curX:.2f}, Y={curY:.2f}, Z={curZ:.2f}")

    # Definir posición y orientación deseadas (evitar singularidad)
    pos = [600.0, 0.0, 500.0]     # [x, y, z] en mm

    # Cuaternión modificado para evitar singularidad (roll 0, pitch 90, yaw 0)
    # Cuaternión representando una rotación de 90 grados alrededor del eje Y
    quat = [0.7071, 0.7071, 0.0, 0.0] #(esto corresponde a una rotación de 90 grados sobre el eje Y)
    # Crear mensaje EGM guidance
    sensor_msg = CreateSensorMessage(num, pos, quat)

    # Serializar y enviar al robot
    serialized_msg = sensor_msg.SerializeToString()
    robot_socket.sendto(serialized_msg, addr)

    num += 1