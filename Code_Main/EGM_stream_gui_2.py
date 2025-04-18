import egm_pb2 as egm
import socket
import math
import threading
import time

# Puertos
stream_port = 6511     # EGM Position Stream
guidance_port = 6510   # EGM Position Guidance
computer_ip = "127.0.0.1"

# Variable global para almacenar el feedback más reciente
latest_feedback = None

def quaternion_to_euler(qw, qx, qy, qz):
    # Conversión de cuaternión a ángulos de Euler (Z-Y-X)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    rx = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (qw * qy - qz * qx)
    ry = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    rz = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(rx), math.degrees(ry), math.degrees(rz)

def listen_stream():
    global latest_feedback
    stream_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    stream_socket.bind((computer_ip, stream_port))
    print(f"[INFO] Escuchando feedback en {computer_ip}:{stream_port}")

    while True:
        data, _ = stream_socket.recvfrom(1024)
        message = egm.EgmRobot()
        message.ParseFromString(data)
        latest_feedback = message.feedBack
        
        if latest_feedback is None:
            time.sleep(0.01)
            continue

        # Extraer posición y orientación
        pos_fb = latest_feedback.cartesian.pos
        ori_fb = latest_feedback.cartesian.orient
        joints = latest_feedback.joints.joints
        #j5 = joints[4]

        # Convertir cuaternión a euler
        rx, ry, rz = quaternion_to_euler(ori_fb.u0, ori_fb.u1, ori_fb.u2, ori_fb.u3)

        # Imprimir datos
        #print(f"\n[FEEDBACK]")
        print(f"Posición  [mm] : x={pos_fb.x:.1f}, y={pos_fb.y:.1f}, z={pos_fb.z:.1f}")
        #print(f"Orientación [°]: rx={rx:.1f}, ry={ry:.1f}, rz={rz:.1f}")
        #print(f"Joints    [°] : J1={joints[0]:.1f}, J2={joints[1]:.1f}, J3={joints[2]:.1f}, " +
        #      f"J4={joints[3]:.1f}, J5={joints[4]:.1f}, J6={joints[5]:.1f}")

def CreateSensorMessage(seq_number, pos, quat):
    egmSensor = egm.EgmSensor()
    header = egmSensor.header
    header.seqno = seq_number
    header.mtype = egm.EgmHeader.MessageType.MSGTYPE_CORRECTION

    planned = egmSensor.planned
    pose = planned.cartesian
    position = pose.pos
    orientation = pose.orient

    position.x, position.y, position.z = pos
    orientation.u0, orientation.u1, orientation.u2, orientation.u3 = quat

    return egmSensor

# Iniciar hilo para escuchar el feedback del robot
threading.Thread(target=listen_stream, daemon=True).start()

# Socket para enviar corrección de posición
guidance_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
guidance_socket.bind((computer_ip, guidance_port))
print(f"[INFO] Enviando correcciones desde {computer_ip}:{guidance_port}")

seq = 0

try:
    while True:
        if latest_feedback is None:
            time.sleep(0.01)
            continue

        joints = latest_feedback.joints.joints
        j5 = joints[4]

        # Verificamos si está en una singularidad (por ejemplo, J5 ≈ 0° o ≈ 180°)
        #if abs(j5) < 5.0 or abs(abs(j5) - 180.0) < 5.0:
        #    print(f"[WARN] Singularidad detectada en J5 = {j5:.2f}° -> no se envía comando")
        #    time.sleep(0.1)
        #    continue

        # Posición deseada
        pos = [600.0, 0.0, 500.0]  # en mm
        quat = [0.7071, 0.7071, 0.0, 0.0]  # 90° rotación sobre Y

        # Crear y enviar mensaje
        sensor_msg = CreateSensorMessage(seq, pos, quat)
        serialized_msg = sensor_msg.SerializeToString()
        guidance_socket.sendto(serialized_msg, ("127.0.0.1", 6510))
        print(f"[INFO] Enviado comando: Seq={seq}, Pos={pos}, Quat={quat}")

        #print(f"[OK] Comando enviado. J5 = {j5:.2f}°")
        seq += 1
        time.sleep(0.02)  # Tasa de envío 50 Hz

except KeyboardInterrupt:
    print("\n[INFO] Interrumpido por el usuario. Cerrando sockets.")
    guidance_socket.close()
