import socket
import time
from egm_pb2 import EgmSensor, EgmRobot, EgmHeader
from scipy.spatial.transform import Rotation as R

# Configuración
UDP_IP = "127.0.0.1"
UDP_PORT = 6510
TOLERANCE = 1.0  # mm
JOINT_TOL = 3.0  # grados de tolerancia para la posición de seguridad
RATE = 0.004  # 4 ms

# Lista de objetivos [x, y, z, rx, ry, rz]
targets = [
    [500, 0, 500, -180, 0, 180],
    [500, 100, 500, -180, 0, 180],
    [400, 100, 600, -180, 0, 180],
    [300, 0, 700, -180, 0, 180]
]

# Posición segura de las articulaciones [J1, J2, J3, J4, J5, J6]
safe_joint_position = [0, 0, 0, 0, 90, 90]

"""
def euler_deg_to_quaternion(rx, ry, rz):
    rx, ry, rz = [math.radians(a) for a in (rx, ry, rz)]

    cx = math.cos(rx / 2)
    sx = math.sin(rx / 2)
    cy = math.cos(ry / 2)
    sy = math.sin(ry / 2)
    cz = math.cos(rz / 2)
    sz = math.sin(rz / 2)

    q0 = cx * cy * cz + sx * sy * sz
    q1 = sx * cy * cz - cx * sy * sz
    q2 = cx * sy * cz + sx * cy * sz
    q3 = cx * cy * sz - sx * sy * cz

    return [q0, q1, q2, q3]
"""

def euler_to_quaternion(rx_deg, ry_deg, rz_deg):
    r = R.from_euler('xyz', [rx_deg, ry_deg, rz_deg], degrees=True)
    q = r.as_quat()  # retorna [x, y, z, w]
    return [q[3], q[0], q[1], q[2]]  # [u0, u1, u2, u3] formato EGM

def pose_close(p1, p2, tol=TOLERANCE):
    return all(abs(a - b) < tol for a, b in zip(p1, p2))

def joints_close(joints, safe_joints, tol=JOINT_TOL):
    return all(abs(j - s) < tol for j, s in zip(joints, safe_joints))

def build_egm_sensor_message(seq, pose):
    msg = EgmSensor()
    msg.header.seqno = seq
    msg.header.tm = seq  # usar el número de secuencia como timestamp
    msg.header.mtype = EgmHeader.MSGTYPE_CORRECTION

    msg.planned.cartesian.pos.x = pose[0]
    msg.planned.cartesian.pos.y = pose[1]
    msg.planned.cartesian.pos.z = pose[2]

    q = euler_to_quaternion(pose[3], pose[4], pose[5])
    msg.planned.cartesian.orient.u0 = q[0]
    msg.planned.cartesian.orient.u1 = q[1]
    msg.planned.cartesian.orient.u2 = q[2]
    msg.planned.cartesian.orient.u3 = q[3]

    return msg.SerializeToString()

def get_robot_pose(data):
    msg = EgmRobot()
    msg.ParseFromString(data)
    p = msg.feedBack.cartesian.pos
    o = msg.feedBack.cartesian.orient
    return [p.x, p.y, p.z, o.u0, o.u1, o.u2, o.u3]

def get_robot_joints(data):
    msg = EgmRobot()
    msg.ParseFromString(data)
    joints = msg.feedBack.joints.joints  # lista de 6 articulaciones
    return joints

# Crear socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(0.1)

print(">> Cliente EGM iniciado")


seq = 1
current_target = 0
waiting = False
pose_sent = False
last_pose = None
first_position_sent = False

while current_target < len(targets):
    try:
        data, addr = sock.recvfrom(1024)
        actual_pose = get_robot_pose(data)
        joints = get_robot_joints(data)

        # Mostrar la posición actual del robot
        #print(f"Posición actual → X: {actual_pose[0]:.1f}, Y: {actual_pose[1]:.1f}, Z: {actual_pose[2]:.1f}, "
        #      f"RX: {actual_pose[4]:.3f}, RY: {actual_pose[5]:.3f}, RZ: {actual_pose[6]:.3f}")

        # Seguridad: Solo antes de enviar la primera posición
        if not first_position_sent and current_target == 0:
            if not joints_close(joints, safe_joint_position):
                #print("⛔ Articulaciones fuera de la posición segura. Esperando...")
                print(f"[JOINTS] J1-J6: {[round(j, 2) for j in joints]}")
                #print(f"[POSE] XYZ: {actual_pose[:3]}")
                continue
            else:
                print("✔ Posición segura verificada. Enviando primer objetivo.")
                first_position_sent = True        

        target = targets[current_target]

        # Enviar posición objetivo continuamente
        pose_data = build_egm_sensor_message(seq, target)
        sock.sendto(pose_data, addr)
        seq += 1

        # Verificar si se alcanzó la posición
        if pose_close(actual_pose[:3], target[:3]):
            print(f"✔ Objetivo #{current_target + 1} alcanzado: {actual_pose[:3]}")
            current_target += 1
            time.sleep(0.5)  # pequeña pausa antes del siguiente target

        time.sleep(RATE)

    except socket.timeout:
        continue
    except KeyboardInterrupt:
        print(">> Cliente EGM detenido por el usuario")
        break