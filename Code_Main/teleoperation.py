import socket
import time
from egm_pb2 import EgmSensor, EgmRobot, EgmHeader
from scipy.spatial.transform import Rotation as R

# Configuración
UDP_IP = "127.0.0.1"
UDP_PORT = 6510
TOLERANCE = 1.0  # mm
RATE = 0.004  # 4 ms

# Lista de objetivos [x, y, z, rx, ry, rz]
targets = [
    [500, 0, 500, -180, 0, 180],
    [500, 100, 500, -180, 0, 180],
    [400, 100, 600, -180, 0, 180],
    [300, 0, 700, -180, 0, 180]
]

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

# Crear socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(0.1)

print(">> Cliente EGM iniciado")

seq = 1
current_target = 0
addr = None

while current_target < len(targets):
    try:
        data, addr = sock.recvfrom(1024)
        actual_pose = get_robot_pose(data)
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