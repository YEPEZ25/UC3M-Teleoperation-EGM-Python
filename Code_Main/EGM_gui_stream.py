import socket
import time
import math
import multiprocessing
import egm_pb2 as egm

ROBOT_IP = "127.0.0.1"
PORT_SEND = 6510  # Guidance
PORT_RECV = 6511  # Feedback stream

# ==========================
# UTILS
# ==========================

def euler_to_quaternion(rx, ry, rz):
    rx, ry, rz = map(math.radians, [rx, ry, rz])
    cy = math.cos(rz * 0.5)
    sy = math.sin(rz * 0.5)
    cp = math.cos(ry * 0.5)
    sp = math.sin(ry * 0.5)
    cr = math.cos(rx * 0.5)
    sr = math.sin(rx * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qw, qx, qy, qz

def quaternion_to_euler(qw, qx, qy, qz):
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    rx = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (qw * qy - qz * qx)
    ry = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    rz = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(rx), math.degrees(ry), math.degrees(rz)

# ==========================
# FEEDBACK PROCESS (Listener)
# ==========================

def feedback_listener(current_pose):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ROBOT_IP, PORT_RECV))

    while True:
        data, _ = sock.recvfrom(1024)
        message = egm.EgmRobot()
        message.ParseFromString(data)

        joints = message.feedBack.joints.joints
        pos = message.feedBack.cartesian.pos
        orient = message.feedBack.cartesian.orient
        rx, ry, rz = quaternion_to_euler(orient.u0, orient.u1, orient.u2, orient.u3)

        current_pose['x'] = pos.x
        current_pose['y'] = pos.y
        current_pose['z'] = pos.z
        current_pose['rx'] = rx
        current_pose['ry'] = ry
        current_pose['rz'] = rz

# ==========================
# MAIN CONTROL LOOP
# ==========================

def main():
    # Lista de movimientos (X, Y, Z, Rx, Ry, Rz)
    waypoints = [
        (200,-100, 400, 0, 0, 0),
        (300,-100, 400, 0, 0, 30),
        (300, 100, 400, 0, 30, 0),
        (200, 100, 400, 30, 0, 0)
    ]

    # Estado compartido de posición
    manager = multiprocessing.Manager()
    current_pose = manager.dict(x=0, y=0, z=0, rx=0, ry=0, rz=0)

    # Iniciar escucha en proceso separado
    listener = multiprocessing.Process(target=feedback_listener, args=(current_pose,))
    listener.start()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("Iniciando movimientos guiados por EGM...")
    for i, (x_des, y_des, z_des, rx_des, ry_des, rz_des) in enumerate(waypoints):
        print(f"→ Enviando destino {i+1}")
        reached = False
        while not reached:
            # Crear y enviar paquete
            msg = egm.EgmSensor()
            msg.header.tm = int(time.time() * 1000) % (2**32)
            msg.header.seqno = i + 1

            cmd = msg.planned  # <- CAMBIO AQUÍ

            cmd.cartesian.pos.x = x_des
            cmd.cartesian.pos.y = y_des
            cmd.cartesian.pos.z = z_des

            qw, qx, qy, qz = euler_to_quaternion(rx_des, ry_des, rz_des)
            cmd.cartesian.orient.u0 = qw
            cmd.cartesian.orient.u1 = qx
            cmd.cartesian.orient.u2 = qy
            cmd.cartesian.orient.u3 = qz

            sock.sendto(msg.SerializeToString(), (ROBOT_IP, PORT_SEND))

            # Comprobar si llegó (tolerancia)
            tol_pos = 2.0  # mm
            tol_ang = 2.0  # grados
            err = (
                abs(current_pose['x'] - x_des) < tol_pos and
                abs(current_pose['y'] - y_des) < tol_pos and
                abs(current_pose['z'] - z_des) < tol_pos and
                abs(current_pose['rx'] - rx_des) < tol_ang and
                abs(current_pose['ry'] - ry_des) < tol_ang and
                abs(current_pose['rz'] - rz_des) < tol_ang
            )
            if err:
                print(f"✓ Posición {i+1} alcanzada.")
                time.sleep(1)
                reached = True

            time.sleep(0.01)  # Espera 10ms

    print("✔ Todos los movimientos completados.")
    listener.terminate()
    sock.close()

# ==========================
# EJECUCIÓN
# ==========================

if __name__ == "__main__":
    main()
