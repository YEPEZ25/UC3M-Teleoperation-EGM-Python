import egm_pb2 as egm
import socket
import math

computer_ip = "127.0.0.1"  # Para simulación en RobotStudio
robot_port = 6511
num = 0

robot_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
robot_socket.bind((computer_ip, robot_port))

print(f"Listening for joint feedback on {computer_ip}:{robot_port}")

# Convertir a ángulos de Euler (en grados)
# Asumiendo convención Tait-Bryan (Z-Y-X)
def quaternion_to_euler(qw, qx, qy, qz):
    # Roll (Rx)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    rx = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (Ry)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        ry = math.copysign(math.pi / 2, sinp)
    else:
        ry = math.asin(sinp)

    # Yaw (Rz)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    rz = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(rx), math.degrees(ry), math.degrees(rz)

try:
    while True:
        data, addr = robot_socket.recvfrom(1024)
        #print(f"Received message from {addr}")

        message = egm.EgmRobot()
        message.ParseFromString(data)

        Seq = message.header.seqno
        Time = message.header.tm

        # ✅ ACCESO CORRECTO A LAS ARTICULACIONES
        joints = message.feedBack.joints.joints
        CurX=message.feedBack.cartesian.pos.x
        CurY=message.feedBack.cartesian.pos.y
        CurZ=message.feedBack.cartesian.pos.z
        q = message.feedBack.cartesian.orient
        qw, qx, qy, qz = q.u0, q.u1, q.u2, q.u3
        
        rx, ry, rz = quaternion_to_euler(qw, qx, qy, qz)

        #print(f"SeqNum={Seq}, Time={Time}")
        #for i, joint_angle in enumerate(joints[:6]):
        #    print(f"  J{i+1} = {joint_angle:.2f}°")
        print(f"J1: {joints[0]:.2f}° J2: {joints[1]:.2f}° J3: {joints[2]:.2f}° J4: {joints[3]:.2f}° J5: {joints[4]:.2f}° J6: {joints[5]:.2f}° X: {CurX:.2f} Y: {CurY:.2f} Z: {CurZ:.2f} RX: {rx:.2f} RY: {ry:.2f} RZ: {rz:.2f}")
        #num += 1

except KeyboardInterrupt:
    print("\nPrograma interrumpido por el usuario.")
    robot_socket.close()
    print("Socket cerrado. Salida limpia.")