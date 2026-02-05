#!/usr/bin/env python3
import sys
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# =============================
# Longitudes reales (m)
# =============================
L1 = 0.216          # brazo
L2 = 0.37           # antebrazo + centro del gripper

# Altura real del hombro respecto a base_link
SHOULDER_Z = 0.80 + 0.35 - 0.08   # = 1.07 m

# Nombre de los joints (URDF)
JOINT_NAMES = [
    'torso_shoulder',        # pitch (Y)
    'shoulder_arm_motor',    # yaw (Z)
    'elbow',                 # pitch (Y)
    'wrist'
]


def ik_planar(x, z):
    """
    Cinemática inversa planar (X-Z), codo abajo
    x, z referidos a base_link
    """

    # Coordenadas relativas al hombro
    dx = x
    dz = z - SHOULDER_Z

    r = math.sqrt(dx*dx + dz*dz)

    # Chequeo de alcance
    if r > (L1 + L2) or r < abs(L1 - L2):
        raise ValueError(f"Objetivo fuera de alcance: r={r:.3f} m")

    # =============================
    # Codo (elbow-down)
    # =============================
    cos_q2 = (r*r - L1*L1 - L2*L2) / (2*L1*L2)
    cos_q2 = max(min(cos_q2, 1.0), -1.0)
    q_elbow = math.acos(cos_q2)

    # =============================
    # Hombro
    #
    # q=0 apunta hacia -Z
    # =============================
    phi = math.atan2(dx, -dz)
    psi = math.atan2(
        L2 * math.sin(q_elbow),
        L1 + L2 * math.cos(q_elbow)
    )

    q_shoulder = phi - psi

    return q_shoulder, q_elbow


def ik_3d(x, y, z):
    """
    IK 3D:
    - yaw sobre Z
    - IK planar reutilizada
    """

    # =============================
    # Ángulo que define el plano
    # =============================
    q_yaw = math.atan2(y, x)

    # =============================
    # Proyección al plano X-Z
    # =============================
    x_planar = math.sqrt(x*x + y*y)
    z_planar = z

    # IK planar existente
    q_shoulder, q_elbow = ik_planar(x_planar, z_planar)

    return q_shoulder, q_yaw, q_elbow


class IKPublisher(Node):

    def __init__(self, goal):
        super().__init__('ik_joint_state_publisher')

        x, y, z = goal  # coordenadas absolutas en base_link

        try:
            q_shoulder, q_yaw, q_elbow = ik_3d(x, y, z)
        except ValueError as e:
            self.get_logger().error(str(e))
            rclpy.shutdown()
            sys.exit(1)

        # Posiciones articulares
        self.positions = [
            q_shoulder,
            q_yaw,
            q_elbow,
            0.0       # wrist (no usada)
        ]

        self.publisher = self.create_publisher(
            JointState, '/joint_states', 10
        )
        self.timer = self.create_timer(0.1, self.publish)

        self.get_logger().info(
            f"Objetivo base_link: x={x:.3f}, y={y:.3f}, z={z:.3f}"
        )
        self.get_logger().info(
            f"Solución (rad): shoulder={q_shoulder:.3f}, yaw={q_yaw:.3f}, elbow={q_elbow:.3f}"
        )

    def publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = self.positions
        self.publisher.publish(msg)


def main():
    rclpy.init()

    if len(sys.argv) == 4:
        goal = [
            float(sys.argv[1]),
            float(sys.argv[2]),
            float(sys.argv[3])
        ]
    else:
        goal = [0.40, -0.25, 0.95]
        print("Uso:")
        print("  ros2 run manipulation_planner ik_joint_state_publisher.py x y z")
        print("Usando objetivo por defecto:", goal)

    node = IKPublisher(goal)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

