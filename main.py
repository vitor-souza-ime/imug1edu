import os
import sys
import threading
import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# ROS2 / UNITREE ENVIRONMENT
# ============================================================

os.environ.setdefault(
    'AMENT_PREFIX_PATH',
    '/home/u7/unitree_ros2/install/unitree_hg:'
    '/home/u7/unitree_ros2/install/unitree_go:'
    '/home/u7/unitree_ros2/install/unitree_api:'
    '/opt/ros/jazzy'
)

os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')

sys.path.insert(0, '/opt/ros/jazzy/lib/python3.12/site-packages')
sys.path.insert(0, '/home/u7/unitree_ros2/install/unitree_hg/local/lib/python3.12/dist-packages')

# ============================================================
# ROS2 IMPORTS
# ============================================================

import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy

from unitree_hg.msg import LowState

# ============================================================
# ROS2 NODE
# ============================================================

class PoseNode(Node):

    def __init__(self):

        super().__init__('g1_pose_estimator')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            LowState,
            '/lowstate',
            self.callback,
            qos
        )

        self.latest_msg = None
        self.lock = threading.Lock()

        self.message_count = 0

        self.get_logger().info("G1 Pose Estimator Started")

    def callback(self, msg):

        with self.lock:

            self.latest_msg = msg
            self.message_count += 1

# ============================================================
# SKELETON MODEL
# ============================================================

class SkeletonModel:

    def __init__(self):

        # Approximate body dimensions (meters)

        self.thigh = 0.35
        self.shin  = 0.35
        self.trunk = 0.45

        self.upper_arm = 0.25
        self.forearm   = 0.25

        self.hip_offset = 0.08
        self.shoulder_offset = 0.18

    # ========================================================
    # SAGITTAL VIEW
    # ========================================================

    def sagittal(self, msg):

        motors = msg.motor_state

        pitch = msg.imu_state.rpy[1]

        pelvis = np.array([0.0, 0.8])

        torso_top = pelvis + np.array([
            -self.trunk * np.sin(pitch),
             self.trunk * np.cos(pitch)
        ])

        # LEFT LEG

        hip_l  = motors[2].q
        knee_l = motors[3].q

        knee_left = pelvis + np.array([
            self.thigh * np.sin(hip_l),
           -self.thigh * np.cos(hip_l)
        ])

        ankle_left = knee_left + np.array([
            self.shin * np.sin(hip_l + knee_l),
           -self.shin * np.cos(hip_l + knee_l)
        ])

        # RIGHT LEG

        hip_r  = motors[8].q
        knee_r = motors[9].q

        knee_right = pelvis + np.array([
            self.thigh * np.sin(hip_r),
           -self.thigh * np.cos(hip_r)
        ])

        ankle_right = knee_right + np.array([
            self.shin * np.sin(hip_r + knee_r),
           -self.shin * np.cos(hip_r + knee_r)
        ])

        # LEFT ARM

        sh_l = motors[15].q
        el_l = motors[17].q

        elbow_l = torso_top + np.array([
            -self.upper_arm * np.sin(sh_l),
            -self.upper_arm * np.cos(sh_l)
        ])

        hand_l = elbow_l + np.array([
            -self.forearm * np.sin(sh_l + el_l),
            -self.forearm * np.cos(sh_l + el_l)
        ])

        # RIGHT ARM

        sh_r = motors[22].q
        el_r = motors[24].q

        elbow_r = torso_top + np.array([
            -self.upper_arm * np.sin(sh_r),
            -self.upper_arm * np.cos(sh_r)
        ])

        hand_r = elbow_r + np.array([
            -self.forearm * np.sin(sh_r + el_r),
            -self.forearm * np.cos(sh_r + el_r)
        ])

        return {

            "pelvis": pelvis,
            "torso": torso_top,

            "knee_left": knee_left,
            "ankle_left": ankle_left,

            "knee_right": knee_right,
            "ankle_right": ankle_right,

            "elbow_l": elbow_l,
            "hand_l": hand_l,

            "elbow_r": elbow_r,
            "hand_r": hand_r
        }

    # ========================================================
    # CORONAL VIEW
    # ========================================================

    def coronal(self, msg):

        motors = msg.motor_state

        roll = msg.imu_state.rpy[0]

        pelvis_center = np.array([0.0, 0.8])

        torso_top = pelvis_center + np.array([
            self.trunk * np.sin(roll),
            self.trunk * np.cos(roll)
        ])

        hip_left  = pelvis_center + np.array([-self.hip_offset, 0])
        hip_right = pelvis_center + np.array([ self.hip_offset, 0])

        # LEFT LEG

        hip_roll_l = motors[1].q

        knee_left = hip_left + np.array([
            -self.thigh * np.sin(hip_roll_l),
            -self.thigh * np.cos(hip_roll_l)
        ])

        ankle_left = knee_left + np.array([
            -self.shin * np.sin(hip_roll_l),
            -self.shin * np.cos(hip_roll_l)
        ])

        # RIGHT LEG

        hip_roll_r = motors[7].q

        knee_right = hip_right + np.array([
             self.thigh * np.sin(hip_roll_r),
            -self.thigh * np.cos(hip_roll_r)
        ])

        ankle_right = knee_right + np.array([
             self.shin * np.sin(hip_roll_r),
            -self.shin * np.cos(hip_roll_r)
        ])

        # SHOULDERS

        shoulder_left  = torso_top + np.array([-self.shoulder_offset, 0])
        shoulder_right = torso_top + np.array([ self.shoulder_offset, 0])

        # LEFT ARM

        sh_l = motors[16].q

        elbow_l = shoulder_left + np.array([
            -self.upper_arm * np.sin(sh_l),
            -self.upper_arm * np.cos(sh_l)
        ])

        hand_l = elbow_l + np.array([
            -self.forearm * np.sin(sh_l),
            -self.forearm * np.cos(sh_l)
        ])

        # RIGHT ARM

        sh_r = motors[23].q

        elbow_r = shoulder_right + np.array([
             self.upper_arm * np.sin(sh_r),
            -self.upper_arm * np.cos(sh_r)
        ])

        hand_r = elbow_r + np.array([
             self.forearm * np.sin(sh_r),
            -self.forearm * np.cos(sh_r)
        ])

        return {

            "hip_left": hip_left,
            "hip_right": hip_right,

            "torso_bottom": pelvis_center,
            "torso_top": torso_top,

            "knee_left": knee_left,
            "ankle_left": ankle_left,

            "knee_right": knee_right,
            "ankle_right": ankle_right,

            "shoulder_left": shoulder_left,
            "shoulder_right": shoulder_right,

            "elbow_l": elbow_l,
            "hand_l": hand_l,

            "elbow_r": elbow_r,
            "hand_r": hand_r
        }

# ============================================================
# CENTER OF MASS ESTIMATION
# ============================================================

def compute_com(sag):

    points = np.array([

        sag["pelvis"],
        sag["torso"],

        sag["knee_left"],
        sag["ankle_left"],

        sag["knee_right"],
        sag["ankle_right"],

        sag["elbow_l"],
        sag["hand_l"],

        sag["elbow_r"],
        sag["hand_r"]

    ])

    com = np.mean(points, axis=0)

    return com

# ============================================================
# POSTURAL STATE
# ============================================================

def estimate_posture(rpy):

    roll  = np.degrees(rpy[0])
    pitch = np.degrees(rpy[1])

    if abs(roll) < 5 and abs(pitch) < 5:
        return "STABLE"

    if abs(roll) < 12 and abs(pitch) < 12:
        return "SLIGHT IMBALANCE"

    return "CRITICAL IMBALANCE"

# ============================================================
# DRAW FUNCTIONS
# ============================================================

def draw_sagittal(ax, p, com, posture):

    ax.clear()

    ax.set_title("Sagittal Plane", fontsize=14)

    # TORSO

    ax.plot(
        [p["pelvis"][0], p["torso"][0]],
        [p["pelvis"][1], p["torso"][1]],
        linewidth=5
    )

    # LEFT LEG

    ax.plot(
        [p["pelvis"][0], p["knee_left"][0], p["ankle_left"][0]],
        [p["pelvis"][1], p["knee_left"][1], p["ankle_left"][1]],
        'o-',
        linewidth=4
    )

    # RIGHT LEG

    ax.plot(
        [p["pelvis"][0], p["knee_right"][0], p["ankle_right"][0]],
        [p["pelvis"][1], p["knee_right"][1], p["ankle_right"][1]],
        'o-',
        linewidth=4
    )

    # LEFT ARM

    ax.plot(
        [p["torso"][0], p["elbow_l"][0], p["hand_l"][0]],
        [p["torso"][1], p["elbow_l"][1], p["hand_l"][1]],
        'o-',
        linewidth=3
    )

    # RIGHT ARM

    ax.plot(
        [p["torso"][0], p["elbow_r"][0], p["hand_r"][0]],
        [p["torso"][1], p["elbow_r"][1], p["hand_r"][1]],
        'o-',
        linewidth=3
    )

    # CENTER OF MASS

    ax.scatter(
        com[0],
        com[1],
        s=250,
        marker='X',
        label='COM'
    )

    ax.text(
        com[0] + 0.03,
        com[1],
        "COM",
        fontsize=12
    )

    # GROUND

    ax.axhline(y=0, linestyle='--')

    # TEXT

    ax.text(
        -0.95,
        1.45,
        f"Postural State: {posture}",
        fontsize=12
    )

    ax.set_xlim(-1, 1)
    ax.set_ylim(-0.2, 1.6)

    ax.set_aspect('equal')

    ax.grid(True)

# ============================================================

def draw_coronal(ax, p, posture):

    ax.clear()

    ax.set_title("Coronal Plane", fontsize=14)

    # TORSO

    ax.plot(
        [p["torso_bottom"][0], p["torso_top"][0]],
        [p["torso_bottom"][1], p["torso_top"][1]],
        linewidth=5
    )

    # HIPS

    ax.plot(
        [p["hip_left"][0], p["hip_right"][0]],
        [p["hip_left"][1], p["hip_right"][1]],
        linewidth=4
    )

    # SHOULDERS

    ax.plot(
        [p["shoulder_left"][0], p["shoulder_right"][0]],
        [p["shoulder_left"][1], p["shoulder_right"][1]],
        linewidth=4
    )

    # LEFT LEG

    ax.plot(
        [p["hip_left"][0], p["knee_left"][0], p["ankle_left"][0]],
        [p["hip_left"][1], p["knee_left"][1], p["ankle_left"][1]],
        'o-',
        linewidth=4
    )

    # RIGHT LEG

    ax.plot(
        [p["hip_right"][0], p["knee_right"][0], p["ankle_right"][0]],
        [p["hip_right"][1], p["knee_right"][1], p["ankle_right"][1]],
        'o-',
        linewidth=4
    )

    # LEFT ARM

    ax.plot(
        [p["shoulder_left"][0], p["elbow_l"][0], p["hand_l"][0]],
        [p["shoulder_left"][1], p["elbow_l"][1], p["hand_l"][1]],
        'o-',
        linewidth=3
    )

    # RIGHT ARM

    ax.plot(
        [p["shoulder_right"][0], p["elbow_r"][0], p["hand_r"][0]],
        [p["shoulder_right"][1], p["elbow_r"][1], p["hand_r"][1]],
        'o-',
        linewidth=3
    )

    ax.text(
        -0.95,
        1.45,
        f"Postural State: {posture}",
        fontsize=12
    )

    ax.axhline(y=0, linestyle='--')

    ax.set_xlim(-1, 1)
    ax.set_ylim(-0.2, 1.6)

    ax.set_aspect('equal')

    ax.grid(True)

# ============================================================
# MAIN
# ============================================================

def main():

    rclpy.init()

    node = PoseNode()

    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True
    )

    spin_thread.start()

    model = SkeletonModel()

    plt.ion()

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 8))

    last_print = 0

    try:

        while rclpy.ok():

            with node.lock:

                if node.latest_msg is None:
                    continue

                msg = node.latest_msg
                count = node.message_count

            sagittal_pose = model.sagittal(msg)
            coronal_pose  = model.coronal(msg)

            com = compute_com(sagittal_pose)

            posture = estimate_posture(msg.imu_state.rpy)

            draw_sagittal(ax1, sagittal_pose, com, posture)
            draw_coronal(ax2, coronal_pose, posture)

            fig.suptitle(
                f"Unitree G1 Pose Estimation | Messages Received: {count}",
                fontsize=16
            )

            # ====================================================
            # CLI DEBUG
            # ====================================================

            if count - last_print > 100:

                last_print = count

                roll  = np.degrees(msg.imu_state.rpy[0])
                pitch = np.degrees(msg.imu_state.rpy[1])
                yaw   = np.degrees(msg.imu_state.rpy[2])

                print("\n================================================")
                print(f"Messages Received : {count}")
                print(f"Postural State    : {posture}")
                print(f"Roll              : {roll:.2f} deg")
                print(f"Pitch             : {pitch:.2f} deg")
                print(f"Yaw               : {yaw:.2f} deg")
                print(f"COM X             : {com[0]:.3f} m")
                print(f"COM Y             : {com[1]:.3f} m")
                print("================================================")

            plt.pause(0.03)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()

# ============================================================

if __name__ == '__main__':
    main()
