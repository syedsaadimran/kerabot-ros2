import math
import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
from scipy.spatial.transform import Rotation as R

def main():
    rclpy.init()
    node = Node("find_inverted_quat")
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)

    print("Listening for TF base_link -> L70IE_Finger...")
    time_start = node.get_clock().now()
    t = None
    while (node.get_clock().now() - time_start).nanoseconds * 1e-9 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            t = tf_buffer.lookup_transform("base_link", "L70IE_Finger", rclpy.time.Time())
            break
        except Exception:
            pass

    if t is not None:
        q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        r = R.from_quat(q)
        rot_mat = r.as_matrix()
        print(f"Current End-Effector Pose:")
        print(f"  Translation (m) : [{t.transform.translation.x:.4f}, {t.transform.translation.y:.4f}, {t.transform.translation.z:.4f}]")
        print(f"  Quat (xyzw)     : [{q[0]:.4f}, {q[1]:.4f}, {q[2]:.4f}, {q[3]:.4f}]")
        print(f"  RPY (deg)       : {r.as_euler('xyz', degrees=True)}")
        print(f"  Rotation Matrix :\n{rot_mat}")
        print(f"  End-effector Z-axis in base_link: {rot_mat[:, 2]}")
        print(f"  End-effector X-axis in base_link: {rot_mat[:, 0]}")

    # Compute ideal pointing-down orientation:
    # We want the tool approach axis (Z-axis of L70IE_Finger) to point in -Z_base direction: [0, 0, -1]
    # Standard upside-down orientation: pitch = -180 deg or roll = 180 deg
    # In Euler RPY (degrees): (180, 0, 0) or (0, 180, 0) or (-180, 0, 0)
    for roll, pitch, yaw in [(180, 0, 0), (-180, 0, 0), (0, 180, 0), (0, -180, 0), (90, 180, 0)]:
        r_test = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
        q_test = r_test.as_quat()
        mat = r_test.as_matrix()
        z_dir = mat[:, 2]
        print(f"\nRPY ({roll:4d}, {pitch:4d}, {yaw:4d}) -> Quat [{q_test[0]:.4f}, {q_test[1]:.4f}, {q_test[2]:.4f}, {q_test[3]:.4f}] -> Tool Z-dir: [{z_dir[0]:.3f}, {z_dir[1]:.3f}, {z_dir[2]:.3f}]")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
