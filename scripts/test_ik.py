import sys
import numpy as np
sys.path.append("/home/saad/kerabot_ws/scripts")
from peel_place_benchmark_suite import solve_ik_6dof, fk_kerabot_6dof, check_strict_self_collision

PICK_XYZ  = np.array([0.25, -0.30, 0.22])
PLACE_XYZ = np.array([-0.25, -0.30, 0.22])

q_pick = solve_ik_6dof(PICK_XYZ, yaw_angle=0)
print("q_pick:", q_pick)
if q_pick:
    pos, _, _ = fk_kerabot_6dof(q_pick)
    print("q_pick FK EE pos:", pos, "Error:", np.linalg.norm(pos - PICK_XYZ))

q_place = solve_ik_6dof(PLACE_XYZ, yaw_angle=0)
print("q_place:", q_place)
if q_place:
    pos, _, _ = fk_kerabot_6dof(q_place)
    print("q_place FK EE pos:", pos, "Error:", np.linalg.norm(pos - PLACE_XYZ))
