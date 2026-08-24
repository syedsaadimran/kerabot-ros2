import sys
import numpy as np
sys.path.append("/home/saad/kerabot_ws/scripts")
from peel_place_benchmark_suite import fk_kerabot_6dof

q1 = [2.4112368979017575, -0.8611778493527322, 1.8416346949523703, 0.9535226084575836, 2.75041722469178, 0.0]
pos_ee, links, _ = fk_kerabot_6dof(q1)

print("EE pos:", pos_ee)
for i, l in enumerate(links):
    print(f"links[{i}] (z={l[2]:.3f}): {l}, dist_to_EE={np.linalg.norm(pos_ee - l):.4f}")
