import sys
sys.path.append("/home/saad/kerabot_ws/scripts")
from peel_place_benchmark_suite import check_strict_self_collision, fk_kerabot_6dof

q_zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
print("q_zero self-collision check (True means valid/no collision):", check_strict_self_collision(q_zero))

candidates = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, -0.5, 1.0, 0.0, -0.5, 0.0],
    [0.0, -0.8, 1.5, 0.0, -0.7, 0.0],
    [0.0, -0.6, 1.2, 0.0, -0.6, 0.0],
]

for i, q in enumerate(candidates):
    valid = check_strict_self_collision(q)
    pos, links, _ = fk_kerabot_6dof(q)
    print(f"Candidate {i}: q={q} -> Valid={valid}, EE_pos={pos}, min_link_z={min(l[2] for l in links)}")
