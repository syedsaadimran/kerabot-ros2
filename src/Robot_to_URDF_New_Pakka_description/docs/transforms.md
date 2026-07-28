# Transformation Matrices - Robot_to_URDF_New_Pakka

Homogeneous transformation matrices between consecutive frames.
Convention: URDF RPY (XYZ extrinsic / ZYX intrinsic).

## Notation

### Frames

| Index | Link |
|-------|------|
| $L_{0}$ | base_link |
| $L_{1}$ | L110I_Shoulder |
| $L_{2}$ | L110I_shoulder_2 |
| $L_{3}$ | J2J3_Shoulder |
| $L_{4}$ | Wrist_Motor |
| $L_{5}$ | L70IE_Finger |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | Revolute_1 | continuous (rad) | $L_{0}$ | $L_{1}$ |
| $q_{2}$ | Revolute_2 | continuous (rad) | $L_{1}$ | $L_{2}$ |
| $q_{3}$ | Revolute_3 | continuous (rad) | $L_{2}$ | $L_{3}$ |
| $q_{4}$ | Revolute_4 | continuous (rad) | $L_{3}$ | $L_{4}$ |
| $q_{5}$ | Revolute_5 | continuous (rad) | $L_{4}$ | $L_{5}$ |

Shorthand: $c_i = \cos(q_i)$, $s_i = \sin(q_i)$

### Kinematic Tree

```
L0: base_link
  +-- [continuous] Revolute_1 (q1)
      L1: L110I_Shoulder
        +-- [continuous] Revolute_2 (q2)
            L2: L110I_shoulder_2
              +-- [continuous] Revolute_3 (q3)
                  L3: J2J3_Shoulder
                    +-- [continuous] Revolute_4 (q4)
                        L4: Wrist_Motor
                          +-- [continuous] Revolute_5 (q5)
                              L5: L70IE_Finger
```

## Transforms

## Revolute_1

$L_{0}$ **base_link** -> $L_{1}$ **L110I_Shoulder** (continuous)
  Variable: $q_{1}$

- **origin xyz**: (0, 0, 0.08) m
- **origin rpy**: (-1.570796, 0, 0) rad
- **axis**: (0, 1, 0)

### Local Transform

$T^{0}_{1}(q_{1}) = T_{fixed} \cdot R_{axis}(q_{1})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & -1 & 0 & 0.08 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{1}) = \begin{bmatrix}
c_{1} & 0 & s_{1} & 0 \\
0 & 1 & 0 & 0 \\
-s_{1} & 0 & c_{1} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_2

$L_{1}$ **L110I_Shoulder** -> $L_{2}$ **L110I_shoulder_2** (continuous)
  Variable: $q_{2}$

- **origin xyz**: (0, -0.119, 0.057) m
- **origin rpy**: (3.141593, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{1}_{2}(q_{2}) = T_{fixed} \cdot R_{axis}(q_{2})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & -1 & 0 & -0.119 \\
0 & 0 & -1 & 0.057 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{2}) = \begin{bmatrix}
c_{2} & -s_{2} & 0 & 0 \\
s_{2} & c_{2} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_3

$L_{2}$ **L110I_shoulder_2** -> $L_{3}$ **J2J3_Shoulder** (continuous)
  Variable: $q_{3}$

- **origin xyz**: (0, 0.426, 0.003028) m
- **origin rpy**: (-1.570796, -1.570796, 0) rad
- **axis**: (-1, 0, 0)

### Local Transform

$T^{2}_{3}(q_{3}) = T_{fixed} \cdot R_{axis}(q_{3})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0.426 \\
1 & 0 & 0 & 0.003028 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{3}) = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & c_{3} & s_{3} & 0 \\
0 & -s_{3} & c_{3} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_4

$L_{3}$ **J2J3_Shoulder** -> $L_{4}$ **Wrist_Motor** (continuous)
  Variable: $q_{4}$

- **origin xyz**: (0.053972, 0, 0.314) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$$
T^{3}_{4}(q_{4}) = \begin{bmatrix}
c_{4} & -s_{4} & 0 & 0.053972 \\
s_{4} & c_{4} & 0 & 0 \\
0 & 0 & 1 & 0.314 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_5

$L_{4}$ **Wrist_Motor** -> $L_{5}$ **L70IE_Finger** (continuous)
  Variable: $q_{5}$

- **origin xyz**: (0.0595, 0, 0.13) m
- **origin rpy**: (-0.000787, -1.570796, 0.002101) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{4}_{5}(q_{5}) = T_{fixed} \cdot R_{axis}(q_{5})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -0.001315 & -0.999999 & 0.0595 \\
0 & 0.999999 & -0.001315 & 0 \\
1 & 0 & 0 & 0.13 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{5}) = \begin{bmatrix}
c_{5} & -s_{5} & 0 & 0 \\
s_{5} & c_{5} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Global Transform Chains

Transform from root $L_0$ to any link, as product of local transforms along the kinematic chain.

$$T^{0}_{2} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2})\quad (L_0 \to L_{2}: \text{L110I_shoulder_2})$$

$$T^{0}_{3} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3})\quad (L_0 \to L_{3}: \text{J2J3_Shoulder})$$

$$T^{0}_{4} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4})\quad (L_0 \to L_{4}: \text{Wrist_Motor})$$

$$T^{0}_{5} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5})\quad (L_0 \to L_{5}: \text{L70IE_Finger})$$

