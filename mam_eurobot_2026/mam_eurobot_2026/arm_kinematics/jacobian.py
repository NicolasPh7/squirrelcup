import numpy as np
import matplotlib.pyplot as plt

# Anzahl zufälliger Samples
n_samples = 5000

# kinematics_5dof_sympy.py
# Requires: sympy, numpy (optional), scipy (optional for fallback)

import sympy as sp
import numpy as np
from dataclasses import dataclass
from typing import Tuple, Callable, Optional

@dataclass
class DHRow:
    a: float
    alpha: float
    d: float
    theta_offset: float = 0.0  # fixed offset added to q_i

# Symbols for joints
q1, q2, q3, q4, q5 = sp.symbols('q1 q2 q3 q4 q5', real=True)
q = sp.Matrix([q1, q2, q3, q4, q5])

def dh_A(a, alpha, d, theta):
    ca, sa = sp.cos(alpha), sp.sin(alpha)
    ct, st = sp.cos(theta), sp.sin(theta)
    return sp.Matrix([
        [ ct,      -st*ca,   st*sa,   a*ct ],
        [ st,       ct*ca,  -ct*sa,   a*st ],
        [  0,          sa,      ca,     d  ],
        [  0,           0,       0,     1  ],
    ])

# Your DH table
DH_ROWS = (
    DHRow(a=sp.Float(0.0),     alpha=sp.Float(0.0),       d=sp.Float(0.0),     theta_offset=sp.Float(0.0)),
    DHRow(a=sp.Float(4.5),     alpha=sp.pi/2,             d=sp.Float(9.871),   theta_offset=sp.Float(0.0)),
    DHRow(a=sp.Float(-134.32), alpha=sp.Float(0.0),       d=sp.Float(0.0),     theta_offset=0),
    DHRow(a=sp.Float(-87.74),  alpha=sp.Float(0.0),       d=sp.Float(0.0),     theta_offset=-sp.pi/2),
    DHRow(a=sp.Float(0.0),     alpha=0,                   d=sp.Float(0.0),     theta_offset=sp.pi/2),
)


JOINT_LOWER = np.array([-np.pi, -np.pi/2, -np.pi, -np.pi/2, -np.pi/2], dtype=float)
JOINT_UPPER = np.array([ np.pi,  np.pi/2,  np.pi,  np.pi/2,  np.pi/2], dtype=float)

# Chain transform T = A1*A2*...*A5
T = sp.eye(4)
for i, row in enumerate(DH_ROWS):
    theta_i = q[i] + row.theta_offset
    T = T * dh_A(row.a, row.alpha, row.d, theta_i)

R = T[:3, :3]
p = T[:3, 3]

# Pose components: x, y, z, yaw, roll (ZYX convention)
x, y, z = p[0], p[1], p[2]
yaw = sp.atan2(R[1, 0], R[0, 0])    # about Z
roll = sp.atan2(R[2, 1], R[2, 2])   # about X

# FK lambdas
FK_xyzyawroll = sp.lambdify((q1, q2, q3, q4, q5), (x, y, z, yaw, roll), 'numpy')

# Task vector and Jacobian (5x5)
task = sp.Matrix([x, y, z, yaw, roll])
J = task.jacobian(q)  # symbolic 5x5

# Lambdas for Jacobian and rank check
J_lambda = sp.lambdify((q1, q2, q3, q4, q5), J, 'numpy')

def fk_lambda(q_vec):
    return FK_xyzyawroll(*q_vec)  # returns (x, y, z, yaw, roll)

def jacobian_lambda(q_vec):
    return J_lambda(*q_vec)  # 5x5 numpy array

# IK via sympy.nsolve (requires good initial guess and reachable target)
def ik_nsolve(goal_xyzyawroll, q_init, maxsteps=100):
    gx, gy, gz, gyaw, groll = goal_xyzyawroll

    # Angle wrap helper (symbolic): enforce equations modulo 2*pi by using sin/cos equalities
    # Here, use direct atan2 equality with small-angle difference assumption; practical alternative:
    # match sin/cos of angles to avoid branch issues:
    eqs = [
        sp.Eq(x, gx),
        sp.Eq(y, gy),
        sp.Eq(z, gz),
        sp.Eq(R[1, 0], sp.sin(gyaw)),   # yaw: sin-Komponente
        sp.Eq(R[0, 0], sp.cos(gyaw)),   # yaw: cos-Komponente
        sp.Eq(R[2, 1], sp.sin(groll)),  # roll: sin-Komponente
        sp.Eq(R[2, 2], sp.cos(groll)),  # roll: cos-Komponente
    ]


    sol = sp.nsolve(eqs, (q1, q2, q3, q4, q5), tuple(q_init), tol=1e-12, maxsteps=maxsteps, prec=50)
    return [float(sol[i]) for i in range(5)]

# Optional: robust numerical fallback (scipy) if nsolve struggles
def ik_least_squares(goal_xyzyawroll, q_init, bounds=(JOINT_LOWER, JOINT_UPPER), weights=(1,1,1,1,1)):
    import numpy as np
    from scipy.optimize import least_squares

    def residuals(qv):
        xv, yv, zv, yawv, rollv = fk_lambda(qv)
        # angle differences wrapped
        def wrap(a): return (a + np.pi) % (2*np.pi) - np.pi
        r = np.array([xv-goal_xyzyawroll[0], yv-goal_xyzyawroll[1], zv-goal_xyzyawroll[2],
                      wrap(yawv-goal_xyzyawroll[3]), wrap(rollv-goal_xyzyawroll[4])], dtype=float)
        w = np.array(weights, dtype=float)
        return w * r

    n = 5
    if bounds is None:
        lb = np.full(n, -2*np.pi)
        ub = np.full(n, +2*np.pi)
    else:
        lb, ub = bounds

    res = least_squares(residuals, q_init, bounds=(lb, ub), xtol=1e-10, ftol=1e-10, gtol=1e-10, method='trf', jac='2-point')
    return res.x.tolist()

# # Example usage
# if __name__ == "__main__":
#     import numpy as np

#     for qtest in [
#         [0,0,0,0,0],
#         [0.5, -0.3, 0.2, 0.1, -0.4],
#         [1.0, 0.5, -0.5, 0.2, 0.0],
#     ]:
#         print(f"-------------- q0 = {qtest} --------------")
#         q0 = qtest
#         print("FK at q0:", fk_lambda(q0))
#         J0 = jacobian_lambda(q0)
#         print("Jacobian rank at q0:", np.linalg.matrix_rank(J0))

#         goal = [50.0, 10.0, 250.0, 0.2, -0.1]  # x,y,z (mm), yaw, roll (rad)
#         try:
#             q_sol = ik_nsolve(goal, q_init=q0)
#             print("IK (nsolve) q*:", q_sol)
#             print("FK(goal check):", fk_lambda(q_sol))
#         except Exception as e:
#             print("nsolve failed:", e)
#             q_sol = ik_least_squares(goal, q_init=q0)
#             print("IK (LS) q*:", q_sol)
#             print("FK(goal check):", fk_lambda(q_sol))

import numpy as np
import matplotlib.pyplot as plt

# --- Deine Gelenkgrenzen ---
JOINT_LOWER = np.array([-np.pi, -np.pi/2, -np.pi, -np.pi/2, -np.pi/2])
JOINT_UPPER = np.array([ np.pi,  np.pi/2,  np.pi,  np.pi/2,  np.pi/2])

# --- Anzahl zufälliger Samples für Histogramm ---
n_samples = 5000
ranks = []

for _ in range(n_samples):
    q = np.random.uniform(JOINT_LOWER, JOINT_UPPER)
    J = np.nan_to_num(J_lambda(*q), nan=0.0, posinf=0.0, neginf=0.0)
    rank = np.linalg.matrix_rank(J)
    ranks.append(rank)

# --- Histogramm anzeigen ---
plt.figure(figsize=(6,4))
plt.hist(ranks, bins=np.arange(0,7)-0.5, rwidth=0.8, color='steelblue')
plt.xlabel("Jacobian Rank")
plt.ylabel("Count")
plt.title("Jacobian Rank Distribution over 5D Joint Space")
plt.xticks(range(6))
plt.grid(True, linestyle='--', alpha=0.5)
plt.tight_layout()
plt.show()

# --- Heatmap für q1 vs q2 bei fixierten q3–q5 ---
n_grid = 50
q1_vals = np.linspace(JOINT_LOWER[0], JOINT_UPPER[0], n_grid)
q2_vals = np.linspace(JOINT_LOWER[1], JOINT_UPPER[1], n_grid)
rank_grid = np.zeros((n_grid, n_grid))

q_fixed = [0.0, 0.0, 0.0]  # q3, q4, q5 fixiert

for i, q1 in enumerate(q1_vals):
    for j, q2 in enumerate(q2_vals):
        q = [q1, q2] + q_fixed
        J = np.nan_to_num(J_lambda(*q), nan=0.0, posinf=0.0, neginf=0.0)
        rank_grid[j, i] = np.linalg.matrix_rank(J)

# --- Heatmap anzeigen ---
plt.figure(figsize=(6,5))
plt.imshow(rank_grid, extent=[q1_vals[0], q1_vals[-1], q2_vals[0], q2_vals[-1]],
           origin='lower', aspect='auto', cmap='viridis')
plt.colorbar(label='Jacobian Rank')
plt.xlabel('q1 (rad)')
plt.ylabel('q2 (rad)')
plt.title('Jacobian Rank Heatmap (q1 vs q2)')
plt.grid(False)
plt.tight_layout()
plt.show()
