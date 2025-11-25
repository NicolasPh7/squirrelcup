# kinematics_5dof.py
# Requires: sympy, numpy, scipy

import sympy as sp
import numpy as np
from dataclasses import dataclass

@dataclass
class DHRow:
    a: float
    alpha: float
    d: float
    theta_offset: float = 0.0

# Joint symbols
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

# DH table (aus deinem Bild)
# DH_ROWS = (
#     DHRow(a=sp.Float(0.0),     alpha=sp.Float(0.0),       d=sp.Float(0.0),     theta_offset=sp.Float(0.0)),
#     DHRow(a=sp.Float(-4.5),    alpha=sp.pi/2,             d=sp.Float(9.871),   theta_offset=sp.Float(0.0)),
#     DHRow(a=sp.Float(134.32),  alpha=sp.Float(0.0),       d=sp.Float(0.0),     theta_offset=sp.Float(0.0)),
#     DHRow(a=sp.Float(87.74),   alpha=sp.Float(0.0),       d=sp.Float(0.0),     theta_offset=-sp.pi/2),
#     DHRow(a=sp.Float(0.0),     alpha=sp.pi/2,             d=sp.Float(0.0),     theta_offset=-sp.pi/2),
# )
DH_ROWS = (
    DHRow(a=sp.Float(0.0),     alpha=sp.Float(0.0),       d=sp.Float(0.0),     theta_offset=sp.Float(0.0)),
    DHRow(a=sp.Float(4.5),     alpha=sp.pi/2,             d=sp.Float(9.871),   theta_offset=sp.Float(0.0)),
    DHRow(a=sp.Float(-134.32), alpha=sp.Float(0.0),       d=sp.Float(0.0),     theta_offset=0),
    DHRow(a=sp.Float(-87.74),  alpha=sp.Float(0.0),       d=sp.Float(0.0),     theta_offset=-sp.pi/2),
    DHRow(a=sp.Float(0.0),     alpha=0,                   d=sp.Float(0.0),     theta_offset=sp.pi/2),
)
# Joint limits
JOINT_LOWER = np.array([-np.pi, -np.pi/2, -np.pi, -np.pi/2, -np.pi/2], dtype=float)
JOINT_UPPER = np.array([ np.pi,  np.pi/2,  np.pi,  np.pi/2,  np.pi/2], dtype=float)

# Forward kinematics chain
Ts = [sp.eye(4)]
for i, row in enumerate(DH_ROWS):
    theta_i = q[i] + row.theta_offset
    Ai = dh_A(row.a, row.alpha, row.d, theta_i)
    Ts.append(Ts[-1] * Ai)

T_0e = Ts[-1]
R = T_0e[:3, :3]
p = T_0e[:3, 3]

# Pose outputs: position + tool z-axis direction (robust orientation)
x, y, z = p[0], p[1], p[2]
r_iz = R[:, 2]
r_iz_x, r_iz_y = r_iz[0], r_iz[1]

# Task vector and Jacobian
task = sp.Matrix([x, y, z, r_iz_x, r_iz_y])
J_task = task.jacobian(q)

# Lambdify
FK_lambda = sp.lambdify((q1, q2, q3, q4, q5), (x, y, z, r_iz_x, r_iz_y), 'numpy')
J_lambda = sp.lambdify((q1, q2, q3, q4, q5), J_task, 'numpy')

def fk(qv):
    return FK_lambda(*qv)

def J_task_num(qv):
    return np.array(J_lambda(*qv), dtype=float)

# IK via sympy.nsolve
def ik_nsolve(goal, q_init, maxsteps=100):
    gx, gy, gz, grx, gry = goal
    eqs = [
        sp.Eq(x, gx),
        sp.Eq(y, gy),
        sp.Eq(z, gz),
        sp.Eq(r_iz_x, grx),
        sp.Eq(r_iz_y, gry),
    ]
    sol = sp.nsolve(eqs, (q1, q2, q3, q4, q5), tuple(q_init),
                    tol=1e-12, maxsteps=maxsteps, prec=50)
    return [float(sol[i]) for i in range(5)]

# IK via least-squares (robust fallback)
def ik_least_squares(goal, q_init, bounds=(JOINT_LOWER, JOINT_UPPER)):
    from scipy.optimize import least_squares
    def residuals(qv):
        xv, yv, zv, rxv, ryv = fk(qv)
        r = np.array([xv-goal[0], yv-goal[1], zv-goal[2],
                      rxv-goal[3], ryv-goal[4]], dtype=float)
        return r
    lb, ub = bounds
    res = least_squares(residuals, q_init, bounds=(lb, ub),
                        xtol=1e-10, ftol=1e-10, gtol=1e-10,
                        method='trf', jac='2-point')
    return res.x.tolist()

# Example runner function
def run_examples():
    tests = [
        [1.57, 0, 0, 0, 1.57],
        [0.5, -0.3, 0.2, 0.1, -0.4],
        [1.0, 0.5, -0.5, 0.2, 0.6],
        [0.6, -0.8, 1.2, -0.6, 0.3],
    ]
    for q0 in tests:
        fk_out = fk(q0)
        Jn = J_task_num(q0)
        rank = np.linalg.matrix_rank(Jn)
        print(f"q0={q0} FK: {fk_out}  rank(J_task)={rank}")

        goal = [50.0, 10.0, 250.0, 0.2, -0.1]  # Zielpose
        try:
            q_sol = ik_nsolve(goal, q_init=q0)
            print("IK (nsolve) q*:", q_sol)
            print("FK(goal check):", fk(q_sol))
        except Exception as e:
            print("nsolve failed:", e)
            q_sol = ik_least_squares(goal, q_init=q0)
            print("IK (LS) q*:", q_sol)
            print("FK(goal check):", fk(q_sol))

# Only run examples if executed directly
if __name__ == "__main__":
    run_examples()


# How to use
# # test_kinematics.py
# from kinematics_5dof import run_examples, fk, J_task_num, ik_least_squares

# # Einfach die Beispielroutine starten
# run_examples()

# # Oder eigene Tests definieren
# q0 = [0.5, -0.3, 0.2, 0.1, -0.4]
# print("FK custom:", fk(q0))
# print("Jacobian rank:", np.linalg.matrix_rank(J_task_num(q0)))

# goal = [60.0, 20.0, 200.0, 0.1, -0.2]
# q_sol = ik_least_squares(goal, q_init=q0)
# print("IK solution:", q_sol)
# print("FK(goal check):", fk(q_sol))
