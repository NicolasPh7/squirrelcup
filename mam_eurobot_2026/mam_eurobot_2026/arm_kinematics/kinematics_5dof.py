# kinematics_5dof.py
# Requires: sympy, numpy, scipy

import sympy as sp
import numpy as np
from dataclasses import dataclass
from itertools import product
import pdb

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
    m = sp.Matrix([
        [ ct,      -st*ca,   st*sa,   a*ct ],
        [ st,       ct*ca,  -ct*sa,   a*st ],
        [  0,          sa,      ca,     d  ],
        [  0,           0,       0,     1  ],
    ])
    return m

# DH_ROWS = (
#     DHRow(a=sp.Float(0.0),     alpha=sp.Float(0.0),       d=sp.Float(0.0),     theta_offset=sp.Float(0.0)),
#     DHRow(a=sp.Float(-4.5),    alpha=sp.pi/2,             d=sp.Float(9.871),   theta_offset=sp.Float(0.0)),
#     DHRow(a=sp.Float(134.32),  alpha=sp.Float(0.0),       d=sp.Float(0.0),     theta_offset=sp.Float(0.0)),
#     DHRow(a=sp.Float(87.74),   alpha=sp.Float(0.0),       d=sp.Float(0.0),     theta_offset=-sp.pi/2),
#     DHRow(a=sp.Float(0.0),     alpha=sp.pi/2,             d=sp.Float(0.0),     theta_offset=-sp.pi/2),
# )
DH_ROWS = (
    # DHRow(a=sp.Float(75.31),   alpha=sp.Float(0.0),       d=sp.Float(124.86),  theta_offset=-sp.pi/2),
    DHRow(a=sp.Float(4.5),   alpha=sp.Float(0.0),       d=sp.Float(0.0),  theta_offset=sp.Float(0.0)),
    DHRow(a=sp.Float(134.32),     alpha=-sp.pi/2,             d=sp.Float(9.871),   theta_offset=sp.Float(0.0)),
    DHRow(a=sp.Float(87.74), alpha=sp.Float(0.0),       d=sp.Float(0.0),     theta_offset=-sp.pi/2),
    DHRow(a=sp.Float(62.75),  alpha=sp.Float(0.0),              d=sp.Float(0.0),     theta_offset=sp.Float(0.0)),
    DHRow(a=sp.Float(0.0),  alpha=0,       d=sp.Float(0.0),     theta_offset=sp.Float(0.0)),
    # DHRow(a=sp.Float(-62.75),  alpha=sp.Float(0.0),       d=sp.Float(0.0),     theta_offset=sp.Float(0.0)),
)
# Joint limits
JOINT_LOWER = np.array([-np.pi,         -np.pi,     sp.Float(0.0),             -np.pi/4,   -np.pi], dtype=float)
JOINT_UPPER = np.array([ np.pi,  sp.Float(0.0),             np.pi,              np.pi/4,    np.pi], dtype=float)

def debug_transforms(qv):
    # qv: list/tuple mit 5 numerischen Werten
    subs = {q1: qv[0], q2: qv[1], q3: qv[2], q4: qv[3], q5: qv[4]}
    T = sp.eye(4)
    for i, row in enumerate(DH_ROWS):
        theta_i = q[i] + row.theta_offset
        Ai = dh_A(row.a, row.alpha, row.d, theta_i)   # SymPy-Matrix (symbolisch)
        # numerische Version von Ai
        Ai_num = np.array(Ai.subs(subs).evalf().tolist(), dtype=float)
        # kumulierte Transformation (symbolisch), dann numerisch konvertieren
        T = T * Ai
        T_num = np.array(T.subs(subs).evalf().tolist(), dtype=float)

        print(f"Link {i+1}: a={float(row.a)} alpha={float(row.alpha)} d={float(row.d)} theta_offset={float(row.theta_offset)}")
        print(" Ai (numeric):\n", Ai_num)
        print(" T_0i (numeric):\n", T_num)
        print("----")

T_base = sp.Matrix([
    [0,  1, 0,  0.0],
    [-1, 0, 0, -75.31],
    [0,  0, 1, 124.86],
    [0,  0, 0,   1.0]
])

# Forward kinematics chain
Ts = [T_base]
for i, row in enumerate(DH_ROWS):
    theta_i = q[i] + row.theta_offset
    Ai = dh_A(row.a, row.alpha, row.d, theta_i)
    Ts.append(Ts[-1] * Ai)

T_base_ee = Ts[-1]   # jetzt ist das die Transformation von Origin (welt) zum Endeffektor
R = T_base_ee[:3, :3]
p = T_base_ee[:3, 3]

# T_0e = Ts[-1]
# R = T_0e[:3, :3]
# p = T_0e[:3, 3]

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

def ik_all_solutions(goal, q_inits):
    sols = []
    for q0 in q_inits:
        try:
            sol = ik_nsolve(goal, q_init=q0)
            sols.append(sol)
        except Exception:
            try:
                sol = ik_least_squares(goal, q_init=q0)
                sols.append(sol)
            except Exception:
                pass
    # Duplikate entfernen (numerisch nahe Lösungen zusammenfassen)
    unique_sols = []
    for s in sols:
        if not any(np.allclose(s, u, atol=1e-3) for u in unique_sols):
            unique_sols.append(s)
    return unique_sols


def generate_grid_inits(lower, upper, steps=2):
    grids = [np.linspace(l, u, steps) for l, u in zip(lower, upper)]
    return list(product(*grids))


def select_far_from_limits(q_solutions, lower=JOINT_LOWER, upper=JOINT_UPPER):
    def margin(q):
        # Abstand zu Limits pro Gelenk
        dist_to_upper = [u - qi for qi, u in zip(q, upper)]
        dist_to_lower = [qi - l for qi, l in zip(q, lower)]
        # kleinster Abstand zu einem Limit
        return min(dist_to_upper + dist_to_lower)
    return max(q_solutions, key=margin)

def compute_best_q_for_goal(goal):
    q_inits = generate_grid_inits(JOINT_LOWER, JOINT_UPPER, steps=2)
    print(len(q_inits), "Startwerte erzeugt")

    # Toleranz für FK-Check
    tol = 1e-2

    sols = ik_all_solutions(goal, q_inits)

    if sols:
        # FK-Check für alle Lösungen
        valid_sols = []
        for q_sol in sols:
            fk_val = fk(q_sol)
            err = np.linalg.norm(np.array(goal[:3]) - np.array(fk_val[:3]))
            ok = err < tol
            if ok:
                valid_sols.append(q_sol)

        if valid_sols:
            q_best = select_far_from_limits(valid_sols)
            fk_val = fk(q_best)
            err = np.linalg.norm(np.array(goal[:3]) - np.array(fk_val[:3]))
            ok = err < tol
            # pdb.set_trace()
            print(f"Goal=({goal[:3]}) | FK={fk_val[:3]} | Error={err:.3f} | {'OK' if ok else 'FAIL'} | Best q*={q_best}\n")

            return q_best

    else:
        print(f"Goal=({x:.1f},{y:.1f},0) | keine Lösung gefunden\n")

# Example runner function
def run_examples():
    tests = [
        # [1.57, 0, 0, 0, 1.57],
        # [0, 1.57, 0, 0, 0],S
        # [0, -1.57, 0, -1.57, 0],
        [    0.0, 0.0,        0.0, 0.0, 0.0],
        # [      0,   0, 1.57079633, 0.0,   0],
        # [      0,   0, 1.57079633, 0.0,   0],
        # [3.14159,   0, 1.57079633,   0,   0],
        # [0.6, -0.8, 1.2, -0.6, 0.3],
    ]
    for q0 in tests:
        # debug_transforms(q0)
        fk_out = fk(q0)
        Jn = J_task_num(q0)
        rank = np.linalg.matrix_rank(Jn)
        print (f"------------------- q0={q0} ------------------- ")
        print(f"q0={q0} FK: {fk_out}  rank(J_task)={rank}")

        goals = [
            # [50.0, 10.0, 250.0, 0.2, -0.1],
            # [0.0, 31.6, 0, 0, 0.0],
            # [50, 50, 0, 0, 0],
            # [100, 100, 0, 0, 0],
            # [0.0, 50.0, 0, 0, 0.0],
            # [0.0, 63.2, 0, 0, 0.0],
            # [63.2, 0.0, 0, 0, 0.0],
            # [0.0, 73.7, 0, 0, 0.0],
            # [0.0, 84.2, 0, 0, 0.0],
            # # [0.0, 115.8, 0, 0, 0.0],
            # # [0.0, 136.8, 0, 0, 0.0],
            # [0.0, 147.4, 0, 0, 0.0],
            # [0.0, 168.4, 0, 0, 0.0],
            # [0.0, 200.0, 0, 0, 0.0],
        ]


        for goal in goals:
            print (f"||||||||||||||||| goal={goal} ||||||||||||||||| ")
            try:
                q_sol = ik_nsolve(goal, q_init=q0)
                print("IK (nsolve) q*:", q_sol)
                print("FK(goal check):", fk(q_sol))
            except Exception as e:
                print("nsolve failed:", e)
                q_sol = ik_least_squares(goal, q_init=q0)
                print("IK (LS) q*:", q_sol)
                print("FK(goal check):", fk(q_sol), "\n")

        # # Sweep-Bereich definieren

        q_inits = generate_grid_inits(JOINT_LOWER, JOINT_UPPER, steps=2)
        print(len(q_inits), "Startwerte erzeugt")

        x_range = np.linspace(0, 200, 3)   # 0 bis 200 mm, 21 Schritte
        y_range = np.linspace(0, 200, 9)

        # Toleranz für FK-Check
        tol = 1e-2

        # compute_best_q_for_goal([0.5, 0.5, 0, 0, 0])
        results = []
        for x in x_range:
            for y in y_range:
                goal = [x, y, 125.0, 0, 0]
                sols = ik_all_solutions(goal, q_inits)

                if sols:
                    # FK-Check für alle Lösungen
                    valid_sols = []
                    for q_sol in sols:
                        fk_val = fk(q_sol)
                        err = np.linalg.norm(np.array(goal[:3]) - np.array(fk_val[:3]))
                        ok = err < tol
                        results.append((x, y, ok, err, q_sol))
                        if ok:
                            valid_sols.append(q_sol)

                    if valid_sols:
                        q_best = select_far_from_limits(valid_sols)
                        fk_val = fk(q_best)
                        err = np.linalg.norm(np.array(goal[:3]) - np.array(fk_val[:3]))
                        ok = err < tol
                        print(f"Goal=({x:.1f},{y:.1f},0) | FK={fk_val[:3]} | Error={err:.3f} | {'OK' if ok else 'FAIL'} | Best q*={q_best}\n")

                else:
                    print(f"Goal=({x:.1f},{y:.1f},0) | keine Lösung gefunden\n")



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
