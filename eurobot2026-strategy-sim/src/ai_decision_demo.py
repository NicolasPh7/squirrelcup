import os
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

# Config
SRC_DIR = os.path.dirname(__file__)
MAP_IMG = os.path.join(SRC_DIR, "table_bis.png")
COLS, ROWS = 8, 6
ACTIONS = ["UP", "DOWN", "LEFT", "RIGHT", "COLLECT", "DELIVER"]

# key map points (col, row)
POINTS = {
    "nest_y": (0, 0),
    "nest_b": (7, 0),
    "collection": (0, 3),
    "pantry": (7, 3),
    "fridge": (2, 0),
    "thermo": (4, 5),
}


# simple deterministic state for rollouts
class State:
    def __init__(self, pos=(3, 5), carried=0, items=3):
        self.pos = tuple(pos)
        self.carried = int(carried)
        self.items = int(items)
        self.time = 0

    def copy(self):
        s = State(self.pos, self.carried, self.items)
        s.time = self.time
        return s

    def step(self, action, nest="nest_y"):
        x, y = self.pos
        if action == "UP":
            y = min(ROWS - 1, y + 1)
        elif action == "DOWN":
            y = max(0, y - 1)
        elif action == "LEFT":
            x = max(0, x - 1)
        elif action == "RIGHT":
            x = min(COLS - 1, x + 1)
        elif action == "COLLECT":
            if (x, y) == POINTS["collection"] and self.items > 0 and self.carried < 3:
                self.items -= 1
                self.carried += 1
        elif action == "DELIVER":
            if (x, y) == POINTS[nest] and self.carried > 0:
                reward = self.carried * 10
                self.carried = 0
                self.time += 1
                self.pos = (x, y)
                return reward
        self.pos = (x, y)
        self.time += 1
        return 0


# biased random policy for faster meaningful rollouts
def random_policy(state, nest_choice):
    if state.carried > 0:
        target = POINTS[nest_choice]
    elif state.items > 0:
        target = POINTS["collection"]
    else:
        target = POINTS["pantry"]
    x, y = state.pos
    tx, ty = target
    dx = int(np.sign(tx - x))
    dy = int(np.sign(ty - y))
    if random.random() < 0.75:
        if dx != 0 and random.random() < 0.6:
            return "RIGHT" if dx > 0 else "LEFT"
        if dy != 0:
            return "UP" if dy > 0 else "DOWN"
    return random.choice(ACTIONS)


def rollout(init_state, depth=12, nest_choice="nest_y"):
    s = init_state.copy()
    tot = 0
    for _ in range(depth):
        a = random_policy(s, nest_choice)
        tot += s.step(a, nest=nest_choice)
        if random.random() < 0.03:
            s.items += 1
    tot += s.carried * 1
    return tot


def evaluate_action(root_state, action, sims=80, nest_choice="nest_y"):
    acc = 0.0
    for _ in range(sims):
        s = root_state.copy()
        acc += s.step(action, nest=nest_choice)
        acc += rollout(s, depth=10, nest_choice=nest_choice)
    return acc / sims


def best_action_search(root_state, budget=720, nest_choice="nest_y"):
    per_action = max(4, budget // len(ACTIONS))
    scores = {a: 0.0 for a in ACTIONS}
    counts = {a: 0 for a in ACTIONS}
    # initial quick samples
    for a in ACTIONS:
        val = evaluate_action(root_state, a, sims=max(2, per_action // 8), nest_choice=nest_choice)
        scores[a] = val
        counts[a] = 1
    remaining = budget - sum(counts.values())
    for _ in range(remaining // 8):
        # simple UCB-like pick
        ucb = {a: scores[a] + 0.6 * np.sqrt(1 + counts[a]) for a in ACTIONS}
        pick = max(ucb, key=ucb.get)
        val = evaluate_action(root_state, pick, sims=8, nest_choice=nest_choice)
        # incremental mean update
        prev_sum = scores[pick] * counts[pick]
        counts[pick] += 8
        scores[pick] = (prev_sum + val * 8) / counts[pick]
    return scores, counts


# visual demo: map background + iterative planner bars on the right
def demo(save_path=None):
    root = State(pos=(3, 5), carried=0, items=3)
    nest_choice = "nest_y"
    img_ok = os.path.isfile(MAP_IMG)
    fig, ax = plt.subplots(figsize=(12, 7), dpi=120)
    # left: map area
    map_w = COLS
    map_h = ROWS
    ax.set_xlim(-0.5, map_w + 3.5)
    ax.set_ylim(-0.5, map_h - 0.5)
    ax.set_aspect("equal")
    ax.axis("off")

    # background image if present
    if img_ok:
        img = plt.imread(MAP_IMG)
        ax.imshow(img, extent=[0, map_w, 0, map_h], zorder=0)

    # grid lines
    for x in range(COLS + 1):
        ax.plot([x, x], [0, map_h], color="#111", lw=0.9, zorder=2)
    for y in range(ROWS + 1):
        ax.plot([0, COLS], [y, y], color="#111", lw=0.9, zorder=2)

    # draw fixed points
    def draw_box(pt, color, label=None):
        cx, cy = pt
        rect = patches.Rectangle((cx + 0.05, map_h - cy - 1 + 0.05), 0.9, 0.9, facecolor=color, edgecolor="#111", lw=1.1, zorder=3)
        ax.add_patch(rect)
        if label:
            ax.text(cx + 0.5, map_h - cy - 0.5, label, ha="center", va="center", fontsize=8, color="white", weight="bold", zorder=4)

    draw_box(POINTS["nest_y"], "#FFD700", "Nest Y")
    draw_box(POINTS["nest_b"], "#1E90FF", "Nest B")
    draw_box(POINTS["collection"], "#32CD32", "Collect")
    draw_box(POINTS["pantry"], "#8B4513", "Pantry")
    draw_box(POINTS["fridge"], "#4682B4", "Fridge")
    draw_box(POINTS["thermo"], "#DC143C", "Thermo")

    # robot marker
    rx, ry = root.pos
    robot = patches.Circle((rx + 0.5, map_h - ry - 0.5), 0.28, facecolor="#222", edgecolor="white", lw=2, zorder=6)
    ax.add_patch(robot)

    # action text and bars (right side)
    bars = {}
    texts = {}
    xoff = COLS + 0.6
    for i, a in enumerate(ACTIONS):
        y = map_h - i - 0.8
        texts[a] = ax.text(xoff + 0.05, y, f"{a}: ...", va="center", fontsize=10)
        bar = patches.Rectangle((xoff + 1.8, y - 0.2), 0.0, 0.35, facecolor="#2196F3", edgecolor="#0B66A6", zorder=5)
        ax.add_patch(bar)
        bars[a] = bar

    title = ax.text(0.02, 1.02, "Iterative MC Planner — live search", transform=ax.transAxes, fontsize=13, weight="bold")

    # running aggregator
    agg = {"scores": {a: 0.0 for a in ACTIONS}, "sims": {a: 0 for a in ACTIONS}, "round": 0}

    arrow_patch = None

    def update(frame):
        nonlocal arrow_patch
        # small search chunk per frame
        chunk = 360
        scores, counts = best_action_search(root, budget=chunk, nest_choice=nest_choice)
        # merge
        for a in ACTIONS:
            ps = agg["scores"][a] * agg["sims"][a]
            ns = scores[a] * max(1, counts[a])
            tot_n = agg["sims"][a] + max(1, counts[a])
            agg["scores"][a] = (ps + ns) / tot_n
            agg["sims"][a] = tot_n
        agg["round"] += 1

        # update bars and texts
        max_s = max(agg["scores"].values()) + 1e-8
        for a in ACTIONS:
            texts[a].set_text(f"{a}: {agg['scores'][a]:.2f} ({agg['sims'][a]})")
            bars[a].set_width(2.6 * (agg["scores"][a] / max_s))
            bars[a].set_xy((COLS + 2.0, map_h - ACTIONS.index(a) - 0.95))

        # highlight best action with arrow
        best = max(agg["scores"], key=agg["scores"].get)
        title.set_text(f"Round {agg['round']} — best = {best} ({agg['scores'][best]:.2f})")

        if arrow_patch:
            try:
                arrow_patch.remove()
            except Exception:
                pass
            arrow_patch = None

        bx, by = root.pos
        dx, dy = 0, 0
        if best == "UP":
            dy = 1
        elif best == "DOWN":
            dy = -1
        elif best == "LEFT":
            dx = -1
        elif best == "RIGHT":
            dx = 1
        elif best == "COLLECT":
            tx, ty = POINTS["collection"]
            dx, dy = tx - bx, ty - by
        elif best == "DELIVER":
            tx, ty = POINTS[nest_choice]
            dx, dy = tx - bx, ty - by

        arrow_patch = patches.FancyArrow(bx + 0.5, map_h - by - 0.5, dx * 0.7, -dy * 0.7,
                                        width=0.12, length_includes_head=True, head_width=0.25,
                                        facecolor="#E53935", edgecolor="#B71C1C", zorder=7)
        ax.add_patch(arrow_patch)

        # optional save
        if save_path and agg["round"] == 6:
            plt.savefig(save_path, bbox_inches="tight", dpi=180)

        return list(texts.values()) + list(bars.values()) + [title, arrow_patch]

    # layout: leave right space for bars
    fig.subplots_adjust(left=0.02, right=0.92, top=0.96, bottom=0.04)
    anim = FuncAnimation(fig, update, frames=8, interval=900, blit=False, repeat=False)
    plt.show()
    return anim


if __name__ == "__main__":
    demo(save_path=os.path.join(SRC_DIR, "planning_snapshot.png"))