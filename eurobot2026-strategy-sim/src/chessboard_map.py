import os
import matplotlib.pyplot as plt
from matplotlib.patches import Patch, Rectangle

def plot_chessboard_map(rows=6, cols=8, bg_img_path="table_bis.png"):
    # crisp board renderer — legend & labels per PROJ703-Presentation.pdf
    if not os.path.isfile(bg_img_path):
        print(f"Error: '{bg_img_path}' not found in {os.getcwd()}")
        return

    # make room on the right for an external legend
    fig_w = cols * 0.9 + 2.0
    fig_h = rows * 0.9
    fig, ax = plt.subplots(figsize=(fig_w, fig_h), dpi=120)
    img = plt.imread(bg_img_path)
    ax.imshow(img, extent=[0, cols, 0, rows], zorder=0)

    elements = [
        ("Squirrels nest (Yellow team)", "Nest Y", (0, 0), "#FFD700"),
        ("Squirrels nest (Blue team)",   "Nest B", (7, 0), "#1E90FF"),
        ("SIMA starting area (granary)", "SIMA",   (1, 0), "#00BFFF"),
        ("Fridge",                       "Fridge", (2, 0), "#4682B4"),
        ("Loading area",                 "Load",   (3, 0), "#FF8C00"),
        ("Collection area",              "Collect",(0, 3), "#32CD32"),
        ("Pantry",                       "Pantry", (7, 3), "#8B4513"),
        ("Thermometer",                  "Thermo", (4, 5), "#DC143C"),
        ("Cursor",                       "Cursor", (7, 5), "#000000"),
    ]

    # thin grid
    for x in range(cols + 1):
        ax.plot([x, x], [0, rows], color='#000000', lw=0.9, zorder=2)
    for y in range(rows + 1):
        ax.plot([0, cols], [y, y], color='#000000', lw=0.9, zorder=2)

    # markers + clipped in-cell labels
    for full, short, (c, r), color in elements:
        cx, cy = c + 0.5, rows - r - 0.5
        ax.scatter([cx], [cy], s=220, color=color, edgecolors='black', zorder=4)
        cell_rect = Rectangle((c, rows - r - 1), 1, 1)
        cell_rect.set_transform(ax.transData)
        cell_rect.set_visible(False)
        ax.add_patch(cell_rect)
        txt = ax.text(
            cx, cy, short,
            ha='center', va='center', fontsize=9, fontweight='bold',
            color='white' if color not in ("#FFD700", "#32CD32") else 'black',
            bbox=dict(facecolor=color, edgecolor='black', boxstyle='round,pad=0.18', alpha=0.95),
            zorder=5
        )
        txt.set_clip_path(cell_rect)

    # axis labels outside
    for i in range(cols):
        ax.text(i + 0.5, -0.28, chr(65 + i), ha='center', va='top', fontsize=13, fontweight='bold', clip_on=False)
    for i in range(rows):
        ax.text(-0.28, rows - i - 0.5, str(i + 1), ha='right', va='center', fontsize=13, fontweight='bold', clip_on=False)

    ax.set_xlim(0, cols)
    ax.set_ylim(0, rows)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_aspect('equal')
    ax.axis('off')

    legend_items = [
        Patch(facecolor="#FFD700", edgecolor='black', label="Squirrels nest (Yellow team)"),
        Patch(facecolor="#1E90FF", edgecolor='black', label="Squirrels nest (Blue team)"),
        Patch(facecolor="#00BFFF", edgecolor='black', label="SIMA starting area (granary)"),
        Patch(facecolor="#4682B4", edgecolor='black', label="Fridge"),
        Patch(facecolor="#FF8C00", edgecolor='black', label="Loading area"),
        Patch(facecolor="#32CD32", edgecolor='black', label="Collection area"),
        Patch(facecolor="#8B4513", edgecolor='black', label="Pantry"),
        Patch(facecolor="#DC143C", edgecolor='black', label="Thermometer"),
        Patch(facecolor="#000000", edgecolor='black', label="Cursor"),
    ]

    # push map left and place legend to the right (outside)
    fig.subplots_adjust(left=0.06, right=0.72, top=0.97, bottom=0.08)
    ax.legend(handles=legend_items, loc='center left',
              bbox_to_anchor=(0.99, 0.5), bbox_transform=fig.transFigure,
              frameon=True, fontsize=10, title="Legend", title_fontsize=11)
    plt.show()

if __name__ == "__main__":
    plot_chessboard_map()