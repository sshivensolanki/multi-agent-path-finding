#!/usr/bin/env python3
from matplotlib.patches import Circle, Rectangle
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation

COLORS = [
    "#4CC9F0", "#F72585", "#B5179E",
    "#4895EF", "#F9C74F", "#90BE6D",
    "#F3722C", "#577590"
]

class Animation:
    def __init__(self, my_map, starts, goals, paths):

        self.my_map = np.flip(np.transpose(my_map), 1)

        h, w = len(self.my_map), len(self.my_map[0])

        self.starts = [(s[1], w - 1 - s[0]) for s in starts]
        self.goals = [(g[1], w - 1 - g[0]) for g in goals]

        self.paths = [
            [(p[1], w - 1 - p[0]) for p in path]
            for path in paths
        ]

        aspect = h / w
        self.fig = plt.figure(frameon=False, figsize=(6 * aspect, 6))
        self.ax = self.fig.add_subplot(111, aspect='equal')

        self._setup_style(h, w)

        self.patches = []
        self.agents = {}

        self._draw_static_elements(h, w)

        self.T = max(len(p) for p in self.paths) - 1 if self.paths else 0

        self.animation = animation.FuncAnimation(
            self.fig,
            self.animate_func,
            init_func=self.init_func,
            frames=(self.T + 1) * 20,
            interval=25,
            blit=True
        )

    def _setup_style(self, h, w):
        self.ax.set_facecolor("#0f1117")
        self.fig.patch.set_facecolor("#0f1117")

        self.ax.set_xlim(-0.5, h - 0.5)
        self.ax.set_ylim(-0.5, w - 0.5)

        self.ax.set_xticks(np.arange(-0.5, h, 1), minor=True)
        self.ax.set_yticks(np.arange(-0.5, w, 1), minor=True)

        self.ax.set_axisbelow(True)

        self.ax.grid(
            which="minor",
            color="#2a2e39",
            linewidth=0.6,
            alpha=0.6
        )

        self.ax.tick_params(
            left=False,
            bottom=False,
            labelleft=False,
            labelbottom=False
        )

    def _draw_static_elements(self, h, w):

        # border
        self.patches.append(
            Rectangle(
                (-0.5, -0.5), h, w,
                facecolor="none",
                edgecolor="#444",
                linewidth=1.2,
                zorder=2
            )
        )

        # obstacles
        for i in range(h):
            for j in range(w):
                if self.my_map[i][j]:
                    self.patches.append(
                        Rectangle(
                            (i - 0.5, j - 0.5),
                            1, 1,
                            facecolor="#1f2430",
                            edgecolor="#1f2430",
                            zorder=1
                        )
                    )

        # goals
        for i, g in enumerate(self.goals):
            self.patches.append(
                Circle(
                    g,
                    0.22,
                    facecolor=COLORS[i % len(COLORS)],
                    edgecolor="white",
                    alpha=0.25,
                    linewidth=1.0,
                    zorder=3
                )
            )

        # agents
        for i, path in enumerate(self.paths):
            color = COLORS[i % len(COLORS)]

            self.agents[i] = Circle(
                self.starts[i],
                0.32,
                facecolor=color,
                edgecolor="white",
                linewidth=1.2,
                alpha=0.95,
                zorder=10
            )

            self.patches.append(self.agents[i])

    def init_func(self):
        for p in self.patches:
            self.ax.add_patch(p)

        return self.patches

    def animate_func(self, t):
        t = t / 20.0

        for i, path in enumerate(self.paths):
            self.agents[i].center = self.get_state(t, path)

        return self.patches

    @staticmethod
    def get_state(t, path):
        if int(t) <= 0:
            return np.array(path[0])
        elif int(t) >= len(path):
            return np.array(path[-1])
        else:
            a = np.array(path[int(t) - 1])
            b = np.array(path[int(t)])
            return a + (b - a) * (t - int(t))

    def save(self, file_name, speed=1.0):
        self.animation.save(
            file_name,
            fps=int(30 * speed),
            dpi=200,
            savefig_kwargs={"pad_inches": 0}
        )

    def show(self):
        plt.show()
