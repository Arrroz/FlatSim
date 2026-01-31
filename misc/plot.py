import matplotlib.pyplot as plt

# TODO: plotting creates GUI conflicts that make the pyglet window smaller
# TODO: move windows accordingly so they don't spawn on top of one another
# TODO: make the pyglet window the active one after creating the plot; otherwise keyboard events aren't captured until the user returns control to that window
class Plot:

    def __init__(self, data: list[tuple[list]]):
        self.data = data

        self.fig, axes = plt.subplots()
        self.markers = [] # type: list[plt.Line2D]

        for d in self.data:
            line, = axes.plot(d[0], d[1])
            if len(d) > 2:
                line.set_label(d[2])

            marker, = axes.plot(d[0][0], d[1][0], "o", color=line.get_color())
            self.markers.append(marker)
        
        axes.legend()

        plt.ion()
        plt.show()
    
    def update(self, i):
        for d, m in zip(self.data, self.markers):
            m.set_data([d[0][i]], [d[1][i]])
        self.fig.canvas.flush_events()

