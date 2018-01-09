import matplotlib.pyplot as plt
import  matplotlib.animation as animation
from matplotlib import style

class Chart:
    chart_data = []

    style.use('default')
    fig = plt.figure()
    ax1 = fig.add_subplot(1,1,1)

    def set_chart_data(self, data):
        self.chart_data = data

    def animate(self, i):
        xs = []
        ys = []
        for data in self.chart_data:
            x, y = data
            xs.append(x)
            ys.append(y)
        self.ax1.clear()
        self.ax1.plot(xs,ys)

    def draw(self):
        ani = animation.FuncAnimation(self.fig, self.animate, interval=500)
        plt.ion()
        mng = plt.get_current_fig_manager()
        mng.window.state('zoomed')
        plt.show()