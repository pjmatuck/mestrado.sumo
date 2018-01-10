import matplotlib.pyplot as plt
import  matplotlib.animation as animation
from matplotlib import style

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)

bad_chars = '()'

def animate(i):
    graph_data = open('output/arrived_veh_data.xml','r').read()
    lines = graph_data.split('\n')
    xs = []
    ys = []
    for line in lines:
        if len(line) > 1:
            line = line.replace('(',"")
            line = line.replace(')',"")
            # line = line.replace(' ',"")
            x, y = line.split(',')
            xs.append(int(x))
            ys.append(int(y))
    ax1.clear()
    ax1.plot(xs,ys)

ani = animation.FuncAnimation(fig, animate, interval=1000)
plt.show()