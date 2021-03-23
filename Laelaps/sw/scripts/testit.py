import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as ln
import matplotlib.animation as animation

example = 4

if example == 1:
  def update_line(num, data, line):
    #print('dbg', num, data, line)
    line.set_data(data[...,:num])
    return line,

  fig1 = plt.figure()

  data = np.random.rand(2, 25)
  #print('dbg', data)
  l, = plt.plot([], [], 'r-')
  plt.xlim(0, 1)
  plt.ylim(0, 1)
  plt.xlabel('x')
  plt.title('test')
  line_ani = animation.FuncAnimation(fig1, update_line, 25, fargs=(data, l),
    interval=1000, blit=True)
  #line_ani.save('lines.mp4')


elif example == 2:
  fig2 = plt.figure()

  x = np.arange(-9, 10)
  y = np.arange(-9, 10).reshape(-1, 1)
  base = np.hypot(x, y)
  ims = []
  for add in np.arange(15):
    ims.append((plt.pcolor(x, y, base + add, norm=plt.Normalize(0, 30)),))

  im_ani = animation.ArtistAnimation(fig2, ims, interval=50, repeat_delay=3000,
      blit=True)
  #im_ani.save('im.mp4', metadata={'artist':'Guido'})

elif example == 3:
  class Scope:
    def __init__(self, ax, maxt=2, dt=0.02):
        self.ax = ax
        self.dt = dt
        self.maxt = maxt
        self.tdata = [0]
        self.ydata = [0]
        self.line = ln.Line2D(self.tdata, self.ydata)
        self.ax.add_line(self.line)
        self.ax.set_ylim(-.1, 1.1)
        self.ax.set_xlim(0, self.maxt)

    def update(self, y):
        lastt = self.tdata[-1]
        if lastt > self.tdata[0] + self.maxt: # reset the arrays
            self.tdata = [self.tdata[-1]]
            self.ydata = [self.ydata[-1]]
            self.ax.set_xlim(self.tdata[0], self.tdata[0] + self.maxt)
            self.ax.figure.canvas.draw()

        t = self.tdata[-1] + self.dt
        self.tdata.append(t)
        self.ydata.append(y)
        self.line.set_data(self.tdata, self.ydata)
        return self.line,

  def emitter(p=0.03):
    'return a random value with probability p, else 0'
    while True:
        v = np.random.rand(1)
        if v > p:
            yield 0.
        else:
            yield np.random.rand(1)

  fig, ax = plt.subplots()
  scope = Scope(ax)

  # pass a generator in "emitter" to produce data for the update func
  ani = animation.FuncAnimation(fig, scope.update, emitter, interval=10,
    blit=True)

elif example == 4:
  class Scope:
    def __init__(self, ax, maxt=2, dt=0.02):
        self.ax = ax
        self.dt = dt
        self.maxt = maxt
        self.tdata = [0]
        self.ydata = [0]
        self.line = ln.Line2D(self.tdata, self.ydata)
        self.ax.add_line(self.line)
        self.ax.set_ylim(-.1, 1.1)
        self.ax.set_xlim(0, self.maxt)

    def update(self, y):
        lastt = self.tdata[-1]
        if lastt > self.tdata[0] + self.maxt: # reset the arrays
          self.tdata = self.tdata[1:]
          self.ydata = self.ydata[1:]
          self.ax.set_xlim(self.tdata[0], self.tdata[0] + self.maxt)
          self.ax.figure.canvas.draw()

        t = self.tdata[-1] + self.dt
        self.tdata.append(t)
        self.ydata.append(y)
        self.line.set_data(self.tdata, self.ydata)
        return self.line,

  def emitter(p=0.03):
    'return a random value with probability p, else 0'
    while True:
        v = np.random.rand(1)
        if v > p:
            yield 0.
        else:
            yield np.random.rand(1)

  fig, ax = plt.subplots()
  scope = Scope(ax)

  # pass a generator in "emitter" to produce data for the update func
  ani = animation.FuncAnimation(fig, scope.update, emitter, interval=10,
    blit=True)


#-----------------------------
plt.show()
