'''
TODO:

child of logger?

1. run-time plots
2. esc to end
2. save results after execution

https://stackoverflow.com/questions/56087708/how-to-plotting-online-with-pyplot -> doesn't work
https://gist.github.com/dwf/1222883
https://stackoverflow.com/questions/43571924/multiprocessing-show-matplotlib-plot
https://gist.github.com/iwatake2222/2d2a39ac062e6b823822412ef5f1c1ba
https://stackoverflow.com/questions/67111498/matplotlib-animate-in-python-using-multiprocessing
'''
import matplotlib.pyplot as plt
import numpy as np

def plot(t, x=None, v=None, u=None):
    dpi = 120
    width = 600
    height = 1200
    figsize = (width / dpi, height / dpi)
    _, ax = plt.subplots(4, 1, figsize=figsize, dpi=dpi, sharex=True)

    ax[0].set_title('position')
    ax[0].set_ylabel('m')

    ax[1].set_title('quaternion')
    ax[1].set_ylabel('')

    ax[2].set_title('velocity')
    ax[2].set_ylabel('m / s')

    ax[3].set_title('control')
    ax[3].set_ylabel('')

    if x is not None:
        x = np.array(x)
        pos = x[:, :3]
        q = x[:, 3:]
        ax[0].plot(t, pos)
        ax[1].plot(t, q)

    if v is not None:
        ax[2].plot(t, v)

    if u is not None:
        ax[3].plot(t, u)
    plt.show()