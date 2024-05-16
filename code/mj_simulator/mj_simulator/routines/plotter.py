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


def plot(t, x=None, v=None, u=None, x_des=None, separate=False):
    dpi = 120
    width = 1280
    height = 720
    figsize = (width / dpi, height / dpi)

    for i in range(len(x)):
        if type(x[i]) is not int:
            x = x[i:]
            break

    for i in range(len(v)):
        if type(v[i]) is not int:
            v = v[i:]
            break

    for i in range(len(u)):
        if type(u[i]) is not int:
            u = u[i:]
            break

    x = np.array(x) if x is not None else x
    v = np.array(v) if v is not None else v
    u = np.array(u) if u is not None else u

    N = min([len(e) for e in [t, x, v, u] if e is not None])

    if separate:
        if x is not None:
            pos = x[:, :3]
            q = x[:, 3:]
            plt.title('Position')
            plt.ylabel('m')
            plt.xlabel('time, s')
            plt.plot(t[:N], pos[:N, :], label=[r'$p_x$', r'$p_y$', r'$p_z$'])
            plt.legend()
            plt.show()

            plt.title('Quaternion')
            plt.xlabel('time, s')
            plt.plot(t[:N], q[:N, :], label=[
                        r'$q_0$', r'$q_1$', r'$q_2$', r'$q_3$'])
            plt.legend()
            plt.show()

        if v is not None:
            plt.title('Velocity')
            plt.ylabel('m/s')
            plt.xlabel('time, s')
            plt.plot(t[:N], v[:N, :], label=[r'$v_x$', r'$v_y$',
                        r'$v_z$', r'$w_x$', r'$w_y$', r'$w_z$'])
            plt.legend()
            plt.show()

        if u is not None:
            plt.title('Control')
            plt.xlabel('time, s')
            plt.plot(t[:N], u[:N, :])
            plt.legend()
            plt.show()
    else:

        fig, ax = plt.subplots(2, 2, figsize=figsize, dpi=dpi, sharex=True)
        fig.suptitle("ROV state plot")

        ax[0][0].set_title('position')
        ax[0][0].set_ylabel('m')

        ax[0][1].set_title('quaternion')
        ax[0][1].set_ylabel('')

        ax[1][0].set_title('velocity')
        ax[1][0].set_ylabel('m / s')
        ax[1][0].set_xlabel('time, s')

        ax[1][1].set_title('control')
        ax[1][1].set_ylabel('')
        ax[1][1].set_xlabel('time, s')

        if x is not None:
            pos = x[:, :3]
            q = x[:, 3:]
            ax[0][0].plot(t[:N], pos[:N, :], label=[r'$p_x$', r'$p_y$', r'$p_z$'])
            ax[0][0].axhline(y = 0, color = 'k', linestyle = '--', alpha=0.5)
            ax[0][0].grid()
            ax[0][1].plot(t[:N], q[:N, :], label=[
                        r'$q_0$', r'$q_1$', r'$q_2$', r'$q_3$'])
            ax[0][0].legend()
            ax[0][1].legend()
            ax[0][1].grid()

        if v is not None:
            ax[1][0].plot(t[:N], v[:N, :], label=[r'$v_x$', r'$v_y$',
                        r'$v_z$', r'$w_x$', r'$w_y$', r'$w_z$'])
            ax[1][0].legend()
            ax[1][0].grid()

        if u is not None:
            ax[1][1].plot(t[:N], u[:N, :])
            ax[1][1].grid()

        plt.show()

        plt.plot(t[900:N], pos[900:N, :], label=[r'$p_x$', r'$p_y$', r'$p_z$'])
        plt.legend()
        plt.grid()

        plt.show()

def plot_trajectory(x=None, x_des=None):

    if type(x[0]) is int:
        x = x[1:]

    x = np.array(x) if x is not None else x
    x_des = np.array(x_des) if x is not None else x_des

    if x is not None and x_des is not None:
        pos = x[:, :2]
        pos_des= x_des[:, :2]
        plt.title('Trajectory')
        plt.ylabel('y, m')
        plt.xlabel('x, m')
        plt.plot(pos[:, 0], pos[:, 1], label='real')
        plt.plot(pos_des[:, 0], pos_des[:, 1], label='des')
        plt.legend()
        plt.show()