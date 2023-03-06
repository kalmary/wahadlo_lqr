import numpy as np
import control as ct
import matplotlib.pyplot as plt
import matplotlib.animation as anime
import time

# parameters
g = 9.81  # gravity [kg*m/s^2]
m = 1  # mass [kg]
l = 0.5  # length [m]
max_theta = 10
max_omega = 10

deg2rad = np.pi / 180
rad2deg = 180 / np.pi

np.set_printoptions(precision=3, suppress=True)


def arr_abcd(m, g, l):
    A = np.array([[0, 1],
                  [-g / l, 0]])

    B = np.array([[0],
                  [1 / (m * pow(l, 2))]])
    C = np.array([[1],
                  [0]])
    D = 0
    return A, B, C, D


def cost_eff():  # state cost Q matrix (penalizes bad performance) and input cost R matrix (penalizes the effort)
    R = [0.001]
    Q = [[0.001, 0.001],
         [0.001, 0.001]]
    return R, Q


def destination(state_error, a, b, c, d, q, r):
    K, _, _ = ct.lqr(a, b, q, r)  # K, S, E
    u_star = K @ state_error
    return u_star


def act_state(a, state_v_min1, b, input_v_min1):
    state_estimation = (a @ state_v_min1) + (b @ input_v_min1)
    return state_estimation


def graph(graph_err1, graph_err2, t):
    plt.title('State error')
    plt.xlabel('time [s/10]')
    plt.plot(t, graph_err1, color='blue')
    plt.plot(t, graph_err2, color='red')
    plt.show()

def main():
    A, B, C, D = arr_abcd(m, g, l)
    R, Q = cost_eff()
    dt=0.1
    st1 = np.array([10 * deg2rad, np.sqrt(g / l)])  # start <--actual
    st2 = np.array([0, 0])  # finish
    graph_err1=[]
    graph_err2=[]
    t=0
    t_list=[]
    for i in range(0, 5):
        print(f"Iterarion={i}   Current state={st1}     Desired state={st2}")
        graph_err1.append(st1[0]-st2[0])
        graph_err2.append(st1[1]-st2[1])
        state_error=[graph_err1[i], graph_err2[i]]
        print(f"State Error={state_error}")
        opt_ctrl_input = destination(state_error, A, B, C, D, Q, R)
        st1 = act_state(A, st1, B, opt_ctrl_input)

        time.sleep(dt)
        t_list.append(t)
        t+=dt
        if -1e-10 < state_error[0] < 1e-10 and -1e-10 < state_error[1] < 1e-10:
            print("Goal Reached!")
            break
    graph(graph_err1, graph_err2, t_list)

if __name__ == "__main__":
    main()