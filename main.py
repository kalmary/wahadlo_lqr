import numpy as np
import control as ct
import matplotlib.pyplot as plt
import matplotlib.animation as anime

# parameters
g = 9.81  # gravity [kg*m/s^2]
m = 1  # mass [kg]
l = 0.5  # length [m]
max_theta=10
max_omega=10

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
    R = np.ones((1, 1), dtype=float)
    Q = np.ones((2, 2), dtype=float)
    return R, Q


def destination(st1, st2, a, b, c, d, q, r):
    st_error = st1 - st2
    K, _, _ = ct.lqr(a, b, q, r)  # K, S, E
    u_star = K @ st_error
    return u_star


def act_state(a, state_v_min1, b, input_v_min1):

    state_estimation = (a @ state_v_min1) + (b @ input_v_min1)
    return state_estimation


# def graph():

def main():
    A, B, C, D = arr_abcd(m, g, l)
    R, Q = cost_eff()
    st1 = np.array([10 * deg2rad, np.sqrt(g / l)])  # start <--actual
    st2 = np.array([0, 0])  # finish

    for i in range(0, 100):
        print(f"Iterarion={i}   Current state={st1}     Desired state={st2}")
        state_error = st1 - st2
        st_err_magn = np.linalg.norm(state_error)
        print(f"State Error Magnitute={st_err_magn}")
        opt_ctrl_input = destination(st1, st2, A, B, C, D, Q, R)
        st1 = act_state(A, st1, B, opt_ctrl_input)

        if st_err_magn<1e-10:
            print("Goal Reached!")
            break;


if __name__ == "__main__":
    main()
