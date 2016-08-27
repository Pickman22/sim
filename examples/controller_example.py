from motor import motor
from controller import controller
import matplotlib.pyplot as plt

if __name__ == '__main__':

    def position_pid(x, t, *args):
        pid, = args
        return pid.control(x[0])  # Uses position as feedback.

    def velocity_pid(x, t, *args):
        pid, = args
        return pid.control(x[1])  # Uses velocity as feedback.

    ti = 0.
    tf = 5.
    ts = 0.001
    t = ti

    Ts = 2  # Settling time.
    Pos = 0.05  # Maximum overshoot of 5%.

    R = 1.  # Armature resistance.
    Kf = 0.1  # Friction coefficient.
    Kb = 0.01  # Back-emf constant.
    L = 0.05  # Armature inductance.
    Kt = 0.01  # Torque constant.
    J = 0.1  # Rotor's moment of inertia.

    m = motor.DCMotor(ts, R, L, J, Kt, Kf, Kb)
    vel_pid = controller.velocity_controller(Ts, Pos, Kf=Kf, Kb=Kb, L=L, R=R,
                                             J=J, Kt=Kt)
    pos_pid = controller.position_controller(Ts, Pos, Kf=Kf, Kb=Kb, L=L, R=R,
                                             J=J, Kt=Kt)
    pos_pid.ts = ts
    vel_pid.ts = ts
    pos_pid.target = 1.
    vel_pid.target = 1.

    pt, px, pv = motor.motor_sim(m, ti, tf, ts, pos_pid, on_input=position_pid)
    vt, vx, vv = motor.motor_sim(m, ti, tf, ts, vel_pid, on_input=velocity_pid)

    fig, ax = plt.subplots(2, 2)

    ax[0, 0].plot(pt, px[:, 0:2])
    ax[0, 0].legend(['Position', 'Velocity'])
    ax[0, 0].set_title('Position Controller')
    ax[1, 0].plot(pt, pv, label='Voltage')
    ax[1, 0].set_title('Control Signal')
    ax[1, 0].legend()

    ax[0, 1].plot(vt, vx[:, 1], label='Velocity')
    ax[0, 1].set_title('Velocity Controller')
    ax[0, 1].legend()
    ax[1, 1].plot(vt, vv, label='Voltage')
    ax[1, 1].set_title('Control Signal')
    ax[1, 1].legend()
    plt.show()
