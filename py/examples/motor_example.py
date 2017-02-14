import motor
import matplotlib.pyplot as plt

if __name__ == '__main__':

    ti = 0.
    tf = 3.
    ts = 0.002

    Kf = 0.1  # Friction Coefficient.
    Kb = 0.01  # Back-emf Coefficient.
    L = 0.5  # Armature Inductance.
    R = 0.1  # Resistance.
    J = 0.01  # Rotor Inertia.
    Kt = 0.01  # Torque Constant.

    m = motor.IdealDCMotor(ts, R, L, J, Kt, Kb, Kf)
    t, x, v = motor.motor_sim(m, ti, tf, ts, plot=False, input=24.)
    plt.plot(t, x)
    plt.legend(['position', 'veloctiy', 'current'], loc='best')
    plt.show()
