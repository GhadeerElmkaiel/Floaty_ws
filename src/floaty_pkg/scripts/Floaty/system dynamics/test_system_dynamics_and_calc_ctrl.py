import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Define the second order system dynamics
def second_order_system(t, state, omega_n, zeta):
    """
    Defines the second order system dynamics.

    Args:
    - t (float): Time.
    - state (list): State variables [position, velocity].
    - omega_n (float): Natural frequency.
    - zeta (float): Damping ratio.

    Returns:
    - dydt (list): Derivative of state variables [velocity, acceleration].
    """
    position, velocity = state
    dstate_dt = [velocity, -2 * zeta * omega_n * velocity - omega_n**2 * position]
    return dstate_dt


def calc_ctrl_params_second(omega_n, zeta, a, b):
    x_dot_param = -(2*zeta*omega_n+b)/a
    x_param = -(omega_n**2)/a
    return [x_param, x_dot_param]


def third_order_system(t, state, omega_n, zeta, gama, ang_to_acc):
    """
    Defines the second order system dynamics.

    Args:
    - t (float): Time.
    - state (list): State variables [velocity, angle, angle_rate].
    - omega_n (float): Natural frequency.
    - zeta (float): Damping ratio.
    - gama (float): Additional damping ratio.

    Returns:
    - dydt (list): Derivative of state variables [acceleration, angle_rate, angle_acc].
    """
    velocity, angle, angle_rate = state
    angle_acc = -(2*zeta*omega_n + gama)*angle_rate - (2*zeta*omega_n*gama + omega_n**2)*angle - (omega_n**2*gama/ang_to_acc)*velocity
    dstate_dt = [angle*ang_to_acc, angle_rate, angle_acc]
    return dstate_dt


def calc_ctrl_params_third(omega_n, zeta, gama, a, b, c, d, e):
    angle_rate_param = -(b + 2*zeta*omega_n + gama)/a
    angle_param = -(c + 2*zeta*omega_n*gama + omega_n**2)/a
    vel_param = -(d + omega_n**2*gama/e)/a
    return [vel_param, angle_param, angle_rate_param]

# ============================ Pitch and x =============================

# Define simulation parameters

# omega_n = 3.5  # Natural frequency
# zeta = 1.0   # Damping ratio
# gama = 2.5   # Damping ratio
omega_n = np.sqrt(80)+1  # Natural frequency
zeta = 0.7   # Damping ratio
gama = 1.7   # Damping ratio
gama = 0   # Damping ratio
# ang_to_acc = 2
a = -68
b = -0.85
c = -80
d = -9.25
e = 5.86 # ang_to_acc

params = calc_ctrl_params_third(omega_n, zeta, gama, a, b, c, d, e)
print(params)


t_span = (0, 10)  # Simulation time span
# initial_conditions = [1.0, 0.0]  # Initial state [position, velocity]
initial_conditions = [0.0, 0.4, 0.0]  # Initial state [velocity, angle, angle_rate]

# Perform simulation
# sol = solve_ivp(lambda t, y: second_order_system(t, y, omega_n, zeta), t_span, initial_conditions, t_eval=np.linspace(0, 10, 100))
sol = solve_ivp(lambda t, y: third_order_system(t, y, omega_n, zeta, gama, e), t_span, initial_conditions, t_eval=np.linspace(0, 10, 200))

# Plot results
plt.figure(figsize=(10, 5))
# plt.plot(sol.t, sol.y[0], label='Position')
# plt.plot(sol.t, sol.y[1], label='Velocity')
plt.plot(sol.t, sol.y[0], label='Velocity')
plt.plot(sol.t, sol.y[1], label='angle')
plt.plot(sol.t, sol.y[2], label='angle_rate')
plt.xlabel('Time')
plt.ylabel('Response')
plt.title('Third Order System Response')
plt.legend()
plt.grid(True)
plt.show()

print("LQR_K(:,4) = [{}; {}; {}; {}]".format(-params[0]/2, params[0]/2, params[0]/2, -params[0]/2))
print("LQR_K(:,8) = [{}; {}; {}; {}]".format(-params[1]/2, params[1]/2, params[1]/2, -params[1]/2))
print("LQR_K(:,11) = [{}; {}; {}; {}]".format(-params[2]/2, params[2]/2, params[2]/2, -params[2]/2))


# ============================ Roll and y =============================

# Define simulation parameters

# omega_n = 3.5  # Natural frequency
# zeta = 1.0   # Damping ratio
# gama = 2.5   # Damping ratio
omega_n = 8.8 # Natural frequency
zeta = 0.2  # Damping ratio
gama = 0.5   # Damping ratio
# ang_to_acc = 2
# a = -0.324
a = -9
b = -2.6
c = -88
d = 4.25
e = -7.36 # ang_to_acc

# a = -9
# b = 0
# c = 0
# d = 0
# e = -7.36 # ang_to_acc

params = calc_ctrl_params_third(omega_n, zeta, gama, a, b, c, d, e)
print(params)


t_span = (0, 10)  # Simulation time span
# initial_conditions = [1.0, 0.0]  # Initial state [position, velocity]
initial_conditions = [0.0, 0.0, 0.1]  # Initial state [velocity, angle, angle_rate]

# Perform simulation
# sol = solve_ivp(lambda t, y: second_order_system(t, y, omega_n, zeta), t_span, initial_conditions, t_eval=np.linspace(0, 10, 100))
sol = solve_ivp(lambda t, y: third_order_system(t, y, omega_n, zeta, gama, e), t_span, initial_conditions, t_eval=np.linspace(0, 10, 200))

# Plot results
plt.figure(figsize=(10, 5))
# plt.plot(sol.t, sol.y[0], label='Position')
# plt.plot(sol.t, sol.y[1], label='Velocity')
plt.plot(sol.t, sol.y[0], label='Velocity')
plt.plot(sol.t, sol.y[1], label='angle')
plt.plot(sol.t, sol.y[2], label='angle_rate')
plt.xlabel('Time')
plt.ylabel('Response')
plt.title('Third Order System Response')
plt.legend()
plt.grid(True)
plt.show()

# print("LQR_K(:,5) = [{}; {}; {}; {}]".format(params[0]/2, params[0]/2, -params[0]/2, -params[0]/2))
# print("LQR_K(:,7) = [{}; {}; {}; {}]".format(params[1]/2, params[1]/2, -params[1]/2, -params[1]/2))
# print("LQR_K(:,10) = [{}; {}; {}; {}]".format(params[2]/2, params[2]/2, -params[2]/2, -params[2]/2))

print("LQR_K(:,5) = [{}; {}; {}; {}]".format(-params[0]/2, -params[0]/2, params[0]/2, params[0]/2))
print("LQR_K(:,7) = [{}; {}; {}; {}]".format(-params[1]/2, -params[1]/2, params[1]/2, params[1]/2))
print("LQR_K(:,10) = [{}; {}; {}; {}]".format(-params[2]/2, -params[2]/2, params[2]/2, params[2]/2))


# ============================ Yaw =============================

omega_n = 4  # Natural frequency
zeta = 0.3   # Damping ratio
a = 22.2
b = -0.2054

params = calc_ctrl_params_second(omega_n, zeta, a, b)
print(params)


t_span = (0, 10)  # Simulation time span
# initial_conditions = [1.0, 0.0]  # Initial state [position, velocity]
initial_conditions = [0.3, 0.1]  # Initial state [velocity, angle, angle_rate]

# Perform simulation
# sol = solve_ivp(lambda t, y: second_order_system(t, y, omega_n, zeta), t_span, initial_conditions, t_eval=np.linspace(0, 10, 100))
sol = solve_ivp(lambda t, y: second_order_system(t, y, omega_n, zeta), t_span, initial_conditions, t_eval=np.linspace(0, 10, 200))


# Plot results
plt.figure(figsize=(10, 5))
# plt.plot(sol.t, sol.y[0], label='Position')
# plt.plot(sol.t, sol.y[1], label='Velocity')
plt.plot(sol.t, sol.y[0], label='Yaw')
plt.plot(sol.t, sol.y[1], label='Yaw_dot')
plt.xlabel('Time')
plt.ylabel('Response')
plt.title('Second Order System Response')
plt.legend()
plt.grid(True)
plt.show()

print("LQR_K(:,9) = [{}; {}; {}; {}]".format(params[0]/2, params[0]/2, params[0]/2, params[0]/2))
print("LQR_K(:,12) = [{}; {}; {}; {}]".format(params[1]/2, params[1]/2, params[1]/2, params[1]/2))

# ============================ Z =============================

omega_n = 2  # Natural frequency
zeta = 1.1  # Damping ratio
a = 1.3
# b = -2.2591
b = -5.625

params = calc_ctrl_params_second(omega_n, zeta, a, b)
print(params)


t_span = (0, 10)  # Simulation time span
# initial_conditions = [1.0, 0.0]  # Initial state [position, velocity]
initial_conditions = [-0.2, 0.0]  # Initial state [velocity, angle, angle_rate]

# Perform simulation
# sol = solve_ivp(lambda t, y: second_order_system(t, y, omega_n, zeta), t_span, initial_conditions, t_eval=np.linspace(0, 10, 100))
sol = solve_ivp(lambda t, y: second_order_system(t, y, omega_n, zeta), t_span, initial_conditions, t_eval=np.linspace(0, 10, 200))


# Plot results
plt.figure(figsize=(10, 5))
# plt.plot(sol.t, sol.y[0], label='Position')
# plt.plot(sol.t, sol.y[1], label='Velocity')
plt.plot(sol.t, sol.y[0], label='Z')
plt.plot(sol.t, sol.y[1], label='Z_dot')
plt.xlabel('Time')
plt.ylabel('Response')
plt.title('Second Order System Response')
plt.legend()
plt.grid(True)
plt.show()

# I am not sure about the signals
print("LQR_K(:,3) = [{}; {}; {}; {}]".format(params[0]/2, -params[0]/2, params[0]/2, -params[0]/2))
print("LQR_K(:,6) = [{}; {}; {}; {}]".format(params[1]/2, -params[1]/2, params[1]/2, -params[1]/2))