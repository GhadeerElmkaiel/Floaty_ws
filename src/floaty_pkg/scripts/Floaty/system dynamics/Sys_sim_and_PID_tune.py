import numpy as np
import matplotlib.pyplot as plt
import control as ctrl


roll_A = np.array([[-17.5, -126],
          [1, 0]])
roll_B = np.array([[-380],
          [0]])
roll_C = np.array([[1, 0]])
roll_D = np.array([[0]])



pitch_A = np.array([[3, -62],
          [1, 0]])
pitch_B = np.array([[-140],
          [0]])
pitch_C = np.array([[1, 0]])
pitch_D = np.array([[0]])


# 1. Define the State-Space Model of a Second-Order System
# Let's use a mass-spring-damper system as our example.
# The states are [position, velocity]
# The input is the force applied to the mass.
# The output is the position of the mass.



# A = roll_A
# B = roll_B
# C = roll_C
# D = roll_D

A = pitch_A
B = pitch_B
C = pitch_C
D = pitch_D

# Create the state-space system object
system = ctrl.ss(A, B, C, D)

# 2. Define the LQR Cost Function Matrices (Q and R)
# The LQR cost function is J = integral(x'Qx + u'Ru) dt
# Q penalizes state deviations from the origin.
# R penalizes control effort.

# Q matrix: We want to penalize both position and velocity errors.
# Let's penalize position error more than velocity error.
Q = np.diag([10.0, 1.0])

# R matrix: This is a scalar since we have a single input.
# A smaller R allows for more control effort.
R = np.array([[25]])

# 3. Calculate the Optimal LQR Gain (K)
# The lqr function returns the gain matrix K, the solution to the
# Riccati equation S, and the closed-loop eigenvalues E. [8, 1]
K, S, E = ctrl.lqr(system, Q, R)
print(f"LQR Gain (K): {K}")

# 4. Simulate the Closed-Loop System
# We will simulate the system's response to an initial condition.
# Let's say the system starts at position 1.0 with zero velocity.
initial_state = [1.0, 0.0]

# Create the closed-loop system with the LQR controller
# The new A matrix for the closed-loop system is (A - BK)
A_cl = A - B @ K
closed_loop_system = ctrl.ss(A_cl, B, C, D)

# Time vector for simulation
T = np.linspace(0, 10, 500)

# Simulate the response to the initial condition
t, yout = ctrl.initial_response(closed_loop_system, T=T, X0=initial_state)

# 5. Plot the Results
plt.figure(figsize=(10, 6))
plt.plot(t, yout)
plt.title("LQR Controller Response for a Second-Order System")
plt.xlabel("Time (s)")
plt.ylabel("Position")
plt.grid(True)
plt.show()