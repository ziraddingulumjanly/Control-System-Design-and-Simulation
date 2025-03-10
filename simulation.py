import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

A = np.array([[-0.0499,  0.0499,   0,       0],
              [ 0.0499, -0.0667,   0,       0],
              [ 0,       0,      -0.0251,   0],
              [ 0,       0,       0.0335,  -0.0335]])

B = np.array([[0.00510,  0],
              [0,         0],
              [0.0377,    0],
              [0,         0]])
B_eff = B[:, 0]  # shape (4,)

C = np.array([[0, 2, 0, 0],
              [0, 0, 0, 0.1]])


Kp1 = 2.5982
Ki1 = 0.0332
Kd1 = 29.0047
tau1 = 1e-3  

Kp2 = 13.7632
Ki2 = 0.2754
Kd2 = 153.4397
tau2 = 1e-3



def closed_loop_ode(t, state):
    x = state[0:4]
    i1 = state[4]    # integrator for H2
    d1 = state[5]    # derivative filter state for H2
    i2 = state[6]    # integrator for T2
    d2 = state[7]    # derivative filter state for T2

    # Outputs:
    y1 = 2.0 * x[1]
    y2 = 0.1 * x[3]

    # Step changes in reference:

    r1 = 1.519 if t < 100 else 1.6
    r2 = 45.0  if t < 500 else 48.0

    # Tracking errors
    e1 = r1 - y1
    e2 = r2 - y2

    # PID outputs for each loop
    u1 = Kp1*e1 + Ki1*i1 + Kd1*d1
    u2 = Kp2*e2 + Ki2*i2 + Kd2*d2

    # Summation of both controllers' outputs
    u = u1 + u2

    # Plant dynamics: dx/dt = A*x + B_eff * u
    dxdt = A @ x + B_eff * u

    dy1_dt = 2.0 * dxdt[1]
    dy2_dt = 0.1 * dxdt[3]

    # Integrators
    di1_dt = e1
    di2_dt = e2

    # Derivative filters:
    dd1_dt = (-d1 - dy1_dt) / tau1
    dd2_dt = (-d2 - dy2_dt) / tau2

    # Combine derivatives
    dstatedt = np.zeros(8)
    dstatedt[0:4] = dxdt
    dstatedt[4] = di1_dt
    dstatedt[5] = dd1_dt
    dstatedt[6] = di2_dt
    dstatedt[7] = dd2_dt

    return dstatedt
# Simulation Setup

t_start = 0.0
t_final = 600.0
t_eval = np.linspace(t_start, t_final, 6001)  # 0.1s steps

x0_plant = np.array([0, 0.7595, 0, 450])

# Initial controller states (i1, d1, i2, d2) = 0
ctrl_init = np.zeros(4)

# Full initial state
init_state = np.concatenate((x0_plant, ctrl_init))

# Solve ODE
sol = solve_ivp(
    closed_loop_ode, 
    [t_start, t_final], 
    init_state, 
    t_eval=t_eval, 
    method='RK45'
)

# Extract solution
t = sol.t
x_hist = sol.y[0:4, :]  # plant states over time

# Compute outputs
H2 = 2.0 * x_hist[1, :]   # Water level
T2 = 0.1 * x_hist[3, :]   # Temperature

# Plot
plt.figure(figsize=(10, 8))

# Water Level
plt.subplot(2, 1, 1)
plt.plot(t, H2, 'b-', label='H2 (Water Level)')
plt.axvline(100, color='k', linestyle='--', label='r1 step at t=100s')
plt.ylabel('H2 (m)')
plt.legend()
plt.title('Closed-Loop Response with Summed PID Controllers')

# Temperature
plt.subplot(2, 1, 2)
plt.plot(t, T2, 'r-', label='T2 (Temperature)')
plt.axvline(500, color='k', linestyle='--', label='r2 step at t=500s')
plt.xlabel('Time (s)')
plt.ylabel('T2 (°C)')
plt.legend()

plt.tight_layout()

plt.savefig("CombinedPID_Response.pdf", dpi=600, format="pdf")
plt.show()
