# Cruise Control System using PID Controller

This project implements a PID-based Cruise Control System for a vehicle.  
The controller regulates the vehicle speed in the presence of aerodynamic drag, rolling resistance, and road-slope disturbances.
- The main goal in designing the cruisecontrol algorithm is to maintain 
  vehicle speed smoothly but accurately.

#How to Run the Project?
You can open and inspect the C++ implementation included in this repository, but to actually run and test the system, you will need MATLAB and Simulink.
C++ code will be compiled within the C block in Simulink automatically.
In the Simulink model, set the desired reference speed in the Step block. Running the simulation will show the system response in the Scope block, allowing 
you to see how the vehicle tracks the reference input.

## System (Car) Modeling

### Non-linear Longitudinal Vehicle Model
$$
m\frac{du}{dt} = F_x - mg\sin\theta - f\, mg\cos\theta - \frac{1}{2}\rho A C_d (u + u_w)^2
$$

Where:  
- \(u\): vehicle speed  
- \(F_x\): traction force  
- \(\theta\): road slope angle  
- \(u_w\): wind speed  

### Linearization around an operating point
Let  
$$
u = u_0 + \delta u, \quad F_x = F_{x0} + \delta F_x, \quad \theta = \theta_0 + \delta\theta
$$

Using first-order Taylor expansion (keeping linear terms):

$$
m \, \delta\dot{u} = \delta F_x - mg \cos\theta_0 \, \delta\theta+ f \, m g \sin\theta_0 \, \delta\theta- \rho A C_d (u_0 + u_w) \, \delta u
$$


### Linear first-order model (Laplace domain)
After rearranging and taking Laplace transform:

$$
(\tau s + 1)\,\Delta U(s) = K \big(\Delta F_x(s) + D(s)\big)
$$

or in time-domain approximation:

$$
\tau \,\delta\dot{u} + \delta u = K(\delta F_x + d)
$$

with

$$
\tau = \frac{m}{\rho C_d A (u_0 + u_w)}, \qquad
K = \frac{1}{\rho C_d A (u_0 + u_w)}
$$

Disturbance:

$$
d = mg (\sin\theta_0 - \cos\theta_0)\,\delta\theta
$$

## System Parameters (example values)
| Parameter | Value | Description |
|---|---:|---|
| \(u_0\) | 20 m/s | nominal speed |
| \(m\) | 1000 kg | vehicle mass |
| \(A\) | 1 m² | frontal area |
| \(C_d\) | 0.5 | drag coefficient |
| \(\rho\) | 1.202 kg/m³ | air density |
| \(f\) | 0.015 | rolling resistance |
| \(g\) | 9.81 m/s² | gravity |
| \(u_w\) | 2 m/s | wind speed |
| \(\theta_0\) | 0 rad | slope angle |

## PID Controller
Controller:

$$
C(s) = PID(s) = k_p + k_d s + \frac{k_i}{s}
$$

$$
L(s) = C(s) G(s) \quad \text{with} \quad G(s) = \frac{K}{\tau s + 1}
$$

Closed-loop denominator:

$$
s(\tau s + 1) + K\big( k_p s + k_d s^2 + k_i \big) = 0
$$

Collect terms → characteristic polynomial:

$$
(\tau + K k_d)\, s^2 + (1 + K k_p)\, s + K k_i = 0
$$

Normalize  to match standard second-order:

$$
s^2 + 2\zeta\omega_n s + \omega_n^2 = 0
$$
-Thus, the controller gains (i.e., parameters KPand KI) must be selected to achievegood performance for the closed-loop system. 
This requires consideration of a numberof factors, such as stability, steady-state errors, and transient response

Thus coefficient matching gives:

$$
2\zeta\omega_n = \frac{1 + K k_p}{\tau + K k_d}, \qquad
\omega_n^2 = \frac{K k_i}{\tau + K k_d}
$$

From these we solve for controller gains (for a chosen \(k_d\)):

$$
k_p = \frac{2\zeta\omega_n(\tau + K k_d) - 1}{K},\qquad
k_i = \frac{\omega_n^2(\tau + K k_d)}{K},\qquad
k_d= \frac{1 - \tau}{K}\
$$
$$


## Design criteria 
- Settling time \(T_s = 3\) second and zeta=5 (because we want the system to be critically damped) are choosen as design requirement (settling time and system response) and from the 
  given below equation  we can easily compute our natural frequency of the system. Then by just putting those values basicly, PID 
  coefficients can be calculated in our project
- Relation: $$
\left( T_s \approx \frac{4}{\zeta \omega_n} \right)
$$

## Simulation
- The simulation is done in MATLAB / Simulink with C++ coder block implemented in simulink model 

### Euler Solver Description
The system is simulated using a fixed-step solver in Simulink. Specifically, we use **ODE1 (Euler method)** with a **fixed step size of 0.01 seconds**. This simple first-order method integrates the system equations at each time step, providing us computationally efficient solution. 

> Note: While Euler method is easy to implement, smaller step sizes improve accuracy, especially for fast-changing or nonlinear systems.


## References
1. Lecture notes: "Vehicle Dynamics and Control", Yildiz Technical University.  
2. A. G. Ulsoy, H. Peng, M. Cakmaci, Automotive Control Systems, Cambridge University Press.
