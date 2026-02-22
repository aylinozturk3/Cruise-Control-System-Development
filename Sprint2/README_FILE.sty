##Cruise Control System using PID Controller
#Sprint 2 – Object-Oriented Principals Implementation

#MOTIVATION
This sprint extends the Cruise Control System developed in Sprint 1 by restructuring the project using 
object oriented programming principles.
The system regulates the longitudinal vehicle speed under the influence of aerodynamic drag, rolling resistance, wind disturbance, and road friction variations. Multiple vehicle, weather, and road configurations are supported.

The primary goal of the cruise control algorithm is to maintain vehicle speed smoothly and accurately under varying operating conditions. We also add real acceleration values from the car companies to make the respond more realistic

##HOW TO RUN CODE?
1. Compile the project using a standard C++ compiler in terminal:
"g++ main.cpp CruiseControl.cpp -o cruise_control"
"./cruise_control"
2. Then you as an user need to choose car type, weather and road conditions as well. Also another important part is to set your desired velocity and simulation time.
3. After compiling the code, you will get a CSV file named "sim_output.csv" as a output. You can directly run the MATLAB code "data_visualization.m" that is given in Sprint2 for visualization. 
There is also ASCII velocity–time visualization in terminal printed as velocity time-series.

##SYSTEM ARCHITECTURE
The software is structured using object-oriented principles:
- CruiseControlSystem:
  *Stores vehicle, environment, and road parameters
  *Computes linearized model coefficients and PID gains
  *Executes closed-loop simulation
  *Limits acceleration via limitAcceleration with the real acceleration values
- EulerSolver:
  *Implements the ODE1(Euler) integration method as class in our code
  *Used for numerical time integration
- Main File:
  *Handles user interactions and provides built-in visualization and data export

###VEHICLE MODEL
We continue to use the same linearized model that is used in Sprint1.

## Non-linear Longitudinal Vehicle Model
$$
m\frac{du}{dt} = F_x - mg\sin\theta - f\, mg\cos\theta - \frac{1}{2}\rho A C_d (u + u_w)^2
$$
Parameters definitions given are as follows:  
- \(u\): vehicle speed  
- \(F_x\): traction force  
- \(\theta\): road slope angle  
- \(u_w\): wind speed  

We linearized it around an operating point
$$
u = u_0 + \delta u, \quad F_x = F_{x0} + \delta F_x, \quad \theta = \theta_0 + \delta\theta
$$
Using first-order Taylor expansion (keeping linear terms):

$$
m \, \delta\dot{u} = \delta F_x - mg \cos\theta_0 \, \delta\theta+ f \, m g \sin\theta_0 \, \delta\theta- \rho A C_d (u_0 + u_w) \, \delta u
$$

Our linearized model after the laplace transform is turning out to be as following:
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

##PID Controller & Coefficient Matching
Controller:

$$
C(s) = PID(s) = k_p + k_d s + \frac{k_i}{s}
$$

$$
L(s) = C(s) G(s) \quad \text{with} \quad G(s) = \frac{K}{\tau s + 1}
$$
Closed-loop denominator becomes as:
$$
(\tau + K k_d)\, s^2 + (1 + K k_p)\, s + K k_i = 0
$$

Normalize  to match standard second-order: (This equation is used for second-order controller to design controller bsed on zeta and wn values.)

$$
s^2 + 2\zeta\omega_n s + \omega_n^2 = 0
$$

From these we solve for controller gains (for a chosen \(k_d\)):

$$
$$
k_p = \frac{2\zeta\omega_n(\tau + K k_d) - 1}{K},\qquad
k_i = \frac{\omega_n^2(\tau + K k_d)}{K},\qquad
k_d= \frac{1 - \tau}{K}\
$$
$$

##HOW TO CHOOSE DESIGN CRITERIA
- Relation between settling time and Zeta were given as: $$
\left( T_s \approx \frac{4}{\zeta \omega_n} \right)
$$
Settling time is the time that we want our system to reach equilibrium point
We choose zeta based on our system requirements. We choose zeta as 1. Because we want our system to be critically damped system so that system do not show 
any overshoot and reach desired value within considerabely enough amoung of time.

## CAR VALUES
Car types lists in our code consists of "SEDAN","SUV","SPORT". All the informations which are mass, drag coeffienct, frontal area and rolling resistance taken from 
technical specifications of the car company.

##SYSTEM RESPONSE ANALYSIS
We will analyze our system in terms of controller parameters such as overhoot, settling time, transient time in MATLAB with the build-in function called "step"
This values taken while our choosen parameters are Sedan, No Wind, Flat conditions:

RiseTime: 4.9167 - Time takes from 10 percent to 90 percent of the steady-state value.
TransientTime: 6.1742 -Transient time is the duration of the transient response, that is, the period from the initial change until the system reaches near steady-state.
SettlingTime: 6.1736 - Settling time is the time it takes for the system response to remain within a certain percentage (commonly ±2 percent or ±5 percent) of the steady-state value.
Overshoot: 0 - Overshoot is the amount by which the system's response exceeds its final steady-state value during the transient phase

Meanwhile this one's parameters are Sedan, Stormy, ICY conditions: (with maximum acceleration as 4.5 m/s^2)
RiseTime: 4.9208
TransientTime: 6.1790
SettlingTime: 6.1784
Overshoot: 0

We can see that from numerical analysis controller do its job very well with no overshoot and settling time is approximately 6 second to reach 100 km/h
which is a realistic results of the project

##Numerical Simulation
The system is simulated in the time domain using a fixed-step Euler (ODE1) method. The state update equation is:
$$
u_{k+1} = u_k + \dot{u}_k \Delta t
$$
with a fixed step size:
$$
\Delta t = 0.01
$$

#Simulation loop logic explained briefly:
1. Compute error=targetSpeed-currentVelocity
2.Update Integral (Ki), Derivative (Kd)
3. Compute control signal=Kp*error+
5.Compute velocity change = (K*controlOutput-velocity)/tau
6. Apply limitAcceleration() so that our car does not exceed max acceleration that can be achievable by this car
7.Update velocity using Euler (ODE1) step
8. Store {time,velocity} for history

As a result of Sprint2, our car models ensures realistic vehicle acceleration behavior.

##Sprint 2 Extensions
In Sprint 2, the system is implemented using object-oriented design. Separate classes are defined for the vehicle system,
controller logic, and numerical solver. Multiple vehicle configurations, weather conditions, and road friction levels are supported. 
Furthermore, we make use of real acceleration values from technical specification and our system model is accelerated with constant acceleration based on those values.
Additionally, terminal-based visualization and CSV export are provided for result analysis in MATLAB in terms of velocity and acceleration graphs and some values.

##References:
 1. Lecture notes: "Vehicle Dynamics and Control", M. Tolga Emirler, Yildiz Technical University 
 2. A. G. Ulsoy, H. Peng, M. Cakmaci, Automotive Control Systems, Cambridge University Press.
    
