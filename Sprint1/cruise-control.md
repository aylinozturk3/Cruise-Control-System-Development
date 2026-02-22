# Project: Simple Cruise Control Algorithm

Idea contributed by Yannis Horstmann (yannis.horstmann@tum.de) and Tobias Wittmann (ge84dik@mytum.de).

## Motivation:

We want to implement a [PID-Controller](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller) based cruise control algorithm for a simplified car model. Driving resistances such as rolling resistance and aerodynamic drag will be implemented in a simplified model and simulated. A PID-controller will be used to manipulate system behavior and control the car’s velocity to a user-defined value.

For that we will use [the linearized model described here](https://ctms.engin.umich.edu/CTMS/index.php?example=CruiseControl&section=SimulinkModeling) which assumes that rolling resistance and aerodynamic drag depend linearly on the velocity. This resource also includes material about the PID controller and examples of open-loop and closed-loop behaviour.

To implement this project, basic knowledge about control systems and PID control will be helpful.

## Sprint 1 (basics)

In this sprint, the system equations and the controller equations are implemented into code. Additionally, to simulate the system behaviour, a simple solver using [Euler Method](https://en.wikipedia.org/wiki/Euler_method) is implemented in this spint. [This resource](https://www.codesansar.com/numerical-methods/eulers-method-using-cpp-output.htm) could help with the implementation of the solver. The user should be able to input initial conditions and the desired velocity. A time series vector of velocity of the car should be returned for possible visualization with external tools.

### Definition of done:

- Read Me file and basic documentation
- Implementation of system equations and PID-controller
- Simple solver working
- Output of a timeseries vector of the car's velocity for external visualization (e.g. Matlab)

## Sprint 2 (OOP)

In Sprint 2, the projects components shall be implemented in an object-oriented way. This means implementing classes for the "system" (the car), the "controller" and the "solver". In addition to that, we want to create a built-in visualization method to make the system behavior observable to the user.

### Defintion of done:

- Object-oriented implementation of system, controller, and solver  
- Visualization of velocity of car for depending on initial conditions and control parameters

## Sprint 3 (performance and/or STL)

The last sprint is about the structure of our code and finding ways to improve it. We and identify the sections of the code that take disproportionately long. Using the methods that we will learn from the lecture, we will try to reduce the runtime as much as possible.

### Definition of done:

- Analyze how much time each section of code takes
- Reduce runtime of slowest code-sections