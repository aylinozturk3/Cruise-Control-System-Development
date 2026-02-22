Cruise Control System
##Overview

This project implements a Cruise Control System for vehicles, designed and developed in three stages to demonstrate modeling, control, and software design techniques. 
The system regulates vehicle speed under varying road and environmental conditions.

Development Stages
1️⃣ System Modeling & Controller Design:

Derived system dynamics and equations of motion

Linearized the vehicle model

Designed a classical feedback controller to regulate speed


2️⃣ Object-Oriented Programming Implementation:

Refactored the system into OOP classes

Enabled user selection of:

Vehicle type

Road profile

Air resistance conditions

Simulated vehicle response in a modular and extendable framework


3️⃣ Code Optimization:

Improved computational efficiency and readability

Reduced simulation time

Structured code for maintainability and scalability

Added CMake build files for easy compilation

Features
-Vehicle speed regulation under different conditions
-Modular, object-oriented design for easy extension
-Simulation of realistic driving scenarios
-Adjustable parameters for vehicle, road, and environment

Workflow:

C++ code simulates vehicle dynamics and controller response

Simulation results exported to CSV files

MATLAB used to load CSV files and generate graphs and performance plots
