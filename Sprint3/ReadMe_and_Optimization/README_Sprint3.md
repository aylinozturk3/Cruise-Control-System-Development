##Cruise Control System using PID Controller
#Sprint 3 – Optimization of the Slowest Code Sections

#MOTIVATION
This sprint extends the Cruise Control System developed in Sprint 2 by optimizing the slowest code sections
This final sprint focuses on the Performance Optimization of the Cruise Control System developed in previous phases. The goal was to transform a purely functional simulation into a high-performance, hardware-aware system. By integrating real-world acceleration data from automotive manufacturers and optimizing the computational core,
we achieved a simulation that is both realistic and extremely efficient.

##Definition of done:
-Analyze how much time each section of code takes
-Reduce runtime of slowest code-sections


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

## How do we get calculation of each function?
This report documents the systematic optimization of the Cruise Control Simulation system. By utilizing GPROF for profiling and applying Hardware-Aware Programming techniques, the execution efficiency was improved by several orders of magnitude.
GPROF is a profiling tool that measures how much time the CPU spends in each function. 

In the original, unoptimized version of the code, the GPROF flat profile identified three primary bottlenecks which are:
1-CruiseControlSystem::runSimulation -40.00 percentage: High overhead due to inefficient loop logic.
2-std::vector::emplace_back / push_back -20.00 percentage:   
  Without memory reservation, the vector was constantly reallocating memory. 
  Every time the capacity was reached, the CPU had to find a new memory block and copy all existing data.
3-EulerSolver::step - 20.00 percentage: Computational overhead from repeated divisions and memory references.


##  Implemented Optimization Strategies
   1- LOOP UNROLLING:
   It is used inside the main simulation loop within "CruiseControlSystem::runSimulation"
   Implementation: The standard for loop was "unrolled" by a factor of 4. Instead of incrementing the counter by 1 (i++), the loop now increments by 4 (i += 4), processing four simulation steps in a single iteration. A "Tail Loop" was added at the end to handle cases 
   where the total number of steps is not a multiple of 4. By unrolling the loop, we reduced these control instructions by 75 percentage. This allows the CPU's Instruction Pipeline to stay filled with actual physics calculations rather than being interrupted by loop management logic.
   It was very useful to reduce the memory interaction in every increment in the loop.
   We also add tail Loop which handle remaining steps when (expected_size mod 4 != 0).


   2- REPLACING DIVISON BY MULTIPLY:
   Instead of calculating error / dt inside the simulation loop which also executes millions of times, pre-calculation is made outside the loop as "const double inv_dt = 1.0 / dt;". Inside the loop, all divisions were replaced with a multiplication operation by "inv_dt"
   In modern CPU architectures, a division instruction is high-latency which is taking approximatelly 20–40 clock cycles whereas a multiplication instruction is low-latenc
  
   3- PREALLOCATION OF VECTOR AND DIRECT INDEXING:
   It is used in the management of the "std::vector<SimData> history" container. 
   At the start of the simulation where the total number of steps was calculated, and the vector was pre-sized using "history.resize(expected_size)". Data was then stored using Direct Indexing such as (history[i] = ...).
   We use that method because reallocating memory is a slow operation that involves finding a new memory block and copying all data into new memory. By using resize and direct indexing, we bypassed the "capacity check" logic hidden inside push_back and wrote data directly to sequential memory addresses.

   4- INLINE FUNCTION AND REGISTER OPTIMIZATION:
   It is used in the "EulerSolver::step" and "limitAcceleration" functions.
   Function Inlining is one of the most powerful optimizations for small, frequently used functions. Additionally, parameters were switched from "pass-by-reference" to "pass-by-value.
   By passing double values directly rather than their memory addresses (references), we enable the compiler to perform Register Allocation. This keeps the data within the CPU's internal registers which is way faster than pass by reference. It uses so called local registers.
   The compiler can easily merge this math into the main loop, allowing it to use CPU Registers for the variables instead of fetching them from RAM, which is the fastest way to process data.

   5- USAGE OF "UNLIKELY" IN FUNCTIONS - Branch Prediction Optimization:
    It keeps the most common instructions flowing smoothly through the Instruction Pipeline without interruptions.
    We used in this method in "CruiseControlSystem::runSimulation" as in for loop. It keeps the most common instructions flowing smoothly through the Instruction Pipeline 
    without interruptions. This keeps the CPU's branch predictor from loading unnecessary instructions into the cache.

   6- USE OF "std::clamp" in "CruiseControlSystem::limitAcceleration":
   In the original version, we used multiple if-else blocks to manually handle acceleration saturation. In the optimized version, we replaced this logic with std::clamp from the <algorithm> library.
   This ensures the requested PID output is mathematically bounded within the real-world performance specs of the selected car type (e.g., SUV vs. SPORT), preventing unrealistic simulation behavior.

## Result of Sprint 3
The primary goal of Sprint 3 was to eliminate the bottlenecks identified in the initial profiling phase. By shifting from a purely functional implementation to a hardware-aware architecture, we achieved significant performance gains.
Initially, the system was heavily Compute-Bound, with CruiseControlSystem::runSimulation consuming 40 percentage of the total execution time. This was primarily due to high instruction overhead and inefficient arithmetic.
The primary objective of Sprint 3 was to eliminate the performance bottlenecks identified during the initial profiling phase. By transitioning from a standard functional implementation to a Hardware-Aware Architecture and methods that we learned from the lecture notes, we achieved significant efficiency gains while maintaining physical accuracy.
The final GPROF report confirms that the Cruise Control System is no longer limited by software inefficiencies. The remaining execution time is spent on unavoidable hardware tasks such as printing data and memory bus writes. 


## References
 1. Lecture notes: "Vehicle Dynamics and Control", M. Tolga Emirler, Yildiz Technical University 
 2. A. G. Ulsoy, H. Peng, M. Cakmaci, Automotive Control Systems, Cambridge University Press.
    
