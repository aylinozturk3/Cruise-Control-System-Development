#ifndef CONTROLLER_H
#define CONTROLLER_H

extern "C" void compute_pid_gains(double m, double u0, double* Kp, double* Ki, double* Kd);

#endif
