#ifndef EULERSOLVER_H
#define EULERSOLVER_H
//EULER ODE1 method is used, because our system is first order and linear

// ADDED FOR OPTIMIZATION-inline- reduces the function overcall expense
class EulerSolver{
	public:
		inline double step(double state,double derivative, double dt) const {
			return state+derivative*dt;
		}
		
};
#endif
