#ifndef EULERSOLVER_H
#define EULERSOLVER_H
//EULER ODE1 method is used, because our system is first order and linear
class EulerSolver{
	public:
		double step(double& state,double& derivative, double& dt) const {
			return state+derivative*dt;
		}
		
};
#endif
