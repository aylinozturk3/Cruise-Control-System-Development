#include "controller.h" // WICHTIG: Hier binden wir den Header ein
#include <cmath>

// --- Klassen ---
class VehicleModel {
    double m, u0, A;
    const double rho = 1.202, Cd = 0.4, uw = 2.0;
public:
    VehicleModel(double mass, double velocity_op) { updateState(mass, velocity_op); }
    void updateState(double mass, double velocity_op) {
        m = mass; u0 = velocity_op;
        A = 1.6 + 0.00056 * (m - 765.0);
    }
    double getK_plant() const { 
        double denominator = rho * A * Cd * (u0 + uw);
            if (std::abs(denominator) < 1e-10) throw std::runtime_error("Division by zero");
            return 1.0 / denominator;
    }
    double getTau() const { return m / (rho * A * Cd * (u0 + uw)); }
};

class PIDGainCalculator {
    double Ts, zeta;
public:
    PIDGainCalculator(double sample_time, double damping) : Ts(sample_time), zeta(damping) {}
    void calculate(const VehicleModel& v, double* Kp, double* Ki, double* Kd) {
        double Tau = v.getTau(), K = v.getK_plant();
        double omega_n = 4.0 / (Ts * zeta);
        *Kp = ((2.0*zeta*omega_n*Tau) - 1.0)/K;
        *Ki = (Tau*omega_n*omega_n)/K;
        *Kd = (1.0 - K)/Tau;
    }
};

// --- Wrapper Implementierung ---
static std::unique_ptr<VehicleModel> myCar;  // Use smart pointers for automatic cleanup
static std::unique_ptr<PIDGainCalculator> myTuner;

// Hier muss KEIN extern "C" mehr stehen, da es im Header definiert ist,
// aber zur Sicherheit lassen wir es, damit der Linker es findet.
extern "C" void compute_pid_gains(double m, double u0, double* Kp, double* Ki, double* Kd) {
    if (!myCar) { myCar = new VehicleModel(m, u0); myTuner = new PIDGainCalculator(sample_time, damping); }
    myCar->updateState(m, u0);
    myTuner->calculate(*myCar, Kp, Ki, Kd);
}
