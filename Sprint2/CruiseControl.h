#ifndef CRUISE_CONTROL_H
#define CRUISE_CONTROL_H

#include <vector>

struct VehicleParams {
    double mass;
    double dragCoeff;
    double frontalArea;
    double rollingResistance;
    double maxAcceleration;      // Maximum acceleration (m/s²) from specs
    double maxDeceleration;      // Maximum braking/deceleration (m/s²)
    double maxVelocity;          // Top speed (m/s)
};

struct Environment {
    double airDensity;
    double windSpeed;
};

struct Road{
    double friction;
};

struct SimData {
    double time;
    double velocity;
};

enum class CarType { SEDAN, SUV, SPORT };
enum class WeatherCondition { NO_WIND, WINDY, STORMY };
enum class RoadType{FLAT, ICY};

class CruiseControlSystem {
private:
    VehicleParams vehicle;
    Environment environment; // Renamed from 'weather' to avoid conflict 
    Road road;
    double Kp, Ki, Kd; 
    double dt = 0.01; // Matches Simulink solver [cite: 100]
    double limitAcceleration(double requestedAccel, double currentVel) const;
public:
    void setModel(CarType car, WeatherCondition weatherInput, RoadType roadtype); 
    void calculateGains(double targetSpeed, double Ts, double zeta);
    std::vector<SimData> runSimulation(double targetSpeed, double simulationTime);
    double getActualAcceleration(double currentVel) const;

    // Physics explicitly uses targetSpeed as the linearization point
    double getTau(double targetMs) const;
    double getK(double targetMs) const;
    double getMaxAccel() const { return vehicle.maxAcceleration * road.friction; }
    double getMaxDecel() const { return vehicle.maxDeceleration * road.friction; }
    double getMaxSpeed() const { return vehicle.maxVelocity; }
};

#endif