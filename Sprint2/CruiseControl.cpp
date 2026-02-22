#include "CruiseControl.h"
#include "EulerSolver.h"
#include <cmath>

void CruiseControlSystem::setModel(CarType car, WeatherCondition weatherInput, RoadType roadtype) {
    // Presets from source logic [cite: 1, 37]
    switch(car) {
        case CarType::SEDAN: vehicle = {1705.0, 0.27 , 2.2 ,0.014, 4.5, -9.0, 72.2};break; // 2020 BMW 320d- getting values from technical specifications
        case CarType::SUV:   vehicle = {1880.0, 0.34, 2.5, 0.018, 4.8, -8.5, 66.7}; break;// 2020 BMWX3 xDrive30d
        case CarType::SPORT: vehicle = {1760.0, 0.33, 2.07, 0.012, 7.0, -10.0, 83.3};break; // 2020 BMW Z4 break;
    }
    switch(weatherInput) {
        case WeatherCondition::NO_WIND: environment = {1.202, 0.0}; break;
        case WeatherCondition::WINDY:   environment = {1.202, 5.0}; break;
        case WeatherCondition::STORMY:  environment = {1.15, 15.0}; break; 
    }
    switch(roadtype) {
        case RoadType::FLAT: road.friction= static_cast<double>(1.00); break;
        case RoadType::ICY:  road.friction= static_cast<double>(0.55); break;
    }
}

double CruiseControlSystem::getTau(double targetMs) const {
    return vehicle.mass / (environment.airDensity * vehicle.dragCoeff * vehicle.frontalArea * (targetMs + environment.windSpeed)* road.friction);
}

double CruiseControlSystem::getK(double targetMs) const {
    double effectiveWindFactor = 1.0 + (environment.windSpeed / 20.0); 
    return 1.0 / (environment.airDensity * vehicle.dragCoeff * vehicle.frontalArea * targetMs * effectiveWindFactor * road.friction);
}

void CruiseControlSystem::calculateGains(double targetSpeedMs, double Ts, double zeta) {
    double Tau = getTau(targetSpeedMs);
    double K = getK(targetSpeedMs);
    double omega_n = 4.0 / (Ts * zeta);
    double conditionFactor = 1.0;
    if (road.friction < 0.7) conditionFactor = 1.5; // Icy roads
    if (environment.windSpeed > 10.0) conditionFactor *= 1.3; // Stormy 
    Kp = ((2.0 * zeta * omega_n * Tau) - 1.0) / K;
    Ki = (Tau * omega_n * omega_n) / K;
    Kd = (1.0 - K) / Tau;
}
double CruiseControlSystem::limitAcceleration(double requestedAccel, double currentVel) const {
    double limitedAccel = requestedAccel;
    
    // 1. Check max acceleration
    if (limitedAccel > vehicle.maxAcceleration) {
        limitedAccel = vehicle.maxAcceleration;
    }
    
    // 2. Check max braking (negative acceleration)
    if (limitedAccel < vehicle.maxDeceleration) {
        limitedAccel = vehicle.maxDeceleration;
    }
    
    // 3. Can't accelerate if already at max speed
    if (currentVel >= vehicle.maxVelocity && limitedAccel > 0) {
        limitedAccel = 0;
    }
    
    return limitedAccel;
}
std::vector<SimData> CruiseControlSystem::runSimulation(double targetSpeedMs, double simulationTime) {
    std::vector<SimData> history;
    EulerSolver solver;      // <-- kesin tanımla
    double velocity = 0.0, integral = 0.0, prevError = 0.0;
    double Tau = getTau(targetSpeedMs), K = getK(targetSpeedMs);
    
     if (targetSpeedMs > vehicle.maxVelocity) {
        targetSpeedMs = vehicle.maxVelocity;  // Just limit it
    }
    for (double t = 0; t <= simulationTime; t += dt) {
        double error = targetSpeedMs - velocity;
       
        double derivative = (error - prevError) / dt;
        double controlOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);

        double dv = (K * controlOutput - velocity) / Tau;
         dv = limitAcceleration(dv, velocity);

        // Euler step
        velocity = solver.step(velocity, dv, dt);  
        if (velocity > vehicle.maxVelocity) {
            velocity = vehicle.maxVelocity;
        }
        if (velocity < 0) {
            velocity = 0;
        }
        
        prevError = error;
        history.push_back({t, velocity});
    }
    return history;
}

