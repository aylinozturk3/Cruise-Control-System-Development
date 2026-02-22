#include "CruiseControl.h"
#include "EulerSolver.h"
#include <cmath>
#include <algorithm>

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
    double denominator = (environment.airDensity * vehicle.dragCoeff * vehicle.frontalArea * (targetMs + environment.windSpeed)* road.friction);
    return vehicle.mass / std::max(denominator, 1e-9);
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
    //ADDED FOR OPTIMIZATON- USE OF STL LIBRARY  std::clamp(value, min limit, max limit)
    double limitedAccel = std::clamp(requestedAccel, vehicle.maxDeceleration, vehicle.maxAcceleration);
    
    if (currentVel > vehicle.maxVelocity && limitedAccel > 0) {
        return 0.0;
    }
    return limitedAccel;

    
}

std::vector<SimData> CruiseControlSystem::runSimulation(double targetSpeedMs, double simulationTime) {
    const double inv_dt = 1.0 / dt; // ADDED FOR OPT.- to reduce run time computation- division is more expensive than multiplying
    double t = 0.0;

    std::vector<SimData> history;
    EulerSolver solver;     
    double velocity = 0.0, integral = 0.0, prevError = 0.0;
    double Tau = getTau(targetSpeedMs), K = getK(targetSpeedMs);
    
    //ADDED FOR OPT. - WE RESERVE PLACES FOR VECTOR- because copying values everytime from vector is too expensive
    size_t expected_size = static_cast<size_t>(simulationTime *inv_dt) + 1;
    history.resize(expected_size);
    size_t i=0;
    
    if (targetSpeedMs > vehicle.maxVelocity)[[unlikely]] {
        targetSpeedMs = vehicle.maxVelocity;  // Just limit it
    }
 
    // ADDED - LOOP UNROLLING FOR OPTIMIZATION 
    // Aim to reduce the loop expense by 25% 
    for (; i + 3 < expected_size; i += 4) {
        
        // STEP 1
        double error = targetSpeedMs - velocity;
        double derivative = (error - prevError) * inv_dt;
        integral += error * dt;
        integral = std::clamp(integral, -0.17, 0.17); // to prevent integral wind up
        double dv = limitAcceleration((K * ((Kp * error) + (Ki * integral) + (Kd * derivative)) - velocity) / Tau, velocity);
        velocity = solver.step(velocity, dv, dt);
        if (velocity > vehicle.maxVelocity) [[unlikely]] velocity = vehicle.maxVelocity;
        history[i] = {t, velocity};
        t += dt; prevError = error;

        // STEP 2
        error = targetSpeedMs - velocity;
        derivative = (error - prevError) * inv_dt;
        integral += error * dt;
        integral = std::clamp(integral, -0.17, 0.17); //
        dv = limitAcceleration((K * ((Kp * error) + (Ki * integral) + (Kd * derivative)) - velocity) / Tau, velocity);
        velocity = solver.step(velocity, dv, dt);
        if (velocity > vehicle.maxVelocity) [[unlikely]] velocity = vehicle.maxVelocity;
        history[i+1] = {t, velocity};
        t += dt; prevError = error;

        // STEP 3
        error = targetSpeedMs - velocity;
        derivative = (error - prevError) * inv_dt;
        integral += error * dt;
        integral = std::clamp(integral, -0.17, 0.17); //
        dv = limitAcceleration((K * ((Kp * error) + (Ki * integral) + (Kd * derivative)) - velocity) / Tau, velocity);
        velocity = solver.step(velocity, dv, dt);
        if (velocity > vehicle.maxVelocity) [[unlikely]] velocity = vehicle.maxVelocity;
        history[i+2] = {t, velocity};
        t += dt; prevError = error;

        // STEP 4
        error = targetSpeedMs - velocity;
        derivative = (error - prevError) * inv_dt;
        integral += error * dt;
        integral = std::clamp(integral, -0.17, 0.17); //
        dv = limitAcceleration((K * ((Kp * error) + (Ki * integral) + (Kd * derivative)) - velocity) / Tau, velocity);
        velocity = solver.step(velocity, dv, dt);
        if (velocity > vehicle.maxVelocity) [[unlikely]] velocity = vehicle.maxVelocity;
        history[i+3] = {t, velocity};
        t += dt; prevError = error;
    }

    // 3. TAIL LOOP - for the loops which are left at the end
    for (; i < expected_size; ++i) {
        double error = targetSpeedMs - velocity;
        double derivative = (error - prevError) * inv_dt;
        integral += error * dt;
        integral = std::clamp(integral, -0.17, 0.17); //
        double dv = limitAcceleration((K * ((Kp * error) + (Ki * integral) + (Kd * derivative)) - velocity) / Tau, velocity);
        velocity = solver.step(velocity, dv, dt);
        history[i]={t, velocity};
        t += dt; prevError = error;
    }
    
    return history;
}

