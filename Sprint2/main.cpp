#include <iostream>
#include <vector> 
#include <iomanip>
#include "CruiseControl.h"
#include "EulerSolver.h"
#include <fstream>
#include <iostream>

// --- PRINTING VECTOR DATA ---
void printVectorData(const std::vector<SimData>& data) {
    std::cout << "\n--- Raw Simulation Data (km/h) ---\n";
    std::cout << std::setw(10) << "Time(s)" << " | " << std::setw(15) << "Velocity(km/h)" << "\n";
    std::cout << "-------------------------------\n";
    
    // Printing every 10th result to keep terminal clean
    for (size_t i = 0; i < data.size(); i += 10) {
        std::cout << std::fixed << std::setprecision(2) << std::setw(10) << data[i].time 
                  << " | " << std::setw(15) << data[i].velocity * 3.6 << "\n";
    }
}

// --- PLOTTING DATA ---
void plotGraph(const std::vector<SimData>& data, double targetMs, double targetKmh) {
    const int width = 60;
    const int height = 15;
    double maxVKmh = targetKmh * 1.2;

    std::cout << "\n--- Velocity (km/h) vs. Time ---\n";
    for (int i = height; i >= 0; --i) {
        double currentKmh = (maxVKmh / height) * i;
        std::cout << std::fixed << std::setprecision(0) << std::setw(5) << currentKmh << " | ";
        
        for (int j = 0; j < width; ++j) {
            int idx = (data.size() / width) * j;
            if (idx >= data.size()) idx = data.size() - 1;
            
            // Convert current velocity data point to km/h for plotting
            double valKmh = data[idx].velocity * 3.6;
            
            int pointY = (valKmh / maxVKmh) * height;
            int targetY = (targetKmh / maxVKmh) * height;

            if (pointY == i) std::cout << "*";      
            else if (targetY == i) std::cout << "-"; 
            else std::cout << " ";
        }
        std::cout << "\n";
    }
    std::cout << "        " << std::string(width, '=') << "\n        Time (s) ->\n";
}

void exportCSV(const std::vector<SimData>& data) {
    std::ofstream file("sim_output.csv");
    file << "time,velocity\n";
    for (auto& d : data)
        file << d.time << "," << d.velocity << "\n";
    std::cout << "\nCSV exported as sim_output.csv\n";
}


int main() {
    CruiseControlSystem cc;
    int carChoice, weatherChoice, roadChoice;
    double speedKmh, simDuration;

    std::cout << "1. Vehicle Type (1:Sedan, 2:SUV, 3:Sport): ";
    std::cin >> carChoice;
    std::cout << "2. Weather (1:No Wind, 2:Windy, 3:Stormy): ";
    std::cin >> weatherChoice;
    std::cout << "3. Road (1:Flat, 2:Icy): ";
    std::cin >> roadChoice;
    std::cout << "4. Target Speed (km/h): ";
    std::cin >> speedKmh;
    std::cout << "5. Simulation Duration (seconds): ";
    std::cin >> simDuration;

    
    double targetMs = speedKmh / 3.6;
    cc.setModel(static_cast<CarType>(carChoice-1), static_cast<WeatherCondition>(weatherChoice-1),static_cast<RoadType>(roadChoice-1));
    cc.calculateGains(targetMs, 3.0, 1);
    double maxSpeedKmh = cc.getMaxSpeed() * 3.6;
    if (speedKmh > maxSpeedKmh) {
        std::cout << "\n⚠️  Warning: Target speed " << speedKmh 
                  << " km/h exceeds vehicle max of " << maxSpeedKmh 
                  << " km/h. Limiting to " << maxSpeedKmh << " km/h.\n";
        targetMs = cc.getMaxSpeed();
    }

    
std::cout << "\n--- Vehicle Specifications ---\n";
    std::cout << "Max Acceleration: " << cc.getMaxAccel() << " m/s² (" 
              << cc.getMaxAccel() * 3.6 << " km/h/s)\n";
    std::cout << "Max Deceleration: " << cc.getMaxDecel() << " m/s²\n";
    std::cout << "Top Speed: " << cc.getMaxSpeed() * 3.6 << " km/h\n";
    std::cout << "------------------------------\n";
    auto results = cc.runSimulation(targetMs, simDuration);

	// Visual ASCII Graph
	plotGraph(results, targetMs, speedKmh);
	
	// Print values in terminal
	printVectorData(results);
	
	// Save as CSV for MATLAB
	exportCSV(results);
	
	return 0;
}


