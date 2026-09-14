#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include "uav_system.hpp"

using namespace mrs_multirotor_simulator; 

struct FlightEvent {
    double time; 
    double pitch, roll, speed;
};

int main() {
    UavSystem uav;
    uav.setStatePos(Eigen::Vector3d(0, 0, 100), Eigen::Vector3d(0, 0, 0));

    std::vector<FlightEvent> plan = {
        {0.0,   0.00,  0.00, 20.0},
        {5.0,   0.10,  0.00, 25.0},
        {15.0,  0.00,  0.00, 20.0},
        {20.0,  0.05,  0.30, 22.0},
        {30.0,  0.00,  0.00, 20.0},
        {35.0, -0.08,  0.00, 15.0},
        {45.0,  0.00,  0.00, 20.0}
    };
    
    int next_event = 0;
    double dt = 0.01; 
    int total_steps = 50.0 / dt; // Simulace na 50 vteřin

    std::cout << "========================================================================\n";
    std::cout << std::setw(8)  << "Cas [s]"
              << std::setw(12) << "X [m]"
              << std::setw(12) << "Y [m]"
              << std::setw(12) << "Z [m]"
              << std::setw(15) << "Rychlost [m/s]"
              << std::setw(13) << "Pitch [deg]\n";
    std::cout << "========================================================================\n";
    std::cout << std::fixed << std::setprecision(2);

    for (int step = 0; step <= total_steps; step++) {
        double t = step * dt;

        // 1. Změna povelu, pokud nastal správný čas
        if (next_event < plan.size() && t >= plan[next_event].time) {
            uav.setInput(plan[next_event].pitch, plan[next_event].roll, plan[next_event].speed);
            
            std::cout << "------- Novy povel: Pitch " << plan[next_event].pitch 
                      << ", Roll " << plan[next_event].roll 
                      << ", Speed " << plan[next_event].speed << " -------\n";
            next_event++;
        }

        // 2. Výpočet fyziky
        uav.makeStep(dt);

        // 3. Výpis dat každou 1 vteřinu (100 kroků)
        if (step % 100 == 0) {
            auto state = uav.getState();
            
            double speed = state.v.norm();
            double pitch_deg = std::asin(-state.R(2, 0)) * (180.0 / M_PI);

            std::cout << std::setw(8)  << t
                      << std::setw(12) << state.x(0)
                      << std::setw(12) << state.x(1)
                      << std::setw(12) << state.x(2)
                      << std::setw(15) << speed
                      << std::setw(13) << pitch_deg << "\n";
        }
    }

    return 0;
}