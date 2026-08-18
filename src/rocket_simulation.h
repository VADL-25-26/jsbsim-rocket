#pragma once
#include <algorithm>
#include <fstream>
#include <memory>
#include <FGFDMExec.h>
#include "config.h"
#include "simulation_types.h"


class RocketSimulation {
public:
    RocketSimulation();
    ~RocketSimulation() = default;

    bool step();
    [[nodiscard]] SimulationSnapshot getSnapshot() const noexcept;
    void setFinAngle(double angleDeg);

private:
    /*
     * RocketSimulation Constructor helper functions
    */
    void setInitialConditions();
    void setAtmosphericConditions();
    void csvInit();

    /*
     * RocketSimulation step() helper functions
    */
    void getState();
    void deployParachute();
    void motorIgnition();
    void motorShutdown();
    void motorHandler();
    void printInfo();
    void logCSV();

    // Member variables
    RocketState rocket_state_;
    MotorState motor_state_;
    std::ofstream output_file_;

    bool reached_apogee = false;

    double motor_burn_time_s = 2.3;

    float liftoff_threshold_agl;
    bool did_liftoff = false;

    // TIME DATA
    double last_time = 0.0;
    double dt = 0.0;
    double simulation_time_s = 0.0; // Everything relies on this time 
    double time_s = simulation_time_s - PRELAUNCH_DURATION_S; // Only used for normalizing time in graphs
    double last_print_s = 0.0;

    std::unique_ptr<JSBSim::FGFDMExec> fdmExec;
};