#pragma once

enum class MotorStatus {
    PendingIgnition,
    Burning,
    Burnout
};

enum class ParachuteStatus {
    None,
    DrogueDescent,
    MainDescent
};


struct RocketState {
    // Altitude / Pressure
    double altitude_ft = 0.0;
    double altitude_agl_ft = 0.0;
    double pressure_kpa = 0.0;

    // Velocity
    double vel_north_ft_s = 0.0;
    double vel_east_ft_s = 0.0;
    double vel_down_ft_s = 0.0;
    double vel_magnitude_ft_s = 0.0;
    double vel_vertical_ft_s = 0.0; // = -vel_down_ft_s for positive vertical vel

    // Acceleration
    double accel_x = 0.0;
    double accel_y = 0.0;
    double accel_z = 0.0;

    double accel_ned_n = 0.0;
    double accel_ned_e = 0.0;
    double accel_ned_d = 0.0;

    // Attitude
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;

    // Angular Acceleration
    double gyro_x = 0.0;
    double gyro_y = 0.0;
    double gyro_z = 0.0;

    // CG-x and Mass
    double cg_x = 0.0;
    double mass = 0.0;
    
    // 3D Position
    double current_lat = 0.0;
    double current_lon = 0.0;
    double x_pos_ft = 0.0;
    double y_pos_ft = 0.0;

    // Parachutes
    ParachuteStatus parachute_status = ParachuteStatus::None;

    // Fin angle
    double fin_angle_deg = 0.0;

};  

struct MotorState {
    // Motor Status
    MotorStatus motor_status = MotorStatus::PendingIgnition;

    // Motor Info
    double motor_thrust = 0.0;
    double propellant_remaining = 0.0;
    double burn_time = 0.0;
    double delivered_impulse_lbf = 0.0;
};

struct SimulationSnapshot {
    RocketState rocket;
    MotorState motor;

    double simulation_time_s = 0.0;
    double time_s = 0.0;
};