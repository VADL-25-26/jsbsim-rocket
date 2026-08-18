#pragma once

enum class ParachuteConfig {
    SingleDeployment,
    DoubleDeployment
};


// Simulation Configuration
inline constexpr double SIMULATION_FREQUENCY = 200.0;
inline constexpr char ROCKET_MODEL_XML[] = "summer_subscale_2026";
inline constexpr double PRELAUNCH_DURATION_S = 1.0;
inline constexpr double LIFTOFF_ALT_THRESHOLD_FT = 10.0;
inline constexpr double MOTOR_SHUTDOWN_THRESHOLD = 0.01;

// Gallatin, Tennessee launch conditions.
inline constexpr double LAUNCH_TERRAIN_ELEVATION_FT = 545.0;
inline constexpr double ROCKET_CG_HEIGHT_FT = 3.28084;
inline constexpr double LAUNCH_LATITUDE_DEG = 36.388303;
inline constexpr double LAUNCH_LONGITUDE_DEG = -86.44759;
inline constexpr double RAIL_TILT_FROM_VERTICAL_DEG = 5.25;
inline constexpr double RAIL_AZIMUTH_DEG = 270.0;

// Logging
inline constexpr char CSV_FILE_NAME[] = "sim3.csv";
inline constexpr double PRINT_INTERVAL = 0.1f;


// Rocket Configuration
inline constexpr ParachuteConfig PARACHUTE_CONFIG = ParachuteConfig::SingleDeployment;
inline constexpr float MAIN_PARACHUTE_DEPLOY_FT = 550.0f; // Only applies for dual deployment


// Math
inline constexpr float FT_TO_M = 0.3048f;
inline constexpr double RAD_TO_DEG = 180.0 / M_PI;
inline constexpr double DEG_TO_RAD = M_PI / 180.0;
