#include "rocket_simulation.h"
#include <memory>
#include <stdexcept>
#include <models/FGInertial.h>
#include <models/FGPropulsion.h>
#include <models/FGMassBalance.h>
#include <models/FGFCS.h>
#include <models/propulsion/FGTank.h>
#include <models/FGAuxiliary.h>
#include <initialization/FGInitialCondition.h>
#include <models/FGAccelerations.h>
#include "config.h"
#include "simulation_types.h"
#include <cmath>

/**
 * @brief Constructs a new Rocket Simulation:: Rocket Simulation object
 * 
 */
RocketSimulation::RocketSimulation() {
    // Create an instance of the JSBSim flight dynamics model executor
    fdmExec = std::unique_ptr<JSBSim::FGFDMExec>(new JSBSim::FGFDMExec());
    fdmExec->SetDebugLevel(0);

    // Configure simulation frequency
    fdmExec->Setdt(1.0 / SIMULATION_FREQUENCY);

    // Load the aircraft configuration file
    std::string aircraftName = ROCKET_MODEL_XML;
    if (!fdmExec->LoadModel(aircraftName)) {
        std::runtime_error("Failed to load aircraft model " + aircraftName);
    }

    // Initialize the JSBSim model
    if (!fdmExec->GetPropagate()->InitModel()) {
        std::runtime_error("Failed to initialize aircraft model");
    }

    setInitialConditions();
    setAtmosphericConditions();

    
    // Store initial position for 3D trajectory tracking
    double initial_latitude = LAUNCH_LATITUDE_DEG;
    double initial_longitude = LAUNCH_LONGITUDE_DEG;
    double initial_altitude = LAUNCH_TERRAIN_ELEVATION_FT;
    double cg_x = 0.0; 
    double mass_lmb = 50.4; // wet mass (lbs)
}

void RocketSimulation::setInitialConditions() {
    fdmExec->GetIC()->SetTerrainElevationFtIC(LAUNCH_TERRAIN_ELEVATION_FT);
    fdmExec->GetIC()->SetAltitudeASLFtIC(ROCKET_CG_HEIGHT_FT + LAUNCH_TERRAIN_ELEVATION_FT);
    fdmExec->GetIC()->SetLatitudeDegIC(LAUNCH_LATITUDE_DEG);
    fdmExec->GetIC()->SetLongitudeDegIC(LAUNCH_LONGITUDE_DEG);
    fdmExec->GetIC()->SetThetaDegIC(90.0 - RAIL_TILT_FROM_VERTICAL_DEG);
    fdmExec->GetIC()->SetPhiDegIC(0.0);        // No roll
    fdmExec->GetIC()->SetPsiDegIC(RAIL_AZIMUTH_DEG);
    fdmExec->GetIC()->SetVNorthFpsIC(0.0);
    fdmExec->GetIC()->SetVEastFpsIC(0.0);
    fdmExec->GetIC()->SetVDownFpsIC(0.0);       // No initial velocity
    fdmExec->GetIC()->SetPRadpsIC(0.0);          // No roll rate
    fdmExec->GetIC()->SetQRadpsIC(0.0);         // No pitch rate
    fdmExec->GetIC()->SetRRadpsIC(0.0);         // No yaw rate
    setFinAngle(0.0f);


    fdmExec->RunIC();
    fdmExec->SetHoldDown(true); // Holds vehicle position until motor burn

    csvInit(); 
}

void RocketSimulation::setAtmosphericConditions() {
    // Enable realistic atmospheric turbulence and wind for final testing
    fdmExec->SetPropertyValue("atmosphere/turb-rate", 0.1);      // Moderate turbulence
    fdmExec->SetPropertyValue("atmosphere/turb-gain", 1.0);      // Normal gain
    fdmExec->SetPropertyValue("atmosphere/wind-north-fps", 7.33); // 5 mph north wind (7.33 ft/s)
    fdmExec->SetPropertyValue("atmosphere/wind-east-fps", 0.0);   // No east wind
    fdmExec->SetPropertyValue("atmosphere/wind-down-fps", 0.0);   // No vertical wind
}


void RocketSimulation::setFinAngle(double angleDeg) {
    rocket_state_.fin_angle_deg = angleDeg;
    double angleRad = angleDeg * DEG_TO_RAD;
    fdmExec->SetPropertyValue("fcs/racs_fin_1_pos_rad", angleRad);
    fdmExec->SetPropertyValue("fcs/racs_fin_2_pos_rad", angleRad);
    fdmExec->SetPropertyValue("fcs/racs_fin_3_pos_rad", angleRad);
    fdmExec->SetPropertyValue("fcs/racs_fin_4_pos_rad", angleRad);
}


bool RocketSimulation::step() {
    // Run the JSBSim simulation for one time step
    fdmExec->Run();

    getState();
    motorHandler();
    deployParachute();
    printInfo();
    logCSV();

    // Check for numerical divergence and terminate gracefully
    if (rocket_state_.vel_magnitude_ft_s > 10000.0 || rocket_state_.altitude_ft > 100000.0 || std::isnan(rocket_state_.vel_magnitude_ft_s) || std::isnan(rocket_state_.altitude_ft)) {
        std::cout << "ERROR: Numerical divergence detected!" << std::endl;
        std::cout << "Rocket Velocity: " << rocket_state_.vel_magnitude_ft_s << std::endl;
        std::cout << "Rocket Altitude: " << rocket_state_.altitude_ft << std::endl;
        return true;
    }

    // Check for liftoff
    if (rocket_state_.vel_magnitude_ft_s > 0.0f && rocket_state_.altitude_agl_ft > LIFTOFF_ALT_THRESHOLD_FT && !did_liftoff) {
        std::cout << "Liftoff detected at " << rocket_state_.altitude_agl_ft << " ft" << std::endl;
        did_liftoff = true;
    }

    
    if (did_liftoff && reached_apogee && rocket_state_.altitude_agl_ft < 5.0) {  // Only terminate after apogee and very low altitude
        std::cout << "Rocket has reached the ground after flight." << std::endl;
        return true;
    }

    // Update time
    last_time = simulation_time_s;

    return false;
}


void RocketSimulation::getState() {
    // Get current state
    simulation_time_s = fdmExec->GetSimTime();
    dt = simulation_time_s - last_time;
    time_s = simulation_time_s - PRELAUNCH_DURATION_S;


    // Get altitude (ft) and pressure (kpa)
    rocket_state_.pressure_kpa = static_cast<float>(fdmExec->GetPropertyValue("atmosphere/P-psf") * 0.04788025898); // psf to kPa
    rocket_state_.altitude_ft = fdmExec->GetPropagate()->GetAltitudeASL();
    rocket_state_.altitude_agl_ft = rocket_state_.altitude_ft - LAUNCH_TERRAIN_ELEVATION_FT;
    
    // Get velocity components (ft/s)
    rocket_state_.vel_north_ft_s = fdmExec->GetPropagate()->GetVel(1); // North velocity (ft/s)
    rocket_state_.vel_east_ft_s = fdmExec->GetPropagate()->GetVel(2); // East velocity (ft/s) 
    rocket_state_.vel_down_ft_s = fdmExec->GetPropagate()->GetVel(3); // Down velocity (ft/s)
    
    // Velocity magnitude (ft/s)
    rocket_state_.vel_magnitude_ft_s = std::hypot(
        rocket_state_.vel_north_ft_s, 
        rocket_state_.vel_east_ft_s, 
        rocket_state_.vel_down_ft_s
    );

    // Vertical velocity of rocket (positive is up) (ft/s)
    rocket_state_.vel_vertical_ft_s = -rocket_state_.vel_down_ft_s;

    // Body acceleration vector (ft/s²). JSBSim body axes are X forward, Y right, Z down.
    JSBSim::FGColumnVector3 a_body_ft_s2 = fdmExec->GetAccelerations()->GetBodyAccel();
    rocket_state_.accel_x = a_body_ft_s2(1);
    rocket_state_.accel_y = a_body_ft_s2(2);
    rocket_state_.accel_z = a_body_ft_s2(3);

    // Transform from body frame to NED frame
    JSBSim::FGMatrix33 T_body_to_ned = fdmExec->GetPropagate()->GetTb2l();
    JSBSim::FGColumnVector3 accel_ned = T_body_to_ned * a_body_ft_s2;
    rocket_state_.accel_ned_n = accel_ned(1);
    rocket_state_.accel_ned_e = accel_ned(2);
    rocket_state_.accel_ned_d = accel_ned(3);

    // Attitude of rocket
    rocket_state_.roll_deg = static_cast<float>(fdmExec->GetPropagate()->GetEuler(1) * RAD_TO_DEG);
    rocket_state_.pitch_deg = static_cast<float>(fdmExec->GetPropagate()->GetEuler(2) * RAD_TO_DEG);
    rocket_state_.yaw_deg = static_cast<float>(fdmExec->GetPropagate()->GetEuler(3) * RAD_TO_DEG);

    // JSBSim body angular rates: P=roll, Q=pitch, R=yaw, in rad/s.
    rocket_state_.gyro_x = static_cast<float>(fdmExec->GetPropagate()->GetPQR(1));
    rocket_state_.gyro_y = static_cast<float>(fdmExec->GetPropagate()->GetPQR(2));
    rocket_state_.gyro_z = static_cast<float>(fdmExec->GetPropagate()->GetPQR(3));

    // find cg-x location
    rocket_state_.cg_x = fdmExec->GetMassBalance()->GetXYZcg(1);
    rocket_state_.mass_lmb = fdmExec->GetMassBalance()->GetMass() * 32.174; //mass is stored in slugs, convert to lbm
    
    // Calculate 3D position relative to launch point
    rocket_state_.current_lat = fdmExec->GetPropagate()->GetLocation().GetLatitudeDeg();
    rocket_state_.current_lon = fdmExec->GetPropagate()->GetLocation().GetLongitudeDeg();
    
    // Convert to local coordinates in feet (X=North, Y=East, Z=Up)
    rocket_state_.x_pos_ft = (rocket_state_.current_lat - LAUNCH_LATITUDE_DEG) * 364000.0;  // North-South in feet
    rocket_state_.y_pos_ft = (rocket_state_.current_lon - LAUNCH_LONGITUDE_DEG) * 364000.0 * cos(LAUNCH_LONGITUDE_DEG * 3.14159265359 / 180.0);  // East-West in feet

    if (motor_state_.motor_status == MotorStatus::Burnout && rocket_state_.vel_vertical_ft_s < 0.0) {
        reached_apogee = true;
    }
}


void RocketSimulation::deployParachute() {
    auto& status = rocket_state_.parachute_status;

    // Parachute deployment at apogee
    if (reached_apogee && status == ParachuteStatus::None) {
        if (PARACHUTE_CONFIG == ParachuteConfig::DoubleDeployment) {
            fdmExec->SetPropertyValue("external_reactions/drogue_chute/drogue_open", 1); 
            status = ParachuteStatus::DrogueDescent;

        } else if (PARACHUTE_CONFIG == ParachuteConfig::SingleDeployment){
            fdmExec->SetPropertyValue("external_reactions/main_chute/main_open", 1); 
            status = ParachuteStatus::MainDescent;
        }

        // std::cout << "Drogue chute deployed at " << altitude << " ft" << std::endl;
    }

    // If in dual deployment, deploy main chute below 550 feet (per requirements) 
    if (status == ParachuteStatus::DrogueDescent && rocket_state_.altitude_agl_ft < MAIN_PARACHUTE_DEPLOY_FT) {
        fdmExec->SetPropertyValue("external_reactions/main_chute/main_open", 1); 
        status = ParachuteStatus::MainDescent;

        // std::cout << "Main chute deployed at " << altitude << " ft" << std::endl;
    }
}


void RocketSimulation::motorIgnition() {
    fdmExec->GetFCS()->SetThrottleCmd(0, 1.0);  // Set throttle to 100% for solid rocket ignition
    fdmExec->SetHoldDown(false); // Releases vehicle from held position

    fdmExec->GetPropulsion()->GetEngine(0)->SetRunning(true);  // Also set engine to running state
    
    // Disable ground contacts during powered flight to prevent false contact detection
    // Try multiple property naming conventions
    fdmExec->SetPropertyValue("gear/unit[0]/spring-coeff-lbs_ft", 0.0);  
    fdmExec->SetPropertyValue("gear/unit[1]/spring-coeff-lbs_ft", 0.0);  
    fdmExec->SetPropertyValue("gear/unit[0]/damping-coeff-lbs_ft_sec", 0.0);  
    fdmExec->SetPropertyValue("gear/unit[1]/damping-coeff-lbs_ft_sec", 0.0);  
    
    // Also try moving contact points far away
    fdmExec->SetPropertyValue("gear/unit[0]/location-z-in", -1000.0);  // Move launch rail far down
    fdmExec->SetPropertyValue("gear/unit[1]/location-z-in", -1000.0);  // Move nose contact far down
    
    // Alternative property names
    fdmExec->SetPropertyValue("contact/unit[0]/spring-coeff-lbs_ft", 0.0);
    fdmExec->SetPropertyValue("contact/unit[1]/spring-coeff-lbs_ft", 0.0);
    
    // Log data
    std::cout << "Igniting engine at t=" << time_s << "s" << std::endl;

    // Update motor status
    motor_state_.motor_status = MotorStatus::Burning;
}


void RocketSimulation::motorShutdown() {
    // Record burn time
    motor_state_.burn_time = simulation_time_s - PRELAUNCH_DURATION_S;

    // Turn off Engine
    fdmExec->GetPropulsion()->GetEngine(0)->SetRunning(false);
    fdmExec->GetFCS()->SetThrottleCmd(0, 0.0);
    
    // Force fuel tank to zero to prevent further table lookups
    fdmExec->GetPropulsion()->GetTank(0)->SetContents(0.0);
    
    // Update motor status
    motor_state_.motor_status = MotorStatus::Burnout;

    // Print
    std::cout << "Engine shutdown at t=" << simulation_time_s << "s (fuel=" << motor_state_.propellant_remaining 
                << "lbs, burn_time=" << motor_state_.burn_time << "s)" << std::endl;
    std::cout << "TOTAL IMPULSE DELIVERED: " << motor_state_.delivered_impulse_lbf << " lbf⋅s (expected: 973.27 lbf⋅s)" << std::endl;
}

void RocketSimulation::motorHandler() {
    motor_state_.motor_thrust =
        fdmExec->GetPropulsion()->GetEngine(0)->GetThrust();

    motor_state_.propellant_remaining =
        fdmExec->GetPropulsion()->GetTank(0)->GetContents();

    switch (motor_state_.motor_status) {
        case MotorStatus::PendingIgnition:
            if (simulation_time_s >= PRELAUNCH_DURATION_S) {
                motorIgnition();
            }
            break;

        case MotorStatus::Burning:
            if (dt > 0.0) {
                motor_state_.delivered_impulse_lbf +=
                    motor_state_.motor_thrust * dt;
            }

            if (motor_state_.motor_thrust <
                MOTOR_SHUTDOWN_THRESHOLD) {
                motorShutdown();
            }
            break;

        case MotorStatus::Burnout:
            break;
    }
}


void RocketSimulation::printInfo() {
    if (last_print_s < simulation_time_s - PRINT_INTERVAL) {
        std::cout << rocket_state_.vel_magnitude_ft_s << '\n';

        // Updating last print time
        last_print_s = simulation_time_s;
    }
}


SimulationSnapshot RocketSimulation::getSnapshot() const noexcept {
    SimulationSnapshot snapshot;
    snapshot.rocket = rocket_state_;
    snapshot.motor = motor_state_;
    snapshot.simulation_time_s = simulation_time_s;
    snapshot.time_s = time_s;

    return snapshot;
}


void RocketSimulation::logCSV() {
    output_file_
        << simulation_time_s << ','
        << time_s << ','
        << rocket_state_.altitude_ft << ','
        << rocket_state_.altitude_agl_ft << ','
        << rocket_state_.pressure_kpa << ','
        << rocket_state_.vel_north_ft_s << ','
        << rocket_state_.vel_east_ft_s << ','
        << rocket_state_.vel_down_ft_s << ','
        << rocket_state_.vel_magnitude_ft_s << ','
        << rocket_state_.vel_vertical_ft_s << ','
        << rocket_state_.accel_x << ','
        << rocket_state_.accel_y << ','
        << rocket_state_.accel_z << ','
        << rocket_state_.accel_ned_n << ','
        << rocket_state_.accel_ned_e << ','
        << rocket_state_.accel_ned_d << ','
        << rocket_state_.roll_deg << ','
        << rocket_state_.pitch_deg << ','
        << rocket_state_.yaw_deg << ','
        << rocket_state_.gyro_x << ','
        << rocket_state_.gyro_y << ','
        << rocket_state_.gyro_z << ','
        << rocket_state_.cg_x << ','
        << rocket_state_.mass_lmb << ','
        << rocket_state_.current_lat << ','
        << rocket_state_.current_lon << ','
        << rocket_state_.x_pos_ft << ','
        << rocket_state_.y_pos_ft << ','
        << static_cast<int>(rocket_state_.parachute_status) << ','
        << static_cast<int>(motor_state_.motor_status) << ','
        << motor_state_.motor_thrust << ','
        << motor_state_.propellant_remaining << ','
        << motor_state_.burn_time << ','
        << motor_state_.delivered_impulse_lbf
        << '\n';
}

void RocketSimulation::csvInit() {
    const std::string output_csv = std::string("../sim_results/") + CSV_FILE_NAME;
    output_file_.open(output_csv);

    if (!output_file_.is_open()) {
        throw std::runtime_error("Failed to open output file: " + output_csv);
    }

    // Configure to 6 decimal points always;
    output_file_ << std::fixed << std::setprecision(6);
    
    // Write row names
    output_file_
        << "simulation_time_s,"
        << "time_s,"
        << "altitude_ft,"
        << "altitude_agl_ft,"
        << "pressure_kpa,"
        << "vel_north_ft_s,"
        << "vel_east_ft_s,"
        << "vel_down_ft_s,"
        << "vel_magnitude_ft_s,"
        << "vel_vertical_ft_s,"
        << "accel_x,"
        << "accel_y,"
        << "accel_z,"
        << "accel_ned_n,"
        << "accel_ned_e,"
        << "accel_ned_d,"
        << "roll_deg,"
        << "pitch_deg,"
        << "yaw_deg,"
        << "gyro_x,"
        << "gyro_y,"
        << "gyro_z,"
        << "cg_x,"
        << "mass_lmb,"
        << "current_lat,"
        << "current_lon,"
        << "x_pos_ft,"
        << "y_pos_ft,"
        << "parachute_status,"

        << "motor_status,"
        << "motor_thrust,"
        << "propellant_remaining,"
        << "burn_time,"
        << "delivered_impulse_lbf_s"
        << "\n";
}
