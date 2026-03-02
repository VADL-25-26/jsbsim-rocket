#include <iostream>
#include <fstream>
#include <iomanip>
#include <memory>
#include <cmath>
#define _USE_MATH_DEFINES
#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>
#include <models/FGInertial.h>
#include <models/FGPropulsion.h>
#include <models/FGMassBalance.h>
#include <models/FGFCS.h>
#include <models/propulsion/FGTank.h>
#include <models/FGAuxiliary.h>
#include "models/FGAccelerations.h"
#include "RK4.h"
#include <iostream>

int main(int argc, char* argv[]) {    
    // Create an instance of the JSBSim flight dynamics model executor
    std::unique_ptr<JSBSim::FGFDMExec> fdmExec(new JSBSim::FGFDMExec());

    // Set the simulation to run at 200 Hz (matching VN100 speed)
    fdmExec->Setdt(1.0 / 200.0);

    // select which rocket to simulate
    std::string aircraftName = "midscale";

    if (argc == 2) {
        aircraftName = argv[1];
    } else if (argc > 2) {
        std::cerr << "Usage: " << argv[0] << " [aircraft_name]" << std::endl;
        return 1;
    }

    // Load the aircraft configuration file
    if (!fdmExec->LoadModel(aircraftName)) {
        std::cerr << "Failed to load the aircraft model" << std::endl;
        return 1;
    }

    // Initialize the JSBSim model
    if (!fdmExec->GetPropagate()->InitModel()) {
        std::cerr << "Failed to initialize the aircraft model" << std::endl;
        return 1;
    }

    // Set initial conditions for launch into wind - REALISTIC LAUNCH TECHNIQUE (CORRECTED)
    fdmExec->GetIC()->SetAltitudeASLFtIC(5);    // Start slightly higher to avoid ground contact
    fdmExec->GetIC()->SetLatitudeDegIC(34.90115786777616);
    fdmExec->GetIC()->SetLongitudeDegIC(-86.61568310338117);

    double rail_tilt_from_vertical_deg = 0;  // 0 = straight up
    double rail_azimuth_deg = 180.0;           // direction you want it to lean toward ( 180 is Point south (into north wind))

    fdmExec->GetIC()->SetThetaDegIC(90.0 - rail_tilt_from_vertical_deg);
    fdmExec->GetIC()->SetPsiDegIC(rail_azimuth_deg);
    fdmExec->GetIC()->SetPhiDegIC(0.0);                  // No roll
       


    fdmExec->GetIC()->SetVNorthFpsIC(0.0);
    fdmExec->GetIC()->SetVEastFpsIC(0.0);
    fdmExec->GetIC()->SetVDownFpsIC(0.0);          // No initial velocity
    fdmExec->GetIC()->SetPRadpsIC(0.0);            // No roll rate
    fdmExec->GetIC()->SetQRadpsIC(0.0);            // No pitch rate
    fdmExec->GetIC()->SetRRadpsIC(0.0);            // No yaw rate

    // Initialize FCS properties
    fdmExec->SetPropertyValue("fcs/elevator-cmd-norm", 0.0);
    fdmExec->SetPropertyValue("fcs/pitch-trim-cmd-norm", 0.0);
    fdmExec->SetPropertyValue("fcs/aileron-cmd-norm", 0.0);
    fdmExec->SetPropertyValue("fcs/rudder-cmd-norm", 0.0);

    // initialize variables for ACS
    bool acs_deployed = false;
    double goal_apogee = 1600; 
    bool acs_enabled = false;

    // Initialize variables for parachute deployment
    bool drogue_deployed = false;
    bool main_deployed = false;
    double max_altitude = 0.0;
    bool reached_apogee = false;

    // Initialize the model
    fdmExec->RunIC();
    
    // Enable realistic atmospheric turbulence and wind for final testing
    fdmExec->SetPropertyValue("atmosphere/turb-rate", 0.1);      // Moderate turbulence
    fdmExec->SetPropertyValue("atmosphere/turb-gain", 1.0);      // Normal gain

    auto mph_to_fps = [](double mph){ return mph * 1.4666666667; };

    // Wind Speed
    double wind_north_mph = 0.0;   // + = wind toward north in JSBSim NED convention (check your sign expectation)
    double wind_east_mph  = 0.0;

    fdmExec->SetPropertyValue("atmosphere/wind-north-fps", mph_to_fps(wind_north_mph));
    fdmExec->SetPropertyValue("atmosphere/wind-east-fps",  mph_to_fps(wind_east_mph));
    fdmExec->SetPropertyValue("atmosphere/wind-down-fps", 0.0);
    
    // Debug: Check current wind conditions
    std::cout << "\n=== REALISTIC ATMOSPHERIC CONDITIONS ===" << std::endl;
    std::cout << "Wind North: " << fdmExec->GetPropertyValue("atmosphere/wind-north-fps") << " fps (5 mph)" << std::endl;
    std::cout << "Wind East:  " << fdmExec->GetPropertyValue("atmosphere/wind-east-fps") << " fps" << std::endl;
    std::cout << "Wind Down:  " << fdmExec->GetPropertyValue("atmosphere/wind-down-fps") << " fps" << std::endl;
    std::cout << "Wind Mag:   " << fdmExec->GetPropertyValue("atmosphere/wind-mag-fps") << " fps" << std::endl;
    std::cout << "Turb Rate:  " << fdmExec->GetPropertyValue("atmosphere/turb-rate") << std::endl;
    std::cout << "=========================================" << std::endl;

    // Open an output file to save the trajectory data
    std::ofstream outputFile("midscale_trajectory.csv");
    outputFile 
    << "Time,X_ft,Y_ft,Z_ft,Altitude,Vertical_Velocity,Vertical_Acceleration,"
       "Drogue_Deployed,Main_Deployed,"
       "CG_x_in,Mass_lbs,Pred_Apogee_ft\n";

    // Initialize motor ignition sequence
    bool motor_ignited = false;
    bool engine_shutdown = false;  // Track engine shutdown state
    double ignition_time = 0.001; // Quick ignition after simulation start
    double total_impulse = 0.0;  // Track total impulse delivered
    double last_time = 0.0;      // For impulse integration, and acceleration derivation
    double shutdown_time = 0.0; // for RK4 check
    double print_interval = 0.1;
    
    // Store initial position for 3D trajectory tracking
    double initial_latitude = 34.90115786777616;  // Launch latitude - Bragg Farm
    double initial_longitude = -86.61568310338117; // Launch longitude
    double initial_altitude = 5;   // Launch altitude
    double cg_x = 0.0; 
    double mass = 47.4; // wet mass (lbs)

    // initialize variables for acceleration derivation
    double last_vertical_velocity = 0.0;
    double vertical_acceleration = 0.0;

    // Initialize RK4 model
    Rk4 predictor(10, 2.14, 47.08/2.205, 0.01824); // (hz, CD, drymass [kg], cross-section area [m^2])
    double predicted_apogee = 0;

    // -------------------------------------------------------------------------
    // RAIL FRICTION MODEL (constant force along rocket axis while on the rail)
    //
    // Requires the vehicle XML to include an external_reactions force named
    // "rail_friction" with properties:
    //   external_reactions/rail_friction/on        (0 or 1)
    //   external_reactions/rail_friction/force_lbs (force magnitude in lbf)
    //
    // We assume a 14 ft rail and apply a resistive force along BODY -X (opposes
    // thrust direction +X) until the rocket has traveled 14 ft along body +X.
    //
    // This version models:
    //   F_fric = mu * (N_preload + W*sin(tilt_from_vertical))
    // where N_preload is a constant button/rail preload (lbf).
    // -------------------------------------------------------------------------
    constexpr double PI = 3.14159265358979323846;
    constexpr double rail_length_ft = 14.0;     // assumed rail length (ft)
    const double rail_mu = 0.20;                // realistic ~0.15 - 0.30
    const double preload_per_button_lbf = 47.4 / 3.0; // tune (constant preload at each button)
    const double preload_total_lbf = 3.0 * preload_per_button_lbf;

    double rail_s_ft = 0.0;   // integrated distance along body +X (ft)
    bool on_rail = false;     // becomes true at ignition, false after rail exit

    // Initialize rail friction properties (off initially)
    fdmExec->SetPropertyValue("external_reactions/rail_friction/on", 0.0);
    fdmExec->SetPropertyValue("external_reactions/rail_friction/force_lbs", 0.0);

    auto compute_rail_friction_lbf = [&]() -> double {
        // weight in lbf
        const double weight_lbf = fdmExec->GetPropertyValue("inertial/weight-lbs");
        const double tilt_rad = rail_tilt_from_vertical_deg * (PI / 180.0);

        // crude estimate of lateral (rail-normal) load from gravity due to rail tilt
        const double N_from_tilt_lbf = weight_lbf * std::sin(tilt_rad);

        // total normal force used for Coulomb friction
        const double N_total_lbf = preload_total_lbf + std::abs(N_from_tilt_lbf);

        return rail_mu * N_total_lbf;
    };
    
    std::cout << "Starting K1800 rocket simulation" << std::endl;
    std::cout << "Expected apogee (no ACS): ~1750ft" << std::endl;
    std::cout << "Motor ignition scheduled for t=" << ignition_time << " seconds" << std::endl;

    float liftoff_threshold_agl = 10.0f;
    bool did_liftoff = false;

    // Run the simulation loop - extended for high altitude flight
    while (fdmExec->GetSimTime() < 120.0 && fdmExec->GetPropagate()->GetAltitudeASL() >= -10.0) {
        // Run the JSBSim simulation for one time step
        fdmExec->Run();

        // Get current state
        double time = fdmExec->GetSimTime();
        double altitude = fdmExec->GetPropagate()->GetAltitudeASL();
        double dt = time - last_time;
        
        // Get velocity components for proper apogee detection
        double vx = fdmExec->GetPropagate()->GetVel(1); // X velocity (forward/aft)
        double vy = fdmExec->GetPropagate()->GetVel(2); // Y velocity (left/right)  
        double vz = fdmExec->GetPropagate()->GetVel(3); // Z velocity (up/down)
        
        // For vertical rocket, use velocity magnitude for display but VZ for apogee detection
        double velocity_magnitude = sqrt(vx*vx + vy*vy + vz*vz);
        double vertical_velocity = -vz; // In JSBSim: positive Z is down, so -Z is up
        
        // Get body acceleration
        /* double a_x = fdmExec->GetAccelerations()->GetUVWdot(1);
        double a_y = fdmExec->GetAccelerations()->GetUVWdot(2);
        double a_z = fdmExec->GetAccelerations()->GetUVWdot(3); */

        /* // integrate vertical acceleration
        vertical_acceleration = (vertical_velocity - last_vertical_velocity) / dt;
        last_vertical_velocity = vertical_velocity; */

        // Body acceleration vector (ft/s²)
        JSBSim::FGColumnVector3 a_body = fdmExec->GetAccelerations()->GetBodyAccel();

        // Transform from body to local (Earth) frame
        JSBSim::FGMatrix33 T_body_to_local = fdmExec->GetPropagate()->GetTb2l();
        JSBSim::FGColumnVector3 a_local = T_body_to_local * a_body;

        // Vertical (upward) acceleration in local frame
        vertical_acceleration = -a_local(3);  // Note JSBSim uses +Z down

        // Check for numerical divergence and terminate gracefully
        if (velocity_magnitude > 10000.0 || altitude > 100000.0 || std::isnan(velocity_magnitude) || std::isnan(altitude)) {
            std::cout << "ERROR: Numerical divergence detected!" << std::endl;
            std::cout << "Time=" << time << "s, Alt=" << altitude << "ft, Vel=" << velocity_magnitude << "ft/s" << std::endl;
            std::cout << "VX=" << vx << " VY=" << vy << " VZ=" << vz << std::endl;
            std::cout << "Vertical_vel=" << vertical_velocity << "ft/s" << std::endl;
            std::cout << "Terminating simulation to prevent crash..." << std::endl;
            break;
        }

        /* // Simple velocity debugging during motor burn only
        if (motor_ignited && time < 1.0 && fmod(time, 0.5) < 0.01) {
            std::cout << "DEBUG: Altitude=" << altitude << "ft, Vertical_vel=" << vertical_velocity << "ft/s" << std::endl;
        }

        // Debug angular orientation during early flight
        if (time < 5.0 && fmod(time, 0.2) < 0.01) {
            double pitch_deg = fdmExec->GetPropagate()->GetEuler(2) * 180.0 / 3.14159; // Theta (pitch)
            double yaw_deg = fdmExec->GetPropagate()->GetEuler(3) * 180.0 / 3.14159;   // Psi (yaw) 
            double roll_deg = fdmExec->GetPropagate()->GetEuler(1) * 180.0 / 3.14159;  // Phi (roll)
            
            std::cout << "ORIENTATION t=" << std::fixed << std::setprecision(2) << time 
                      << "s: Pitch=" << std::setprecision(1) << pitch_deg 
                      << "°, Yaw=" << yaw_deg << "°, Roll=" << roll_deg << "°" << std::endl;
        } */

        // Ignite motor at scheduled time using throttle setting for solid rockets
        if (!motor_ignited && time >= ignition_time) {
            std::cout << "Igniting engine at t=" << time << "s" << std::endl;
            fdmExec->GetFCS()->SetThrottleCmd(0, 1.0);  // Set throttle to 100% for solid rocket ignition
            auto engine = fdmExec->GetPropulsion()->GetEngine(0);
            engine->SetRunning(true);  // Also set engine to running state
            
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
            
            std::cout << "Ground contacts disabled for powered flight" << std::endl;

            // -----------------------------------------------------------------
            // TURN ON RAIL FRICTION AT IGNITION
            // (will remain on until we integrate 14 ft along the rail)
            // -----------------------------------------------------------------
            rail_s_ft = 0.0;
            on_rail = true;

            const double F_fric_lbf = compute_rail_friction_lbf();
            fdmExec->SetPropertyValue("external_reactions/rail_friction/force_lbs", F_fric_lbf);
            fdmExec->SetPropertyValue("external_reactions/rail_friction/on", 1.0);

            std::cout << "Rail friction enabled: mu=" << rail_mu
                      << ", preload_total=" << preload_total_lbf << " lbf"
                      << ", tilt=" << rail_tilt_from_vertical_deg << " deg"
                      << ", F_fric=" << F_fric_lbf << " lbf" << std::endl;
            
            motor_ignited = true;
        }

        // ---------------------------------------------------------------------
        // RAIL FRICTION APPLICATION (distance-based cut-off at 14 ft)
        //
        // Integrate distance traveled along the rocket body +X axis, then disable
        // the constant friction force once the rail length is exceeded.
        // ---------------------------------------------------------------------
        if (motor_ignited && on_rail && dt > 0.0) {
            // Body-axis forward speed (ft/s). In JSBSim body axes: X forward, Y right, Z down.
            double u_fps = fdmExec->GetPropertyValue("velocities/u-fps");

            // Integrate only forward motion (avoid subtracting due to noise)
            if (u_fps > 0.0) rail_s_ft += u_fps * dt;

            // Optionally update friction during burn (weight changes slightly)
            const double F_fric_lbf = compute_rail_friction_lbf();
            fdmExec->SetPropertyValue("external_reactions/rail_friction/force_lbs", F_fric_lbf);
            fdmExec->SetPropertyValue("external_reactions/rail_friction/on", 1.0);

            if (rail_s_ft >= rail_length_ft) {
                on_rail = false;
                fdmExec->SetPropertyValue("external_reactions/rail_friction/on", 0.0);
                fdmExec->SetPropertyValue("external_reactions/rail_friction/force_lbs", 0.0);

                std::cout << "Rail exit detected at t=" << time
                          << "s (rail_s=" << rail_s_ft << " ft). Rail friction disabled." << std::endl;
            }
        }

        /* // Debug output for engine state during motor burn phase
        if (motor_ignited && time < 3.0) {  // Shortened from 5.0s to cover the 2.1s burn + coast
            if (time - ignition_time < 0.2 || fmod(time, 0.5) < 0.01) {  // Show for first 0.2s, then every 0.5s
                auto engine = fdmExec->GetPropulsion()->GetEngine(0);
                double throttle = fdmExec->GetFCS()->GetThrottlePos(0);
                double propellant_remaining = fdmExec->GetPropulsion()->GetTank(0)->GetContents();
                double propellant_consumed = 3.9 - propellant_remaining;
                double burn_percentage = (propellant_consumed / 3.9) * 100.0;
                
                // Integrate impulse
                double dt = time - last_time;
                if (dt > 0) {
                    total_impulse += engine->GetThrust() * dt;
                }
                
                std::cout << "t=" << std::fixed << std::setprecision(2) << time << "s: "
                          << "Thrust=" << std::setprecision(1) << engine->GetThrust() << "lbs, " 
                          << "Fuel=" << std::setprecision(2) << propellant_remaining << "lbs, "
                          << "Burn=" << std::setprecision(1) << burn_percentage << "%, "
                          << "Impulse=" << std::setprecision(1) << total_impulse << "lbf⋅s" << std::endl;
            }
        } */
        
        // Continue impulse integration even after debug output stops
        if (motor_ignited && !engine_shutdown) {
            auto engine = fdmExec->GetPropulsion()->GetEngine(0);
            if (dt > 0) {
                total_impulse += engine->GetThrust() * dt;
            }
        }
        
        last_time = time;

        // MECO based on engine burn time given by manufacturer
        if (motor_ignited && !engine_shutdown) {
            double propellant_remaining = fdmExec->GetPropulsion()->GetTank(0)->GetContents();
            double burn_time = time - ignition_time;
            
            if (burn_time >= 1.67) {  // MECO after 1.4 s (K1800 burn time)
                auto engine = fdmExec->GetPropulsion()->GetEngine(0);
                engine->SetRunning(false);
                fdmExec->GetFCS()->SetThrottleCmd(0, 0.0);
                
                // Force fuel tank to zero to prevent further table lookups
                fdmExec->GetPropulsion()->GetTank(0)->SetContents(0.0);
                
                engine_shutdown = true;
                std::cout << "Engine shutdown at t=" << time << "s (fuel=" << propellant_remaining 
                          << "lbs, burn_time=" << burn_time << "s)" << std::endl;
                std::cout << "TOTAL IMPULSE DELIVERED: " << total_impulse << " lbf⋅s (expected: 973.27 lbf⋅s)" << std::endl;
            
                shutdown_time = time;
            }
        }

        if (velocity_magnitude > 0.0f && altitude > liftoff_threshold_agl && !did_liftoff) {
            std::cout << "Liftoff detected at " << altitude << " ft" << std::endl;
            did_liftoff = true;
        }

        // Track maximum altitude and detect apogee more robustly
        if (did_liftoff && altitude > max_altitude) {
            max_altitude = altitude;
        }
        
        // Detect apogee when vertical velocity becomes negative after liftoff
        if (did_liftoff && !reached_apogee && vertical_velocity < -20.0 && altitude > 200.0) { // Very conservative thresholds
            reached_apogee = true;
            std::cout << "Apogee reached at " << max_altitude << " ft (current alt: " << altitude << " ft)" << std::endl;
        }

        // Deploy ACS when predicted apogee exceeds goal apogee
        if (acs_enabled && !acs_deployed && predicted_apogee >= goal_apogee && engine_shutdown && time > shutdown_time + 0.5){
            fdmExec->SetPropertyValue("aero/ACSangle", 90*(PI/180)); // set acs to 90 deg (needs radians)
            acs_deployed = true;
            std::cout << "t=" << time << "s, Pred. Apogee: " << predicted_apogee << "ft, ACS Deployed at " << altitude << " ft" << std::endl;

        }

        // Deploy drogue chute at apogee
        if (reached_apogee && !drogue_deployed) {
            fdmExec->SetPropertyValue("external_reactions/drogue_chute/drogue_open", 1); 
            drogue_deployed = true;
            std::cout << "Drogue chute deployed at " << altitude << " ft" << std::endl;
        }

        // Deploy main chute below 550 feet (per requirements) 
        if (reached_apogee && !main_deployed && altitude < 550.0) {
            fdmExec->SetPropertyValue("external_reactions/main_chute/main_open", 1); 
            main_deployed = true;
            std::cout << "Main chute deployed at " << altitude << " ft" << std::endl;
        }

        // Print and save trajectory data with improved output formatting
        if (fmod(time, print_interval) < 0.001) {  

            std::cout << std::fixed << std::setprecision(1);
            std::cout << "t=" << time << "s: Alt=" << altitude << "ft, Vel=" << velocity_magnitude << "ft/s";
            
            // Show flight phase information
            if (!did_liftoff) {
                std::cout << " [ON PAD]";
            } else if (!engine_shutdown) {
                std::cout << " [POWERED FLIGHT]";
            } else if (!reached_apogee) {
                std::cout << " [COASTING UP]";
            } else if (!drogue_deployed) {
                std::cout << " [FALLING]";
            } else if (!main_deployed) {
                std::cout << " [DROGUE DESCENT]";
            } else {
                std::cout << " [MAIN CHUTE]";
            }
            std::cout << std::endl;
        }

        if (acs_deployed) {
            print_interval = 10; // lengthen print interval after acs deployment
        }

        // find cg-x location
        cg_x = fdmExec->GetMassBalance()->GetXYZcg(1);
        mass = fdmExec->GetMassBalance()->GetMass() * 32.174; //mass is stored in slugs, convert to lbm
        
        // Calculate 3D position relative to launch point
        predicted_apogee = 3.28084 * predictor.rk4_apogee_predictor(altitude*0.3048,vertical_velocity*0.3048); // convert units
        double current_lat = fdmExec->GetPropagate()->GetLocation().GetLatitudeDeg();
        double current_lon = fdmExec->GetPropagate()->GetLocation().GetLongitudeDeg();
        double current_alt = fdmExec->GetPropagate()->GetAltitudeASL();
        
        // Convert to local coordinates in feet (X=North, Y=East, Z=Up)
        double x_pos = (current_lat - initial_latitude) * 364000.0;  // North-South in feet
        double y_pos = (current_lon - initial_longitude) * 364000.0 * cos(initial_latitude * 3.14159265359 / 180.0);  // East-West in feet
        double z_pos = current_alt - initial_altitude;  // Height above launch point in feet
        
        outputFile << time << "," << x_pos << "," << y_pos << "," << z_pos << "," 
        << altitude << "," << vertical_velocity << "," << vertical_acceleration << "," 
        //<< a_x << "," << a_y << "," << a_z << ","
        << drogue_deployed << "," << main_deployed << ","
        << cg_x << "," << mass << "," << predicted_apogee << "\n";

        if (did_liftoff && reached_apogee && altitude < 5.0) {  // Only terminate after apogee and very low altitude
            std::cout << "Rocket has reached the ground after flight." << std::endl;
            break;
        }

        /* // Add detailed monitoring during descent phase
        if (reached_apogee && altitude < 700.0 && time > 11.0) {
            if (fmod(time, 0.1) < 0.01) {  // Every 0.1 seconds during critical descent
                double alpha = fdmExec->GetAuxiliary()->Getalpha() * 180.0/3.14159; // Convert to degrees
                double beta = fdmExec->GetAuxiliary()->Getbeta() * 180.0/3.14159;
                double mach = fdmExec->GetAuxiliary()->GetMach();
                double qbar = fdmExec->GetAuxiliary()->Getqbar();
                
                std::cout << "DESCENT DEBUG t=" << std::fixed << std::setprecision(1) << time 
                          << "s: Alt=" << altitude << "ft, VMag=" << velocity_magnitude 
                          << "ft/s, VZ=" << vertical_velocity << "ft/s" << std::endl;
                std::cout << "  Alpha=" << alpha << "deg, Beta=" << beta 
                          << "deg, Mach=" << mach << ", Qbar=" << qbar << "psf" << std::endl;
            }
        } */
    }

    std::cout << "Simulation complete." << std::endl;
    std::cout << "Maximum altitude reached: " << max_altitude << " ft" << std::endl;
    std::cout << "Time to reach the ground: " << fdmExec->GetSimTime() << " s" << std::endl;

    outputFile.close();
    return 0;
}