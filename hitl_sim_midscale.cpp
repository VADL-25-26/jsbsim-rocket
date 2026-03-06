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
#include <iostream>
#include <asio.hpp>
#include <atomic>
#include <thread>
#include <chrono>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <unordered_map>

constexpr size_t PACKET_LEN = 36;

uint16_t vn_crc16(const uint8_t* data, size_t length)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc = (crc >> 8) | (crc << 8);
        crc ^= data[i];
        crc ^= (crc & 0xFF) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0xFF) << 5;
    }
    return crc;
}


void build_packet(uint8_t* packet,
                float yaw, float pitch, float roll,
                float ax, float ay, float az,
                float pressure)
{
    packet[0] = 0xFA;
    packet[1] = 0x01;   // msg id
    packet[2] = 0x05;   // group flags
    packet[3] = 0x00;
    packet[4] = 0x00;
    packet[5] = 0x00;

    std::memcpy(&packet[6],  &yaw,      4);
    std::memcpy(&packet[10],  &pitch,    4);
    std::memcpy(&packet[14], &roll,     4);
    std::memcpy(&packet[18], &ax,       4);
    std::memcpy(&packet[22], &ay,       4);
    std::memcpy(&packet[26], &az,       4);
    std::memcpy(&packet[30], &pressure, 4);

    uint16_t crc = vn_crc16(&packet[1], 33);

    packet[34] = (crc >> 8) & 0xFF;
    packet[35] = crc & 0xFF;
}

void config_thread(std::thread& t, int core_id, int priority) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);
    pthread_setaffinity_np(t.native_handle(), sizeof(cpu_set_t), &cpuset);

    sched_param sch;
    sch.sched_priority = priority;
    
    int result = pthread_setschedparam(t.native_handle(), SCHED_FIFO, &sch);
    
    if (result != 0) {
        // Just a warning, doesn't kill the app
        // On WSL, this is expected unless running as sudo
        static bool warned = false;
        if (!warned) {
            std::cout << "[System] Note: Real-time priority not available (WSL/Permissions). Running in Best-Effort mode." << std::endl;
            warned = true;
        }
    }
}


class SerialLink {
public:
    SerialLink(const std::string& dev, unsigned baud)
        : io_(),
          port_(io_, dev),
          running_(true)
    {
        port_.set_option(asio::serial_port_base::baud_rate(baud));
        port_.set_option(asio::serial_port_base::character_size(8));
        port_.set_option(asio::serial_port_base::parity(
            asio::serial_port_base::parity::none));
        port_.set_option(asio::serial_port_base::stop_bits(
            asio::serial_port_base::stop_bits::one));
        port_.set_option(asio::serial_port_base::flow_control(
            asio::serial_port_base::flow_control::none));


        int fd = port_.native_handle();
        struct serial_struct ser_info;
        if (ioctl(fd, TIOCGSERIAL, &ser_info) >= 0) {
            ser_info.flags |= ASYNC_LOW_LATENCY;
            ioctl(fd, TIOCSSERIAL, &ser_info);
        }
        ::tcflush(port_.native_handle(), TCIFLUSH);

        start_rx();

        io_thread_ = std::thread([this]() {
            io_.run();
        });
    }

    ~SerialLink() {
        running_ = false;
        io_.stop();
        if (io_thread_.joinable())
            io_thread_.join();
    }

    void send(const void* data, size_t len) {
        asio::write(port_, asio::buffer(data, len));
    }

    bool acs_deployed() const {
        return acs_deploy.load(std::memory_order_acquire);
    }

    bool pds_deployed() const {
        return pds_deploy.load(std::memory_order_acquire);
    }

    bool prs_deployed() const {
        return prs_deploy.load(std::memory_order_acquire);
    }

    int get_fd() {
        return port_.native_handle();
    }

    std::thread& io_thread() {
        return io_thread_;
    }

private:
    void start_rx() {
        port_.async_read_some(
            asio::buffer(rx_buf_),
            [this](const asio::error_code& ec, size_t n) {
                if (!ec && running_) {
                    on_rx(n);
                    start_rx();
                }
            });
    }
    void on_rx(size_t n) {
        // Append new bytes to our persistent "waiting" string
        rx_accumulator.append(rx_buf_.data(), n);

        for (auto it = msg_map.begin(); it != msg_map.end(); ) {
            const std::string& trigger = it->first;
            std::atomic<bool>* flag = it->second;
            
            size_t pos = rx_accumulator.find(trigger);
            if (pos != std::string::npos) {
                flag->store(true, std::memory_order_release);
                std::cout << "[HIL] Command Recieved: " << trigger << std::endl;

                rx_accumulator.erase(0, trigger.length());

                // Remove trigger from map since it is guaranteed actuation is a one-time event
                msg_map.erase(it);
                // Only expect one command per RX Cycle
                return;
            }
            else {
                ++it; // Only increment if we didn't erase
            }
        }
        if (rx_accumulator.size() > 1000) {
            rx_accumulator.erase(0, 500); 
        }
    }


    asio::io_context io_;
    asio::serial_port port_;
    std::thread io_thread_;
    std::atomic<bool> running_;
    std::array<char, 256> rx_buf_;
    std::atomic<bool> acs_deploy{false};
    std::atomic<bool> prs_deploy{false};
    std::atomic<bool> pds_deploy{false};
    std::string rx_accumulator;
    std::unordered_map<std::string, std::atomic<bool>*> msg_map {
        {"ACS_PWM_CHANGED\n", &acs_deploy},
        {"PRS_PWM_CHANGED\n", &prs_deploy},
        {"PDS_PWM_CHANGED\n", &pds_deploy}
    };
};

int main(int argc, char* argv[]) {
    using clock = std::chrono::steady_clock;
    constexpr auto STEP = std::chrono::milliseconds(5); // 200 Hz

    auto next_tick = clock::now() + STEP;
    uint8_t output_packet[PACKET_LEN];

    SerialLink stm32("/dev/ttyACM0", 115200);  // adjust ttyS*
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    ::tcflush(stm32.get_fd(), TCIFLUSH);

    config_thread(stm32.io_thread(), 1, 80);

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    
    // Create an instance of the JSBSim flight dynamics model executor
    std::unique_ptr<JSBSim::FGFDMExec> fdmExec(new JSBSim::FGFDMExec());

    // Set the simulation to run at 200 Hz (matching VN100 speed)
    fdmExec->Setdt(1.0 / 400.0);

    // select which rocket to simulate
    std::string aircraftName = "fullscale";

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
    fdmExec->GetIC()->SetAltitudeASLFtIC(10.5);    // Start slightly higher to avoid ground contact
    fdmExec->GetIC()->SetLatitudeDegIC(34.90115786777616);
    fdmExec->GetIC()->SetLongitudeDegIC(-86.61568310338117);
    fdmExec->GetIC()->SetThetaDegIC(90.0);         // 0° forward tilt (subtract from 90)
    fdmExec->GetIC()->SetPhiDegIC(0.0);            // No roll
    fdmExec->GetIC()->SetPsiDegIC(180.0);          // Point south (into north wind)
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

    bool acs_enabled = true;
    bool acs_active = false;

    bool prs_active = false;
    bool pds_active = false;

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
    fdmExec->SetPropertyValue("atmosphere/wind-north-fps", 0); 
    fdmExec->SetPropertyValue("atmosphere/wind-east-fps", 0.0);   // No east wind
    fdmExec->SetPropertyValue("atmosphere/wind-down-fps", 0.0);   // No vertical wind
    
    // // Debug: Check current wind conditions
    // std::cout << "\n=== REALISTIC ATMOSPHERIC CONDITIONS ===" << std::endl;
    // std::cout << "Wind North: " << fdmExec->GetPropertyValue("atmosphere/wind-north-fps") << " fps (5 mph)" << std::endl;
    // std::cout << "Wind East:  " << fdmExec->GetPropertyValue("atmosphere/wind-east-fps") << " fps" << std::endl;
    // std::cout << "Wind Down:  " << fdmExec->GetPropertyValue("atmosphere/wind-down-fps") << " fps" << std::endl;
    // std::cout << "Wind Mag:   " << fdmExec->GetPropertyValue("atmosphere/wind-mag-fps") << " fps" << std::endl;
    // std::cout << "Turb Rate:  " << fdmExec->GetPropertyValue("atmosphere/turb-rate") << std::endl;
    // std::cout << "=========================================" << std::endl;

    // Open an output file to save the trajectory data
    std::ofstream outputFile("hitl_midscale_trajectory.csv");
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
    double mass = 50.4; // wet mass (lbs)

    // initialize variables for acceleration derivation
    double last_vertical_velocity = 0.0;
    double vertical_acceleration = 0.0;

    
    // std::cout << "Starting L1940 rocket simulation" << std::endl;
    // std::cout << "Motor ignition scheduled for t=" << ignition_time << " seconds" << std::endl;

    float liftoff_threshold_agl = 10.0f;
    bool did_liftoff = false;

    // Run the simulation loop - extended for high altitude flight
    while (fdmExec->GetSimTime() < 120.0 && fdmExec->GetPropagate()->GetAltitudeASL() >= -10.0) {
        // Run the JSBSim simulation for one time step
        fdmExec->Run();

        // Get current state
        double time = fdmExec->GetSimTime();
        double altitude = fdmExec->GetPropagate()->GetAltitudeASL();
        float pressure = 101.325 * powf32((1.0f - (altitude / 145366.45)), (1.0f/0.190284f)); // Altitude to pressure (ft to kPa
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
            // std::cout << "ERROR: Numerical divergence detected!" << std::endl;
            // std::cout << "Time=" << time << "s, Alt=" << altitude << "ft, Vel=" << velocity_magnitude << "ft/s" << std::endl;
            // std::cout << "VX=" << vx << " VY=" << vy << " VZ=" << vz << std::endl;
            // std::cout << "Vertical_vel=" << vertical_velocity << "ft/s" << std::endl;
            // std::cout << "Terminating simulation to prevent crash..." << std::endl;
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
            // std::cout << "Igniting engine at t=" << time << "s" << std::endl;
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
            
            // std::cout << "Ground contacts disabled for powered flight" << std::endl;
            
            motor_ignited = true;
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
            
            if (burn_time >= 2.3) {  // MECO after 2.3 s (L1940 burn time)
                auto engine = fdmExec->GetPropulsion()->GetEngine(0);
                engine->SetRunning(false);
                fdmExec->GetFCS()->SetThrottleCmd(0, 0.0);
                
                // Force fuel tank to zero to prevent further table lookups
                fdmExec->GetPropulsion()->GetTank(0)->SetContents(0.0);
                
                engine_shutdown = true;
                // std::cout << "Engine shutdown at t=" << time << "s (fuel=" << propellant_remaining 
                //           << "lbs, burn_time=" << burn_time << "s)" << std::endl;
                // std::cout << "TOTAL IMPULSE DELIVERED: " << total_impulse << " lbf⋅s (expected: 973.27 lbf⋅s)" << std::endl;
            
                shutdown_time = time;
            }
        }

        if (velocity_magnitude > 0.0f && altitude > liftoff_threshold_agl && !did_liftoff) {
            // std::cout << "Liftoff detected at " << altitude << " ft" << std::endl;
            did_liftoff = true;
        }

        // Track maximum altitude and detect apogee more robustly
        if (did_liftoff && altitude > max_altitude) {
            max_altitude = altitude;
        }
        
        // Detect apogee when vertical velocity becomes negative after liftoff
        if (did_liftoff && !reached_apogee && vertical_velocity < -20.0 && altitude > 200.0) { // Very conservative thresholds
            reached_apogee = true;
            // std::cout << "Apogee reached at " << max_altitude << " ft (current alt: " << altitude << " ft)" << std::endl;
        }

        // Deploy ACS when predicted apogee exceeds goal apogee
        if (!acs_active && acs_enabled && stm32.acs_deployed() && engine_shutdown && time > shutdown_time + 0.5){
            fdmExec->SetPropertyValue("aero/ACSangle", 90*(M_PI/180)); // set acs to 90 deg (needs radians)
            std::cout << "t= " << time << "s: ACS deployed at " << altitude << " ft" << std::endl;
            acs_active = true;
        }

        // Deploy drogue chute at apogee
        if (reached_apogee && !drogue_deployed) {
            fdmExec->SetPropertyValue("external_reactions/drogue_chute/drogue_open", 1); 
            drogue_deployed = true;
            // std::cout << "Drogue chute deployed at " << altitude << " ft" << std::endl;
        }

        // Deploy main chute below 550 feet (per requirements) 
        if (reached_apogee && !main_deployed && altitude < 550.0) {
            fdmExec->SetPropertyValue("external_reactions/main_chute/main_open", 1); 
            main_deployed = true;
            // std::cout << "Main chute deployed at " << altitude << " ft" << std::endl;
        }

        // Print and save trajectory data with improved output formatting
        if (fmod(time, print_interval) < 0.001) {  
            // std::cout << std::fixed << std::setprecision(1);
            // std::cout << "t=" << time << "s: Alt=" << altitude << "ft, Vel=" << velocity_magnitude << "ft/s, Pressure=" << pressure << "kPa";
            
            // // Show flight phase information
            // if (!did_liftoff) {
            //     std::cout << " [ON PAD]";
            // } else if (!engine_shutdown) {
            //     std::cout << " [POWERED FLIGHT]";
            // } else if (!reached_apogee) {
            //     std::cout << " [COASTING UP]";
            // } else if (!drogue_deployed) {
            //     std::cout << " [FALLING]";
            // } else if (!main_deployed) {
            //     std::cout << " [DROGUE DESCENT]";
            // } else {
            //     std::cout << " [MAIN CHUTE]";
            // }
            // std::cout << std::endl;
        }

        if (acs_active) {
            print_interval = 10; // lengthen print interval after acs deployment
        }

        // find cg-x location
        cg_x = fdmExec->GetMassBalance()->GetXYZcg(1);
        mass = fdmExec->GetMassBalance()->GetMass() * 32.174; //mass is stored in slugs, convert to lbm
        
        // Calculate 3D position relative to launch point
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
        << cg_x << "," << mass << "," << "\n";

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

        // Send HIL packet to STM32
        build_packet(output_packet, 0x00, 0x00, 0x00, 0x00, 0x00, vertical_acceleration * 0.3048f, pressure);
        stm32.send(output_packet, PACKET_LEN);

        auto now = clock::now();

        if (now > next_tick + STEP) {
            // We missed a deadline — resync
            next_tick = now + STEP;
        } else {
            next_tick += STEP;
        }
        std::this_thread::sleep_until(next_tick);
        
        // std::cout << "[HIL] Sent packet at t=" << time << "s" << std::endl;
    }

    std::cout << "Simulation complete." << std::endl;
    std::cout << "Maximum altitude reached: " << max_altitude << " ft" << std::endl;
    std::cout << "Time to reach the ground: " << fdmExec->GetSimTime() << " s" << std::endl;

    outputFile.close();
    return 0;
}
