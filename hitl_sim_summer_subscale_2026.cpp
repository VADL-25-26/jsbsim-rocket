#include <iostream>
#include <fstream>
#include <iomanip>
#include <memory>
#include <cmath>
#define _USE_MATH_DEFINES
#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>
#include <models/FGInertial.h>
#include <models/FGAtmosphere.h>
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
#include <vector>
#include <sstream>
#include <cstdlib>
#include "RK4.h"

constexpr size_t PACKET_LEN = 48;
constexpr float FT_TO_M = 0.3048f;
constexpr double RAD_TO_DEG = 180.0 / M_PI;

// Test hook for HIL roll detection. JSBSim's current midscale model has no
// natural roll disturbance, so P/gyro_x can be exactly zero for the whole run.
// Keep this enabled while validating the Teensy roll integration path; disable
// it once a physical roll disturbance or ACS torque model is connected.
constexpr bool HIL_ENABLE_ROLL_RATE_TEST = false;
constexpr double HIL_ROLL_TEST_START_S = 2.35;
constexpr double HIL_ROLL_TEST_END_S = 2.75;
constexpr float HIL_ROLL_TEST_RATE_RADPS = 1.0f;

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
                float pressure,
                float gx, float gy, float gz)
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
    std::memcpy(&packet[34], &gx,       4);
    std::memcpy(&packet[38], &gy,       4);
    std::memcpy(&packet[42], &gz,       4);

    uint16_t crc = vn_crc16(&packet[1], PACKET_LEN - 3);

    packet[46] = (crc >> 8) & 0xFF;
    packet[47] = crc & 0xFF;
}


struct AttitudeReplaySample {
    double time_s;
    float yaw_deg;
    float pitch_deg;
    float roll_deg;
    float gyro_x;
    float gyro_y;
    float gyro_z;
};

std::vector<std::string> split_csv_line(const std::string& line)
{
    std::vector<std::string> cells;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
        cells.push_back(cell);
    }
    return cells;
}

bool parse_double_cell(const std::vector<std::string>& row, int index, double& value)
{
    if (index < 0 || static_cast<size_t>(index) >= row.size()) {
        return false;
    }
    char* end = nullptr;
    value = std::strtod(row[index].c_str(), &end);
    return end != row[index].c_str();
}

int column_index(const std::unordered_map<std::string, int>& columns, const std::string& name)
{
    auto it = columns.find(name);
    return it == columns.end() ? -1 : it->second;
}

bool load_attitude_replay(const std::string& path, std::vector<AttitudeReplaySample>& samples)
{
    std::ifstream input(path);
    if (!input.is_open()) {
        std::cerr << "Failed to open attitude replay CSV: " << path << std::endl;
        return false;
    }

    std::string line;
    if (!std::getline(input, line)) {
        std::cerr << "Attitude replay CSV is empty: " << path << std::endl;
        return false;
    }

    std::vector<std::string> header = split_csv_line(line);
    std::unordered_map<std::string, int> columns;
    for (size_t i = 0; i < header.size(); ++i) {
        columns[header[i]] = static_cast<int>(i);
    }

    const int time_col = column_index(columns, "System Time (ms)");
    const int phase_col = column_index(columns, "Phase");
    const int yaw_col = column_index(columns, "Yaw (deg)");
    const int pitch_col = column_index(columns, "Pitch (deg)");
    const int roll_col = column_index(columns, "Roll (deg)");
    const int gx_col = column_index(columns, "Gyro X");
    const int gy_col = column_index(columns, "Gyro Y");
    const int gz_col = column_index(columns, "Gyro Z");

    if (time_col < 0 || yaw_col < 0 || pitch_col < 0 || roll_col < 0 ||
        gx_col < 0 || gy_col < 0 || gz_col < 0) {
        std::cerr << "Attitude replay CSV is missing System Time, YPR, or Gyro columns." << std::endl;
        return false;
    }

    struct RawRow {
        double time_ms;
        double phase;
        AttitudeReplaySample sample;
    };

    std::vector<RawRow> rows;
    while (std::getline(input, line)) {
        std::vector<std::string> row = split_csv_line(line);
        double time_ms = 0.0;
        double yaw = 0.0;
        double pitch = 0.0;
        double roll = 0.0;
        double gx = 0.0;
        double gy = 0.0;
        double gz = 0.0;
        double phase = 0.0;

        if (!parse_double_cell(row, time_col, time_ms) ||
            !parse_double_cell(row, yaw_col, yaw) ||
            !parse_double_cell(row, pitch_col, pitch) ||
            !parse_double_cell(row, roll_col, roll) ||
            !parse_double_cell(row, gx_col, gx) ||
            !parse_double_cell(row, gy_col, gy) ||
            !parse_double_cell(row, gz_col, gz)) {
            continue;
        }

        if (phase_col >= 0) {
            parse_double_cell(row, phase_col, phase);
        }

        rows.push_back({time_ms, phase, {0.0, static_cast<float>(yaw), static_cast<float>(pitch),
            static_cast<float>(roll), static_cast<float>(gx), static_cast<float>(gy), static_cast<float>(gz)}});
    }

    if (rows.empty()) {
        std::cerr << "Attitude replay CSV has no usable samples." << std::endl;
        return false;
    }

    double launch_time_ms = rows.front().time_ms;
    for (const RawRow& row : rows) {
        if (row.phase >= 2.0) {
            launch_time_ms = row.time_ms;
            break;
        }
    }

    samples.clear();
    samples.reserve(rows.size());
    for (RawRow row : rows) {
        row.sample.time_s = (row.time_ms - launch_time_ms) / 1000.0;
        if (row.sample.time_s >= 0.0) {
            samples.push_back(row.sample);
        }
    }

    if (samples.empty()) {
        std::cerr << "Attitude replay CSV has no samples at or after launch/boost." << std::endl;
        return false;
    }

    std::cout << "Loaded attitude replay: " << path
              << " (" << samples.size() << " samples, "
              << samples.front().time_s << "s to " << samples.back().time_s << "s)" << std::endl;
    return true;
}

bool sample_attitude_replay(
    const std::vector<AttitudeReplaySample>& samples,
    double time_s,
    AttitudeReplaySample& out)
{
    if (samples.empty()) {
        return false;
    }

    if (time_s <= samples.front().time_s) {
        out = samples.front();
        return true;
    }
    if (time_s >= samples.back().time_s) {
        out = samples.back();
        return true;
    }

    size_t upper = 1;
    while (upper < samples.size() && samples[upper].time_s < time_s) {
        ++upper;
    }

    const AttitudeReplaySample& a = samples[upper - 1];
    const AttitudeReplaySample& b = samples[upper];
    const double span = b.time_s - a.time_s;
    const float alpha = span > 0.0 ? static_cast<float>((time_s - a.time_s) / span) : 0.0f;

    auto lerp = [alpha](float left, float right) {
        return left + (right - left) * alpha;
    };

    out.time_s = time_s;
    out.yaw_deg = lerp(a.yaw_deg, b.yaw_deg);
    out.pitch_deg = lerp(a.pitch_deg, b.pitch_deg);
    out.roll_deg = lerp(a.roll_deg, b.roll_deg);
    out.gyro_x = lerp(a.gyro_x, b.gyro_x);
    out.gyro_y = lerp(a.gyro_y, b.gyro_y);
    out.gyro_z = lerp(a.gyro_z, b.gyro_z);
    return true;
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

    // Dedicated 2026 summer subscale HIL executable. Keep the aircraft fixed
    // so midscale/fullscale command-line choices cannot affect this run.
    const std::string aircraftName = "summer_subscale_2026";
    std::string attitudeReplayPath;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--attitude-replay") {
            if (i + 1 >= argc) {
                std::cerr << "Usage: " << argv[0] << " [--attitude-replay flight.csv]" << std::endl;
                return 1;
            }
            attitudeReplayPath = argv[++i];
        } else {
            std::cerr << "Usage: " << argv[0] << " [--attitude-replay flight.csv]" << std::endl;
            return 1;
        }
    }

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

    std::vector<AttitudeReplaySample> attitudeReplay;
    if (!attitudeReplayPath.empty() && !load_attitude_replay(attitudeReplayPath, attitudeReplay)) {
        return 1;
    }

    double motor_burn_time_s = 2.3;
    bool single_chute_recovery = false;
    const bool is_summer_subscale = aircraftName == "summer_subscale_2026";
    if (is_summer_subscale) {
        motor_burn_time_s = 1.12;
        single_chute_recovery = true;
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

    // Gallatin, Tennessee launch conditions.
    const double launch_terrain_elevation_ft = is_summer_subscale ? 545.0 : 0.0;
    const double launch_altitude_ft =
        is_summer_subscale ? launch_terrain_elevation_ft + 3.28084 : 10.5;
    const double launch_latitude_deg = is_summer_subscale ? 36.388303 : 34.90115786777616;
    const double launch_longitude_deg = is_summer_subscale ? -86.44759 : -86.61568310338117;
    const double rail_tilt_from_vertical_deg = is_summer_subscale ? 5.25 : 0.0;
    const double rail_azimuth_deg = is_summer_subscale ? 270.0 : 180.0;

    fdmExec->GetIC()->SetTerrainElevationFtIC(launch_terrain_elevation_ft);
    fdmExec->GetIC()->SetAltitudeASLFtIC(launch_altitude_ft);
    fdmExec->GetIC()->SetLatitudeDegIC(launch_latitude_deg);
    fdmExec->GetIC()->SetLongitudeDegIC(launch_longitude_deg);
    fdmExec->GetIC()->SetThetaDegIC(90.0 - rail_tilt_from_vertical_deg);
    fdmExec->GetIC()->SetPhiDegIC(0.0);            // No roll
    fdmExec->GetIC()->SetPsiDegIC(rail_azimuth_deg);
    fdmExec->GetIC()->SetVNorthFpsIC(0.0);
    fdmExec->GetIC()->SetVEastFpsIC(0.0);
    fdmExec->GetIC()->SetVDownFpsIC(0.0);          // No initial velocity
    fdmExec->GetIC()->SetPRadpsIC(0.0);            // No roll rate
    fdmExec->GetIC()->SetQRadpsIC(0.0);            // No pitch rate
    fdmExec->GetIC()->SetRRadpsIC(0.0);            // No yaw rate

    auto set_racs_roll_angle = [&](double angle_rad) {
        fdmExec->SetPropertyValue("aero/RACS_roll_cmd_rad", angle_rad);
        fdmExec->SetPropertyValue("fcs/racs_fin_1_pos_rad", angle_rad);
        fdmExec->SetPropertyValue("fcs/racs_fin_2_pos_rad", angle_rad);
        fdmExec->SetPropertyValue("fcs/racs_fin_3_pos_rad", angle_rad);
        fdmExec->SetPropertyValue("fcs/racs_fin_4_pos_rad", angle_rad);
    };

    // Initialize FCS properties
    fdmExec->SetPropertyValue("fcs/elevator-cmd-norm", 0.0);
    fdmExec->SetPropertyValue("fcs/pitch-trim-cmd-norm", 0.0);
    fdmExec->SetPropertyValue("fcs/aileron-cmd-norm", 0.0);
    fdmExec->SetPropertyValue("fcs/rudder-cmd-norm", 0.0);
    set_racs_roll_angle(0.0);
    fdmExec->SetPropertyValue("aero/RACS_cant_active", 1.0);

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

    const double desired_launch_temperature_c = 31.5;
    const double desired_launch_temperature_r =
        (desired_launch_temperature_c + 273.15) * 1.8;
    const double standard_launch_temperature_r =
        fdmExec->GetPropertyValue("atmosphere/T-R");
    fdmExec->SetPropertyValue("atmosphere/delta-T",
                              desired_launch_temperature_r - standard_launch_temperature_r);
    fdmExec->GetAtmosphere()->Run(false);
    if (is_summer_subscale) {
        constexpr double target_pad_pressure_kpa = 101.76098;
        constexpr double kpa_to_psf = 20.885434273;
        const double current_pad_pressure_psf = fdmExec->GetPropertyValue("atmosphere/P-psf");
        const double current_sea_level_pressure_psf = fdmExec->GetPropertyValue("atmosphere/P-sl-psf");
        fdmExec->SetPropertyValue("atmosphere/P-sl-psf",
            current_sea_level_pressure_psf *
            (target_pad_pressure_kpa * kpa_to_psf / current_pad_pressure_psf));
        fdmExec->GetAtmosphere()->Run(false);
    }
    
    // Enable realistic atmospheric turbulence and wind for final testing
    fdmExec->SetPropertyValue("atmosphere/turb-rate", 0.1);
    fdmExec->SetPropertyValue("atmosphere/turb-gain", 1.0);
    fdmExec->SetPropertyValue("atmosphere/wind-north-fps", 0.0);
    fdmExec->SetPropertyValue("atmosphere/wind-east-fps",
                              is_summer_subscale ? (7.5 * 5280.0 / 3600.0) : 0.0);
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
    const std::string output_csv = is_summer_subscale ? "hitl_summer_subscale_trajectory.csv" : "hitl_midscale_trajectory.csv";
    std::ofstream outputFile(output_csv);
    outputFile 
    << "Time,X_ft,Y_ft,Z_ft,Altitude,Vertical_Velocity,Vertical_Acceleration,"
       "Drogue_Deployed,Main_Deployed,"
       "CG_x_in,Mass_lbs,Pred_Apogee_ft,"
       "VN_Accel_X_mps2,VN_Accel_Y_mps2,VN_Accel_Z_mps2,"
       "Gyro_X_radps,Gyro_Y_radps,Gyro_Z_radps,"
       "Yaw_deg,Pitch_deg,Roll_deg,Pressure_kPa\n";

    // Initialize motor ignition sequence
    bool motor_ignited = false;
    bool engine_shutdown = false;  // Track engine shutdown state
    double ignition_time = 0.001; // Quick ignition after simulation start
    double total_impulse = 0.0;  // Track total impulse delivered
    double last_time = 0.0;      // For impulse integration, and acceleration derivation
    double shutdown_time = 0.0; // for RK4 check
    double print_interval = 0.1;
    
    // Store initial position for 3D trajectory tracking
    double initial_latitude = launch_latitude_deg;
    double initial_longitude = launch_longitude_deg;
    double initial_altitude = launch_altitude_ft;
    double cg_x = 0.0; 
    double mass = 50.4; // wet mass (lbs)

    // initialize variables for acceleration derivation
    double last_vertical_velocity = 0.0;
    double vertical_acceleration = 0.0;

    // Keep the HIL CSV predicted-apogee column populated like the standalone sim.
    Rk4 predictor(10, 0.56, 47.08/2.205, 0.01824); // (hz, ascent CD, drymass [kg], cross-section area [m^2])
    double predicted_apogee = 0.0;

    
    // std::cout << "Starting L1940 rocket simulation" << std::endl;
    // std::cout << "Motor ignition scheduled for t=" << ignition_time << " seconds" << std::endl;

    float liftoff_threshold_agl = 10.0f;
    bool did_liftoff = false;

    // Run the simulation loop - extended for high altitude flight
    while (fdmExec->GetSimTime() < 120.0 && fdmExec->GetPropagate()->GetAltitudeASL() >= launch_terrain_elevation_ft - 10.0) {
        // Run the JSBSim simulation for one time step
        fdmExec->Run();

        // Get current state
        double time = fdmExec->GetSimTime();
        double altitude = fdmExec->GetPropagate()->GetAltitudeASL();
        double altitude_agl = altitude - launch_terrain_elevation_ft;
        float pressure = static_cast<float>(fdmExec->GetPropertyValue("atmosphere/P-psf") * 0.04788025898); // psf to kPa
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

        // Body acceleration vector (ft/s²). JSBSim body axes are X forward, Y right, Z down.
        JSBSim::FGColumnVector3 a_body = fdmExec->GetAccelerations()->GetBodyAccel();

        // HIL frame expected by the current Teensy flight code:
        // accel_x is the rocket longitudinal/boost axis used for launch detect.
        float jsb_x_forward = static_cast<float>(a_body(1)) * FT_TO_M;
        float jsb_y_right = static_cast<float>(a_body(2)) * FT_TO_M;
        float jsb_z_down = static_cast<float>(a_body(3)) * FT_TO_M;

        float vn_ax = jsb_x_forward;
        float vn_ay = jsb_y_right;
        float vn_az = -jsb_z_down;

        float roll_deg = static_cast<float>(fdmExec->GetPropagate()->GetEuler(1) * RAD_TO_DEG);
        float pitch_deg = static_cast<float>(fdmExec->GetPropagate()->GetEuler(2) * RAD_TO_DEG);
        float yaw_deg = static_cast<float>(fdmExec->GetPropagate()->GetEuler(3) * RAD_TO_DEG);

        // JSBSim body angular rates: P=roll, Q=pitch, R=yaw, in rad/s.
        float gyro_x = static_cast<float>(fdmExec->GetPropagate()->GetPQR(1));
        float gyro_y = static_cast<float>(fdmExec->GetPropagate()->GetPQR(2));
        float gyro_z = static_cast<float>(fdmExec->GetPropagate()->GetPQR(3));

        AttitudeReplaySample replay_sample{};
        if (sample_attitude_replay(attitudeReplay, time, replay_sample)) {
            yaw_deg = replay_sample.yaw_deg;
            pitch_deg = replay_sample.pitch_deg;
            roll_deg = replay_sample.roll_deg;
            gyro_x = replay_sample.gyro_x;
            gyro_y = replay_sample.gyro_y;
            gyro_z = replay_sample.gyro_z;
        } else if (HIL_ENABLE_ROLL_RATE_TEST &&
            time >= HIL_ROLL_TEST_START_S &&
            time <= HIL_ROLL_TEST_END_S) {
            gyro_x = HIL_ROLL_TEST_RATE_RADPS;
        }

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
            
            if (burn_time >= motor_burn_time_s) {
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

        if (velocity_magnitude > 0.0f && altitude_agl > liftoff_threshold_agl && !did_liftoff) {
            // std::cout << "Liftoff detected at " << altitude_agl << " ft" << std::endl;
            did_liftoff = true;
        }

        // Track maximum altitude and detect apogee more robustly
        if (did_liftoff && altitude_agl > max_altitude) {
            max_altitude = altitude_agl;
        }
        
        // Detect apogee when vertical velocity becomes negative after liftoff
        if (did_liftoff && !reached_apogee && vertical_velocity < -20.0 && altitude_agl > 200.0) { // Very conservative thresholds
            reached_apogee = true;
            // std::cout << "Apogee reached at " << max_altitude << " ft (current alt: " << altitude << " ft)" << std::endl;
        }

        // Deploy ACS when predicted apogee exceeds goal apogee
        if (!acs_active && acs_enabled && stm32.acs_deployed() && engine_shutdown && time > shutdown_time + 0.5){
            fdmExec->SetPropertyValue("aero/ACSangle", 90*(M_PI/180)); // drag deployment angle, radians
            set_racs_roll_angle(0.0); // active RACS command stays zero; fixed fin cant is modeled in XML
            fdmExec->SetPropertyValue("aero/RACS_cant_active", 0.0); // ACS/chute phase should not keep injecting fixed ascent roll torque
            std::cout << "t= " << time << "s: ACS deployed at " << altitude << " ft" << std::endl;
            acs_active = true;
        }

        // Deploy the first/recovery chute at apogee. For summer_subscale_2026 this is the only chute.
        if (reached_apogee && !drogue_deployed) {
            fdmExec->SetPropertyValue("external_reactions/drogue_chute/drogue_open", 1); 
            fdmExec->SetPropertyValue("aero/RACS_cant_active", 0.0);
            drogue_deployed = true;
            // std::cout << "Drogue chute deployed at " << altitude << " ft" << std::endl;
        }

        // Deploy main chute below 550 feet (per requirements) 
        if (!single_chute_recovery && reached_apogee && !main_deployed && altitude_agl < 550.0) {
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
        predicted_apogee = 3.28084 * predictor.rk4_apogee_predictor(altitude_agl*0.3048, vertical_velocity*0.3048); // convert units
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
        << cg_x << "," << mass << "," << predicted_apogee << ","
        << vn_ax << "," << vn_ay << "," << vn_az << ","
        << gyro_x << "," << gyro_y << "," << gyro_z << ","
        << yaw_deg << "," << pitch_deg << "," << roll_deg << "," << pressure << "\n";

        if (did_liftoff && reached_apogee && altitude_agl < 5.0) {  // Only terminate after apogee and very low altitude
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
        build_packet(output_packet, yaw_deg, pitch_deg, roll_deg, vn_ax, vn_ay, vn_az, pressure, gyro_x, gyro_y, gyro_z);
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
    std::cout << "Maximum altitude reached: " << max_altitude << " ft AGL" << std::endl;
    std::cout << "Time to reach the ground: " << fdmExec->GetSimTime() << " s" << std::endl;

    outputFile.close();
    return 0;
}
