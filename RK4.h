#pragma once
#include <cmath>

class Rk4 {
public:
    // default values
    Rk4(double frequency = 10.0, double coeff_drag = 3.0, double mass = 5.0, double area = 1.0)
        : FREQUENCY(frequency),
          MAX_TIME(16.0),
          GRAV_CONST(9.80665),
          COEFF_DRAG(coeff_drag),
          AIR_DENSITY(1.225),
          MASS(mass),
          AREA(area) {}

    // RK4 model - everything going into this model is in METRIC
    double rk4_apogee_predictor(double h0, double v0) {
        double dt = 1.0 / FREQUENCY;
        int max_steps = static_cast<int>(MAX_TIME / dt);

        double h = h0; // height (m)
        double v = v0; // velocity (m/s)

        for (int i = 0; i < max_steps; ++i) {
            if (v <= 0.0) break;

            // Fourth-order RK4 integration
            auto [k1_v, k1_h] = find_derivatives(v);
            auto [k2_v, k2_h] = find_derivatives(v + 0.5 * dt * k1_v);
            auto [k3_v, k3_h] = find_derivatives(v + 0.5 * dt * k2_v);
            auto [k4_v, k4_h] = find_derivatives(v + dt * k3_v);

            v += (dt / 6.0) * (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v);
            h += (dt / 6.0) * (k1_h + 2.0 * k2_h + 2.0 * k3_h + k4_h);
        }
        return h;
    }

private:
    // Constants
    double FREQUENCY;
    double MAX_TIME;
    double GRAV_CONST;
    double COEFF_DRAG;
    double AIR_DENSITY;
    double MASS;
    double AREA;

    // Compute derivatives: dv/dt and dh/dt
    std::pair<double, double> find_derivatives(double velocity) {
        double drag = 0.5 * COEFF_DRAG * AREA * AIR_DENSITY * velocity * velocity * ((velocity >= 0.0) ? 1.0 : -1.0);
        double dv = -GRAV_CONST - drag / MASS;
        double dh = velocity;
        return {dv, dh};
    }
};
