#include "rocket_simulation.h"


int main(int argc, char* argv[]) {
    RocketSimulation sim;
    int steps = 0;
    while (steps < 100000000) {
        if (sim.step()) {
            break;
        }
        steps++;
    }

    return 0;
}