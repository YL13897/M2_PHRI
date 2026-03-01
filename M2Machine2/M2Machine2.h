
#ifndef M2_MACHINE_H
#define M2_MACHINE_H

#include "RobotM2.h"
#include "StateMachine.h"
#include "FLNLHelper.h"

// State Classes
#include "M2States2.h"

// Top-level state machine wrapper for M2 robot runtime.
// - Owns the RobotM2 instance and the simple TCP/UDP UI helper.
// - Wires up states and transitions and proxies periodic hw updates.
class M2Machine : public StateMachine {
public:
    M2Machine();
    ~M2Machine();

    // Set up robot, logging, and UI server
    void init();
    // Tear down UI and state machine
    void end();

    // Per-cycle hardware/state update tick
    void hwStateUpdate();

    RobotM2* robot() { return static_cast<RobotM2*>(_robot.get()); } // typed getter

    std::shared_ptr<FLNLHelper> UIserver = nullptr; // UI/command server
    // Session/subject identifier provided by Unity
    std::string sessionId = "UNSET";
};

#endif /* M2_MACHINE_H */
