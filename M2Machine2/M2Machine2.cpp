/*
 * =====================================================================================
 *
 *      Project:   M2 Human–Robot Interaction Experiment Framework
 *      Module:    M2Machine / M2States
 *      Purpose:   State machine implementation for M2 robot control, trial logic,
 *                 effort computation, deterministic perturbation scheduling, and
 *                 Unity interface synchronization.
 *
 * =====================================================================================
 */

 
#include "M2Machine2.h"

// Wall-clock time helper (seconds since epoch) for logs
static inline double system_time_sec() {
    using namespace std::chrono;
    return duration_cast<duration<double>>(system_clock::now().time_since_epoch()).count();
}

// Transition guard: calibration finished -> leave CalibState
static bool endCalib(StateMachine& sm) {
    return (sm.state<M2CalibState>("CalibState"))->isCalibDone();
}

// Transition guard: Standby -> ProbMove when UI sends BGIN
static bool toProbOnBtn12(StateMachine& SM){
    auto& sm = static_cast<M2Machine&>(SM);

    if (sm.UIserver && sm.UIserver->isCmd()) {
        std::string cmd; std::vector<double> v;
        sm.UIserver->getCmd(cmd, v);

        // MODIFIED: "STRT_PROB" -> "BGIN"
        if (cmd == "BGIN") {
            sm.UIserver->clearCmd();
            sm.UIserver->sendCmd(std::string("OK"));
            spdlog::info("[TRANS] accepting BGIN -> toProb");
            return true;
        }

        else {
            sm.UIserver->clearCmd();
        }
    }
    return false;
}

// Transition guard: ProbMove finished -> Standby
static bool probMoveFinished(StateMachine& sm){
    return sm.state<M2ProbMoveState>("ProbMoveState")->isFinished();
}

M2Machine::M2Machine() {
    setRobot(std::make_unique<RobotM2>("M2_MELB"));

    // Register states
    addState("CalibState",   std::make_shared<M2CalibState>(robot()));
    addState("StandbyState", std::make_shared<M2StandbyState>(robot(),this));
    // The constructor call remains the same
    addState("ProbMoveState",std::make_shared<M2ProbMoveState>(robot(),this));

    // Wire transitions
    addTransition("CalibState", &endCalib,"StandbyState");
    // MODIFIED: Transition name is updated for clarity
    addTransition("ProbMoveState", &probMoveFinished, "StandbyState");
    addTransition("StandbyState", &toProbOnBtn12, "ProbMoveState");

    setInitState("CalibState");
}
M2Machine::~M2Machine() {
}

void M2Machine::init() {
    spdlog::debug("M2Machine::init()");
    if (robot()->initialise()) {
        // Basic machine-wide CSV logger (position/velocity/force)
        logHelper.initLogger("M2MachineLog", "logs/M2Machine.csv", LogFormat::CSV, true);
        logHelper.add(runningTime(),                 "Time (s)");
        logHelper.add(robot()->getEndEffPosition(),  "Position");
        logHelper.add(robot()->getEndEffVelocity(),  "Velocity");
        logHelper.add(robot()->getEndEffForce(),     "Force");
        logHelper.startLogger();
        // Initialise a default session id with epoch seconds if Unity hasn't set one yet
        if (sessionId == "UNSET") {
            auto now = std::chrono::system_clock::now();
            auto secs = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
            sessionId = std::to_string(secs);
        }
        // UI server used to exchange simple commands and telemetry with Unity
        UIserver = std::make_shared<FLNLHelper>(*robot(), "0.0.0.0");
    } else {
        spdlog::critical("Failed robot initialisation. Exiting...");
        std::raise(SIGTERM);
    }
}

void M2Machine::end() {
    if (running() && UIserver) 
        UIserver->closeConnection();
    StateMachine::end();
}


// UI connection management and state update loop. Called in main loop at 100Hz, but only sends state to UI at 40Hz to reduce network load.
static bool connected = false;
static auto lastCheck = std::chrono::steady_clock::now();

void M2Machine::hwStateUpdate() {
    auto now = std::chrono::steady_clock::now();

    // Check UI connection every second, and block until reconnected if lost
    if (UIserver && std::chrono::duration<double,std::milli>(now - lastCheck).count() > 1000.0) {
        connected = UIserver->isConnected();
        if (!connected) {
            spdlog::critical("UI down, waiting reconnect...");
            UIserver->reconnect();       // This will block until reconnected, and then return true
            connected = UIserver->isConnected();
            spdlog::info("UI reconnected");
        }
        lastCheck = now;
    }

    StateMachine::hwStateUpdate();

    // Send state to UI at 40Hz to reduce network load (but still have smooth updates in Unity)
    static auto lastSend = std::chrono::steady_clock::now();
    if (connected && std::chrono::duration<double,std::milli>(now - lastSend).count() >= 25.0) {
        UIserver->sendState();
        lastSend = now;
    }
}
