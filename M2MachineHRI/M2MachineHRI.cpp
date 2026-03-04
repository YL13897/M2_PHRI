/*
    M2MachineHRI.cpp: 
    Top-level state machine implementation for M2 robot control, trial logic, 
    effort computation, deterministic perturbation scheduling, 
    and Unity interface synchronization.
*/

 #include "M2MachineHRI.h"

// ----------------------------------------------------------------------------
// --- Transition guards implementations ---

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
static bool toProbOnBtn(StateMachine& SM){
    auto& sm = static_cast<M2MachineHRI&>(SM);

    if (sm.UIserver && sm.UIserver->isCmd()) {
        std::string cmd; std::vector<double> v;
        sm.UIserver->getCmd(cmd, v);

        // If "BGIN" command received, clear it and transition to ProbMove
        if (cmd == "BGIN") {
            sm.UIserver->clearCmd();
            sm.UIserver->sendCmd(std::string("OK"));
            spdlog::info("[TRANS] accepting BGIN -> toProb");
            return true;
        }
        // If unknown command received, clear it and log a warning
        else {
            spdlog::warn("Unexpected cmd='{}' received. Ignoring.", cmd);
            sm.UIserver->clearCmd();
        }
    }
    return false;
}

// Transition guard: ProbMove finished -> Standby
static bool probMoveFinished(StateMachine& sm){
    return sm.state<M2ProbMoveState>("ProbMoveState")->isFinished();
}

// -----------------------------------------------------------------------------
// --- State method implementations ---
// Core state implementations for M2 machine: Calibration, Standby, Probabilistic Move (TO_A / WAIT_START / TRIAL)
M2MachineHRI::M2MachineHRI() {

    // Create and own the robot instance
    setRobot(std::make_unique<RobotM2>("M2_MELB")); 
    // Register states
    addState("CalibState",   std::make_shared<M2CalibState>(robot()));
    addState("StandbyState", std::make_shared<M2StandbyState>(robot(),this));
    addState("ProbMoveState",std::make_shared<M2ProbMoveState>(robot(),this));

    // Wire transitions
    addTransition("CalibState", &endCalib,"StandbyState");
    addTransition("StandbyState", &toProbOnBtn, "ProbMoveState");
    addTransition("ProbMoveState", &probMoveFinished, "StandbyState");

    // Set initial state to calibration
    setInitState("CalibState"); 
}

// Destructor
M2MachineHRI::~M2MachineHRI() {
}

// Initialize robot, logging, and UI server
void M2MachineHRI::init() {
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

// Tear down UI and state machine
void M2MachineHRI::end() {
    if (running() && UIserver) 
        UIserver->closeConnection();
    StateMachine::end();
}


// -----------------------------------------------------------------------------
// --- M2MachineHRI main update loop ---

// UI connection management and state update loop. Called in main loop at 100Hz, but only sends state to UI at 40Hz to reduce network load.
// 100Hz: default setting, see: CANOpenRobotController/src/core/application.cpp -> app_programControlLoop()
static bool connected = false;
static auto lastCheck = std::chrono::steady_clock::now();
// This is the main update loop for the state machine, called at 100Hz. It handles UI command draining, phase control, and state-specific logic.
void M2MachineHRI::hwStateUpdate() {
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
    // Run the current state's duringCode() and handle transitions
    StateMachine::hwStateUpdate(); 

    // Send state to UI at 40Hz to reduce network load (but still have smooth updates in Unity)
    static auto lastSend = std::chrono::steady_clock::now();
    if (connected && std::chrono::duration<double,std::milli>(now - lastSend).count() >= 25.0) { // 25ms = 40Hz
        UIserver->sendState();
        lastSend = now;
    }
}
