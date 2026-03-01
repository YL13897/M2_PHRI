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


#ifndef M2_STATES_H_DEF
#define M2_STATES_H_DEF

#include "RobotM2.h"
#include "State.h"
#include "StateMachine.h"
#include <random>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <deque>
using namespace std;


double timeval_to_sec(struct timespec *ts);

class M2Machine;


// Base state with standardized entry/during/exit and console banners
class M2TimedState : public State {
   protected:

    RobotM2 *robot;                               /*<!Pointer to state machines robot object*/

    M2TimedState(RobotM2 *M2, const char *name = NULL): State(name), robot(M2){};
   private:
    void entry(void) final {
        std::cout
        << "==================================" << std::endl
        << " STARTING  " << getName() << std::endl
        << "----------------------------------" << std::endl
        << std::endl;

        //Actual state entry
        entryCode();
    };
    void during(void) final {
        //Actual state during
        duringCode();
    };
    void exit(void) final {
        exitCode();
        std::cout
        << "----------------------------------" << std::endl
        << "EXIT "<< getName() << std::endl
        << "==================================" << std::endl
        << std::endl;
    };

   public:
    virtual void entryCode(){};
    virtual void duringCode(){};
    virtual void exitCode(){};
    
};

// Joint stop seeking + encoder calibration; exits when calibrated
class M2CalibState : public M2TimedState {

   public:
    M2CalibState(RobotM2 *M2, const char *name = "M2 Calib State"):M2TimedState(M2, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    bool isCalibDone() {return calibDone;}

   private:
     VM2 stop_reached_time;
     bool at_stop[2];
     bool calibDone=false;
};

// Transparent idle (zero commanded force) with light CSV logging
class M2StandbyState : public M2TimedState {
public:
    M2StandbyState(RobotM2* M2, M2Machine* mach, const char* name = "M2 Standby")
        : M2TimedState(M2, name), machine(mach) {}

    void entryCode() override;
    void duringCode() override;
    void exitCode() override;


private:
    M2Machine* machine = nullptr;

    // Logging helpers/fields
    std::ofstream standbyCsv_;
    int  standbyLogEveryN_ = 10;  // log every 10 iterations to reduce I/O
    int  standbyIter_ = 0;
    bool standbyRecording_ = true; // set false to disable logging

    void openStandbyCSV_();
    void closeStandbyCSV_();
    void writeStandbyCSV_(double t, double sys_t, const std::string& sid,
                          const VM2& pos, const VM2& vel,
                          const VM2& fcmd, const VM2& fsense);
};


// Probabilistic move block: TO_A -> WAIT_START (preload check) -> TRIAL
// Handles UI commands, scoring, deterministic LEFT/UP schedule, and CSV logs
class M2ProbMoveState : public M2TimedState {
public:
    M2ProbMoveState(RobotM2* M2, M2Machine* mach, const char* name="M2 Probabilistic Move");

    void entryCode() override;
    void duringCode() override;
    void exitCode() override;

    bool isFinished() const { return finishedFlag; }

    // --- Perturbation force arrays loaded from CSV ---
    std::vector<double> upPerturbForce;
    std::vector<double> upPerturbForce2;
    std::vector<double> leftPerturbForce;
    std::vector<double> leftPerturbForce2;
    // size_t perturbIndex = 0;
    bool injectingUp = false;
    bool injectingLeft = false;


    // --- WAIT_START hold-at-A configuration ---
    bool waitHoldLatched_ = false;  
    double k_hold = 1000.0; // Default 350        
    double d_hold = 35.0;            

    double k_hold_cmd = 1000.0; // Default 350
    double d_hold_cmd = 35.0;

    bool hold_log_once = false;
    // --- Perturbation CSV loading functions ---
    std::vector<double> loadColumnFromCSV(const std::string& path, int colIndex, double tStart, double tEnd);
    void loadPerturbationForces();

    bool trialIsLeft = true;
    // bool trialIsLeft = false;


    double F_const_up = 50.0;   // Set the constant perturbation force magnitude (N)
    double F_const_left = -30.0; // Make sure left force is negative

    bool softWallEnabled = false;

// public:
    // experiment config
    // VM2 A{0.45, 0.002};
    // VM2 C{0.45, 0.302};
    // VM2 A{0.45, 0.002};
    // VM2 C{0.45, 0.222};
    // VM2 A{0.32, 0.002};
    // VM2 C{0.32, 0.222};
    VM2 A{0.32, 0.050};
    VM2 C{0.32, 0.300};


    const int seed = 1111111;
    // const int seed = 1456070;
    // const int seed = 1661471;

    
    // double epsA = 0.05;
    // double epsC = 0.05;
    double epsC = 0.08;  //default 0.05
    
    double lastTrpsT_ = -1.0;
    double trpsMinInterval_ = 0.5; // 20Hz

    std::vector<VM2> trialEndPositions_;
    // bool sendPosOnlyOnTimeout_ = false;    
    
    // double k = 150;
    // double d = 6;
    double k = 300;
    double d = 15;

    double robotForceMagUp  = 17.5;
    double robotForceMagLeft= 17.5;
    
    // double internalForceDur  = 1.2;
    double trialMaxTime      = 0.5; // maximum trial time (s)
    double  trialExtendTime = 0.2; // extra time after timeout to reach C

    const double x_min = 0.15;   // left boundary (m)
    const double x_max = 0.45;   // left boundary (m)
    const double k_wall = 800.0; // wall stiffness N/m
    const double d_wall = 40.0;  // wall damping N·s/m
    const double y_max = 0.40;   // upper boundary (m)

    VM2    internalForce     = VM2::Zero();

    double userForceScale   = 5;
    double forceSaturation   = 80.0;
    double Dv               = 5.0;

    double rampUp       = 0.5;
    double rampDown     = 0.2;

    double probLeft = 0.5;
    int BlockID = 1;

    bool enablePIDToA = false;
    double KpToA = 5.0;
    double KiToA = 30.0;
    double KdToA = 1.0;
    VM2    iErrToA = VM2::Zero();
    double iToA_max = 15.0;

    // --- Meta parameters received from UI ---
    int    meta_scoreMode   = 1;
    int    meta_targetSucc  = 10;
    int    meta_maxTrials   = 10;

    // double V2_Smax = 110.0;

    // ToA related variables
    // double holdTimeA  = 0.25;
    double holdTimeA  = 1;
    double epsA_hold  = 0.10;
    double inBandSince = 0.0;
    VM2    Xi;
    double T_toA  = 2.0;
    double t0_toA = 0.0;

    M2Machine* machine = nullptr;

    // Helper methods (no changes needed)
    VM2 impedance(const VM2& X0, const VM2& X, const VM2& dX, const VM2& dXd = VM2::Zero());
    VM2 readUserForce();
    void decideInternalForceDirection();
    void resetToAPlan(const VM2& Xnow);
    void resetToAIntegrators();
    void writeCSV(double t, const VM2& pos, const VM2& vel, const VM2& fInternal, const VM2& fUser, double effort);
    void applyForce(const VM2& F);
    bool startTrialSignal();


private:
    void handleInput();
    void openCSV();
    // Deterministic schedule for LEFT/UP within a 10-trial block: -1=LEFT, +1=UP
    std::vector<int> trialSchedule_;
    std::vector<double> trialProb_; // 每个 trial 对应的概率值
    double currentTrialProb_ = 0.0;   // 当前试次概率
    int currentTrialDir_  = 0;     // 当前试次方向：-1=LEFT，+1=UP

    bool randomizeOrBlock = true;

    size_t trialIdx_ = 0;
    void buildDeterministicSchedule();
    void buildDeterministicSchedule_random();
    void dumpScheduleCSV_(const std::vector<int>& dir, const std::vector<double>& prob);
    std::vector<int> fullDir_;
    std::vector<double> fullProb_;

    // MERGED: Re-introducing enum for internal state management
    enum Phase {
        TO_A,
        WAIT_START,
        TRIAL
    };
    Phase currentPhase;

    // MERGED: Flags to simulate entryCode() for each phase
    bool initToA = true;
    bool initTrial = true;
    bool pendingStart = false;  // captured STRT; consumed only in WAIT_START
    bool betweenTrials = false; // true only in WAIT_START between trials; allows S_MD/S_MT/S_TS

    // MERGED: Variables from M2TrialState are now here
    double trialStartTime = 0.0;
    double effortIntegral = 0.0;

    double rawEffortIntegral = 0.0;

    double baselineImpulseN = 0.0;
    
    bool finishedFlag = false;

    // Scoring-related state variables
    enum ScoringMode { V1_COUNT_SUCCESS, V2_EFFORT_DISTANCE };
    ScoringMode currentMode;
    int successfulTrials = 0;
    int totalTrialsV1 = 0;
    double totalScoreV2 = 0.0;
    int totalTrialsV2 = 0;

    // ToA notification flag
    bool atA_notified_ = false;

    // --- UI command debounce ---
    // STRT debounce (seconds)
    double lastStrtTime = -1.0;
    double strtMinInterval = 1.0;
    std::mt19937 rng;
    std::ofstream csv;
    // --- Add to M2ProbMoveState (private) ---
    int txSeq_ = 0;

    // --- Preload detection (WAIT_START) ---
    struct WaitSample {
        double t;      // state running() time
        VM2    pos;    // end-eff position
        VM2    vel;    // end-eff velocity
        VM2    force;  // sensed end-eff force
    };
    std::deque<WaitSample> waitBuf_;           // rolling buffer of recent WAIT_START samples
    double preloadThresholdN_ = 10.0;           // adjustable threshold (N), default 3N
    // double preloadWindowSec_  = 0.200;         // window (s), default 200ms
    double preloadWindowSec_  = 0.200; 
    bool   preloadSatisfied_  = false;         // result for the upcoming trial
    std::ofstream preloadWinCsv_;              // raw 200ms window dump
    // std::ofstream trialTagsCsv_;               // per-trial tags (preload yes/no)
    std::ofstream preloadCsv_;                 // merged preload window log
    void openPreloadCSVs_();
    void closePreloadCSVs_();
    // void writePreloadWindow_(int trialIdxForMode, double tNow);
    // void writeTrialTag_(int trialIdxForMode, int mode, bool flag, double tNow);
    void writePreloadWindow_(int trialIdxForMode, double tNow, int mode, bool preloadFlag);
    double computeFxMin_(double tNow);

    static bool isPrintableAscii(const std::string& s) {
        for (unsigned char ch : s) {
            if (ch < 0x20 || ch > 0x7E) { 
                if (ch!='\n' && ch!='\r' && ch!='\t') return false;
            }
        }
        return true;
    }

    static std::string hexDump(const std::string& s) {
        std::ostringstream h;
        h.setf(std::ios::hex, std::ios::basefield);
        h.setf(std::ios::uppercase);
        for (unsigned char ch : s) {
            h << std::setw(2) << std::setfill('0') << (int)ch << ' ';
        }
        return h.str();
    }

    void sendUI_(const std::string& msg);

};

#endif

