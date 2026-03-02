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


#ifndef M2_STATES_H
#define M2_STATES_H

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

class M2MachineHRI;


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
    M2StandbyState(RobotM2* M2, M2MachineHRI* mach, const char* name = "M2 Standby")
        : M2TimedState(M2, name), machine(mach) {}

    void entryCode() override;
    void duringCode() override;
    void exitCode() override;


private:
    M2MachineHRI* machine = nullptr;

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
    M2ProbMoveState(RobotM2* M2, M2MachineHRI* mach, const char* name="M2 Probabilistic Move");

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
    double k_hold = 1000.0; // Default 350        
    double d_hold = 35.0;            

    double k_hold_cmd = 1000.0; // Default 350
    double d_hold_cmd = 35.0;


    double F_const_up = 50.0;   // Set the constant perturbation force magnitude (N)
    double F_const_left = -30.0; // Make sure left force is negative

    bool softWallEnabled = false;

// public:
    // experiment config
    VM2 A{0.32, 0.050};

    
    std::vector<VM2> trialEndPositions_;
    
    double k = 300;
    double d = 15;

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


    bool enablePIDToA = false;
    double KpToA = 5.0;
    double KiToA = 30.0;
    double KdToA = 1.0;
    VM2    iErrToA = VM2::Zero();
    double iToA_max = 15.0;


    // ToA related variables
    double holdTimeA  = 1;
    double epsA_hold  = 0.10;
    double inBandSince = 0.0;
    VM2    Xi;
    double T_toA  = 2.0;
    double t0_toA = 0.0;

    M2MachineHRI* machine = nullptr;

    // Helper methods (no changes needed)
    VM2 impedance(const VM2& X0, const VM2& X, const VM2& dX, const VM2& dXd = VM2::Zero());
    VM2 readUserForce();
    void decideInternalForceDirection();
    void resetToAPlan(const VM2& Xnow);
    void resetToAIntegrators();
    void writeCSV(double t, const VM2& pos, const VM2& vel, const VM2& fInternal, const VM2& fUser, double effort);
    void applyForce(const VM2& F);


private:
    void openCSV();

    // MERGED: Re-introducing enum for internal state management
    enum Phase {
        TO_A,
        WAIT_START,
        TRIAL
    };
    Phase currentPhase;

    int txSeq_ = 0;

    // MERGED: Flags to simulate entryCode() for each phase
    bool initToA = true;
    bool initTrial = true;
    bool pendingStart = false;  // captured STRT; consumed only in WAIT_START
    bool betweenTrials = false; // true only in WAIT_START between trials; allows S_MD/S_MT/S_TS

    // MERGED: Variables from M2TrialState are now here
    double trialStartTime = 0.0;
    double effortIntegral = 0.0;
    
    bool finishedFlag = false;


    // ToA notification flag
    bool atA_notified_ = false;

    // --- UI command debounce ---
    // STRT debounce (seconds)
    double lastStrtTime = -1.0;
    double strtMinInterval = 1.0;
    std::mt19937 rng;
    std::ofstream csv;

    // --- Preload detection (WAIT_START) ---
    struct WaitSample {
        double t;      // state running() time
        VM2    pos;    // end-eff position
        VM2    vel;    // end-eff velocity
        VM2    force;  // sensed end-eff force
    };


    void sendUI_(const std::string& msg);

    // -------------------------------------------------------------
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



};

#endif

