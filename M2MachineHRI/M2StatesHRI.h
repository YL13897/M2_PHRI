/* 
    M2StatesHRI.h:
        Core state implementations for M2 machine
        - Calibration, Standby, Probabilistic Move (TO_A / WAIT_START / TRIAL)
        - UI command handling and CSV logging
*/

#ifndef M2_STATES_H
#define M2_STATES_H

#include "RobotM2.h"
#include "State.h"
#include "StateMachine.h"
#include <fstream>
using namespace std;

class M2MachineHRI;

// Base state with standardized entry/during/exit and console banners
class M2TimedState : public State {
    protected:
        RobotM2 *robot;  /*<!Pointer to state machines robot object*/
        // Constructor takes robot pointer and optional name
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

// M2CalibState: Joint stop seeking + encoder calibration; exits when calibrated
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

// M2StandbyState: Limp idle, then pre-positions to A safely.
class M2StandbyState : public M2TimedState {
    public:
        M2StandbyState(RobotM2* M2, M2MachineHRI* mach, const char* name = "M2 Standby")
            : M2TimedState(M2, name), machine(mach) {}

        void entryCode() override;
        void duringCode() override;
        void exitCode() override;
        bool setA(double x, double y);

    private:
        M2MachineHRI* machine = nullptr;
        VM2 A{0.32, 0.12};
        VM2 Xi;
        bool isMoving = false;
        bool holdActive = false;
        double tMoveStart = 0.0;
        double T_move = 4.0;
        bool safetyTripped = false;
};


// Probabilistic move block: TO_A -> WAIT_START -> TRIAL
// Handles UI commands, X-axis trial control, and CSV logs.
class M2ProbMoveState : public M2TimedState {
    public:
        M2ProbMoveState(RobotM2* M2, M2MachineHRI* mach, const char* name="M2 Probabilistic Move");

        void entryCode() override;
        void duringCode() override;
        void exitCode() override;

        bool isFinished() const { return finishedFlag; }
        bool setA(double x, double y);
        
        // --- Experiment config ---
        VM2 A{0.32, 0.12};
        bool safetyTripped = false;
        
        // --- Workspace guard ---
        bool workspaceGuardEnabled = false;

        // --- Force config ---
        double forceSaturation   = 80.0;
        bool trialCsvEnabled_ = true;

        // --- Trial admittance control ---
        double admM = 0.8;
        double admB = 5.0;
        double admVelLimit = 0.6;
        double admLockK = 2.0;
        double admLockD = 0.2;

        // --- ToA related variables ---
        bool atA_hold = false;
        double holdTimeA  = 0.5;
        double inBandSince = 0.0;
        VM2    Xi;
        double T_toA  = 4.0;
        double t0_toA = 0.0;

        M2MachineHRI* machine = nullptr;

        // --- Helper methods ---
        VM2 impedance(const VM2& X0, const VM2& X, const VM2& dX, const VM2& dXd = VM2::Zero());
        void resetToAPlan(const VM2& Xnow);
        void openCSV();
        void writeCSV(double tTrial, const VM2& pos, const VM2& vel, const VM2& interactionForce, const VM2& endEffForce, const VM2& fInternal, double effort);
        void applyForce(const VM2& F);

    private:
        void startAdmittance();
        void stopAdmittance();

        // Enum for internal state management
        enum Phase {
            TO_A,
            WAIT_START,
            TRIAL
        };
        // Current phase of the trial block
        Phase currentPhase;

        // Impedance control gains: Used in VM2 M2ProbMoveState::impedance
        double k = 300;
        double d = 15;

        // Flags to simulate entryCode() for each phase
        bool initToA = true;
        bool initTrial = true;
        bool holdActive_ = false;
        bool admittanceActive_ = false;
        bool pendingStart = false;  // captured TRBG; consumed only in WAIT_START
        bool rwstAckPending_ = false; // defer RWOK until TO_A has reached A and entered WAIT_START
        
        // Session finish flag for top-level transition
        bool finishedFlag = false;
        
        // Commands part: Mode setting
        enum HRIMode { V1_HRI, V2_PHRI };
        HRIMode HRIMode_ = V2_PHRI;

        // Disturbance state from Unity (DSTR -1/0/+1). In pHRI we use native M2 disturbance force.
        bool disturbanceActive_ = false;
        double disturbanceDirection_ = -1.0;
        double disturbanceForceMagnitude_ = 18.0;
        // Safety fallback: auto-clear disturbance if DSTR=0 is missed.
        double disturbanceAutoOffSec_ = 0.40; // based on Unity setup: T=L_zone/v_forward = 2 x 10 / 50 = 0.4 s
        double disturbanceExpireAt_ = -1.0;
        // TRIAL part: scoring and trial end detection
        double trialStartTime = 0.0;
        double trialDurationSec = 3600.0;
        int trialIndex_ = 0;

        // --- UI command debounce ---
        // TRBG debounce (seconds)
        double lastStartTime = -1.0; // last accepted TRBG time for debounce
        double startMinInterval = 0.5; // minimum interval between accepted TRBG commands
        
        std::ofstream csv; // CSV file stream for logging
};

#endif
