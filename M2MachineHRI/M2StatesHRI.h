/* 
    M2StatesHRI.h and M2MachineHRI.cpp:
        Core state implementations for M2 machine
        - Calibration, Standby, Probabilistic Move (TO_A / WAIT_START / TRIAL)
        - UI command handling and CSV logging
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

//  M2StandbyState: Transparent idle (zero commanded force)
class M2StandbyState : public M2TimedState {
    public:
        M2StandbyState(RobotM2* M2, M2MachineHRI* mach, const char* name = "M2 Standby")
            : M2TimedState(M2, name), machine(mach) {}

        void entryCode() override;
        void duringCode() override;
        void exitCode() override;

    private:
        M2MachineHRI* machine = nullptr;
};


// Probabilistic move block: TO_A -> WAIT_START -> TRIAL
// Handles UI commands, scoring, deterministic LEFT/UP schedule, and CSV logs
class M2ProbMoveState : public M2TimedState {
    public:
        M2ProbMoveState(RobotM2* M2, M2MachineHRI* mach, const char* name="M2 Probabilistic Move");

        void entryCode() override;
        void duringCode() override;
        void exitCode() override;

        // Check if the trial block is finished (used for transition back to Standby)
        bool isFinished() const { return finishedFlag; }
        

        
        // --- Experiment config ---
        VM2 A{0.32, 0.050};
        std::vector<VM2> trialEndPositions_;
        
        // --- Workspace limits and wall config ---
        bool softWallEnabled = false; // only enable walls after reaching A
        const double x_min = 0.15;   // left boundary (m)
        const double x_max = 0.45;   // left boundary (m)
        const double k_wall = 800.0; // wall stiffness N/m
        const double d_wall = 40.0;  // wall damping N·s/m
        const double y_max = 0.40;   // upper boundary (m)

        // --- User force config ---
        double userForceScale   = 1.0;  // scale factor for user force
        double forceSaturation   = 80.0;

        // --- ToA related variables ---
        double holdTimeA  = 1;
        double epsA_hold  = 0.10;
        double inBandSince = 0.0;
        VM2    Xi;
        double T_toA  = 2.0;
        double t0_toA = 0.0;

        M2MachineHRI* machine = nullptr;

        // --- Helper methods ---
        VM2 impedance(const VM2& X0, const VM2& X, const VM2& dX, const VM2& dXd = VM2::Zero());
        VM2 readUserForce();
        void resetToAPlan(const VM2& Xnow);
        void openCSV();
        void writeCSV(double t, const VM2& pos, const VM2& vel, const VM2& fInternal, const VM2& fUser, double effort);
        void applyForce(const VM2& F);

    private:
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

        // Sequence number for UI messages, for debugging
        int txSeq_ = 0; 

        // Flags to simulate entryCode() for each phase
        bool initToA = true;
        bool initTrial = true;
        bool pendingStart = false;  // captured STRT; consumed only in WAIT_START
        bool betweenTrials = false; // true only in WAIT_START between trials; allows S_MD/S_MT/S_TS
        
        // Notification flags for UI commands
        bool atA_notified_ = false;
        bool finishedFlag = false;
        
        // Commands part: Mode setting
        enum HRIMode { V1_HRI, V2_PHRI };
        HRIMode HRIMode_ = V2_PHRI;
        enum CtrlMode { V1_POS, V2_VEL };
        CtrlMode CtrlMode_ = V2_VEL;



        // TRIAL part: scoring and trial end detection
        double trialStartTime = 0.0;
        double effortIntegral = 0.0;
        double rawEffortIntegral = 0.0;


        // --- UI command debounce ---
        // STRT debounce (seconds)
        double lastStrtTime = -1.0; // last accepted STRT command time for debounce
        double strtMinInterval = 1.0; // minimum interval between accepted STRT commands
        
        std::ofstream csv; // CSV file stream for logging

        // --- Preload detection (WAIT_START) ---
        struct WaitSample {
            double t;      // state running() time
            VM2    pos;    // end-eff position
            VM2    vel;    // end-eff velocity
            VM2    force;  // sensed end-eff force
        };

        // Helper to send a command to the UI server with logging
        void sendUI_(const std::string& msg);

        
        // -------------------------------------------------------------
        // ------------------------ Optional ---------------------------

        std::mt19937 rng; // Random number generator for deterministic perturbation scheduling
        
        // --- Debugging helpers for UI command processing ---

        // Check if a string is printable ASCII (for logging/debugging)
        static bool isPrintableAscii(const std::string& s) {
            for (unsigned char ch : s) {
                if (ch < 0x20 || ch > 0x7E) { 
                    if (ch!='\n' && ch!='\r' && ch!='\t') return false;
                }
            }
            return true;
        }

        /* 
        The hexDump function converts each byte in a string to a two-digit uppercase hexadecimal string, useful for debugging binary communication data.
        For example, the string "Hi\n" contains 'H' (0x48), 'i' (0x69), and newline '\n' (0x0A), and its hexadecimal representation is "48 69 0A".
        This is especially useful for debugging communication protocols where non-printable characters may be present.
        */
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

