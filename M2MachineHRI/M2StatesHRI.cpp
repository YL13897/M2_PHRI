/* 
    M2StatesHRI.h and M2MachineHRI.cpp:
        Core state implementations for M2 machine
        - Calibration, Standby, Probabilistic Move (TO_A / WAIT_START / TRIAL)
        - UI command handling and CSV logging
*/

#include <chrono>
#include <spdlog/spdlog.h>
#include "M2StatesHRI.h"
#include "M2MachineHRI.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <sstream>
#include <random>


// ----------------------------------------------------------------------------
// Local wall-clock helper for CSV timestamps
static inline double system_time_sec() {
    using namespace std::chrono;
    return duration_cast<duration<double>>(system_clock::now().time_since_epoch()).count();
}

// Minimum-jerk trajectory helper (position/velocity/optional acceleration)
static inline double MinJerk(const VM2& X0, const VM2& Xf, double T, double t,
                             VM2& Xd, VM2& dXd, VM2* ddXd=nullptr){
    if (T <= 0) { 
        Xd = Xf; 
        dXd.setZero(); 
        if (ddXd) 
            ddXd->setZero(); 
        return 1.0; 
    }
    if (t < 0) 
        t = 0; 
    else if (t > T) 
        t = T;

    const double s = t / T;
    const double s2 = s*s, s3 = s2*s, s4 = s3*s, s5 = s4*s;
    const VM2 dX = (Xf - X0);

    // position
    Xd  = X0 + dX * (10*s3 - 15*s4 + 6*s5);
    // velocity
    dXd = dX * ((30*s2 - 60*s3 + 30*s4) / T);
    // acceleration (optional)
    if (ddXd) *ddXd = dX * ((60*s - 180*s2 + 120*s3) / (T*T));

    return s;
}

// Simple clamp helper
template <typename T>
static inline T clamp_compat(T v, T lo, T hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}


// ----------------------------------------------------------------------------
// --- M2CalibState implementation ---

// Begin calibration: enter torque mode and start stop-seek routine
void M2CalibState::entryCode() {
    calibDone=false;
    for(unsigned int i=0; i<2; i++) {
        stop_reached_time[i] = .0;
        at_stop[i] = false;
    }
    robot->decalibrate();
    robot->initTorqueControl();
    robot->printJointStatus();
    std::cout << "Calibrating (keep clear)..." << std::flush;
}

// Drive joints toward stops; apply calibration once conditions met
void M2CalibState::duringCode() {
    VM2 tau(0, 0);
    VM2 vel=robot->getVelocity();
    double b = 3;
    for(unsigned int i=0; i<vel.size(); i++) {
        tau(i) = -std::min(std::max(20 - b * vel(i), .0), 20.);
        if(stop_reached_time(i)>1) {
            at_stop[i]=true;
        }
        if(std::abs(vel(i))<0.005) {
            stop_reached_time(i) += dt();
        }
    }
    if(robot->isCalibrated()) {
        robot->setEndEffForceWithCompensation(VM2::Zero(), false);
        calibDone=true;
    }
    else {
        if(at_stop[0] && at_stop[1]) {
            robot->applyCalibration();
            std::cout << "OK." << std::endl;
        }
        else {
            robot->setJointTorque(tau);
            if(iterations()%100==1) {
                std::cout << "." << std::flush;
            }
        }
    }
}

// Leave with zero force command and compensation active
void M2CalibState::exitCode() {
    robot->setEndEffForceWithCompensation(VM2::Zero());
}


// ----------------------------------------------------------------------------
// --- M2StandbyState implementation ---

// Enter standby: torque control mode with zero commanded force
void M2StandbyState::entryCode() {
    robot->initTorqueControl();
}

// Idle loop: zero commanded force, snapshot kinematics, and periodic status print
void M2StandbyState::duringCode() {
    // Commanded force in Standby is zero
    VM2 F_cmd = VM2::Zero();

    // Apply the commanded force
    robot->setEndEffForceWithCompensation(F_cmd, true);

    // Snapshot kinematics
    VM2 X  = robot->getEndEffPosition();
    VM2 dX = robot->getEndEffVelocity();
    VM2 Fs = robot->getEndEffForce();

    // Periodic status print
    if (iterations()%500==1) robot->printStatus();
}

// Exit standby: zero force and close CSV
void M2StandbyState::exitCode() {
    robot->setEndEffForceWithCompensation(VM2::Zero());
}


// ----------------------------------------------------------------------------
// --- M2ProbMoveState implementation ---

// Construct probabilistic move state; machine is used for UI/session utilities
M2ProbMoveState::M2ProbMoveState(RobotM2* M2, M2MachineHRI* mach, const char* name)
    : M2TimedState(M2, name), machine(mach) {}

// Initialize ProbMove: torque mode, reset flags, open CSVs, load perturbations
void M2ProbMoveState::entryCode() {

    robot->initTorqueControl();
    robot->setEndEffForceWithCompensation(VM2::Zero(), false);
    trialEndPositions_.clear();
    currentPhase = TO_A;
    finishedFlag = false;
    initToA = true;
    initTrial = true;
    pendingStart  = false;
    softWallEnabled = false;  
    atA_notified_ = false;
    openCSV();

}

// Main loop: drain UI, then run phase switch (TO_A / WAIT_START / TRIAL / ...)
void M2ProbMoveState::duringCode() {

    // === GLOBAL COMMAND DRAIN === (STRT/)
    {
        int guard = 1024; // prevent infinite loop, a single `duringCode()` loop can read a maximum of 1024 commands.
        while (guard-- > 0 && machine && machine->UIserver && machine->UIserver->isCmd()) {
            std::string c; std::vector<double> a;
            machine->UIserver->getCmd(c, a);
            
            // Simple command parsing with trimming and case normalization
            auto trim = [](std::string s){
                auto notspace = [](int ch){ return !std::isspace(ch); };
                s.erase(s.begin(), std::find_if(s.begin(), s.end(), notspace));
                s.erase(std::find_if(s.rbegin(), s.rend(), notspace).base(), s.end());
                return s;
            };
            std::string cu = trim(c);
            std::transform(cu.begin(), cu.end(), cu.begin(), [](unsigned char ch){ return std::toupper(ch); });

            // Handle STRT command with debounce and pending flag
            if (cu.rfind("STRT", 0) == 0) {
                double now = running();
                if (pendingStart) {
                    spdlog::warn("STRT ignored: already pending (phase={}, Δt={:.3f}s)", (int)currentPhase, (now - lastStrtTime));
                    machine->UIserver->clearCmd();
                    continue; 
                }
                if (lastStrtTime >= 0.0 && (now - lastStrtTime) < strtMinInterval) {
                    spdlog::warn("STRT ignored due to debounce (Δt={:.3f}s < {:.3f}s)", (now - lastStrtTime), strtMinInterval);
                    machine->UIserver->clearCmd();
                    continue; 
                }
                pendingStart = true;
                lastStrtTime = now;
                machine->UIserver->clearCmd();
                spdlog::info("GLOBAL: STRT captured (pendingStart=true, t={:.3f})", now);
                break; 
            } 
            
            // Handle mode setting commands (S_MD, S_CT) only if betweenTrials is true
            if (cu.rfind("S_MD",0)==0 || cu.rfind("S_CT",0)==0) {
                if (betweenTrials) {
                    if (cu.rfind("S_MD",0)==0 && !a.empty()) {
                        HRI_Mode = (int)std::round(a[0]);
                        HRIMode_ = (HRI_Mode == 2) ? V2_PHRI : V1_HRI;
                        spdlog::info("BETWEEN-TRIALS: S_MD -> mode={}", HRI_Mode);
                        if (machine && machine->UIserver) machine->UIserver->sendCmd("OK");

                    } else if (cu.rfind("S_CT",0)==0 && !a.empty()) {
                        Ctrl_Mode = (int)std::round(a[0]);
                        CtrlMode_ = (Ctrl_Mode == 2) ? V2_VEL : V1_POS;
                        spdlog::info("BETWEEN-TRIALS: S_CT -> mode={}", Ctrl_Mode);
                        if (machine && machine->UIserver) machine->UIserver->sendCmd("OK");

                    } else {
                        if (machine && machine->UIserver) machine->UIserver->sendCmd("ERR ARG");
                    }
                } else {
                    if (machine && machine->UIserver) machine->UIserver->sendCmd("BUSY");
                    spdlog::warn("PARAM LOCKED: '{}' rejected (phase={}, betweenTrials=0)", cu, (int)currentPhase);
                }
                machine->UIserver->clearCmd();
                continue;
            }


            // If unknown command
            spdlog::warn("GLOBAL: unknown cmd='{}' (trim='{}') @phase={}", c, cu, (int)currentPhase);
            machine->UIserver->clearCmd();
        }
    }


    // === END GLOBAL COMMAND DRAIN ===
    // Phase controller: TO_A -> WAIT_START -> TRIAL
    switch (currentPhase) {
        
        // --- TO_A: move/hold near A ---
        case TO_A: {
            // This block simulates M2ToAState (move/hold near A)
            if (initToA) {
                // Simulate entryCode() for TO_A
                resetToAPlan(robot->getEndEffPosition());
                initToA = false;
            }

            VM2 X = robot->getEndEffPosition();
            VM2 dX = robot->getEndEffVelocity();
            VM2 F_cmd = VM2::Zero();

            VM2 Xd, dXd;
            MinJerk(Xi, A, T_toA, running() - t0_toA, Xd, dXd);

            double k_pos = 4.0;
            F_cmd = impedance(Xd, X, dX, dXd) + k_pos * (Xd - X);
            applyForce(F_cmd);

            // Transition condition check
            double distA = (A - X).norm();
            bool atA_hold = false;
            if (distA < epsA_hold) {
                if (inBandSince == 0.0) {
                    inBandSince = running();}
                else if ((running() - inBandSince) >= holdTimeA) {
                    atA_hold = true;
                    if (machine && machine->UIserver && !atA_notified_) {         
                        machine->UIserver->sendCmd("AT_A"); 
                        atA_notified_ = true;
                        spdlog::info("Checked, atA_hold!");
                    }     
                }
            } else {
                inBandSince = 0.0;
                atA_notified_ = false;
            }

            if (atA_hold) {
                softWallEnabled = true;
                betweenTrials = true;
                currentPhase = WAIT_START;
                spdlog::info("TO_A -> WAIT_START (betweenTrials=1)");
            }
            break;
        }

        // --- WAIT_START: transition to TRIAL ---
        case WAIT_START: {
            // This block simulates WAIT_START: 
            // {
            //     WaitSample s;
            //     s.t     = running();
            //     s.pos   = robot->getEndEffPosition();
            //     s.vel   = robot->getEndEffVelocity();
            //     s.force = robot->getEndEffForce();
            // }

            VM2 X = robot->getEndEffPosition();
            double distToA = (A - X).norm();
            bool atA_hold = false;           
            if (distToA < epsA_hold) {
                if (inBandSince == 0.0) inBandSince = running();
                else if ((running() - inBandSince) >= holdTimeA) {
                    atA_hold = true;
                    if (machine && machine->UIserver && !atA_notified_) {
                        machine->UIserver->sendCmd("AT_A");  // Send p[0] = currentTrialProb_
                        atA_notified_ = true;
                        spdlog::info("WAIT_START: Checked, atA_hold!");
                    }
                }
            } else {
                inBandSince = 0.0;
                atA_notified_ = false;
            }

            if (pendingStart && atA_hold) {
                // On STRT: evaluate last preload window and log

                const double tNow = running();

                pendingStart  = false;
                betweenTrials = false;
                initTrial     = true;
                currentPhase  = TRIAL;

                if (machine && machine->UIserver) machine->UIserver->sendCmd("OK");
                spdlog::info("WAIT_START: pendingStart consumed -> TRIAL (atA_hold=1)");
                break; 

            } else if (pendingStart && !atA_hold) {
                if (iterations() % 1000 == 1) {
                    spdlog::info("WAIT_START: STRT pending but not at A (dist={:.3f} <? {:.3f})", distToA, epsA_hold);
                }
            }

        }
        
        
        case TRIAL: {

            if (initTrial) {
                // Simulate entryCode() for TRIAL
                trialStartTime = running();
                effortIntegral = 0.0;
                rawEffortIntegral = 0.0;

                if (machine && machine->UIserver) {
                    // std::ostringstream oss;
                    // oss.setf(std::ios::fixed);
                    // oss.precision(3);

                    // oss << "TRIAL_BEGIN t=" << trialStartTime
                    //     << " HRI_Mode=" << (HRIMode_ == V2_PHRI ? 2 : 1)
                    //     << " Ctrl_Mode=" << (CtrlMode_ == V2_VEL ? 2 : 1)

                    // std::string outBegin = oss.str();
                    // sendUI_(outBegin);

                    {
                        std::vector<double> p; 
                        p.push_back(trialStartTime);                      // t
                        p.push_back((HRIMode_ == V2_PHRI) ? 2 : 1);       // HRI_Mode
                        p.push_back((CtrlMode_ == V2_VEL) ? 2 : 1);       // Ctrl_Mode
                        machine->UIserver->sendCmd("TRBG", p);  // TRial BeGin with params
                        spdlog::info("Log: TRBG");
                    }
                }
                initTrial = false;
            }

            VM2 X = robot->getEndEffPosition();
            VM2 dX = robot->getEndEffVelocity();
            double tTrial = running() - trialStartTime;


            






















        }

    }
}

// Cleanup on ProbMove exit: zero forces, close CSVs, send session summary
void M2ProbMoveState::exitCode() {
    robot->setEndEffForceWithCompensation(VM2::Zero());
    if (csv.is_open()) csv.close();

    // Send session summary message to UI
    if (machine && machine->UIserver) {
        std::ostringstream oss; // session summary can be expanded with more metadata as needed
        oss.setf(std::ios::fixed); // fixed-point for consistent formatting
        oss.precision(3);
        std::string out = oss.str();
        sendUI_(out); // Placeholder for session summary command; can be expanded with actual summary data
        {
            machine->UIserver->sendCmd("SESS");
            spdlog::info("Log:SESS");
        }
    }
}


// -----------------------------------------------------------------------------
// --- M2ProbMoveState helper methods ---

// Send a plain-text/structured line back to UI
void M2ProbMoveState::sendUI_(const std::string& msg) {
    const int seq = ++this->txSeq_; // increment sequence for debugging
    const size_t L = msg.size();
    if (machine && machine->UIserver) {
        machine->UIserver->sendCmd(msg);
    } 
}

// Compute impedance control force based on current state and optional desired acceleration
VM2 M2ProbMoveState::impedance(const VM2& X0, const VM2& X, const VM2& dX, const VM2& dXd) {
    Eigen::Matrix2d K = Eigen::Matrix2d::Identity() * k;
    Eigen::Matrix2d D = Eigen::Matrix2d::Identity() * d;
    // return K * (X0 - X) + D * (dXd - dX);
    return K * (X0 - X) - D * dX;
}

// Read user force from joystick axes, scaled by userForceScale
VM2 M2ProbMoveState::readUserForce() {
    VM2 f = VM2::Zero();
    if (robot->joystick) {
        const double ax0 = robot->joystick->getAxis(0);
        const double ax1 = robot->joystick->getAxis(1);
        f(0) = userForceScale * ax0;
        f(1) = userForceScale * ax1;
        spdlog::debug("[EFFORT] readUserForce axes=({:.3f},{:.3f}) scale={} -> F_user=({:.3f},{:.3f})",
                      ax0, ax1, userForceScale, f(0), f(1));
    } else {
        spdlog::warn("[EFFORT] readUserForce: joystick is null; returning (0,0)");
    }
    return f;
}

// Reset TO_A trajectory plan with current position as new start, used when entering TO_A
void M2ProbMoveState::resetToAPlan(const VM2& Xnow) {
    Xi = Xnow;
    t0_toA = running();
    inBandSince = 0.0;
}

// Apply force command with optional soft wall constraints
void M2ProbMoveState::applyForce(const VM2& F) {

    // const double x_min = 0.15;   // left boundary (m)
    // const double x_max = 0.45;   // left boundary (m)
    // const double k_wall = 800.0; // wall stiffness N/m
    // const double d_wall = 40.0;  // wall damping N·s/m
    // const double y_max = 0.40;   // upper boundary (m)

    VM2 F_cmd = F;

    if (softWallEnabled) {
        //  read current position and velocity
        VM2 X  = robot->getEndEffPosition();
        VM2 dX = robot->getEndEffVelocity();


        // left wall
        if (X(0) < x_min) {
            double pen = x_min - X(0);
            double F_wall = k_wall * pen - d_wall * dX(0);
            if (F_wall > 0.0) F_cmd(0) += F_wall;
        }
        // right wall
        if (X(0) > x_max) {
            double pen = X(0) - x_max;
            double F_wall = k_wall * pen + d_wall * dX(0);
            if (F_wall > 0.0) F_cmd(0) -= F_wall;
        }

        // upper wall
        if (X(1) > y_max) {
        double penY = X(1) - y_max;   // penetration depth (upper wall)
            double F_wall_y = k_wall * penY + d_wall * dX(1);  
            // Only apply when pushing into the wall
            if (F_wall_y > 0.0) F_cmd(1) -= F_wall_y;   // push downward
        }
    }

    // Clamp forces
    for (int i=0; i<2; ++i)
        F_cmd(i) = clamp_compat(F_cmd(i), -forceSaturation, forceSaturation);

    robot->setEndEffForceWithCompensation(F_cmd, true);
}


// -----------------------------------------------------------------------------
// --- CSV logging helpers for ProbMoveState ---

// Open logs/M2ProbMove_<session>.csv with header
void M2ProbMoveState::openCSV() {

    const std::string sid = (machine && !machine->sessionId.empty()) ? machine->sessionId : std::string("UNSET");
    const std::string fname = std::string("logs/M2ProbMove_") + sid + ".csv";
    csv.open(fname, std::ios::out | std::ios::app);
    if (!csv.is_open()) {
        spdlog::error("Failed to open ProbMove CSV: {}", fname);
        return;
    }
    if (csv.tellp() == 0) {
        csv << "time,sys_time,session_id,pos_x,pos_y,vel_x,vel_y,internal_fx,internal_fy,user_fx,user_fy,effort\n";
    }
}

// Write a row to the ProbMove CSV with current trial data and metadata
void M2ProbMoveState::writeCSV(double t, const VM2& pos, const VM2& vel,
    const VM2& fInternal, const VM2& fUser, double effort) {
    if (!csv.is_open()) return;
    const double sys_t = system_time_sec();
    const std::string sid = (machine ? machine->sessionId : std::string("UNSET"));
    csv << std::fixed << std::setprecision(6)
        << t << "," << sys_t << "," << sid << ","
        << pos(0) << "," << pos(1) << ","
        << vel(0) << "," << vel(1) << ","
        << fInternal(0) << "," << fInternal(1) << ","
        << fUser(0) << "," << fUser(1) << ","
        << effort << "\n";
}