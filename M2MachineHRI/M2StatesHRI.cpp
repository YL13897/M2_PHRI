// Core state implementations for M2 machine
// - Calibration, Standby, Probabilistic Move (TO_A / WAIT_START / TRIAL)
// - UI command handling and CSV logging
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

double timeval_to_sec(struct timespec *ts)
{
    return (double)(ts->tv_sec + ts->tv_nsec / 1000000000.0);
}

template <typename T>
static inline T clamp_compat(T v, T lo, T hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}


// ----------------------------------------------------------------------------
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
// Enter standby: torque control + open CSV
void M2StandbyState::entryCode() {
    robot->initTorqueControl();
    openStandbyCSV_();
    standbyIter_ = 0;
}

// Idle loop: apply zero force (with compensation), snapshot data, and log sparsely
void M2StandbyState::duringCode() {
    // Commanded force in Standby is zero (pure transparent/compensated mode)
    VM2 F_cmd = VM2::Zero();

    // Apply the commanded force
    robot->setEndEffForceWithCompensation(F_cmd, true);

    // Snapshot kinematics
    VM2 X  = robot->getEndEffPosition();
    VM2 dX = robot->getEndEffVelocity();
    VM2 Fs = robot->getEndEffForce();

    // Periodic status print
    if (iterations()%500==1) 
        robot->printStatus();

    // Lightweight logging every N iterations
    if (standbyRecording_ && (++standbyIter_ % standbyLogEveryN_ == 0)) {
        const double sys_t = system_time_sec();
        const std::string sid = (machine ? machine->sessionId : std::string("UNSET"));
        writeStandbyCSV_(running(), sys_t, sid, X, dX, F_cmd, Fs);
    }
}
// Exit standby: zero force and close CSV
void M2StandbyState::exitCode() {
    robot->setEndEffForceWithCompensation(VM2::Zero());
    // closeStandbyCSV_();
}

// Open logs/Standby_<session>.csv (append, create header on first open)
void M2StandbyState::openStandbyCSV_() {
    // Append mode to keep a continuous session log
    // standbyCsv_.open("logs/StandbyLog.csv", std::ios::out | std::ios::app);
    const std::string sid = (machine && !machine->sessionId.empty()) ? machine->sessionId : std::string("UNSET");

    const std::string fname = std::string("logs/Standby_") + sid + ".csv";

    standbyCsv_.open(fname, std::ios::out | std::ios::app);
    if (!standbyCsv_.is_open()) {
        spdlog::error("Failed to open Standby CSV: {}", fname);
        return;
    }
    if (standbyCsv_.tellp() == 0) {
        standbyCsv_ << "time,sys_time,session_id,pos_x,pos_y,vel_x,vel_y,fcmd_x,fcmd_y,fs_x,fs_y\n";
    }
}

// Close standby CSV if open
void M2StandbyState::closeStandbyCSV_() {
    if (standbyCsv_.is_open()) standbyCsv_.close();
}

// Append one row to standby CSV
void M2StandbyState::writeStandbyCSV_(double t, double sys_t, const std::string& sid,
                                      const VM2& pos, const VM2& vel, const VM2& fcmd, const VM2& fsense) {
    if (!standbyCsv_.is_open()) return;
    standbyCsv_ << std::fixed << std::setprecision(6)
                << t << "," << sys_t << "," << sid << ","
                << pos(0) << "," << pos(1) << ","
                << vel(0) << "," << vel(1) << ","
                << fcmd(0) << "," << fcmd(1) << ","
                << fsense(0) << "," << fsense(1) << "\n";
}




// ----------------------------------------------------------------------------

// Send a plain-text/structured line back to UI
void M2ProbMoveState::sendUI_(const std::string& msg) {
    const int seq = ++this->txSeq_;
    const size_t L = msg.size();

    if (machine && machine->UIserver) {
        machine->UIserver->sendCmd(msg);
    } 
}

// Construct probabilistic move state; machine is used for UI/session utilities
M2ProbMoveState::M2ProbMoveState(RobotM2* M2, M2MachineHRI* mach, const char* name)
    : M2TimedState(M2, name), machine(mach) {}

// Initialize ProbMove: torque mode, reset flags, open CSVs, load perturbations
void M2ProbMoveState::entryCode() {

    robot->initTorqueControl();
    robot->setEndEffForceWithCompensation(VM2::Zero(), false);
    // rng.seed(std::random_device{}());
    
    currentPhase = TO_A;
    finishedFlag = false;
    initToA = true;
    initTrial = true;
    pendingStart  = false;
    softWallEnabled = false;  
    atA_notified_ = false;
}

// Main loop: drain UI, then run phase switch (TO_A / WAIT_START / TRIAL / ...)
void M2ProbMoveState::duringCode() {

    // === GLOBAL COMMAND DRAIN === (RSTA/HALT/STRT/param set/etc.)
    
    {
        int guard = 256; // prevent infinite loop, a single `duringCode()` loop can read a maximum of 256 commands.
        while (guard-- > 0 && machine && machine->UIserver && machine->UIserver->isCmd()) {
            std::string c; std::vector<double> a;
            machine->UIserver->getCmd(c, a);
            

            auto trim = [](std::string s){
                auto notspace = [](int ch){ return !std::isspace(ch); };
                s.erase(s.begin(), std::find_if(s.begin(), s.end(), notspace));
                s.erase(std::find_if(s.rbegin(), s.rend(), notspace).base(), s.end());
                return s;
            };
            std::string cu = trim(c);
            std::transform(cu.begin(), cu.end(), cu.begin(), [](unsigned char ch){ return std::toupper(ch); });

            
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
                spdlog::info("GLOBAL: STRT captured (pendingStart=1, t={:.3f})", now);
                break; 
            } 

            // If unknown command
            spdlog::warn("GLOBAL: unknown cmd='{}' (trim='{}') @phase={}", c, cu, (int)currentPhase);
            machine->UIserver->clearCmd();
        }
    }


    // === END GLOBAL COMMAND DRAIN ===
    // Phase controller: TO_A -> WAIT_START -> TRIAL
    switch (currentPhase) {

        // In M2States.cpp, inside M2ProbMoveState::duringCode()
        case WAIT_START: {
            // Keep recent samples for preload window analysis until STRT is consumed
            // Sample and maintain rolling buffer
            {
                WaitSample s;
                s.t     = running();
                s.pos   = robot->getEndEffPosition();
                s.vel   = robot->getEndEffVelocity();
                s.force = robot->getEndEffForce();
            }

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
                currentPhase  = TRIAL;
                initTrial     = true;
                
                if (machine && machine->UIserver) machine->UIserver->sendCmd("OK");
                spdlog::info("WAIT_START: pendingStart consumed -> TRIAL (atA_hold=1)");
                break; 
            } else if (pendingStart && !atA_hold) {
                if (iterations() % 1000 == 1) {
                    spdlog::info("WAIT_START: STRT pending but not at A (dist={:.3f} <? {:.3f})", distToA, epsA_hold);
                }
            }

        }

        case TO_A: {
            // This block simulates M2ToAState (move/hold near A)
            if (initToA) {
                // Simulate entryCode() for TO_A
                resetToAPlan(robot->getEndEffPosition());
                resetToAIntegrators();
                initToA = false;
            }

            VM2 X = robot->getEndEffPosition();
            VM2 dX = robot->getEndEffVelocity();
            VM2 F_cmd = VM2::Zero();

            VM2 Xd, dXd;
            MinJerk(Xi, A, T_toA, running() - t0_toA, Xd, dXd);

            double k_pos = 4.0;
            F_cmd = impedance(Xd, X, dX, dXd) + k_pos * (Xd - X);

            if (enablePIDToA) {
                VM2 e = (Xd - X);
                VM2 de = (dXd - dX);
                iErrToA += e * dt();
                for (int i = 0; i < 2; i++) {
                    double lim = (iToA_max > 1e-9 ? (iToA_max / std::max(1e-9, KiToA)) : 0.0);
                    iErrToA(i) = clamp_compat(iErrToA(i), -lim, +lim);
                }
                VM2 F_pid = KpToA * e + KiToA * iErrToA + KdToA * de;
                F_cmd += F_pid;
            }

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
                        spdlog::info("Checked, atA_hold! ");
                    }     
                }
            } else {
                inBandSince = 0.0;
                atA_notified_ = false;
            }

            if (atA_hold) {
                softWallEnabled = true;
                currentPhase = WAIT_START;
                betweenTrials = true;       
                spdlog::info("TO_A -> WAIT_START (betweenTrials=1)");
            }
            break;
        }

        case TRIAL: {

        }

    }
}

// Cleanup on ProbMove exit: zero forces, close CSVs, send session summary
void M2ProbMoveState::exitCode() {
    robot->setEndEffForceWithCompensation(VM2::Zero());
    if (csv.is_open()) csv.close();

    if (machine && machine->UIserver) {
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss.precision(3);
        std::string out = oss.str();
        sendUI_(out);
        {
            machine->UIserver->sendCmd("SESS");
            spdlog::info("Log:SESS");
        }
    }
}

// ... impedance, readUserForce, etc. methods remain the same ...
VM2 M2ProbMoveState::impedance(const VM2& X0, const VM2& X, const VM2& dX, const VM2& dXd) {
    Eigen::Matrix2d K = Eigen::Matrix2d::Identity() * k;
    Eigen::Matrix2d D = Eigen::Matrix2d::Identity() * d;
    // return K * (X0 - X) + D * (dXd - dX);
    return K * (X0 - X) - D * dX;
}

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


void M2ProbMoveState::resetToAPlan(const VM2& Xnow) {
    Xi = Xnow;
    t0_toA = running();
    inBandSince = 0.0;
}

void M2ProbMoveState::resetToAIntegrators() {
    iErrToA.setZero();
}


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

