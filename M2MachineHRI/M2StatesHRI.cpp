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
#include <cctype>
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

static inline VM2 myVE(const VM2& dX, const VM2& Fm, double B, double M, double dt) {
    const double denom = M + B * dt;
    if (denom <= std::numeric_limits<double>::epsilon()) {
        return VM2::Zero();
    }
    return (Fm * dt + M * dX) / denom;
}

// Simple clamp helper
template <typename T>
static inline T clamp_compat(T v, T lo, T hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

static inline int axis_mode(int axisMode) {
    return axisMode == 1 ? 1 : 0;
}

static inline VM2 axis_A(int axisMode) {
    return axis_mode(axisMode) == 1 ? VM2(0.50, 0.25) : VM2(0.32, 0.20);
}

static inline double axis_wall_min(int axisMode) {
    return axis_mode(axisMode) == 1 ? 0.12 : 0.20;
}

static inline double axis_wall_max(int axisMode) {
    return axis_mode(axisMode) == 1 ? 0.38 : 0.44;
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

// Enter standby: reset moving flag, wait 3 sec
void M2StandbyState::entryCode() {
    spdlog::warn("Entering Standby! Will hold limp for 3s, then move to A.");
    robot->initTorqueControl();
    auto probState = machine ? machine->state<M2ProbMoveState>("ProbMoveState") : nullptr;
    if (probState) A = probState->A;
    spdlog::info("Standby target A set to ({:.3f}, {:.3f})", A(0), A(1));
    isMoving = false;
    safetyTripped = false; // reset safety trip on entry; will be set if excessive speed detected during standby
}

// Idle loop: wait 3 sec at zero force, then min-jerk to point A
void M2StandbyState::duringCode() {
    VM2 X = robot->getEndEffPosition();
    VM2 dX = robot->getEndEffVelocity();
    VM2 F_ext = robot->getEndEffForce();

    if (safetyTripped) {
        robot->setEndEffForceWithCompensation(VM2::Zero(), false);
        return;
    }

    // Guard against dead/offline position sensor returning exactly zero
    if (X.norm() < 0.0001) {
        safetyTripped = true;
        spdlog::error("STANDBY SAFETY TRIP: Sensor offline (Pos at Origin)! Latching forces OFF.");
        robot->setEndEffForceWithCompensation(VM2::Zero(), false);
        return;
    }

    // Guard against physically impossible velocities or disconnected sensor
    if (dX.norm() > 1.5) {
        safetyTripped = true;
        spdlog::error("STANDBY SAFETY TRIP: Speed {:.2f}m/s > 1.5m/s! Latching forces OFF.", dX.norm());
        robot->setEndEffForceWithCompensation(VM2::Zero(), false);
        return;
    }

    if (running() < 2.0) {
        // First 2 seconds of standby: Completely Limp
        robot->setEndEffForceWithCompensation(VM2::Zero(), true);
        if (iterations() % 500 == 1) spdlog::info("Standby (Limp phase): X={:.3f}, Y={:.3f}, Fx={:.2f}, Fy={:.2f}", X(0), X(1), F_ext(0), F_ext(1));
    } else {
        if (!isMoving) {
            isMoving = true;
            Xi = X;
            tMoveStart = running();
            spdlog::info("Standby 2s passed. Auto-moving to Point A...");
        }

        double t_moved = running() - tMoveStart;
        VM2 F_cmd;

        if (t_moved >= T_move) {
            // Trajectory complete: Seamlessly snap to the stiff holding lock
            F_cmd = computeHoldForce(X, dX, A, holdK_);
            if (iterations() % 500 == 1) spdlog::info("Standby (Holding A): X={:.3f}, Y={:.3f}, Fx={:.2f}, Fy={:.2f}", X(0), X(1), F_ext(0), F_ext(1));
        } else {
            // Moving: use soft impedance tracker
            VM2 Xd, dXd;
            MinJerk(Xi, A, T_move, t_moved, Xd, dXd);
            Eigen::Matrix2d K = Eigen::Matrix2d::Identity() * 300.0;
            Eigen::Matrix2d D = Eigen::Matrix2d::Identity() * 15.0;
            F_cmd = K * (Xd - X) + D * (dXd - dX);
            if (t_moved < 0.5) F_cmd *= (t_moved / 0.5);
            if (iterations() % 500 == 1) spdlog::info("Standby Pre-Pos: X={:.3f}, Y={:.3f}, Fx={:.2f}, Fy={:.2f}", X(0), X(1), F_ext(0), F_ext(1));
        }

        for (int i = 0; i < 2; ++i) F_cmd(i) = clamp_compat(F_cmd(i), -60.0, 60.0);
        robot->setEndEffForceWithCompensation(F_cmd, true);
    }
}

// Exit standby: zero force
void M2StandbyState::exitCode() {
    robot->setEndEffForceWithCompensation(VM2::Zero());
}

bool M2StandbyState::ApplyAxisMode(int axisMode) {
    const VM2 nextA = axis_A(axisMode);
    const bool changed = (A - nextA).norm() > 1e-9;
    if (changed) {
        A = nextA;
        isMoving = false;
        safetyTripped = false;
    }
    return changed;
}


// ----------------------------------------------------------------------------
// --- M2ProbMoveState implementation ---

// Construct probabilistic move state; machine is used for UI/session utilities
M2ProbMoveState::M2ProbMoveState(RobotM2* M2, M2MachineHRI* mach, const char* name)
    : M2TimedState(M2, name), machine(mach) {}

bool M2ProbMoveState::ApplyAxisMode(int axisMode) {
    const int nextAxis = axis_mode(axisMode);
    const VM2 nextA = axis_A(nextAxis);
    const bool changed = activeAxis_ != nextAxis || (A - nextA).norm() > 1e-9;

    activeAxis_ = nextAxis;
    A = nextA;
    wallMin_ = axis_wall_min(nextAxis);
    wallMax_ = axis_wall_max(nextAxis);

    return changed;
}

// Initialize ProbMove: torque mode, reset flags, open CSVs, load perturbations
void M2ProbMoveState::entryCode() {

    robot->initTorqueControl();
    robot->setEndEffForceWithCompensation(VM2::Zero(), false);
    spdlog::info("ProbMove target A set to ({:.3f}, {:.3f}) for activeAxis={}", A(0), A(1), activeAxis_);
    currentPhase = TO_A;
    finishedFlag = false;
    safetyTripped = false;
    initToA = true; // will trigger TO_A entry code on first loop
    initTrial = true; // will trigger TRIAL entry code on first loop
    pendingStart  = false; // no start pending at entry
    // Reset debounce clock on state entry so running()-based delta stays valid.
    lastStartTime = -1.0;
    rwstAckPending_ = false;
    softWallEnabled = false;
    // unityForceCmd_ = VM2::Zero();
    disturbanceActive_ = false;
    disturbanceExpireAt_ = -1.0;
    axisLockEnabled_ = false;
    waitLatchEnabled_ = false;
    admittanceActive_ = false;
    trialIndex_ = 0;
    openCSV();

}

// Main loop: drain UI, then run phase switch (TO_A / WAIT_START / TRIAL), 
    // feedback signal cmds (BUSY/OK), and feedback force cmd (FRC2) handling   
void M2ProbMoveState::duringCode() {
    // === GLOBAL COMMAND DRAIN === (TRBG/RWST/FRC2/DSTR/S_AX/S_MD/S_CT)
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

            
            if (cu.rfind("TRBG", 0) == 0) {
                // TRBG only accepted in WAIT_START phase to prevent pending state confusion
                if (currentPhase != WAIT_START) {
                    if (machine && machine->UIserver) machine->UIserver->sendCmd("BUSY");
                    spdlog::warn("TRBG rejected: phase={} (only WAIT_START accepts TRBG)", (int)currentPhase);
                    machine->UIserver->clearCmd();
                    continue;
                }
                double now = running();
                if (pendingStart) {
                    spdlog::warn("START ignored: already pending (phase={}, Δt={:.3f}s)", (int)currentPhase, (now - lastStartTime));
                    machine->UIserver->clearCmd();
                    continue; 
                }
                if (lastStartTime >= 0.0 && (now - lastStartTime) < startMinInterval) {
                    spdlog::warn("START ignored due to debounce (Δt={:.3f}s < {:.3f}s)", (now - lastStartTime), startMinInterval);
                    machine->UIserver->clearCmd();
                    continue; 
                }
                pendingStart = true;
                lastStartTime = now;
                machine->UIserver->clearCmd();
                spdlog::info("WAIT_START: TRBG captured (pendingStart=true, t={:.3f})", now);
                continue;
            }

            // While in ProbMove, BGIN is not a valid command. Return BUSY to avoid silent loss.
            if (cu.rfind("BGIN", 0) == 0) {
                if (machine && machine->UIserver) machine->UIserver->sendCmd("BUSY");
                spdlog::warn("BGIN rejected in ProbMove: phase={}", (int)currentPhase);
                machine->UIserver->clearCmd();
                continue;
            }

            // Manual return to WAIT_START from TRIAL for quick reconfiguration.
            if (cu.rfind("RWST", 0) == 0) {
                if (currentPhase == TRIAL) {
                    stopAdmittance();
                    // unityForceCmd_.setZero();
                    if (machine && machine->UIserver) {
                        machine->UIserver->sendCmd("TRND");
                        spdlog::info("TRIAL: RWST accepted -> TRND");
                    }
                    pendingStart = false;
                    initToA = true;
                    inBandSince = 0.0;
                    softWallEnabled = false;
                    axisLockEnabled_ = false;
                    waitLatchEnabled_ = false;
                    rwstAckPending_ = true;
                    safetyTripped = false;
                    currentPhase = TO_A; // briefly return to TO_A to reset position, then will move back to WAIT_START due to waitLatchEnabled_
                    spdlog::info("TRIAL: RWST received -> TO_A (defer RWOK until WAIT_START at A)");
                } else if (currentPhase == WAIT_START) {
                    rwstAckPending_ = false;
                    if (machine && machine->UIserver) machine->UIserver->sendCmd("RWOK");
                    spdlog::info("WAIT_START: RWST received -> already in WAIT_START");
                } else {
                    if (machine && machine->UIserver) machine->UIserver->sendCmd("BUSY");
                    spdlog::warn("RWST rejected: phase={} (only TRIAL/WAIT_START accepts RWST)", (int)currentPhase);
                }
                machine->UIserver->clearCmd();
                continue;
            }

            // Manual return to TO_A from WAIT_START
            if (cu.rfind("TO_A", 0) == 0) {
                if (currentPhase != TRIAL) {
                    pendingStart = false;
                    initToA = true;
                    inBandSince = 0.0;
                    softWallEnabled = false;
                    axisLockEnabled_ = false;
                    waitLatchEnabled_ = false;
                    safetyTripped = false;
                    currentPhase = TO_A;
                    if (machine && machine->UIserver) machine->UIserver->sendCmd("OK");
                    spdlog::info("PHASE {}: TO_A received -> TO_A", (int)currentPhase);
                } else {
                    if (machine && machine->UIserver) machine->UIserver->sendCmd("BUSY");
                    spdlog::warn("TO_A rejected: phase={} (TRIAL does not accept TO_A)", (int)currentPhase);
                }
                machine->UIserver->clearCmd();
                continue;
            }


        // ------------------------- For testing ----------------------------------------
        // Continuous Unity->M2 feedback force update (used during TRIAL)
            // if (cu.rfind("FRC2", 0) == 0) {
            //     if (a.size() >= 2) {
            //         unityForceCmd_(0) = a[0];
            //         unityForceCmd_(1) = a[1];
            //     }
            //     machine->UIserver->clearCmd();
            //     continue;
            // }
        // ------------------------------------------------------------------------------


            // Dynamic Standby Stiffness
            if (cu.rfind("STBK", 0) == 0) {
                if (!a.empty()) {
                    waitLatchK_ = a[0];
                    axisLockK_ = a[0];
                    spdlog::info("PHASE {}: Standby K dynamic update to {}", (int)currentPhase, a[0]);
                }
                if (machine && machine->UIserver) machine->UIserver->clearCmd();
                continue;
            }

            // Disturbance active flag and magnitude from Unity: DSTR [val]
            if (cu.rfind("DSTR", 0) == 0) {
                double disturbanceCmd = a.empty() ? 0.0 : a[0];
                // Activate if non-zero (since Unity sends the actual force magnitude)
                disturbanceActive_ = std::abs(disturbanceCmd) > 0.001;
                disturbanceDirection_ = disturbanceCmd >= 0.0 ? 1.0 : -1.0;
                if (disturbanceActive_) {
                    disturbanceForceMagnitude_ = std::abs(disturbanceCmd);
                    disturbanceExpireAt_ = running() + disturbanceAutoOffSec_;
                } else {
                    disturbanceExpireAt_ = -1.0;
                }
                if (machine && machine->UIserver) machine->UIserver->sendCmd("OK");
                machine->UIserver->clearCmd();
                continue;
            }

            // Axis setting command from Unity: S_AX [0=X, 1=Y].
            if (cu.rfind("S_AX",0)==0) {
                if (currentPhase != TRIAL && !a.empty()) {
                    int axisMode = (int)std::round(a[0]);
                    bool changed = ApplyAxisMode(axisMode);
                    auto stbyState = machine ? machine->state<M2StandbyState>("StandbyState") : nullptr;
                    if (stbyState) {
                        stbyState->ApplyAxisMode(axisMode);
                    }
                    if (changed) {
                        pendingStart = false;
                        initToA = true;
                        inBandSince = 0.0;
                        softWallEnabled = false;
                        axisLockEnabled_ = false;
                        waitLatchEnabled_ = false;
                        safetyTripped = false;
                        currentPhase = TO_A;
                    }
                    spdlog::info("PHASE {}: S_AX -> axis={}", (int)currentPhase, activeAxis_);
                    if (machine && machine->UIserver) machine->UIserver->sendCmd("OK");
                } else {
                    if (machine && machine->UIserver) machine->UIserver->sendCmd("BUSY");
                    spdlog::warn("PARAM LOCKED: '{}' rejected (phase={}, only WAIT_START/TO_A allowed)", cu, (int)currentPhase);
                }
                machine->UIserver->clearCmd();
                continue;
            }

            // Handle mode setting commands (S_MD, S_CT) in WAIT_START/TO_A
            if (cu.rfind("S_MD",0)==0 || cu.rfind("S_CT",0)==0) {
                if (currentPhase != TRIAL) {
                    if (cu.rfind("S_MD",0)==0 && !a.empty()) {
                        HRI_Mode = (int)std::round(a[0]);
                        HRIMode_ = (HRI_Mode == 2) ? V2_PHRI : V1_HRI;
                        spdlog::info("PHASE {}: S_MD -> mode={}", (int)currentPhase, HRI_Mode);
                        if (machine && machine->UIserver) machine->UIserver->sendCmd("OK");

                    } else if (cu.rfind("S_CT",0)==0 && !a.empty()) {
                        Ctrl_Mode = (int)std::round(a[0]);
                        CtrlMode_ = (Ctrl_Mode == 1) ? V1_POS : V2_VEL;
                        spdlog::info("PHASE {}: S_CT -> mode={}", (int)currentPhase, Ctrl_Mode);
                        if (machine && machine->UIserver) machine->UIserver->sendCmd("OK");

                    } else {
                        spdlog::warn("WAIT_START: mode cmd '{}' missing args", cu);
                    }
                } else {
                    if (machine && machine->UIserver) machine->UIserver->sendCmd("BUSY");
                    spdlog::warn("PARAM LOCKED: '{}' rejected (phase={}, only WAIT_START/TO_A allowed)", cu, (int)currentPhase);
                }
                machine->UIserver->clearCmd();
                continue;
            }

            // Emergency stop: finish ProbMove and return Standby via top-level transition
            if (cu.rfind("SESS",0)==0) {
                // unityForceCmd_.setZero();
                stopAdmittance();
                waitLatchEnabled_ = false;
                finishedFlag = true;
                spdlog::info("SESS received: finish ProbMove and return Standby");
                machine->UIserver->clearCmd();
                break;
            }

            // If unknown command
            spdlog::warn("GLOBAL: unknown cmd='{}' (trim='{}') @phase={}", c, cu, (int)currentPhase);
            machine->UIserver->clearCmd();
        }
    }
    // === END GLOBAL COMMAND DRAIN ===
    // Safety fallback: if DSTR=0 is dropped, force auto-off after a fixed window.
    if (disturbanceActive_ && disturbanceExpireAt_ >= 0.0 && running() >= disturbanceExpireAt_) {
        disturbanceActive_ = false;
        disturbanceExpireAt_ = -1.0;
        spdlog::warn("Disturbance auto-off triggered after {:.3f}s fallback window", disturbanceAutoOffSec_);
    }

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
            
            // Fade-in force authority over first 0.5s to prevent startup jerk
            double t_elapsed = running() - t0_toA;
            if (t_elapsed < 0.5) {
                F_cmd *= (t_elapsed / 0.5); 
            }
            
            applyForce(F_cmd);

            // Transition condition check
            double distA = (A - X).norm();
            atA_hold = false;
            if (distA < epsA_hold) { // within hold threshold
                if (inBandSince == 0.0) {
                    inBandSince = running();}
                else if ((running() - inBandSince) >= holdTimeA) { // held for required time
                    atA_hold = true; 
                    if (machine && machine->UIserver) {         
                        machine->UIserver->sendCmd("AT_A"); 
                        spdlog::info("Checked, atA_hold!");
                    }     
                }
            } else {
                inBandSince = 0.0;
            }

            if (atA_hold) {
                softWallEnabled = true;
                axisLockEnabled_ = true;
                currentPhase = WAIT_START;
                if (rwstAckPending_) {
                    rwstAckPending_ = false;
                    if (machine && machine->UIserver) machine->UIserver->sendCmd("RWOK");
                    spdlog::info("RWST completed: reached A and entered WAIT_START -> RWOK");
                }
                spdlog::info("TO_A -> WAIT_START");
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

            // Arm WAIT_START latch only after TO_A has confirmed AT_A hold.
            if (atA_hold && !waitLatchEnabled_) {
                waitLatchEnabled_ = true;
                spdlog::info("WAIT_START: wait_latch enabled");
            }

            // Virtual spring-damper around point A to reduce free handle motion during WAIT_START.
            if (waitLatchEnabled_) {
                const VM2 X = robot->getEndEffPosition();
                const VM2 dX = robot->getEndEffVelocity();
                const VM2 F_wait = computeHoldForce(X, dX, A, waitLatchK_, waitLatchD_);
                applyForce(F_wait);
            } else {
                applyForce(VM2::Zero());
            }

            if (pendingStart) {

                pendingStart  = false;
                initTrial     = true;
                waitLatchEnabled_ = false; // unlock latch when entering TRIAL
                currentPhase  = TRIAL;

                if (machine && machine->UIserver) machine->UIserver->sendCmd("OK");
                spdlog::info("WAIT_START: pendingStart consumed -> TRIAL");
                break; 
            }
            break;
        }
        
        // --- TRIAL: main trial loop with feedback control and trial end check ---
        case TRIAL: {

            if (initTrial) {
                // Simulate entryCode() for TRIAL
                trialStartTime = running();
                // effortIntegral = 0.0;
                // rawEffortIntegral = 0.0;
                ++trialIndex_;

                if (machine && machine->UIserver) {
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
            VM2 F_interaction = robot->getInteractionForce();
            VM2 F_endeff = robot->getEndEffForce();
            double tTrial = running() - trialStartTime;

            VM2 F_internal = VM2::Zero();
            // VM2 F_unity = unityForceCmd_;
            VM2 F_unity = VM2::Zero();

            const int activeAxis = activeAxis_ == 1 ? 1 : 0;
            const int lockedAxis = 1 - activeAxis;

            // Four-mode framework:
            // 1. V2_PHRI + V1_POS: implemented
            if (HRIMode_ == V2_PHRI && CtrlMode_ == V1_POS) {
                // Native pHRI force generation on M2 side: disturbance only when active.
                F_unity.setZero();
                if (disturbanceActive_) F_internal(activeAxis) += disturbanceDirection_ * disturbanceForceMagnitude_;
            }
            // 2. V2_PHRI + V2_VEL: implemented
            else if (HRIMode_ == V2_PHRI && CtrlMode_ == V2_VEL) {
                F_unity.setZero();
                if (disturbanceActive_) F_internal(activeAxis) += disturbanceDirection_ * disturbanceForceMagnitude_;
            }
            // 3. V1_HRI + V1_POS: framework reserved (to be implemented)
            else if (HRIMode_ == V1_HRI && CtrlMode_ == V1_POS) {
                // No force feedback in HRI mode.
                F_unity.setZero();
            }
            // 4. V1_HRI + V2_VEL: framework reserved (to be implemented)
            else if (HRIMode_ == V1_HRI && CtrlMode_ == V2_VEL) {
                // No force feedback in HRI mode.
                F_unity.setZero();
            }

            VM2 F_cmd = F_internal + F_unity;
            F_cmd(lockedAxis) = 0.0;
            double cmdEffort = F_cmd.norm();

            if (useAdmittance) {
                startAdmittance();

                if (safetyTripped) {
                    robot->setEndEffVelocity(VM2::Zero());
                    cmdEffort = 0.0;
                } else if (X.norm() < 0.0001) {
                    safetyTripped = true;
                    spdlog::error("PROBMOVE SAFETY TRIP: Sensor offline (Pos at Origin)! Latching velocity OFF.");
                    robot->setEndEffVelocity(VM2::Zero());
                    cmdEffort = 0.0;
                } else if (dX.norm() > 1.5) {
                    safetyTripped = true;
                    spdlog::error("PROBMOVE SAFETY TRIP: Speed {:.2f}m/s > 1.5m/s! Latching velocity OFF.", dX.norm());
                    robot->setEndEffVelocity(VM2::Zero());
                    cmdEffort = 0.0;
                } else {
                    VM2 Fs = F_interaction + F_cmd;
                    VM2 Vd = myVE(dX, Fs, admB, admM, dt());
                    Vd(lockedAxis) = 0.0;

                    const double velLimit = std::abs(admVelLimit);
                    if (velLimit > 0.0) {
                        for (int i = 0; i < 2; ++i) {
                            Vd(i) = clamp_compat(Vd(i), -velLimit, velLimit);
                        }
                    }
                    if (softWallEnabled) {
                        if (X(activeAxis) < wallMin_ && Vd(activeAxis) < 0.0) Vd(activeAxis) = 0.0;
                        if (X(activeAxis) > wallMax_ && Vd(activeAxis) > 0.0) Vd(activeAxis) = 0.0;
                    }

                    if (!std::isfinite(Vd(0)) || !std::isfinite(Vd(1))) {
                        safetyTripped = true;
                        spdlog::error("PROBMOVE SAFETY TRIP: Admittance velocity is not finite. Latching velocity OFF.");
                        robot->setEndEffVelocity(VM2::Zero());
                        cmdEffort = 0.0;
                    } else {
                        robot->setEndEffVelocity(Vd);
                        cmdEffort = Vd.norm();
                    }
                }
            } else {
                stopAdmittance();
                applyForce(F_cmd);
            }

            
            if (tTrial >= trialDurationSec) {
                if (machine && machine->UIserver) {
                    machine->UIserver->sendCmd("TRND");
                    spdlog::info("Log: TRND (tTrial={:.3f}s)", tTrial);
                }
                // unityForceCmd_.setZero();
                stopAdmittance();
                pendingStart = false;
                initTrial = true;
                waitLatchEnabled_ = false;
                currentPhase = WAIT_START;
                inBandSince = 0.0;
                spdlog::info("TRIAL duration reached: TRIAL -> WAIT_START");
                break;
            }

            writeCSV(tTrial, X, dX, F_interaction, F_endeff, F_internal, F_unity, cmdEffort);

            break;

        }

    }
}

// Cleanup on ProbMove exit: zero forces, close CSVs, send session summary
void M2ProbMoveState::exitCode() {
    // unityForceCmd_ = VM2::Zero();
    stopAdmittance();
    waitLatchEnabled_ = false;
    robot->setEndEffForceWithCompensation(VM2::Zero());
    if (csv.is_open()) csv.close();

    // Send session summary message to UI
    if (machine && machine->UIserver) {
        {
            machine->UIserver->sendCmd("SESS");
            spdlog::info("Log:SESS");
        }
    }
}


// -----------------------------------------------------------------------------
// --- M2ProbMoveState helper methods ---

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

    VM2 X_chk  = robot->getEndEffPosition();
    VM2 dX_chk = robot->getEndEffVelocity();

    if (safetyTripped) {
        robot->setEndEffForceWithCompensation(VM2::Zero(), false);
        return;
    }

    // SAFETY GUARDIAN: Stop runaway movement due to sensor faults or excessive speed.
    if (X_chk.norm() < 0.0001) {
        safetyTripped = true;
        spdlog::error("PROBMOVE SAFETY TRIP: Sensor offline (Pos at Origin)! Latching forces OFF.");
        robot->setEndEffForceWithCompensation(VM2::Zero(), false);
        return;
    }

    if (dX_chk.norm() > 1.5) {
        safetyTripped = true;
        spdlog::error("PROBMOVE SAFETY TRIP: Speed {:.2f}m/s > 1.5m/s! Latching forces OFF.", dX_chk.norm());
        robot->setEndEffForceWithCompensation(VM2::Zero(), false);
        return;
    }
    const int activeAxis = activeAxis_ == 1 ? 1 : 0;
    const int lockedAxis = 1 - activeAxis;
    VM2 F_cmd = F;

    if (axisLockEnabled_) {
        VM2 X  = robot->getEndEffPosition();
        VM2 dX = robot->getEndEffVelocity();
        VM2 posCurrent = VM2::Zero();
        VM2 velCurrent = VM2::Zero();
        VM2 posTarget = VM2::Zero();
        posCurrent(lockedAxis) = X(lockedAxis);
        velCurrent(lockedAxis) = dX(lockedAxis);
        posTarget(lockedAxis) = A(lockedAxis);
        const VM2 F_lock = computeHoldForce(posCurrent, velCurrent, posTarget, axisLockK_, axisLockD_);
        F_cmd(lockedAxis) += F_lock(lockedAxis);
    }

    if (softWallEnabled) {
        VM2 X  = robot->getEndEffPosition();
        VM2 dX = robot->getEndEffVelocity();
        const double activeMin = wallMin_;
        const double activeMax = wallMax_;

        if (X(activeAxis) < activeMin) {
            double pen = activeMin - X(activeAxis);
            double F_wall = k_wall * pen - d_wall * dX(activeAxis);
            if (F_wall > 0.0) F_cmd(activeAxis) += F_wall;
        }

        if (X(activeAxis) > activeMax) {
            double pen = X(activeAxis) - activeMax;
            double F_wall = k_wall * pen + d_wall * dX(activeAxis);
            if (F_wall > 0.0) F_cmd(activeAxis) -= F_wall;
        }

        const double yWallMax = axis_wall_max(1);
        if (activeAxis == 0 && X(1) > yWallMax) {
            double penY = X(1) - yWallMax;
            double F_wall_y = k_wall * penY + d_wall * dX(1);
            if (F_wall_y > 0.0) F_cmd(1) -= F_wall_y;
        }
    }

    // Clamp forces
    for (int i=0; i<2; ++i)
        F_cmd(i) = clamp_compat(F_cmd(i), -forceSaturation, forceSaturation);

    robot->setEndEffForceWithCompensation(F_cmd, true);
}

void M2ProbMoveState::startAdmittance() {
    if (admittanceActive_) return;
    robot->initVelocityControl();
    robot->setEndEffVelocity(VM2::Zero());
    admittanceActive_ = true;
}

void M2ProbMoveState::stopAdmittance() {
    if (!admittanceActive_) return;
    robot->setEndEffVelocity(VM2::Zero());
    robot->initTorqueControl();
    admittanceActive_ = false;
}


// -----------------------------------------------------------------------------
// --- CSV logging helpers for ProbMoveState ---

// Open logs/M2ProbMove_<session>.csv with header
void M2ProbMoveState::openCSV() {
    if (!trialCsvEnabled_) return;

    const std::string sid = (machine && !machine->sessionId.empty()) ? machine->sessionId : std::string("UNSET");
    const std::string fname = std::string("logs/M2ProbMove_") + sid + ".csv";
    csv.open(fname, std::ios::out | std::ios::app);
    if (!csv.is_open()) {
        spdlog::error("Failed to open ProbMove CSV: {}", fname);
        return;
    }
    if (csv.tellp() == 0) {
        // csv << "trial_index,time_trial,sys_time,pos_x,pos_y,vel_x,vel_y,handle_fx,handle_fy,internal_fx,internal_fy,user_fx,user_fy,effort,disturbance_active,disturbance_direction\n";
        csv << "trial_index,time_trial,pos_x,pos_y,vel_x,vel_y,interaction_fx,interaction_fy,endeffect_fx,endeffect_fy,internal_fx,internal_fy,effort,disturbance_active,disturbance_direction\n";
    }
}

// Write a row to the ProbMove CSV with current trial data and metadata
void M2ProbMoveState::writeCSV(double tTrial, const VM2& pos, const VM2& vel,
    const VM2& interactionForce, const VM2& endEffForce, const VM2& fInternal, const VM2& fUser, double effort) {
    if (!trialCsvEnabled_) return;
    if (!csv.is_open()) return;
    csv << std::fixed << std::setprecision(6)
        << trialIndex_ << ","
        << tTrial << ","
        << pos(0) << "," << pos(1) << ","
        << vel(0) << "," << vel(1) << ","
        << interactionForce(0) << "," << interactionForce(1) << ","
        << endEffForce(0) << "," << endEffForce(1) << ","
        << fInternal(0) << "," << fInternal(1) << ","
        << effort << ","
        << (disturbanceActive_ ? 1 : 0) << ","
        << disturbanceDirection_ << "\n";
}
