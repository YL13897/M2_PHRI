/* 
    M2StatesHRI.cpp:
        Core state implementations for M2 machine
        - Calibration, Standby, Probabilistic Move (TO_A / WAIT_START / TRIAL)
        - UI command handling and CSV logging
*/

#include <spdlog/spdlog.h>
#include "M2StatesHRI.h"
#include "M2MachineHRI.h"
#include <cmath>
#include <algorithm>
#include <cctype>
#include <iomanip>
#include <limits>


// ----------------------------------------------------------------------------
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
    /* 
    Admittance model:
    M * dv/dt + B * v = Fm
    
    Backward Euler discretisation:
    M * (v_{k+1} - v_k) / dt + B * v_{k+1} = Fm
    
    Rearrange:
    M * v_{k+1} - M * v_k + B * dt * v_{k+1} = Fm * dt
    (M + B * dt) * v_{k+1} = Fm * dt + M * v_k
    
    Therefore:
    v_{k+1} = (Fm * dt + M * v_k) / (M + B * dt)
    
    Here:
    dX = v_k, previous velocity
    return value = v_{k+1}, new velocity command
    */
   
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

static constexpr int X_AXIS = 0;
static constexpr int Y_AXIS = 1;
static constexpr double X_MIN = 0.20;
static constexpr double X_MAX = 0.44;
static constexpr double Y_MIN = 0.10;
static constexpr double Y_MAX = 0.38;
static constexpr double HOLD_POS_EPS = 0.02;
static constexpr double HOLD_VEL_EPS = 0.05;

static inline VM2 clampA(double x, double y) {
    if (!std::isfinite(x)) x = 0.32;
    if (!std::isfinite(y)) y = 0.12;
    return VM2(clamp_compat(x, X_MIN, X_MAX), clamp_compat(y, Y_MIN, Y_MAX));
}

static bool startHold(RobotM2* robot, const VM2& target, bool& active) {
    if (active) return true;

    if (!robot->initPositionControl()) {
        robot->initTorqueControl();
        robot->setEndEffForceWithCompensation(VM2::Zero(), false);
        return false;
    }

    if (robot->setEndEffPosition(target) != SUCCESS) {
        robot->initTorqueControl();
        robot->setEndEffForceWithCompensation(VM2::Zero(), false);
        active = false;
        return false;
    }

    active = true;
    return true;
}

static void stopHold(RobotM2* robot, bool& active) {
    if (!active) return;
    robot->initTorqueControl();
    robot->setEndEffForceWithCompensation(VM2::Zero(), false);
    active = false;
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

// Enter standby: reset moving and hold state.
void M2StandbyState::entryCode() {
    spdlog::warn("Entering Standby! Will remain limp for 2s, then move to A.");
    robot->initTorqueControl();
    auto probState = machine ? machine->state<M2ProbMoveState>("ProbMoveState") : nullptr;
    if (probState) A = probState->A;
    spdlog::info("Standby target A set to ({:.3f}, {:.3f})", A(0), A(1));
    isMoving = false;
    holdActive = false;
    safetyTripped = false; // reset safety trip on entry; will be set if excessive speed detected during standby
}

// Stay limp for 2 seconds, move smoothly to A, then enter fixed position hold.
void M2StandbyState::duringCode() {
    VM2 X = robot->getEndEffPosition();
    VM2 dX = robot->getEndEffVelocity();
    VM2 F_ext = robot->getEndEffForce();

    if (safetyTripped) {
        stopHold(robot, holdActive);
        robot->setEndEffForceWithCompensation(VM2::Zero(), false);
        return;
    }

    if (!std::isfinite(X(0)) || !std::isfinite(X(1)) ||
        !std::isfinite(dX(0)) || !std::isfinite(dX(1))) {
        safetyTripped = true;
        stopHold(robot, holdActive);
        spdlog::error("STANDBY SAFETY TRIP: Position or velocity is not finite.");
        robot->setEndEffForceWithCompensation(VM2::Zero(), false);
        return;
    }

    // Guard against dead/offline position sensor returning exactly zero
    if (X.norm() < 0.0001) {
        safetyTripped = true;
        stopHold(robot, holdActive);
        spdlog::error("STANDBY SAFETY TRIP: Sensor offline (Pos at Origin)! Latching forces OFF.");
        robot->setEndEffForceWithCompensation(VM2::Zero(), false);
        return;
    }

    // Guard against physically impossible velocities or disconnected sensor
    if (dX.norm() > 1.5) {
        safetyTripped = true;
        stopHold(robot, holdActive);
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
            stopHold(robot, holdActive);
            isMoving = true;
            Xi = X;
            tMoveStart = running();
            spdlog::info("Standby 2s passed. Auto-moving to Point A...");
        }

        double t_moved = running() - tMoveStart;
        if (holdActive) {
            if (iterations() % 500 == 1) {
                spdlog::info("Standby (Fixed at A): X={:.3f}, Y={:.3f}, Fx={:.2f}, Fy={:.2f}", X(0), X(1), F_ext(0), F_ext(1));
            }
            return;
        }

        VM2 Xd, dXd;
        MinJerk(Xi, A, T_move, std::min(t_moved, T_move), Xd, dXd);
        Eigen::Matrix2d K = Eigen::Matrix2d::Identity() * 600.0;
        Eigen::Matrix2d D = Eigen::Matrix2d::Identity() * 25.0;
        VM2 F_cmd = K * (Xd - X) + D * (dXd - dX);
        if (t_moved < 0.5) F_cmd *= (t_moved / 0.5);
        for (int i = 0; i < 2; ++i) F_cmd(i) = clamp_compat(F_cmd(i), -60.0, 60.0);
        robot->setEndEffForceWithCompensation(F_cmd, true);
        if (iterations() % 500 == 1)
            spdlog::info("Standby Pre-Pos: X={:.3f}, Y={:.3f}, Fx={:.2f}, Fy={:.2f}", X(0), X(1), F_ext(0), F_ext(1));

        if (t_moved >= T_move && (A - X).norm() <= HOLD_POS_EPS && dX.norm() <= HOLD_VEL_EPS) {
            if (!startHold(robot, A, holdActive)) {
                safetyTripped = true;
                spdlog::error("STANDBY SAFETY TRIP: Could not enter fixed hold.");
            } else {
                spdlog::info("Standby fixed at A=({:.3f}, {:.3f})", A(0), A(1));
            }
        }
    }
}

// Exit standby: zero force
void M2StandbyState::exitCode() {
    stopHold(robot, holdActive);
    robot->setEndEffForceWithCompensation(VM2::Zero());
}

bool M2StandbyState::setA(double x, double y) {
    const VM2 nextA = clampA(x, y);
    const bool changed = (A - nextA).norm() > 1e-9;
    if (changed) {
        stopHold(robot, holdActive);
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

bool M2ProbMoveState::setA(double x, double y) {
    const VM2 nextA = clampA(x, y);
    const bool changed = (A - nextA).norm() > 1e-9;
    A = nextA;
    return changed;
}

// Initialize ProbMove: torque mode, reset flags, open CSVs, load perturbations
void M2ProbMoveState::entryCode() {

    robot->initTorqueControl();
    robot->setEndEffForceWithCompensation(VM2::Zero(), false);
    spdlog::info("ProbMove target A set to ({:.3f}, {:.3f})", A(0), A(1));
    currentPhase = TO_A;
    finishedFlag = false;
    safetyTripped = false;
    initToA = true; // will trigger TO_A entry code on first loop
    initTrial = true; // will trigger TRIAL entry code on first loop
    pendingStart  = false; // no start pending at entry
    // Reset debounce clock on state entry so running()-based delta stays valid.
    lastStartTime = -1.0;
    rwstAckPending_ = false;
    workspaceGuardEnabled = false;
    disturbanceActive_ = false;
    disturbanceExpireAt_ = -1.0;
    holdActive_ = false;
    admittanceActive_ = false;
    trialIndex_ = 0;
    openCSV();

}

// Main loop: drain UI, then run phase switch (TO_A / WAIT_START / TRIAL).
void M2ProbMoveState::duringCode() {
    // === GLOBAL COMMAND DRAIN === (TRBG/RWST/DSTR/S_A/S_MD)
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
                    if (machine && machine->UIserver) {
                        machine->UIserver->sendCmd("TRND");
                        spdlog::info("TRIAL: RWST accepted -> TRND");
                    }
                    pendingStart = false;
                    initToA = true;
                    inBandSince = 0.0;
                    workspaceGuardEnabled = false;
                    rwstAckPending_ = true;
                    safetyTripped = false;
                    currentPhase = TO_A;
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
                    stopHold(robot, holdActive_);
                    pendingStart = false;
                    initToA = true;
                    inBandSince = 0.0;
                    workspaceGuardEnabled = false;
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

            // Runtime standby point from Unity: S_A [x, y].
            if (cu == "S_A") {
                if (currentPhase != TRIAL && a.size() >= 2) {
                    bool changed = setA(a[0], a[1]);
                    auto stbyState = machine ? machine->state<M2StandbyState>("StandbyState") : nullptr;
                    if (stbyState) stbyState->setA(a[0], a[1]);
                    if (changed) {
                        stopHold(robot, holdActive_);
                        pendingStart = false;
                        initToA = true;
                        inBandSince = 0.0;
                        workspaceGuardEnabled = false;
                        safetyTripped = false;
                        currentPhase = TO_A;
                    }
                    spdlog::info("PHASE {}: S_A -> A=({:.3f}, {:.3f})", (int)currentPhase, A(0), A(1));
                    if (machine && machine->UIserver) machine->UIserver->sendCmd("OK");
                } else {
                    if (machine && machine->UIserver) machine->UIserver->sendCmd("BUSY");
                    spdlog::warn("S_A rejected: phase={} or missing coordinates", (int)currentPhase);
                }
                machine->UIserver->clearCmd();
                continue;
            }

            // Handle HRI mode setting in WAIT_START/TO_A.
            if (cu.rfind("S_MD",0)==0) {
                if (currentPhase != TRIAL) {
                    if (!a.empty()) {
                        int hriMode = (int)std::round(a[0]);
                        HRIMode_ = (hriMode == 2) ? V2_PHRI : V1_HRI;
                        spdlog::info("PHASE {}: S_MD -> mode={}", (int)currentPhase, hriMode);
                        if (machine && machine->UIserver) machine->UIserver->sendCmd("OK");
                    } else {
                        spdlog::warn("WAIT_START: S_MD missing args");
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
                stopAdmittance();
                stopHold(robot, holdActive_);
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
            if (!safetyTripped && distA <= HOLD_POS_EPS && dX.norm() <= HOLD_VEL_EPS) {
                if (inBandSince == 0.0) {
                    inBandSince = running();}
                else if ((running() - inBandSince) >= holdTimeA) { // held for required time
                    atA_hold = true;
                }
            } else {
                inBandSince = 0.0;
            }

            if (atA_hold && startHold(robot, A, holdActive_)) {
                workspaceGuardEnabled = true;
                currentPhase = WAIT_START;
                if (machine && machine->UIserver) {
                    machine->UIserver->sendCmd("AT_A");
                    spdlog::info("Fixed at A.");
                }
                if (rwstAckPending_) {
                    rwstAckPending_ = false;
                    if (machine && machine->UIserver) machine->UIserver->sendCmd("RWOK");
                    spdlog::info("RWST completed: reached A and entered WAIT_START -> RWOK");
                }
                spdlog::info("TO_A -> WAIT_START");
            } else if (atA_hold) {
                atA_hold = false;
                inBandSince = 0.0;
                safetyTripped = true;
                spdlog::error("TO_A SAFETY TRIP: Could not enter fixed hold.");
            }
            break;
        }

        // --- WAIT_START: transition to TRIAL ---
        case WAIT_START: {
            if (!startHold(robot, A, holdActive_)) {
                initToA = true;
                inBandSince = 0.0;
                currentPhase = TO_A;
                spdlog::error("WAIT_START: Fixed hold failed; returning to TO_A.");
                break;
            }

            if (pendingStart) {

                pendingStart  = false;
                initTrial     = true;
                stopHold(robot, holdActive_);
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
                safetyTripped = false;
                ++trialIndex_;

                if (machine && machine->UIserver) {
                    {
                        std::vector<double> p; 
                        p.push_back(trialStartTime);                      // t
                        p.push_back((HRIMode_ == V2_PHRI) ? 2 : 1);       // HRI_Mode
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

            if (HRIMode_ == V2_PHRI && disturbanceActive_)
                F_internal(X_AXIS) += disturbanceDirection_ * disturbanceForceMagnitude_;

            VM2 F_cmd = F_internal;
            F_cmd(Y_AXIS) = 0.0;
            double cmdEffort = F_cmd.norm();

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
                VM2 Vadm = myVE(dX, Fs, admB, admM, dt());
                VM2 Vd = VM2::Zero();
                Vd(X_AXIS) = Vadm(X_AXIS);
                Vd(Y_AXIS) = admLockK * (A(Y_AXIS) - X(Y_AXIS)) - admLockD * dX(Y_AXIS);

                const double velLimit = std::abs(admVelLimit);
                if (velLimit > 0.0) {
                    for (int i = 0; i < 2; ++i)
                        Vd(i) = clamp_compat(Vd(i), -velLimit, velLimit);
                }
                if (workspaceGuardEnabled) {
                    if (X(X_AXIS) < X_MIN && Vd(X_AXIS) < 0.0) Vd(X_AXIS) = 0.0;
                    if (X(X_AXIS) > X_MAX && Vd(X_AXIS) > 0.0) Vd(X_AXIS) = 0.0;
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
            
            if (tTrial >= trialDurationSec) {
                if (machine && machine->UIserver) {
                    machine->UIserver->sendCmd("TRND");
                    spdlog::info("Log: TRND (tTrial={:.3f}s)", tTrial);
                }
                stopAdmittance();
                pendingStart = false;
                initTrial = true;
                initToA = true;
                currentPhase = TO_A;
                inBandSince = 0.0;
                spdlog::info("TRIAL duration reached: TRIAL -> TO_A");
                break;
            }

            writeCSV(tTrial, X, dX, F_interaction, F_endeff, F_internal, cmdEffort);

            break;

        }

    }
}

// Cleanup on ProbMove exit: zero forces, close CSVs, send session summary
void M2ProbMoveState::exitCode() {
    stopAdmittance();
    stopHold(robot, holdActive_);
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
    return K * (X0 - X) - D * dX;
}

// Reset TO_A trajectory plan with current position as new start, used when entering TO_A
void M2ProbMoveState::resetToAPlan(const VM2& Xnow) {
    Xi = Xnow;
    t0_toA = running();
    inBandSince = 0.0;
}

// Apply force command with safety checks and saturation
void M2ProbMoveState::applyForce(const VM2& F) {

    VM2 X_chk  = robot->getEndEffPosition();
    VM2 dX_chk = robot->getEndEffVelocity();

    if (safetyTripped) {
        robot->setEndEffForceWithCompensation(VM2::Zero(), false);
        return;
    }

    if (!std::isfinite(X_chk(0)) || !std::isfinite(X_chk(1)) ||
        !std::isfinite(dX_chk(0)) || !std::isfinite(dX_chk(1))) {
        safetyTripped = true;
        spdlog::error("PROBMOVE SAFETY TRIP: Position or velocity is not finite.");
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
    VM2 F_cmd = F;

    // Clamp forces
    for (int i=0; i<2; ++i)
        F_cmd(i) = clamp_compat(F_cmd(i), -forceSaturation, forceSaturation);

    robot->setEndEffForceWithCompensation(F_cmd, true);
}

void M2ProbMoveState::startAdmittance() {
    if (admittanceActive_) return;
    stopHold(robot, holdActive_);
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
        csv << "trial_index,time_trial,pos_x,pos_y,vel_x,vel_y,interaction_fx,interaction_fy,endeffect_fx,endeffect_fy,internal_fx,internal_fy,effort,disturbance_active,disturbance_direction\n";
    }
}

// Write a row to the ProbMove CSV with current trial data and metadata
void M2ProbMoveState::writeCSV(double tTrial, const VM2& pos, const VM2& vel,
    const VM2& interactionForce, const VM2& endEffForce, const VM2& fInternal, double effort) {
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
