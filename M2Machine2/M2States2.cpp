// Core state implementations for M2 machine
// - Calibration, Standby, Probabilistic Move (TO_A / WAIT_START / TRIAL)
// - UI command handling and CSV logging
#include <chrono>
#include <spdlog/spdlog.h>
#include "M2States2.h"
#include "M2Machine2.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <sstream>
#include <random>



// Local wall-clock helper for CSV timestamps
static inline double system_time_sec() {
    using namespace std::chrono;
    return duration_cast<duration<double>>(system_clock::now().time_since_epoch()).count();
}

// Send a plain-text/structured line back to UI
void M2ProbMoveState::sendUI_(const std::string& msg) {
    const int seq = ++this->txSeq_;
    const size_t L = msg.size();

    if (machine && machine->UIserver) {
        machine->UIserver->sendCmd(msg);
    } 
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

// Begin calibration: enter torque mode and start stop-seek routine
void M2CalibState::entryCode(void) {
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
void M2CalibState::duringCode(void) {
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
void M2CalibState::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero());
}
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
    closeStandbyCSV_();
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





// Construct probabilistic move state; machine is used for UI/session utilities
M2ProbMoveState::M2ProbMoveState(RobotM2* M2, M2Machine* mach, const char* name)
    : M2TimedState(M2, name), machine(mach) {}

// Initialize ProbMove: torque mode, reset flags, open CSVs, load perturbations
void M2ProbMoveState::entryCode() {

    robot->initTorqueControl();
    robot->setEndEffForceWithCompensation(VM2::Zero(), false);
    trialEndPositions_.clear();
    finishedFlag = false;
    rng.seed(std::random_device{}());
    openCSV();
    // Preload logs
    openPreloadCSVs_();
    waitBuf_.clear();
    preloadSatisfied_ = false;

    if (meta_scoreMode == 2) {
        currentMode = V2_EFFORT_DISTANCE;
    } else {
        currentMode = V1_COUNT_SUCCESS;
    }
    successfulTrials = 0;
    totalTrialsV1 = 0;
    totalScoreV2 = 0.0;
    totalTrialsV2 = 0;

    currentPhase = TO_A;
    initToA = true;
    initTrial = true;
    pendingStart  = false;
    betweenTrials = false;
    lastStrtTime  = -1.0;  // reset debounce timer

    injectingUp = false;
    injectingLeft = false;
    inBandSince = 0.0;
    waitHoldLatched_ = false;
    // Load perturbation forces from CSV
    loadPerturbationForces();
    if (randomizeOrBlock == 1) buildDeterministicSchedule_random();
    else buildDeterministicSchedule();
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

            
            if (cu.rfind("HALT", 0) == 0) {
                finishedFlag = true;
                if (machine && machine->UIserver) machine->UIserver->sendCmd("OK");
                machine->UIserver->clearCmd();
                k_hold = k_hold_cmd; 
                d_hold = d_hold_cmd;
                spdlog::info("GLOBAL: HALT -> finishedFlag=1");

                continue;
            }
            
            if (cu.rfind("RSTA", 0) == 0) {
                currentPhase   = TO_A;
                initToA        = true;
                inBandSince    = 0.0;
                effortIntegral = 0.0;
                betweenTrials  = false;
                waitHoldLatched_ = false;
                if (machine && machine->UIserver) machine->UIserver->sendCmd("OK");
                machine->UIserver->clearCmd();
                spdlog::info("GLOBAL: RSTA -> TO_A");
                continue;
            }
            
            if (cu.rfind("S_PB", 0) == 0 && !a.empty()) {
                probLeft = clamp_compat(a[0], 0.0, 1.0);
                BlockID = a[1];
                if (randomizeOrBlock == 1) buildDeterministicSchedule_random();
                else buildDeterministicSchedule();
                if (machine && machine->UIserver) machine->UIserver->sendCmd("OK");
                machine->UIserver->clearCmd();
                // spdlog::info("GLOBAL: S_PB applied -> pLeft={:.3f}", currentTrialProb_);
                spdlog::info("GLOBAL: S_PB applied -> BlockID={}", BlockID);
                continue;
            }
            
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
            
            if (cu.rfind("S_MD",0)==0 || cu.rfind("S_MT",0)==0 || cu.rfind("S_TS",0)==0) {
                if (betweenTrials) {
                    if (cu.rfind("S_MD",0)==0 && !a.empty()) {
                        meta_scoreMode = (int)std::round(a[0]);
                        // Immediately apply to the runtime scoring mode so UI and scoring stay consistent
                        currentMode = (meta_scoreMode == 2) ? V2_EFFORT_DISTANCE : V1_COUNT_SUCCESS;
                        spdlog::info("BETWEEN-TRIALS: S_MD -> mode={} (currentMode now = {})",
                                     meta_scoreMode, (currentMode == V2_EFFORT_DISTANCE ? 2 : 1));
                        if (machine && machine->UIserver) machine->UIserver->sendCmd("OK");
                    } else if (cu.rfind("S_MT",0)==0 && !a.empty()) {
                        meta_maxTrials = std::max(1, (int)std::round(a[0]));
                        spdlog::info("BETWEEN-TRIALS: S_MT -> maxTrials={}", meta_maxTrials);
                        if (machine && machine->UIserver) machine->UIserver->sendCmd("OK");
                    } else if (cu.rfind("S_TS",0)==0 && !a.empty()) {
                        meta_targetSucc = std::max(1, (int)std::round(a[0]));
                        spdlog::info("BETWEEN-TRIALS: S_TS -> targetSucc={}", meta_targetSucc);
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
            if (cu.rfind("S_SID", 0) == 0 && !a.empty()) {
                long long sidNum = (long long)std::llround(a[0]);
                if (machine) machine->sessionId = std::to_string(sidNum);
                if (machine && machine->UIserver) machine->UIserver->sendCmd("OK");
                machine->UIserver->clearCmd();
                spdlog::info("GLOBAL: S_SID -> sessionId={}", (machine ? machine->sessionId : std::string("null")));
                continue;
            }

            // // Allow adjusting preload threshold/window from UI
            // if (cu.rfind("S_PLT",0)==0 && !a.empty()) {
            //     preloadThresholdN_ = a[0];
            //     if (machine && machine->UIserver) machine->UIserver->sendCmd("OK");
            //     machine->UIserver->clearCmd();
            //     spdlog::info("GLOBAL: S_PLT -> preloadThresholdN_={}", preloadThresholdN_);
            //     continue;
            // }
            // if (cu.rfind("S_PLW",0)==0 && !a.empty()) {
            //     preloadWindowSec_ = std::max(0.0, a[0]);
            //     if (machine && machine->UIserver) machine->UIserver->sendCmd("OK");
            //     machine->UIserver->clearCmd();
            //     // spdlog::info("GLOBAL: S_PLW -> preloadWindowSec_={}", preloadWindowSec_);
            //     continue;
            // }

            // Directional constant force update
            if (cu.rfind("FRC2",0)==0 && a.size() >= 2) {

                // const double F_UP_MIN = 0.0,  F_UP_MAX = 120.0;    // Upward positive
                // const double F_L_MIN  = -80.0, F_L_MAX  = 0.0;    // Leftward negative
                // F_const_up   = clamp_compat(a[0], F_UP_MIN, F_UP_MAX);
                // F_const_left = clamp_compat(a[1], F_L_MIN, F_L_MAX);

                F_const_up = a[0];
                F_const_left = a[1];
                if (F_const_left > 0 && trialIsLeft){F_const_left = -F_const_left;} // leftward force must be negative or zero
                else if (F_const_left < 0 && !trialIsLeft){F_const_left = -F_const_left;} // rightward force must be positive or zero
                
                if (machine && machine->UIserver)
                    machine->UIserver->sendCmd("OK");
                machine->UIserver->clearCmd();
                spdlog::info("GLOBAL: FRC2 -> F_const_up={}, F_const_left={}", F_const_up, F_const_left);
                continue;
            }

            // Hold stiffness/damping update
            if (cu.rfind("HLKD",0)==0 && a.size() >= 2) {

                if (waitHoldLatched_) {
                    // 正在 hold，先排队，等释放后再应用
                    k_hold_cmd = a[0];
                    d_hold_cmd = a[1];
                    // spdlog::info("GLOBAL: HLKD queued (latched) k_hold={}, d_hold={}", k_hold_cmd, d_hold_cmd);
                } else {
                    // 不在 hold，立即生效
                    k_hold_cmd = a[0];
                    d_hold_cmd = a[1];
                    k_hold = k_hold_cmd; 
                    d_hold = d_hold_cmd;
                    // spdlog::info("GLOBAL: HLKD applied k_hold={}, d_hold={}", k_hold, d_hold);
                }


                if (machine && machine->UIserver)
                    machine->UIserver->sendCmd("OK");
                machine->UIserver->clearCmd();
                // spdlog::info("GLOBAL: HLKD -> k_hold={}, d_hold={}", k_hold_cmd, d_hold_cmd);
                continue;
            }

            if (cu.rfind("Q_PB", 0) == 0) { // query pLeft
                if (randomizeOrBlock == 1){
                    double pLeft = probLeft;
                    if (trialSchedule_.empty()) {
                        buildDeterministicSchedule_random();
                    }
                    if (!trialSchedule_.empty()) {
                        size_t idx = trialIdx_ % trialSchedule_.size();
                        pLeft = (idx < trialProb_.size()) ? trialProb_[idx] : probLeft;
                    }
                    std::vector<double> pLeft_Q;
                    pLeft_Q.push_back(pLeft);
                    machine->UIserver->sendCmd("R_PB", {pLeft_Q});
                    machine->UIserver->clearCmd();
                    spdlog::info("GLOBAL: Q_PB -> pLeft={:.3f}", pLeft);
                    continue;
                }
            }
        

            // If unknown command
            spdlog::warn("GLOBAL: unknown cmd='{}' (trim='{}') @phase={}", c, cu, (int)currentPhase);
            machine->UIserver->clearCmd();
        }
    }


    // === END GLOBAL COMMAND DRAIN ===
    // Phase controller: TO_A -> WAIT_START -> TRIAL
    switch (currentPhase) {
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
                        std::vector<double> p;
                        p.push_back(currentTrialProb_);
                        machine->UIserver->sendCmd("AT_A", p);  // Send p[0] = currentTrialProb_
                        atA_notified_ = true;
                        spdlog::info("Checked, atA_hold! pLeft={:.3f}", currentTrialProb_);
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
                waitBuf_.push_back(s);
                const double tCut = s.t - preloadWindowSec_;
                while (!waitBuf_.empty() && waitBuf_.front().t < tCut) {
                    waitBuf_.pop_front();
                }
            }

            VM2 X = robot->getEndEffPosition();
            double distToA = (A - X).norm();
            bool atA_hold = false;           
            if (distToA < epsA_hold) {
                if (inBandSince == 0.0) inBandSince = running();
                else if ((running() - inBandSince) >= holdTimeA) {
                    atA_hold = true;
                    if (machine && machine->UIserver && !atA_notified_) {
                        std::vector<double> p;
                        p.push_back(currentTrialProb_);
                        machine->UIserver->sendCmd("AT_A", p);  // Send p[0] = currentTrialProb_
                        atA_notified_ = true;
                        spdlog::info("WAIT_START: Checked, atA_hold! pLeft={:.3f}", currentTrialProb_);
                    }     
                
                }
            } else {
                inBandSince = 0.0;
                atA_notified_ = false;
            }
            if (pendingStart && atA_hold) {
                // On STRT: evaluate last preload window and log
                // Compute preloadSatisfied_ over the last preloadWindowSec_
                const double tNow = running();
                const double tMin = tNow - preloadWindowSec_;
                // bool windowCovered = (!waitBuf_.empty() && waitBuf_.front().t <= tMin);
                bool windowCovered = (!waitBuf_.empty());
                // bool allAbove = true;
                bool allAbove;

                double sumNorm = 0.0;
                int count = 0;
                for (const auto& s : waitBuf_) {
                    if (s.t < tMin) continue;

                    double fn = s.force.head<2>().norm();
                    sumNorm += fn;
                    count++;}
                    // if (s.force(0) < preloadThresholdN_) { allAbove = false; break; }

                    // if (s.force.head<2>().norm() < preloadThresholdN_) { 
                        
                    //     allAbove = false; break; 
                    // }
                    if (count > 0) {
                        double meanNorm = sumNorm / count;
                        spdlog::info("meanNorm ={:.3f}", meanNorm);
                        allAbove = (meanNorm >= preloadThresholdN_);}
                
                preloadSatisfied_ = (windowCovered && allAbove);

                // Determine upcoming trial index for current mode
                int nextTrialIdx = (currentMode == V1_COUNT_SUCCESS) ? totalTrialsV1 : totalTrialsV2;
                writePreloadWindow_(nextTrialIdx, tNow, (currentMode == V1_COUNT_SUCCESS ? 1 : 2), preloadSatisfied_);
                // writeTrialTag_(nextTrialIdx, (currentMode == V1_COUNT_SUCCESS ? 1 : 2), preloadSatisfied_, tNow);

                pendingStart  = false;
                betweenTrials = false;  
                currentPhase  = TRIAL;
                initTrial     = true;
                waitHoldLatched_ = false; // release A hold when trial begins
                if (machine && machine->UIserver) machine->UIserver->sendCmd("OK");
                spdlog::info("WAIT_START: pendingStart consumed -> TRIAL (atA_hold=1)");
                break; 
            } else if (pendingStart && !atA_hold) {
                
                if (iterations() % 1000 == 1) {
                    spdlog::info("WAIT_START: STRT pending but not at A (dist={:.3f} <? {:.3f})", distToA, epsA_hold);
                }
            }


            // 在 WAIT_START 分支内，日志前取当前力
            VM2 F_wait = waitBuf_.empty() ? VM2::Zero() : waitBuf_.back().force;
            double F_wait_norm = F_wait.norm();
            
            if (iterations() % 1000 == 1) {
                spdlog::info(
                    "WAIT_START cfg: pLeft={:.3f}, mode={}, targetSucc={}, maxTrials={}, V1={}/{}, V2Score={:.3f}({}), finishedFlag={}",
                    currentTrialProb_, meta_scoreMode, meta_targetSucc, meta_maxTrials,
                    successfulTrials, totalTrialsV1, totalScoreV2, totalTrialsV2,
                    finishedFlag ? 1 : 0
                );

                spdlog::info("F=({:.3f},{:.3f}), |F|={:.3f}", F_wait(0), F_wait(1), F_wait_norm);
            }

            if (atA_hold) {
                waitHoldLatched_ = true;
            }


            if (waitHoldLatched_) {
                
                VM2 Xh  = robot->getEndEffPosition();
                VM2 dXh = robot->getEndEffVelocity();
                Eigen::Matrix2d K = Eigen::Matrix2d::Identity() * k_hold;
                Eigen::Matrix2d D = Eigen::Matrix2d::Identity() * d_hold;
                if (!hold_log_once) {
                    spdlog::info("HLKD -> k_hold={}, d_hold={}", k_hold, d_hold);
                    hold_log_once = true; 
                }
                VM2 F_hold = K * (A - Xh) + D * (VM2::Zero() - dXh);
                applyForce(F_hold);  
            } else {
                hold_log_once = false;
                // robot->setEndEffForceWithCompensation(VM2::Zero());
                this->applyForce(VM2::Zero());
            }
  
            break;
        }

        case TRIAL: {
            // This block simulates M2TrialState (apply internal force, score, log)
            if (initTrial) {
                // Simulate entryCode() for TRIAL
                trialStartTime = running();
                decideInternalForceDirection();
                effortIntegral = 0.0;
                rawEffortIntegral = 0.0;
                
                if (injectingUp)       baselineImpulseN = 0;   // default value 4000;
                else if (injectingLeft) baselineImpulseN = 0;  // default value 5270;
                else                    baselineImpulseN = 0.0;  

                if (machine && machine->UIserver) {
                    std::ostringstream oss;
                    oss.setf(std::ios::fixed);
                    oss.precision(3);
                    int curTrialForMode = (currentMode == V1_COUNT_SUCCESS) ? totalTrialsV1 : totalTrialsV2;
                    oss << "TRIAL_BEGIN t=" << trialStartTime
                        << " dir=" << (internalForce(0) < -1e-9 ? "SIDE" : (internalForce(1) > 1e-9 ? "UP" : "NONE"))
                        << " pLeft=" << currentTrialProb_
                        << " mode=" << (currentMode == V1_COUNT_SUCCESS ? 1 : 2)
                        << " cur_trial=" << curTrialForMode
                        << " max_trial=" << meta_maxTrials;
                    // Tag preload (0/1) evaluated in WAIT_START
                    oss << " preload=" << (preloadSatisfied_ ? 1 : 0);
                    std::string outBegin = oss.str();
                    sendUI_(outBegin);
                    {
                        std::vector<double> p;
                        int dirCode = (internalForce(0) < -1e-9) ? -1 : ((internalForce(1) > 1e-9) ? 1 : 0);
                        int curTrialForMode = (currentMode == V1_COUNT_SUCCESS) ? totalTrialsV1 : totalTrialsV2;
                        p.push_back(trialStartTime);                             // t
                        p.push_back((double)dirCode);                            // dirCode
                        // p.push_back(probLeft);                                   // pLeft
                        p.push_back(currentTrialProb_); 
                        p.push_back((double)(currentMode == V1_COUNT_SUCCESS ? 1 : 2)); // mode
                        p.push_back((double)curTrialForMode);                    // cur
                        p.push_back((double)meta_maxTrials);                     // max
                        machine->UIserver->sendCmd("TRBG", p);
                        spdlog::info("TRBBBBB");
                    }
                }
                spdlog::info(
                    "TRIAL_BEGIN cfg: pLeft={:.3f}, mode={}, targetSucc={}, maxTrials={}, V1={}/{}, V2Score={:.3f}({})",
                    // probLeft,
                    currentTrialProb_,
                    meta_scoreMode,
                    meta_targetSucc,
                    meta_maxTrials,
                    successfulTrials,
                    totalTrialsV1,
                    totalScoreV2,
                    totalTrialsV2
                );
                initTrial = false;
            }

            VM2 X = robot->getEndEffPosition();
            VM2 dX = robot->getEndEffVelocity();

            double n = (C - X).norm(); // distance to target C
            double tTrial = running() - trialStartTime;

            // Guard: do not declare success in the first 80 ms to avoid dur=0 "instant success"
            const double minTrialEvalTime = 0.08;

            // Transition condition check
            // bool reachedC = (n < epsC) && (tTrial >= minTrialEvalTime);
            bool reachedC = (n < epsC) && (tTrial >= minTrialEvalTime) && X(1) < (C[1]+ epsC); // also require not too high
            // bool timeout = (tTrial >= (trialMaxTime + trialExtendTime));
            bool timeout = (tTrial >= (trialMaxTime));
            bool timeoutTrial = (tTrial >= trialMaxTime);

            // bool leftAfterReach = false;       // 曾经离开过 C 区
            // static bool inCZone = false;       // 当前是否在 C 区内 
            // if (reachedC) {
            //     inCZone = true;                 // 一旦达到 C 区，标记为在 C 区内
            // }
            // else {
            //     if (inCZone) {
            //         leftAfterReach = true;     // 如果之前在 C 区内，现在不在，说明曾经离开过 C 区
            //         inCZone = false;           // 更新状态为不在 C 区内
            //     }
            // }

            // === Recorded-perturbation replay (frame-by-frame) ===
            VM2 F_int = VM2::Zero();
            // if (injectingUp) {
            //     if (perturbIndex < upPerturbForce.size()) {
            //         F_int(1) = upPerturbForce[perturbIndex++];
            //     }

            // } else if (injectingLeft) {
            //         if (perturbIndex < leftPerturbForce.size()) {
            //             F_int(0) = leftPerturbForce[perturbIndex++]-10;
            //         }
            // }

            if (trialIsLeft){
                F_const_left = -std::abs(F_const_left);
            }else{
                F_const_left = std::abs(F_const_left);
            }

            // -------------------- Internal force -------------------------
            if (injectingUp) {
                // if (!reachedC && timeoutTrial==false) {
                if (timeoutTrial==false) {
                // Apply upward perturbation only until target is reached
                    F_int(1) = F_const_up;
                    // spdlog::info("F_int(1) = {:.3f}", F_int(1));
                    // if (perturbIndex < upPerturbForce2.size()) {
                    //         F_int(1) = upPerturbForce2[perturbIndex++];
                           
                    // }
                        // else {
                        //     F_int(1) = 0.0;
                        // }

                // // Used for recording perturbation force
                // VM2 X = robot->getEndEffPosition();
                // VM2 dX = robot->getEndEffVelocity();
                // VM2 F_cmd = VM2::Zero();

                // VM2 Xd, dXd;
                // MinJerk(A, C, trialMaxTime, running() - t0_toA, Xd, dXd);
                // // Eigen::Matrix2d K = Eigen::Matrix2d::Identity() * 300;
                // // Eigen::Matrix2d D = Eigen::Matrix2d::Identity() * 10;
                // // F_int = K * (A - C) + D * (dXd - dX);
                // F_int = impedance(Xd, X, dX, dXd);

                } else {                
                // After reaching target, stop perturbation
                    F_int.setZero();
            }
            } 

            else if (injectingLeft) {
        
                if (!reachedC && timeoutTrial==false) {
                        F_int(0) = F_const_left;
                        // spdlog::info("F_int(0) = {:.3f}", F_int(0));
                    }
                // if (perturbIndex < leftPerturbForce.size()) {
                //         F_int(0) = leftPerturbForce2[perturbIndex++];
                //     }}
 
                else {
                        F_int.setZero();
                }
            }

                //                 // Used for recording perturbation force
                // VM2 X = robot->getEndEffPosition();
                // VM2 dX = robot->getEndEffVelocity();
                // VM2 F_cmd = VM2::Zero();

                // VM2 B{0.15, 0.002};
                // VM2 Xd, dXd;
                // MinJerk(A, B, trialMaxTime+0.7, running() - t0_toA, Xd, dXd);
                // // Eigen::Matrix2d K = Eigen::Matrix2d::Identity() * 300;
                // // Eigen::Matrix2d D = Eigen::Matrix2d::Identity() * 10;
                // // F_int = K * (A - C) + D * (dXd - dX);
                // F_int = impedance(Xd, X, dX, dXd);
        // }}

            // -------------------- End internal force safety boundary -------------------------
            VM2 F_cmd = F_int + impedance(robot->getEndEffPosition(), X, dX);


            // const double x_min = 0.10;   // left boundary (m)
            // const double x_max = 0.55;   // left boundary (m)
            // const double k_wall = 1800.0; // wall stiffness N/m
            // const double d_wall = 50.0;  // wall damping N·s/m
            // const double y_max = 0.35;   // upper boundary (m)

            // // Left wall
            // if (X(0) < x_min) {
            //     // penetration depth (negative when past wall) (left wall)
            //     double pen = x_min - X(0);
            //     // Only apply when robot is "beyond" wall (pen > 0)
            //     if (pen > 0.0) {
            //         double F_wall_x = k_wall * pen - d_wall * dX(0); // push back rightward
            //         if (F_wall_x > 0.0) {
            //             F_cmd(0) += F_wall_x;  // add wall force
            //         }
            //     }
            // }

            // // Right wall
            // if (X(0) > x_max) {
            //     double pen = X(0) - x_max;
            //     if (pen > 0.0) {
            //         double F_wall_x = k_wall * pen + d_wall * dX(0);
            //         if (F_wall_x > 0.0) F_cmd(0) -= F_wall_x; // push leftward
            //     }
            // }

            // // Upper wall
            // if (X(1) > y_max) {
            //     double penY = X(1) - y_max;   // penetration depth (upper wall)
            //     if (penY > 0.0) {
            //         double F_wall_y = k_wall * penY + d_wall * dX(1);  
            //         // Only apply when pushing into the wall
            //         if (F_wall_y > 0.0) {
            //             F_cmd(1) -= F_wall_y;   // push downward
            //         }
            //     }
            // }

            applyForce(F_cmd);
            // --- Effort calculation based on commanded force (Standby-style) ---
            VM2 F_user = robot->getEndEffForce();  // sensor-measured interaction force
            const double Fu_norm = F_user.norm();
            const double vv = dX.norm();
            rawEffortIntegral += Fu_norm * vv;
            
            

            effortIntegral = 0.05*std::abs(rawEffortIntegral - baselineImpulseN);
            if(effortIntegral < 3)effortIntegral = 0;


            writeCSV(running(), X, dX, F_int, F_user, effortIntegral);


            if (iterations() % 1000 == 1) {
                spdlog::info(
                    "TRIAL status: t={:.3f}, n={:.3f}, effort={:.3f}, pLeft={:.3f}, mode={}, targetSucc={}, maxTrials={}",
                    (running() - trialStartTime),
                    n,
                    effortIntegral,
                    // probLeft,
                    currentTrialProb_,
                    meta_scoreMode,
                    meta_targetSucc,
                    meta_maxTrials
                );
            }

            VM2 X_end = robot->getEndEffPosition();   
            double dist_end = n;                      

            trialEndPositions_.push_back(X_end);      

            const double now = running();
            // if ((!sendPosOnlyOnTimeout_ || timeout) &&(lastTrpsT_ <0.0 || (now -lastTrpsT_) >=trpsMinInterval_)){
            if (timeout || reachedC && (lastTrpsT_ < 0.0 || (now - lastTrpsT_) >= trpsMinInterval_)) {
                
                if (machine && machine->UIserver) {
                    std::vector<double> q;
                    int curTrialIdx = (currentMode == V1_COUNT_SUCCESS) ? totalTrialsV1+1 : totalTrialsV2+1;
                    q.push_back((double)curTrialIdx);        
                    q.push_back(X_end(0));                   
                    q.push_back(X_end(1));                   
                    q.push_back(dist_end);                   
                    q.push_back(timeout ? 1.0 : 0.0);        
                    machine->UIserver->sendCmd("TRPS", q); 
                    spdlog::info("TRPPPP");
                    lastTrpsT_ = now;

                }


                {
                    std::ostringstream oss;
                    oss.setf(std::ios::fixed); oss.precision(3);
                    oss << "TRIAL_POS t=" << running()
                        << " end=(" << X_end(0) << "," << X_end(1) << ")"
                        << " dist=" << dist_end
                        << " timeout=" << (timeout ? 1 : 0);
                        spdlog::info("EndPosX={}, EndPosY={}", X_end(0), X_end(1));

                    sendUI_(oss.str());
                }
            }
            



            if (timeout) {
                // Scoring logic is now handled here directly
                double trialScore = 0;

                if (currentMode == V1_COUNT_SUCCESS) {
                    totalTrialsV1++;
                    if (reachedC) {
                        successfulTrials++;
                        trialScore = 1; // V1 score: 1 for success, 0 for fail
                    }
                    if (successfulTrials >= meta_targetSucc || totalTrialsV1 >= meta_maxTrials) finishedFlag = true;
                } else { // V2_EFFORT_DISTANCE
                    totalTrialsV2++;
                    const double S_max = 110.0;

                    const double baseScore = S_max - 210 * n - 0.5*effortIntegral;
                    
                    trialScore = baseScore;
                    totalScoreV2 += trialScore;
                    if (totalTrialsV2 >= meta_maxTrials) finishedFlag = true;
                }

                if (machine && machine->UIserver) {
                    std::ostringstream oss;
                    oss.setf(std::ios::fixed);
                    oss.precision(3);
                    oss << "TRIAL_END t=" << running() << " dur=" << tTrial
                        // << " reached=" << (reachedC ? 1 : 0)
                        << " reached=" << (reachedC ? 1 : 0)
                        << " dist=" << n
                        << " effort=" << effortIntegral
                        << " trialScore=" << trialScore
                        << " dir=" << (internalForce(0) < -1e-9 ? "SIDE" : (internalForce(1) > 1e-9 ? "UP" : "NONE"))
                        // << " pLeft=" << probLeft
                        << " pLeft=" << currentTrialProb_
                        << " mode=" << (currentMode == V1_COUNT_SUCCESS ? 1 : 2);
                    int curTrial = (currentMode == V1_COUNT_SUCCESS) ? totalTrialsV1 : totalTrialsV2;
                    oss << " cur_trial=" << curTrial
                        << " max_trial=" << meta_maxTrials;
                    if (currentMode == V1_COUNT_SUCCESS) {
                        oss << " v1_suc=" << successfulTrials
                            << " v1_tar=" << meta_targetSucc;
                    } else {
                        oss << " v2_sco=" << totalScoreV2;
                    }
                    std::string out = oss.str();
                    //sendUI_(out);
                    {
                        std::vector<double> p;
                        int dirCode = (internalForce(0) < -1e-9) ? -1 : ((internalForce(1) > 1e-9) ? 1 : 0);
                        int curTrial = (currentMode == V1_COUNT_SUCCESS) ? totalTrialsV1 : totalTrialsV2;

                        p.push_back(running());               // t
                        p.push_back(tTrial);                  // dur
                        p.push_back(reachedC ? 1.0 : 0.0);   // reached
                        p.push_back(n);                       // dist
                        p.push_back(effortIntegral);          // effort
                        p.push_back(trialScore);              // trialScore
                        p.push_back((double)(currentMode == V1_COUNT_SUCCESS ? 1 : 2)); // mode
                        p.push_back((double)curTrial);        // cur
                        p.push_back((double)meta_maxTrials);  // max
                        if (currentMode == V1_COUNT_SUCCESS) {
                            p.push_back((double)successfulTrials); // v1_suc
                            p.push_back((double)meta_targetSucc);  // v1_tar
                            p.push_back(0.0);                      // v2_sco placeholder
                        } else {
                            p.push_back(0.0);                      // v1_suc placeholder
                            p.push_back(0.0);                      // v1_tar placeholder
                            p.push_back(totalScoreV2);             // v2_sco
                        }
                        p.push_back((double)dirCode);              // dirCode（可选）
                        machine->UIserver->sendCmd("TREN", p);
                        spdlog::info("TREEEEEE");
                        std::ostringstream log;
                        log << "[TREN SEND] ("
                            << "t=" << p[0] << ", dur=" << p[1]
                            << ", reached=" << p[2] << ", dist=" << p[3]
                            << ", effort=" << p[4] << ", trialScore=" << p[5]
                            << ", mode=" << p[6] << ", cur=" << p[7]
                            << ", max=" << p[8]
                            << ", v1_suc=" << p[9] << ", v1_tar=" << p[10]
                            << ", v2_sco=" << p[11]
                            << ", dirCode=" << p.back() << ")";
                        spdlog::info("{}", log.str());
                        
                    }
                }
                if (!finishedFlag) {
                    // Optionally reset perturbation injection state
                    injectingUp = false;
                    injectingLeft = false;
                    // Stay in ProbMove but go back to WAIT_START to await explicit STRT.
                    currentPhase    = WAIT_START;
                    betweenTrials   = true;      // allow param changes between trials
                    initTrial       = true;     // prepare for a fresh TRIAL init when STRT arrives
                    inBandSince     = 0.0;      // reset hold timer
                    effortIntegral  = 0.0;      // reset effort accumulator for next trial
                    // reset preload state/buffer for next trial
                    waitBuf_.clear();
                    preloadSatisfied_ = false;
                    spdlog::info("TRIAL: finished (reachedC={}, timeout={}) -> WAIT_START. Awaiting STRT.", reachedC, timeout);
                    double trenSendTime_ = running();
                    // if ((running() - trenSendTime_) < 0.02 ) {
                    //      
                    // }   
                    return;  // important: exit now to avoid any further TRIAL computations this frame
                }


                // inside TRIAL, when timeout block sets up the next round
                // if (!finishedFlag) {
                //     injectingUp = false;
                //     injectingLeft = false;

                //     currentPhase    = TO_A;     // go recenter first
                //     initToA         = true;
                //     betweenTrials   = false;    // only becomes true when TO_A reaches A
                //     waitHoldLatched_= false;
                //     pendingStart    = false;    // optional: force a fresh STRT after recentering
                //     inBandSince     = 0.0;
                //     effortIntegral  = 0.0;
                //     waitBuf_.clear();
                //     preloadSatisfied_ = false;
                //     spdlog::info("TRIAL finished -> TO_A; will start next trial after reaching A");
                //     return;
                // }

            }
            break;
        }
    }
}

// Cleanup on ProbMove exit: zero forces, close CSVs, send session summary
void M2ProbMoveState::exitCode() {
    robot->setEndEffForceWithCompensation(VM2::Zero());
    waitHoldLatched_ = false;
    if (csv.is_open()) csv.close();
    closePreloadCSVs_();

    if (machine && machine->UIserver) {
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss.precision(3);
        oss << "SESSION_END mode=" << (currentMode == V1_COUNT_SUCCESS ? 1 : 2)
            << " V1_success=" << successfulTrials << " V1_trials=" << totalTrialsV1
            << " V2_score=" << totalScoreV2 << " V2_trials=" << totalTrialsV2;
        std::string out = oss.str();
        sendUI_(out);
        {
            std::vector<double> p;
            p.push_back((double)(currentMode == V1_COUNT_SUCCESS ? 1 : 2));
            p.push_back((double)successfulTrials);
            p.push_back((double)totalTrialsV1);
            p.push_back(totalScoreV2);
            p.push_back((double)totalTrialsV2);
            machine->UIserver->sendCmd("SESS", p);
            spdlog::info("SESSSSS");
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

// void M2ProbMoveState::decideInternalForceDirection() {
//     if (trialSchedule_.empty()) buildDeterministicSchedule();
//     int dirFlag = trialSchedule_[trialIdx_ % trialSchedule_.size()];
//     ++trialIdx_;
//     internalForce = (dirFlag < 0) ? VM2(-robotForceMagLeft, 0.0) : VM2(0.0, +robotForceMagUp);
//     injectingLeft = (internalForce(0) < -1e-9);
//     injectingUp   = (internalForce(1) >  1e-9);
//     spdlog::info("TRIAL direction (deterministic): {}", (dirFlag < 0 ? "SIDE" : "UP"));
// }

void M2ProbMoveState::decideInternalForceDirection() {
    if (trialSchedule_.empty()) {
        if (randomizeOrBlock == 1) buildDeterministicSchedule_random();
        else buildDeterministicSchedule();
    }
    if (trialSchedule_.empty()) {
        spdlog::error("trialSchedule_ empty after build; fallback to UP");
        internalForce = VM2(0.0, +robotForceMagUp);
        injectingLeft = false; injectingUp = true;
        currentTrialProb_ = probLeft; currentTrialDir_ = +1;
        return;
    }

    size_t idx = trialIdx_ % trialSchedule_.size();
    int dirFlag = trialSchedule_[idx];
    currentTrialProb_ = (idx < trialProb_.size()) ? trialProb_[idx] : probLeft;
    currentTrialDir_  = dirFlag;
    ++trialIdx_;

    internalForce = (dirFlag < 0) ? VM2(-robotForceMagLeft, 0.0)
                                  : VM2(0.0, +robotForceMagUp);
    injectingLeft = internalForce(0) < -1e-9;
    injectingUp   = internalForce(1) >  1e-9;
    spdlog::info("TRIAL direction ({}): {} p={:.3f}",
                 (randomizeOrBlock == 1 ? "random" : "deterministic"),
                 (dirFlag < 0 ? "SIDE" : "UP"),
                 currentTrialProb_);
}




// void M2ProbMoveState::decideInternalForceDirection() {
//     // 若 schedule 为空，按开关选择构建方式
//     if (trialSchedule_.empty()) {
//         size_t idx = trialIdx_ % trialSchedule_.size();
//         int dirFlag = trialSchedule_[idx];

//         if (randomizeOrBlock == 1) {
//             buildDeterministicSchedule_random();
//             currentTrialProb_ = trialProb_[idx];
//         } else {
//             buildDeterministicSchedule();
//             currentTrialProb_ = probLeft;
//         }
//         // currentTrialProb_ = (randomizeOrBlock == 1) ? trialProb_[idx] : probLeft;
//         currentTrialDir_  = dirFlag;   // -1 或 +1
//         ++trialIdx_;
//         internalForce = (dirFlag < 0) ? VM2(-robotForceMagLeft, 0.0) : VM2(0.0, +robotForceMagUp);
//         injectingLeft = (internalForce(0) < -1e-9);
//         injectingUp   = (internalForce(1) >  1e-9);
//         spdlog::info("TRIAL direction ({})", (dirFlag < 0 ? "SIDE" : "UP"));
//     }   
// }


void M2ProbMoveState::resetToAPlan(const VM2& Xnow) {
    Xi = Xnow;
    t0_toA = running();
    inBandSince = 0.0;
}

void M2ProbMoveState::resetToAIntegrators() {
    iErrToA.setZero();
}

void M2ProbMoveState::dumpScheduleCSV_(const std::vector<int>& dir,
                                       const std::vector<double>& prob) {
    const std::string sid   = (machine && !machine->sessionId.empty()) ? machine->sessionId : "UNSET";
    const std::string fname = std::string("logs/Schedule_") + sid + ".csv";
    std::ofstream f(fname, std::ios::out);
    if (!f.is_open()) {
        spdlog::error("Failed to open schedule CSV: {}", fname);
        return;
    }
    f << "trialIndex,dir,prob\n";
    for (size_t i = 0; i < dir.size(); ++i) {
        double p = (i < prob.size()) ? prob[i] : probLeft;
        f << i+1 << "," << dir[i] << "," << p << "\n";
    }
    f.close();
}



void M2ProbMoveState::buildDeterministicSchedule() {
    trialSchedule_.clear();
    // const int seed = 1111111;
    // const int seed = 1456070;
    // const int seed = 1661471;
    const int total = 10;
    
    const double eps = 1e-3;
    // 0% case：all UP(+1)
    if (probLeft <= eps) {                 
        trialSchedule_.assign(total, +1);
        trialIdx_ = 0;
        
        return;
    }
    // 100% case：all LEFT(-1)  
    if (probLeft >= 1.0 - eps) {           
        trialSchedule_.assign(total, -1);
        trialIdx_ = 0;
        
        return;
    }
    
    int leftCount = static_cast<int>(std::round(probLeft * static_cast<double>(total)));
    leftCount = std::max(1, std::min(total - 1, leftCount));

    
    std::vector<int> schedule(total, +1);
    for (int i = 0; i < leftCount; ++i) schedule[i] = -1;

    // seed 
    std::mt19937_64 rng(seed);
    spdlog::info("[SCHEDULE] Building seeded schedule with seed={}", seed);

    auto is_strict_alternating = [&](const std::vector<int>& v) -> bool {
        
        for (int i = 0; i < total; ++i) {
            if (v[i] == v[(i + 1) % total]) return false;
        }
        return true;
    };

    
    const bool fifty_fifty = (leftCount * 2 == total);
    int attempts = 0;
    const int max_attempts = 64; 

    do {
        std::shuffle(schedule.begin(), schedule.end(), rng);
        attempts++;
        
        if (!(fifty_fifty && is_strict_alternating(schedule))) break;
    } while (attempts < max_attempts);

    trialSchedule_ = std::move(schedule);
    trialIdx_ = 0;

    trialProb_.assign(trialSchedule_.size(), probLeft);

    spdlog::info("[SCHEDULE] Built seeded schedule (total={}, leftCount={}, seed={}, attempts={})",
                 total, leftCount, seed, attempts);
}



void M2ProbMoveState::buildDeterministicSchedule_random() {


    trialSchedule_.clear();
    trialProb_.clear();
    const int totalTrials = 100;          // 总 trial
    const int blockSize = 10;             // 每个 block trial 数
    const std::vector<double> probList{0.1, 0.3, 0.5, 0.7, 0.9};
    const int n = probList.size();


    // 特殊 BlockID 处理
    if (BlockID == 0) {
        trialSchedule_.assign(blockSize, +1);  // 全向前
        trialProb_.assign(blockSize, 0.0);
        return;
    }
    if (BlockID == 100) {
        trialSchedule_.assign(blockSize, -1);  // 全向左
        trialProb_.assign(blockSize, 1.0);
        return;
    }


    if (fullDir_.empty()) {

        // Step 1：生成每个概率对应 trial
        std::vector<int> trialScheduleFull;
        std::vector<double> trialProbFull;
        trialScheduleFull.reserve(totalTrials);
        trialProbFull.reserve(totalTrials);

        for (double p : probList) {
            int numTrials = totalTrials / n;                 // 每个概率出现次数
            int numLeft = static_cast<int>(std::round(p * numTrials));
            int numForward = numTrials - numLeft;

            for (int i = 0; i < numLeft; ++i) {
                trialScheduleFull.push_back(-1);
                trialProbFull.push_back(p);
            }
            for (int i = 0; i < numForward; ++i) {
                trialScheduleFull.push_back(+1);
                trialProbFull.push_back(p);
            }
        }

        // Step 2：随机打乱整个序列
        std::mt19937_64 rng(seed);
        std::vector<size_t> indices(trialScheduleFull.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::shuffle(indices.begin(), indices.end(), rng);

        std::vector<int> trialScheduleShuffled;
        std::vector<double> trialProbShuffled;
        trialScheduleShuffled.reserve(totalTrials);
        trialProbShuffled.reserve(totalTrials);

        for (size_t idx : indices) {
            trialScheduleShuffled.push_back(trialScheduleFull[idx]);
            trialProbShuffled.push_back(trialProbFull[idx]);
        }

        fullDir_  = trialScheduleShuffled;
        fullProb_ = trialProbShuffled;

        dumpScheduleCSV_(trialScheduleShuffled, trialProbShuffled);
    }
    else {
        spdlog::info("[SCHEDULE] Using cached full schedule for BlockID={}", BlockID);
    }
    // Step 3：根据 BlockID 取当前 block
    int numBlocks = totalTrials / blockSize;
    if (BlockID < 1 || BlockID > numBlocks) {
        BlockID = 1;
    }
    int startIdx = (BlockID - 1) * blockSize;

    trialSchedule_.assign(fullDir_.begin() + startIdx,
                          fullDir_.begin() + startIdx + blockSize);
    trialProb_.assign(fullProb_.begin() + startIdx,
                       fullProb_.begin() + startIdx + blockSize);

    spdlog::info("[SCHEDULE] BlockID={}, blockSize={}, seed={}", BlockID, blockSize, seed);
}



// Open logs/M2ProbMove_<session>.csv and write header if new
void M2ProbMoveState::openCSV() {
    // csv.open("logs/M2ProbMoveState.csv", std::ios::out | std::ios::app);
    const std::string sid = (machine && !machine->sessionId.empty()) ? machine->sessionId : std::string("UNSET");

    const std::string fname = std::string("logs/M2ProbMove_") + sid + ".csv";

    csv.open(fname, std::ios::out | std::ios::app);
    if (!csv.is_open()) {
        spdlog::error("Failed to open ProbMove CSV: {}", fname);
        return;
    }
    if (csv.tellp() == 0) {
        // MODIFIED: Added effort to CSV header
        csv << "time,sys_time,session_id,pos_x,pos_y,vel_x,vel_y,internal_fx,internal_fy,user_fx,user_fy,prob_left,trial_dir,target_succ,max_trials,effort\n";
    }
}

// MODIFIED: Added effort to CSV logging
// Append one row to ProbMove CSV (incl. effort)
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
        << currentTrialProb_ << ","
        << currentTrialDir_ << ","
        << meta_targetSucc << ","
        << meta_maxTrials << ","
        << effort << "\n";
}

// Clamp and send Cartesian force to robot with compensation
// void M2ProbMoveState::applyForce(const VM2& F) {
//     VM2 F_clamped = F;
//     for (int i=0; i<2; ++i) {
//         F_clamped(i) = clamp_compat(F_clamped(i), -forceSaturation, forceSaturation);
//     }
//     robot->setEndEffForceWithCompensation(F_clamped, true);
// }

void M2ProbMoveState::applyForce(const VM2& F) {

    // const double x_min = 0.10;   // left boundary (m)
    // const double x_max = 0.55;   // left boundary (m)
    // const double k_wall = 1800.0; // wall stiffness N/m
    // const double d_wall = 50.0;  // wall damping N·s/m
    // const double y_max = 0.35;   // upper boundary (m)

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



// --- CSV perturbation force loading helpers ---

// Load hard-coded perturbation force sequences (UP and LEFT)
void M2ProbMoveState::loadPerturbationForces() {


    for(int i=0;i<500;i++){
        upPerturbForce2.push_back(F_const_up);
        if (F_const_left > 0 && trialIsLeft){ F_const_left = -F_const_left;}
        else if (F_const_left <0 && !trialIsLeft){ F_const_left = -F_const_left;}
        leftPerturbForce2.push_back(F_const_left);
    }

//     upPerturbForce = {
//     125.850499, 125.850499, 125.850499, 125.850423, 123.660521, 123.127617, 122.706884, 122.506548, 122.409786, 122.137488,
//     122.137488, 122.137488, 122.810319, 122.810319, 122.176493, 122.176493, 121.884557, 121.884557, 121.601987, 121.601987,
//     120.597282, 120.597282, 119.863811, 119.863811, 119.785767, 119.430086, 119.411539, 118.445341, 118.445341, 118.015536,
//     117.929742, 117.497447, 116.679141, 116.725741, 116.611514, 115.774205, 115.774205, 114.245048, 114.774208, 113.860215,
//     113.860215, 113.586611, 112.812911, 112.812911, 112.073014, 111.680344, 111.680344, 110.680287, 109.942276, 109.942276,
//     108.260211, 107.260211, 107.060031, 106.848285, 106.848285, 105.044365, 104.042018, 103.053339, 102.953182, 101.960464,
//     101.960464, 101.880368, 100.743816, 100.643143, 99.310301, 98.479064, 97.497013, 97.486255, 96.177043, 96.118286,
//     94.118286, 94.118286, 93.560969, 92.407682, 92.406824, 91.418255, 90.742504, 90.074504, 89.417555, 88.020984,
//     88.020984, 86.842102, 86.842102, 85.902367, 85.902367, 85.178298, 85.178298, 82.812296, 82.812296, 80.302874,
//     80.302874, 79.560315, 78.556171, 77.596611, 77.596611, 75.922063, 75.922063, 74.349256, 74.349256, 72.687542,
//     72.687542, 71.030276, 70.730026, 69.417727, 69.417727, 67.667426, 66.812788, 65.017652, 65.017652, 63.445737,
//     63.445737, 62.478002, 62.478002, 61.260362, 60.260017, 59.421691, 58.748167, 58.048064, 57.123456, 56.421314,
//     56.231864, 55.471065, 55.471065, 55.100134, 53.780213, 53.780213, 52.620867, 52.490075, 50.790075, 49.108843,
//     49.108843, 47.342424, 46.969012, 46.969012, 45.507118, 45.507118, 44.236965, 44.236965, 42.742805, 41.804089,
//     41.804089, 40.495614, 39.395518, 39.395518, 37.674652, 36.066039, 36.066039, 34.170187, 34.270976, 33.549147,
//     32.456712, 31.017424, 30.812816, 30.812816, 29.551712, 28.659566, 28.659566, 27.895075, 26.810847, 25.810847,
//     25.014744, 24.014744, 23.619207, 22.619207, 21.630034, 20.913684, 20.913684, 19.845225, 18.845225, 17.601359,
//     17.601359, 16.403987, 15.704544, 15.376662, 14.478996, 13.021862, 12.681786, 12.181724, 11.218253, 11.215838
// };

    // upPerturbForce = {
    //     27.887317, 25.231683, 24.887317, 25.576049, 25.920415, 25.920415, 26.264781, 25.920415, 25.576049, 25.920415,
    //     25.576049, 25.576049, 25.920415, 25.920415, 25.576049, 25.576049, 26.609147, 25.920415, 25.231683, 25.920415,
    //     26.264781, 26.609147, 25.920415, 25.920415, 25.576049, 26.264781, 26.264781, 25.920415, 25.920415, 26.264781,
    //     25.920415, 25.920415, 26.609147, 26.609147, 25.920415, 26.264781, 25.576049, 25.576049, 25.576049, 25.231683,
    //     26.264781, 25.920415, 25.920415, 25.920415, 25.231683, 25.920415, 25.576049, 25.576049, 25.576049, 25.576049,
    //     25.231683, 25.231683, 26.609147, 25.576049, 25.920415, 25.920415, 25.576049, 25.576049, 26.264781, 25.231683,
    //     26.609147, 25.576049, 26.264781, 26.609147, 25.920415, 25.920415, 25.576049, 25.920415, 25.920415, 25.920415,
    //     25.920415, 26.264781, 25.920415, 25.920415, 25.920415, 25.920415, 26.264781, 26.264781, 25.231683, 25.920415,
    //     26.264781, 26.609147, 26.264781, 26.609147, 25.920415, 26.609147, 25.920415, 26.264781, 26.264781, 26.264781,
    //     25.920415, 26.264781, 25.920415, 25.576049, 25.576049, 25.576049, 26.264781, 26.264781, 25.576049, 25.920415,
    //     26.609147, 26.264781, 25.920415, 25.231683, 26.609147, 25.920415, 26.953512, 25.576049, 26.264781, 26.264781,
    //     25.920415, 26.264781, 25.576049, 26.264781, 26.264781, 25.920415, 26.609147, 26.609147, 25.920415, 25.920415,
    //     26.953512, 26.264781, 25.920415, 26.264781, 25.576049, 26.609147, 26.264781, 25.920415, 25.920415, 25.576049,
    //     26.609147, 26.264781, 26.264781, 26.264781, 26.609147, 26.609147, 26.264781, 26.953512, 26.609147, 25.576049,
    //     25.920415, 25.576049, 26.264781, 26.264781, 25.920415, 26.264781, 26.609147, 26.609147, 25.920415, 26.953512,
    //     26.609147, 26.609147, 25.231683, 26.953512, 25.920415, 26.264781, 26.609147, 26.264781, 26.953512, 26.264781,
    //     26.264781, 26.953512, 25.920415, 25.920415, 26.264781, 25.920415, 26.953512, 25.576049, 26.609147, 25.576049,
    //     26.953512, 26.264781, 25.920415, 26.609147, 25.231683, 26.264781, 25.576049, 26.264781, 25.920415, 26.264781,
    //     26.953512, 25.920415, 26.264781, 25.920415, 26.264781, 25.231683, 26.609147, 26.264781, 25.920415, 26.953512,
    //     26.609147, 27.297878, 26.609147, 26.264781, 26.609147, 26.264781, 26.953512, 26.264781, 26.953512, 25.920415,
    //     26.609147, 26.953512, 26.609147, 26.953512, 27.297878, 26.264781, 26.264781, 25.920415, 26.264781, 26.609147,
    //     26.953512, 26.953512, 26.953512, 26.609147, 25.576049, 27.297878, 25.920415, 26.609147, 26.264781, 26.264781,
    //     26.609147, 26.264781, 26.609147, 26.609147, 26.953512, 25.920415, 25.576049, 26.609147, 26.609147, 26.264781,
    //     26.264781, 26.264781, 26.264781, 26.953512, 25.920415, 26.264781, 25.920415, 26.264781, 25.920415, 26.609147,
    //     26.264781, 26.609147, 25.920415, 26.953512, 26.953512, 26.953512, 26.264781, 25.920415, 26.609147, 26.609147,
    //     25.920415, 25.576049, 26.609147, 26.609147, 25.576049, 27.297878, 25.576049, 26.609147, 26.264781, 26.264781,
    //     25.920415, 25.920415, 26.609147, 26.264781, 26.609147, 25.920415, 25.920415, 26.264781, 26.264781, 26.609147,
    //     26.953512, 25.920415, 25.920415, 25.920415, 25.920415, 26.609147, 25.576049, 25.920415, 25.920415, 25.920415,
    //     26.609147, 25.920415, 26.609147, 26.609147, 26.609147, 26.953512, 25.920415, 25.920415, 26.264781, 26.264781,
    //     25.920415, 26.264781, 25.920415, 26.609147, 26.609147, 26.609147, 26.609147, 26.953512, 26.609147, 26.609147,
    //     25.576049, 26.609147, 25.920415, 26.609147, 26.609147, 26.264781, 26.264781, 25.576049, 25.920415, 25.920415,
    //     25.920415, 26.953512, 26.609147, 26.609147, 26.264781, 25.920415, 26.264781, 25.920415, 26.264781, 26.953512,
    //     26.264781, 25.920415, 26.609147, 26.609147, 25.920415, 26.609147, 26.953512, 25.920415, 25.920415, 25.920415,
    //     25.920415, 26.609147, 25.920415, 26.264781, 26.264781, 26.264781, 26.609147, 25.920415, 25.920415, 26.264781,
    //     26.264781, 26.264781, 25.920415, 26.609147, 26.264781, 26.264781, 26.264781, 26.264781, 26.264781, 26.609147,
    //     26.609147, 25.920415, 25.920415, 26.264781, 27.297878, 26.609147, 25.576049, 26.264781, 26.609147, 25.920415,
    //     25.576049, 25.920415, 26.609147, 27.297878, 26.609147, 26.264781, 25.920415, 26.609147, 25.576049, 25.920415,
    //     25.920415, 25.920415, 25.920415, 25.920415, 26.609147, 25.920415, 25.231683, 26.264781, 25.576049, 25.920415,
    //     25.920415, 26.264781, 26.264781, 25.576049, 26.264781, 25.576049, 25.920415, 26.609147, 25.920415, 26.264781,
    //     25.576049, 26.264781, 26.609147, 26.264781, 25.576049, 26.264781, 25.920415, 25.920415, 25.920415, 25.576049,
    //     25.920415, 25.920415, 26.609147, 25.576049, 26.264781, 25.920415, 26.264781, 25.920415, 25.920415, 26.264781,
    //     25.920415, 25.920415, 25.231683, 26.264781, 25.576049, 25.920415, 25.920415, 26.264781, 26.264781, 25.576049,
    //     25.576049, 25.576049, 25.920415, 26.264781, 25.231683, 26.609147, 26.609147, 26.264781, 25.920415, 25.920415,
    //     26.264781, 26.264781, 26.609147, 25.576049, 25.920415, 25.920415, 26.609147, 26.264781, 25.576049, 25.231683,
    //     25.231683, 26.264781, 26.264781, 25.920415, 26.609147, 25.576049, 25.920415, 25.920415, 25.920415, 25.920415,
    //     26.264781, 26.264781, 25.576049, 25.920415, 25.576049, 25.231683, 25.920415, 26.264781, 25.920415, 26.953512,
    //     26.609147, 25.920415, 25.920415, 26.264781, 26.953512, 26.264781, 26.264781, 26.264781, 25.576049, 26.609147,
    //     26.264781, 26.264781, 26.264781, 26.264781, 25.920415, 25.576049, 25.920415, 25.920415, 25.231683, 25.576049,
    //     26.264781, 25.231683, 26.609147, 26.264781, 25.920415, 26.264781, 19.033098, 18.688732
    // };
    // leftPerturbForce = {
    //     -12.231683, -12.920415, -12.576049, -12.920415, -12.576049, -12.920415, -12.920415, -12.920415, -13.264781, -12.920415,
    //     -13.264781, -13.264781, -13.264781, -13.264781, -12.920415, -12.920415, -12.920415, -13.264781, -12.920415, -12.576049,
    //     -12.920415, -12.920415, -12.576049, -11.887317, -12.576049, -13.264781, -12.231683, -12.920415, -12.920415, -12.920415,
    //     -12.576049, -13.953512, -13.264781, -12.920415, -12.920415, -12.576049, -12.231683, -12.576049, -12.920415, -12.920415,
    //     -12.231683, -13.264781, -12.920415, -13.953512, -13.264781, -13.609147, -13.264781, -13.264781, -13.609147, -13.264781,
    //     -12.920415, -13.264781, -13.264781, -13.609147, -13.264781, -12.231683, -12.231683, -12.576049, -12.920415, -12.920415,
    //     -13.264781, -13.264781, -12.231683, -12.576049, -12.920415, -13.264781, -12.920415, -12.231683, -13.264781, -13.609147,
    //     -12.576049, -13.264781, -13.264781, -12.920415, -13.264781, -13.609147, -13.609147, -13.264781, -12.920415, -13.264781,
    //     -13.264781, -13.264781, -13.264781, -12.920415, -12.920415, -12.920415, -13.264781, -13.264781, -13.264781, -12.920415,
    //     -13.609147, -13.264781, -13.264781, -13.264781, -13.264781, -13.609147, -12.576049, -13.264781, -13.264781, -13.264781,
    //     -13.264781, -12.920415, -12.920415, -12.920415, -12.920415, -12.576049, -13.609147, -13.264781, -13.609147, -12.920415,
    //     -13.953512, -12.920415, -13.264781, -12.920415, -13.264781, -13.609147, -13.609147, -13.609147, -13.264781, -13.953512,
    //     -13.264781, -13.264781, -13.264781, -13.953512, -13.264781, -13.264781, -13.264781, -13.953512, -13.953512, -13.953512,
    //     -13.264781, -13.264781, -13.609147, -13.264781, -13.264781, -13.609147, -12.920415, -12.576049, -12.920415, -13.264781,
    //     -13.264781, -13.264781, -13.264781, -13.609147, -13.609147, -12.920415, -12.920415, -13.264781, -13.264781, -12.920415,
    //     -13.609147, -13.609147, -13.609147, -13.264781, -12.920415, -13.609147, -13.264781, -13.264781, -13.609147, -13.264781,
    //     -13.264781, -12.576049, -12.920415, -13.609147, -13.609147, -13.609147, -13.264781, -12.576049, -12.920415, -13.953512,
    //     -13.264781, -12.920415, -12.920415, -12.920415, -13.609147, -12.920415, -13.264781, -13.264781, -12.920415, -13.609147,
    //     -13.609147, -13.609147, -13.264781, -13.264781, -13.264781, -13.609147, -12.920415, -13.953512, -13.264781, -13.609147,
    //     -13.609147, -13.264781, -13.953512, -13.264781, -13.264781, -12.920415, -12.920415, -13.609147, -14.297878, -13.264781,
    //     -13.264781, -12.920415, -13.264781, -13.264781, -13.609147, -12.576049, -13.264781, -12.576049, -13.609147, -12.920415,
    //     -13.264781, -13.264781, -13.264781, -13.609147, -13.264781, -13.264781, -14.98661, -13.264781, -12.920415, -12.920415,
    //     -12.920415, -13.264781, -12.920415, -13.609147, -13.264781, -12.576049, -12.920415, -12.920415, -12.576049, -13.264781,
    //     -13.609147, -13.609147, -13.609147, -13.609147, -13.609147, -13.264781, -13.264781, -13.609147, -12.920415, -13.609147,
    //     -13.264781, -13.609147, -13.264781, -13.609147, -13.264781, -13.953512, -13.264781, -13.264781, -13.609147, -13.264781,
    //     -12.576049, -14.297878, -13.264781, -13.609147, -12.920415, -13.953512, -13.264781, -13.609147, -13.953512, -13.609147,
    //     -12.920415, -13.609147, -13.264781, -13.609147, -13.264781, -13.264781, -13.264781, -13.609147, -13.609147, -13.264781,
    //     -13.264781, -13.264781, -12.920415, -12.576049, -13.264781, -13.609147, -13.953512, -13.264781, -13.264781, -12.920415,
    //     -13.609147, -13.264781, -13.264781, -13.264781, -13.264781, -13.264781, -12.920415, -12.920415, -14.297878, -13.264781,
    //     -13.264781, -13.264781, -13.609147, -13.609147, -13.609147, -13.264781, -13.264781, -13.264781, -13.264781, -13.264781,
    //     -13.264781, -12.576049, -13.264781, -13.264781, -13.264781, -13.609147, -13.264781, -12.920415, -12.920415, -13.264781,
    //     -12.920415, -13.264781, -13.264781, -13.609147, -13.264781, -12.920415, -13.264781, -13.264781, -13.609147, -13.264781,
    //     -13.953512, -13.609147, -12.920415, -13.264781, -13.264781, -13.609147, -13.264781, -13.609147, -13.264781, -13.264781,
    //     -13.264781, -13.264781, -13.953512, -12.920415, -13.609147, -13.264781, -12.920415, -13.953512, -13.264781, -12.920415,
    //     -12.920415, -13.264781, -13.609147, -13.953512, -13.609147, -13.953512, -13.609147, -12.920415, -13.609147, -13.609147,
    //     -13.264781, -13.953512, -13.609147, -13.609147, -13.264781, -13.609147, -13.264781, -13.953512, -12.231683, -12.576049,
    //     -12.920415, -12.920415, -13.264781, -13.264781, -13.609147, -12.920415, -13.264781, -12.920415, -13.953512, -12.920415,
    //     -13.264781, -13.264781, -12.576049, -13.609147, -13.264781, -13.264781, -13.264781, -12.576049, -13.264781, -13.609147,
    //     -13.264781, -13.264781, -13.264781, -13.609147, -12.920415, -13.609147, -13.609147, -13.264781, -13.264781, -13.264781,
    //     -13.264781, -13.609147, -13.264781, -12.920415, -12.920415, -13.264781, -12.576049, -13.609147, -13.264781, -13.953512,
    //     -12.920415, -13.264781, -13.609147, -13.264781, -13.264781, -13.953512, -13.609147, -12.920415, -13.609147, -12.920415,
    //     -13.264781, -13.264781, -13.609147, -13.264781, -12.920415, -12.920415, -12.576049, -12.231683, -12.576049, -12.920415,
    //     -13.953512, -13.609147, -13.264781, -12.576049, -12.920415, -12.920415, -13.264781, -12.920415, -12.920415, -12.920415,
    //     -12.920415, -13.264781, -13.609147, -13.264781, -12.576049, -12.576049, -12.920415, -13.264781, -13.264781, -13.264781,
    //     -13.264781, -12.920415, -12.920415, -12.920415, -12.576049, -12.576049, -12.920415, -13.609147, -12.576049, -13.264781,
    //     -12.576049, -11.887317, -13.264781, -13.609147, -12.920415, -12.576049, -12.576049, -12.920415, -12.920415, -12.920415,
    //     -13.264781, -12.920415, -12.576049, -12.576049, -12.576049, -12.920415, -13.264781, -13.264781, -12.920415, -12.576049,
    //     -12.920415, -13.264781, -12.920415, -13.264781, -12.576049, -12.231683, -12.920415, -12.920415, -13.264781, -13.264781,
    //     -12.576049, -12.920415, -13.264781, -12.920415, -13.264781, -12.920415, -12.920415, -12.920415, -13.264781, -12.920415,
    //     -12.920415, -12.576049, -12.576049, -12.576049, -12.920415, -12.231683, -12.576049, -13.609147, -12.576049, -13.264781,
    //     -12.576049, -12.920415, -13.264781, -12.576049, -11.887317, -12.231683, -12.920415, -13.264781, -13.609147, -13.264781,
    //     -12.576049, -12.576049, -13.264781, -13.264781, -13.953512, -13.264781, -12.920415, -13.264781, -12.920415, -13.264781,
    //     -13.264781, -12.231683, -12.920415, -12.576049, -12.920415, -13.264781, -12.920415, -13.264781, -12.920415, -12.920415,
    //     -12.920415, -13.264781, -13.264781, -12.920415, -12.920415, -6.033098
    // };


    spdlog::info("[STATIC] Loaded {} up and {} left perturbation samples (hard-coded).", upPerturbForce.size(), leftPerturbForce.size());
}

// --- Preload CSV helpers ---
// void M2ProbMoveState::openPreloadCSVs_() {
//     const std::string sid = (machine && !machine->sessionId.empty()) ? machine->sessionId : std::string("UNSET");
//     const std::string fwin = std::string("logs/PreloadWindow_") + sid + ".csv";
//     const std::string ftag = std::string("logs/TrialTags_") + sid + ".csv";

//     preloadWinCsv_.open(fwin, std::ios::out | std::ios::app);
//     if (preloadWinCsv_.is_open() && preloadWinCsv_.tellp() == 0) {
//         preloadWinCsv_ << "time,sys_time,session_id,mode,trial,fx,fy,pos_x,pos_y,vel_x,vel_y,threshold,window_s\n";
//     }
//     trialTagsCsv_.open(ftag, std::ios::out | std::ios::app);
//     if (trialTagsCsv_.is_open() && trialTagsCsv_.tellp() == 0) {
//         trialTagsCsv_ << "time,sys_time,session_id,mode,trial,preload,threshold,window_s,fx_min_window\n";
//     }
// }


// --- Preload CSV helpers ---
void M2ProbMoveState::openPreloadCSVs_() {
    const std::string sid = (machine && !machine->sessionId.empty())
                                ? machine->sessionId
                                : std::string("UNSET");

    const std::string fname = std::string("logs/Preload_") + sid + ".csv";

    preloadCsv_.open(fname, std::ios::out | std::ios::app);

    if (preloadCsv_.is_open() && preloadCsv_.tellp() == 0) {
        preloadCsv_ << "time,sys_time,session_id,mode,trial,"
                    << "fx,fy,pos_x,pos_y,vel_x,vel_y,prob_left,"
                    << "preload_flag,trial_dir\n";
    }
}



void M2ProbMoveState::closePreloadCSVs_() {
    // if (preloadWinCsv_.is_open()) preloadWinCsv_.close();
    // if (trialTagsCsv_.is_open())  trialTagsCsv_.close();
    if (preloadCsv_.is_open())  preloadCsv_.close();
}



double M2ProbMoveState::computeFxMin_(double tNow) {
    const double tMin = tNow - preloadWindowSec_;
    double fx_min = std::numeric_limits<double>::infinity();

    for (const auto& s : waitBuf_) {
        if (s.t < tMin) continue;
        fx_min = std::min(fx_min, s.force(0));
    }
    if (!std::isfinite(fx_min))
        fx_min = 0.0;
    return fx_min;
}



void M2ProbMoveState::writePreloadWindow_(int trialIdxForMode,
                                          double tNow, int mode,
                                          bool preloadFlag)
{
    if (!preloadCsv_.is_open()) return;

    const double sys_t = system_time_sec();
    const std::string sid = (machine ? machine->sessionId : std::string("UNSET"));
    // const int mode = (currentMode == V1_COUNT_SUCCESS ? 1 : 2);
    const double tMin = tNow - preloadWindowSec_;
    const double fx_min = computeFxMin_(tNow);

    //  Write all samples in the preload window
    for (const auto& s : waitBuf_) {
        if (s.t < tMin) continue;

        preloadCsv_ << std::fixed << std::setprecision(6)
            << s.t << "," << sys_t << "," << sid << ","
            << mode << "," << (trialIdxForMode+1) << ","
            << s.force(0) << "," << s.force(1) << ","
            << s.pos(0) << "," << s.pos(1) << ","
            << s.vel(0) << "," << s.vel(1) << ","
            // << probLeft << ","
            << currentTrialProb_ << ","
            // << preloadThresholdN_ << "," << preloadWindowSec_ << ","
            << (preloadFlag ? 1 : 0)       // extra: preload success flag
            // << ","<< fx_min 
            << currentTrialDir_ << ","  // extra: trial direction
            << "\n";                    // extra: fx_min
    }
}

// void M2ProbMoveState::writePreloadWindow_(int trialIdxForMode, double tNow) {
//     if (!preloadWinCsv_.is_open()) return;
//     const double sys_t = system_time_sec();
//     const std::string sid = (machine ? machine->sessionId : std::string("UNSET"));
//     const int mode = (currentMode == V1_COUNT_SUCCESS ? 1 : 2);
//     const double tMin = tNow - preloadWindowSec_;
//     for (const auto& s : waitBuf_) {
//         if (s.t < tMin) continue;
//         preloadWinCsv_ << std::fixed << std::setprecision(6)
//             << s.t << "," << sys_t << "," << sid << ","
//             << mode << "," << trialIdxForMode << ","
//             << s.force(0) << "," << s.force(1) << ","
//             << s.pos(0) << "," << s.pos(1) << ","
//             << s.vel(0) << "," << s.vel(1) << ","
//             // << preloadThresholdN_ << "," << preloadWindowSec_ << "\n";
//             << "\n";
//     }
// }

// void M2ProbMoveState::writeTrialTag_(int trialIdxForMode, int mode, bool preloadFlag, double tNow) {
//     if (!trialTagsCsv_.is_open()) return;
//     const double sys_t = system_time_sec();
//     const std::string sid = (machine ? machine->sessionId : std::string("UNSET"));
//     const double tMin = tNow - preloadWindowSec_;
//     double fx_min = std::numeric_limits<double>::infinity();
//     for (const auto& s : waitBuf_) {
//         if (s.t < tMin) continue;
//         fx_min = std::min(fx_min, s.force(0));
//     }
//     if (!std::isfinite(fx_min)) fx_min = 0.0;
//     trialTagsCsv_ << std::fixed << std::setprecision(6)
//         << tNow << "," << sys_t << "," << sid << ","
//         << mode << "," << trialIdxForMode << ","
//         << (preloadFlag ? 1 : 0) 
//         // << "," << preloadThresholdN_ << "," << preloadWindowSec_ << ","<< fx_min 
//         << "\n";
// }

