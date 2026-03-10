using UnityEngine;
using UnityEngine.SceneManagement;

/* 
    This component acts as a bridge between the CORC M2 robot and the Unity rover control, 
    allowing to drive the rover using M2 handle input and send realtime feedback forces to 
    M2 based on the rover state. 
    The main logic is in Update() where we read M2 state, compute the desired steering and 
    feedback forces, and send commands to both M2 and the rover accordingly.   
*/

namespace CORC.Demo 
{
    public class M2RoverBridge : MonoBehaviour
    {
        [Header("Refs")]
        public CORC.CORCM2 m2;
        public RoverHandler rover;
        public M2WorldFollower worldFollower;
        public Transform roverTransform;
        public Rigidbody roverRigidbody;
        public float m2CenterX = 0.32f; // The M2 handle X position that corresponds to the center reference (point A).

        public enum UnityDriveMode
        {
            Mode1_Keyboard = 1,
            Mode2_M2 = 2
        }

        [Header("Unity Mode")]
        public UnityDriveMode unityMode = UnityDriveMode.Mode1_Keyboard;
        bool trialActive = false;
        [Tooltip("1=V1_HRI, 2=V2_PHRI")]
        public int hriModeCode = 2;
        [Tooltip("1=V1_POS, 2=V2_VEL")]
        public int ctrlModeCode = 1;

        [Header("POS mode (handle offset -> steer)")]
        float handleXDeadZone = 0.003f;
        public float handleXToSteer = 5.0f; // steering command per meter of handle offset in POS mode

        [Header("M2 VEL mode (handle offset -> steer)")]
        public float velHandleXDeadZone = 0.003f;
        public float velHandleXToSteer = 5.0f; // steering command per meter of handle offset in VEL mode
        public float velHandleXToTargetVx = 5.0f; // target rover Vx (m/s) per meter of handle offset

        [Header("Shared steering limits")]
        public float steerClamp = 1.0f;
        
        [Header("Common hotkeys")]
        public bool enableHotkeys = true;

        [Header("M2 to Unity mapping")]
        public float targetWorldXScale = 5.0f; // Scaling factor to map M2 handle X movement to Unity world X movement.
        public float worldXCenter = 0.0f; // The world X coordinate that corresponds to the M2 handle center (zero) position. 
        [Tooltip("Unity centerline X that should correspond to M2 point A.")]
        public float unityCenterX = 0.0f; // Unity X coordinate that corresponds to M2 handle point A (the center reference).
        public bool autoSnapOnM2Reconnect = true; // Whether to automatically snap rover to M2-mapped position when M2 reconnects, to prevent jump forces due to position mismatch.

        // POS paradigm switch:
        // true  -> Paradigm 1: rover tracks mapped handle position (virtual spring-damper behavior)
        // false -> Paradigm 2: handle offset directly maps to steering
        public bool enablePosSyncCorrection = false;
        public float sendRateHz = 40.0f;
        public float posCorrKp = 0.05f; // Proportional gain for POS-mode sync correction based on rover X error
        public float posCorrKd = 0.04f;// Derivative gain for POS-mode sync correction based on rover Vx

        bool enableKeyboardInMode1 = true;


        // (1) isPaused=true: completely frozen; 
        // (2) isPaused=false, isDriving=true: normal driving; 
        // (3) isPaused=false, isDriving=false: not moving forward but not frozen (can be continued by external logic)
        private bool isDriving = false;
        private bool isPaused = false;

        private float nextFeedbackSendTime = 0.0f;
        private bool lastM2Connected = false; // Track M2 connection status to detect reconnects for auto-snapping
        private bool hasSentDisturbanceState = false; // To track whether we've sent the disturbance state to M2, to avoid redundant commands
        private bool lastDisturbanceState = false; // To track the last disturbance state sent to M2, for change detection
        private float lastDisturbanceDurationSec = 0f; // Last timeout value sent with DSTR=1.
        private Vector3 roverInitialPosition; // To store the initial pose of the rover for resetting, captured in Awake()
        
        // Flag indicating whether the reference point has been initialized.
        // This is used to ensure that we have a valid reference for computing relative handle movements and corresponding rover target positions, 
        // especially after mode switches or trial starts.
        private bool syncRefReady = false;

        // The M2 handle X position recorded at the start of the trial (or after mode switch), used as the zero point for relative movement.
        private float handleXRef = 0.0f; 
        // The rover X position recorded at the start of the trial (or after mode switch), used as the reference point for computing target X based on handle movement.
        private float roverXRef = 0.0f;


        // ----------------------------------------------------------------------------------------------------------------------
        // ------ Unity lifecycle methods ------

        void Awake()
        {
            if (rover == null)
                rover = FindFirstObjectByType<RoverHandler>();
            if (rover != null && roverTransform == null)
                roverTransform = rover.transform;
            if (rover != null && roverRigidbody == null)
                roverRigidbody = rover.GetComponent<Rigidbody>();
            if (worldFollower == null)
                worldFollower = FindFirstObjectByType<M2WorldFollower>();

            if (roverTransform != null)
            {
                roverInitialPosition = roverTransform.position;
            }

            // Keep centerline consistent
            if (Mathf.Abs(unityCenterX) < 1e-6f) unityCenterX = worldXCenter;

            if (!GlobalRandomSeed.IsInitialized)
                UnityEngine.Random.InitState(GlobalRandomSeed.Seed);
        }

        // ----------------------------------------------------------------------------------------------------------------------
        // --- Public methods to be called by UI or other components ---

        // Switch between keyboard control and M2 control modes. 
        // When switching from M2 to keyboard, also send SESS command to end M2 session and stop the trial.
        public void SetUnityMode(int mode)
        {
            UnityDriveMode targetMode = mode == 2 ? UnityDriveMode.Mode2_M2 : UnityDriveMode.Mode1_Keyboard;

            // If switching from M2 to keyboard mode, end the M2 session and trial to ensure a clean transition.
            if (unityMode == UnityDriveMode.Mode2_M2 && targetMode == UnityDriveMode.Mode1_Keyboard)
            {
                if (m2 != null && m2.IsInitialised() && m2.Client != null && m2.Client.IsConnected())
                    m2.SendCmd("SESS");
                NotifyTrialEnd();
            }

            unityMode = targetMode;

            // Keyboard mode enabled, reset states to prepare for keyboard control.
            if (unityMode == UnityDriveMode.Mode1_Keyboard)
            {   
                trialActive = false;
                enableKeyboardInMode1 = true;
                isPaused = false;
                isDriving = false;
                worldFollower?.ResetBias();
                Debug.Log("Switched to Keyboard control mode.");
            }

            // M2 control mode enabled, reset states to prepare for M2 control.
            else
            {
                trialActive = false;
                enableKeyboardInMode1 = false;
                syncRefReady = false;
                worldFollower?.ResetBias();
                Debug.Log("Switched to M2 control mode.");
            }
        }

        // Apply the HRI and control modes received from M2. 
        public void ApplyM2Modes(int hriMode, int ctrlMode)
        {
            bool ctrlChanged = ctrlModeCode != ctrlMode;
            hriModeCode = hriMode;
            ctrlModeCode = ctrlMode;
            if (ctrlChanged)
            {
                syncRefReady = false;
            }
        }

        // This should be called when M2 sends the BGIN command, indicating the trial starts
        public void NotifyTrialBegin()
        {
            if (unityMode == UnityDriveMode.Mode2_M2)
            {
                trialActive = true;
                syncRefReady = false; 
                if (!isPaused) isDriving = true;
                worldFollower?.ResetBias();
                if (ScoreManager.Instance != null) ScoreManager.Instance.SetScorePaused(false);
            }
        }

        // This should be called when the trial ends
        public void NotifyTrialEnd()
        {
            trialActive = false;
            syncRefReady = false;
            if (unityMode == UnityDriveMode.Mode2_M2 && isDriving) isDriving = false;
            worldFollower?.ResetBias();
            SendFeedbackForce(0f, 0f);
            if (m2 != null && m2.IsInitialised() && m2.Client != null && m2.Client.IsConnected())
                m2.SendCmd("DSTR", new double[] { 0.0 });
            hasSentDisturbanceState = false;
            lastDisturbanceState = false;
            lastDisturbanceDurationSec = 0f;
            if (ScoreManager.Instance != null) ScoreManager.Instance.SetScorePaused(true);
        }

        // Reset the rover to its initial pose, and clear velocities. This can be called on trial start or when needed.
        public void ResetRoverToInitialPose()
        {
            if (roverTransform == null) return;

            roverTransform.position = roverInitialPosition;
            roverTransform.rotation = Quaternion.identity;

            if (roverRigidbody != null)
            {
                roverRigidbody.linearVelocity = Vector3.zero;
                roverRigidbody.angularVelocity = Vector3.zero;
            }

            syncRefReady = false;
        }

        // Reset Unity-side runtime state without reloading scene or disconnecting M2.
        public void SoftResetUnityStateKeepM2Connection()
        {
            trialActive = false;
            syncRefReady = false;
            hasSentDisturbanceState = false;
            lastDisturbanceState = false;
            lastDisturbanceDurationSec = 0f;
            isPaused = false;
            isDriving = false;

            ResetRoverToInitialPose();
            ApplyRoverSteer(0f);
            SendFeedbackForce(0f, 0f);

            if (ScoreManager.Instance != null)
            {
                ScoreManager.Instance.ResetAll();
                ScoreManager.Instance.SetScorePaused(true);
            }

            ForceField.ResetFirstEntryFlag();

            if (GlobalRandomSeed.IsInitialized)
                UnityEngine.Random.InitState(GlobalRandomSeed.Seed);

            var road = FindFirstObjectByType<EndlessRoad>();
            if (road != null)
                road.ResetRoadAroundPlayer();

            var fields = FindObjectsByType<ForceField>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
            foreach (var f in fields)
            {
                if (f == null) continue;
                var go = f.gameObject;
                if (!go.activeSelf) continue;
                go.SetActive(false);
                go.SetActive(true);
            }
        }


        // =======================================================================================
        // --- Sync logic  ---

        // Re-synchronize rover position to current M2 mapping without recalibrating A reference.
        public bool SyncRoverToCurrentM2MappedX()
        {
            if (!TryGetM2HandleX(out float handleX)) return false; // Can't get handle X, can't sync

            EnsureSyncReference(handleX);
            SnapRoverToMappedX(handleX);
            return true;
        }

        // Ensure that the reference points for handle X and rover X are initialized, to allow computing relative movements and target positions.
        private void EnsureSyncReference(float handleX)
        {
            if (syncRefReady) return;
            // Set the reference to the current handle position
            handleXRef = handleX; 
            if (roverTransform != null)
                roverXRef = roverTransform.position.x;
            else
                roverXRef = worldXCenter;

            syncRefReady = true;
        }

        // Compute the target rover X position based on the handle X position, using the reference points and scaling factor.
        private float ComputeTargetXFromHandle(float handleX)
        {
            float handleXRel = handleX - handleXRef;
            return roverXRef + handleXRel * targetWorldXScale;
        }

        // Hard-sync rover X to mapped target, used on reconnect/mode switch to prevent large position mismatches that can cause jump forces..
        private void SnapRoverToMappedX(float handleX)
        {
            if (roverTransform == null) return;

            float targetX = ComputeTargetXFromHandle(handleX);
            Vector3 pos = roverTransform.position;
            pos.x = targetX;
            roverTransform.position = pos;

            if (roverRigidbody != null)
            {
                Vector3 v = roverRigidbody.linearVelocity;
                v.x = 0f;
                roverRigidbody.linearVelocity = v;
                roverRigidbody.angularVelocity = Vector3.zero;
            }
        }


        // =======================================================================================
        // --- POS control logic ---

        // Compute steering for POS mode with two paradigms toggled by usePosSyncCorrection.
        private float ComputeSteerPosMode(float handleX, bool usePosSyncCorrection)
        {
            float handleXRel = handleX - handleXRef;
            if (Mathf.Abs(handleXRel) < handleXDeadZone) handleXRel = 0f;

            float steer;

            if (usePosSyncCorrection && roverTransform != null)
            {
                // Paradigm 1: position tracking controller (virtual spring-damper style)
                float targetX = ComputeTargetXFromHandle(handleX);
                float roverX = roverTransform.position.x;
                float roverVx = roverRigidbody != null ? roverRigidbody.linearVelocity.x : 0f;
                steer = posCorrKp * (targetX - roverX) - posCorrKd * roverVx;
            }
            else
            {
                // Paradigm 2: direct mapping from handle offset to steering
                steer = handleXRel * handleXToSteer;
            }

            return Mathf.Clamp(steer, -steerClamp, steerClamp);
        }


        // =======================================================================================
        // --- M2 feedback logic ---
        // Check feedback send timer and send feedback if it's time. This helps to limit the feedback send rate to M2.
        private bool SendFeedbackTimer()
        {
            float now = Time.time;
            if (now < nextFeedbackSendTime) return false;

            float period = 1.0f / sendRateHz;
            nextFeedbackSendTime = now + period;
            return true;
        }

        private void SendFeedback()
        {
            if (!SendFeedbackTimer()) return;
            SendFeedbackForce(0f, 0f);
        }

        // Send the computed feedback force to M2 using the FRC2 command.
        private void SendFeedbackForce(float fx, float fy)
        {
            if (!trialActive) return;
            if (m2 == null || !m2.IsInitialised() || !m2.Client.IsConnected()) return;
            m2.SendCmd("FRC2", new double[] { fx, fy });
        }

        private void TrySendDisturbanceStateToM2()
        {
            if (!trialActive) return;
            if (m2 == null || !m2.IsInitialised() || m2.Client == null || !m2.Client.IsConnected()) return;

            // Use the same Unity disturbance source u(t) for both local simulation and M2 signaling.
            bool state = ForceField.DisturbanceU > 0.5f;
            float durationSec = Mathf.Max(0f, ForceField.DisturbanceDurationSec);
            bool durationChanged = state && lastDisturbanceState && Mathf.Abs(durationSec - lastDisturbanceDurationSec) > 0.05f;

            if (!hasSentDisturbanceState || state != lastDisturbanceState || durationChanged)
            {
                if (state) m2.SendCmd("DSTR", new double[] { 1.0, durationSec });
                else m2.SendCmd("DSTR", new double[] { 0.0 });
                lastDisturbanceState = state;
                lastDisturbanceDurationSec = state ? durationSec : 0f;
                hasSentDisturbanceState = true;
            }
        }

        private bool TryGetM2HandleX(out float handleX)
        {
            handleX = 0f;
            if (m2 == null || !m2.IsInitialised() || m2.Client == null || !m2.Client.IsConnected()) return false;
            if (m2.State == null || m2.State["X"] == null || m2.State["X"].Length < 1) return false;
            handleX = (float)m2.State["X"][0];
            if (float.IsNaN(handleX) || float.IsInfinity(handleX)) return false;
            return true;
        }

        // If can't get valid handle X input from M2, send zero feedback to prevent unintended behavior.
        private void SendZeroFeedbackAtSendRate()
        {
            if (!trialActive) return;
            if (SendFeedbackTimer()) SendFeedbackForce(0f, 0f);
        }

        // =======================================================================================
        // --- VEL control logic ---

        private float ComputeVelHandleXRel(float handleX)
        {
            float handleXRel = handleX - m2CenterX;
            if (Mathf.Abs(handleXRel) < velHandleXDeadZone) handleXRel = 0f;
            return handleXRel;
        }

        // VEL mode: handle offset sets target lateral speed, steering closes the vx error.
        private float ComputeVelSteerFromAcceleration(float handleXRel)
        {
            float targetVx = handleXRel * velHandleXToTargetVx;
            float currentVx = roverRigidbody != null ? roverRigidbody.linearVelocity.x : 0f;
            float vxError = targetVx - currentVx;
            float steer = vxError * velHandleXToSteer;
            steer = Mathf.Clamp(steer, -steerClamp, steerClamp);
            return steer;
        }


        // =======================================================================================
        // --- Common control application logic ---
        private void ApplyRoverSteer(float steer)
        {
            rover.SetInputRaw(new Vector2(steer, 0f));
        }

        private void PosHriMode(float handleX)
        {
            float steer = ComputeSteerPosMode(handleX, enablePosSyncCorrection);
            ApplyRoverSteer(steer);
            SendFeedbackForce(0f, 0f);
        }

        private void PosPhriMode(float handleX)
        {
            float steer = ComputeSteerPosMode(handleX, enablePosSyncCorrection);
            ApplyRoverSteer(steer);
            SendFeedback();
        }

        private void VelHriMode(float handleXRel)
        {
            float steer = ComputeVelSteerFromAcceleration(handleXRel);
            ApplyRoverSteer(steer);
            SendFeedbackForce(0f, 0f);
        }

        private void VelPhriMode(float handleXRel)
        {
            float steer = ComputeVelSteerFromAcceleration(handleXRel);
            ApplyRoverSteer(steer);
            SendFeedback();
        }

        private void HandleKeyboardModeFixedUpdate()
        {
            if (!enableKeyboardInMode1 || rover == null) return;
            Vector2 input = new Vector2(Input.GetAxis("Horizontal"), Input.GetAxis("Vertical"));
            rover.SetInput(input);
        }


        // ----------------------------------------------------------------------------------------------------------------------
        // --- Unity lifecycle methods ---

        void Update()
        {
            if (rover == null)
                rover = FindFirstObjectByType<RoverHandler>();
            if (rover != null && roverTransform == null)
                roverTransform = rover.transform;
            if (rover != null && roverRigidbody == null)
                roverRigidbody = rover.GetComponent<Rigidbody>();

            bool connectedNow = m2 != null && m2.IsInitialised() && m2.Client != null && m2.Client.IsConnected();

            // Detect M2 reconnect to trigger auto-sync if enabled, to prevent large position mismatches that can cause jump forces.
            if (unityMode == UnityDriveMode.Mode2_M2 && connectedNow && !lastM2Connected && autoSnapOnM2Reconnect)
            {
                SyncRoverToCurrentM2MappedX();
                hasSentDisturbanceState = false;
                lastDisturbanceDurationSec = 0f;
            }
            if (!connectedNow)
            {
                hasSentDisturbanceState = false;
                lastDisturbanceDurationSec = 0f;
            }
            lastM2Connected = connectedNow;

            if (unityMode == UnityDriveMode.Mode2_M2 && trialActive)
                TrySendDisturbanceStateToM2();

            if (enableHotkeys)
            {
                
                bool ctrl = Input.GetKey(KeyCode.LeftControl);
                if (Input.GetKeyDown(KeyCode.T) && !ctrl)
                {
                    Debug.LogWarning("Hotkey T detected");
                    isPaused = false;
                    isDriving = true;
                    // if (ScoreManager.Instance != null) ScoreManager.Instance.SetScorePaused(!trialActive);
                }
                if (Input.GetKeyDown(KeyCode.T) && ctrl)
                {
                    isPaused = true;
                    isDriving = false;
                    // if (ScoreManager.Instance != null) ScoreManager.Instance.SetScorePaused(true);
                }

                if (Input.GetKeyDown(KeyCode.R) && ctrl )
                {
                    // if (unityMode == UnityDriveMode.Mode2_M2)
                    //     SoftResetUnityStateKeepM2Connection();
                    // else
                        SceneManager.LoadScene(SceneManager.GetActiveScene().name);
                }
            }

            if (rover != null)
            {
                rover.SetPreserveLateralVelocity(unityMode == UnityDriveMode.Mode2_M2);
                rover.SetPaused(isPaused);
                rover.SetDriving(isDriving && !isPaused);
            }

            if (ScoreManager.Instance != null)
                ScoreManager.Instance.SetScorePaused(isPaused || (unityMode == UnityDriveMode.Mode2_M2 && !trialActive));

        }    


        void FixedUpdate()
        {
            if (rover == null) return; // Safety check

            if (unityMode == UnityDriveMode.Mode1_Keyboard) // Keyboard control mode
            {
                HandleKeyboardModeFixedUpdate();
                return;
            }

             // Can't get handle X, can't proceed with M2 control
            else if (!TryGetM2HandleX(out float handleX))
            {
                SendZeroFeedbackAtSendRate();
                return;
            }

            // M2 control mode but trial not active, ensure rover is not moving and send zero feedback.
            else if (!trialActive)
            {
                SendZeroFeedbackAtSendRate();
                ApplyRoverSteer(0f);
                return;
            }
            else{ // M2 control mode
                
                EnsureSyncReference(handleX);

                int ctrl = ctrlModeCode;
                int hri = hriModeCode;

                if (ctrl == 1) // POS mode
                {
                    if (hri == 2) PosPhriMode(handleX);// pHRI + POS mode
                    else PosHriMode(handleX); // HRI-only + POS mode
                    return;
                }

                if (ctrl == 2)// VEL mode
                {
                    float handleXRel = ComputeVelHandleXRel(handleX);
                    if (hri == 2) VelPhriMode(handleXRel); // pHRI + VEL mode
                    else VelHriMode(handleXRel); // HRI-only + VEL mode
                    return;
                }

                SendFeedbackForce(0f, 0f);
            }
        }


    }
}
