
using UnityEngine;
using TMPro;
using UnityEngine.UI;
using System;

namespace CORC.Demo
{
    public class M2UiPanel : MonoBehaviour
    {
        [Header("Refs")]
        public CORC.CORCM2 m2;
        public M2RoverBridge bridge;
        public CalibrationManager calibrationManager;

        [Header("Texts (TMP)")]
        public TMP_Text timeTxt;
        public TMP_Text posTxt;
        public TMP_Text velTxt;
        public TMP_Text frcTxt;
        public TMP_Text statusTxt;
        public TMP_Text disturbanceTxt;
        public TMP_Text calibrationTxt;
        public TMP_Text graspForceTxt;
        public Slider kSlider;
        public Slider kSlider2;

        [Header("Buttons")]
        public Button beginSessionBtn;
        public Button confirmUnityModeBtn;
        public Button confirmModeBtn;
        public Button startExperimentBtn;
        public Button returnWaitStartBtn;
        public Button toAButton;
        public Button emergencyStopBtn;
        public Button startRecordingBtn;
        public Button stopRecordingBtn;
        public Button startCalibrBtn;

        [Header("Dropdowns")]
        public TMP_Dropdown unityModeDropdown; // Mode1 keyboard / Mode2 M2
        public TMP_Dropdown hriModeDropdown;   // V1_HRI / V2_PHRI

        [Header("Inputs")]
        public TMP_InputField calibForceInput; // Link to UI text field for participant calibration

        private IM2Proxy proxy;
        private const string SetHriCmd = "S_MD";
        private bool bginReady = false; // Indicates BGOK received after BGIN, i.e., M2 is ready for block start and mode application
        private bool pendingHriApply = false;  // Indicates pending HRI mode setting (S_MD)
        private int pendingHri = 2;
        private readonly Color startRecIdleColor = new Color(0.85f, 0.85f, 0.85f, 1f);
        private readonly Color startRecActiveColor = new Color(0.25f, 0.70f, 0.25f, 1f);
        private ExperimentBlockControl blockControl;
        private bool IsM2Mode => bridge != null && bridge.unityMode == M2RoverBridge.UnityDriveMode.Mode2_M2;
        private bool CanStartBlock => blockControl != null && blockControl.CanStartPreparedBlock();


        // ---------------------------------------------------------------------------------------------
        // ---------------------------------- UI Event Handlers ----------------------------------------

        private int GetHriModeCode()
        {
            int idx = hriModeDropdown ? hriModeDropdown.value : 1; 
            return idx == 0 ? 1 : 2; // 1: V1_HRI, 2: V2_PHRI
        }

        private int GetUnityModeCode()
        {
            int idx = unityModeDropdown ? unityModeDropdown.value : 0;
            return idx == 0 ? 1 : 2; // 1: Mode1_Keyboard, 2: Mode2_M2
        }

        // Set command buttons interactable state based on current mode and M2 readiness
        private void SetCommandButtonsInteractable()
        {
            if (confirmUnityModeBtn) confirmUnityModeBtn.interactable = true;
            if (confirmModeBtn) confirmModeBtn.interactable = true;
            if (startExperimentBtn)
                startExperimentBtn.interactable = IsM2Mode
                    ? bginReady && !pendingHriApply && (CanStartBlock || (bridge != null && bridge.IsEmgHold))
                    : blockControl != null && (!blockControl.HasPreparedBlock || CanStartBlock) && !blockControl.IsRoundComplete;
            if (returnWaitStartBtn) returnWaitStartBtn.interactable = IsM2Mode && bginReady;
            if (toAButton) toAButton.interactable = IsM2Mode;
            if (emergencyStopBtn) emergencyStopBtn.interactable = IsM2Mode;
            if (startCalibrBtn)
                startCalibrBtn.interactable = true;
            
        }

        // Update recording buttons interactable state based on EMG/M2-log recording status.
        private void RecordBtnInteract()
        {
            bool canStart = bridge != null && bridge.CanStartAnyRecording;

            if (startRecordingBtn)
            {
                startRecordingBtn.interactable = canStart;

                if (startRecordingBtn.targetGraphic != null)
                {
                    startRecordingBtn.targetGraphic.color =
                        bridge != null && bridge.AnyRecordingActive ? startRecActiveColor : startRecIdleColor;
                }
            }

            if (stopRecordingBtn)
                stopRecordingBtn.interactable = bridge != null && bridge.AnyRecordingActive;
        }

        //  Set status text with color
        private void SetStatus(Color color, string msg)
        {
            if (!statusTxt) return;
            statusTxt.color = color;
            statusTxt.text = msg;
        }

        private void UpdateCalibrationText()
        {
            if (calibrationTxt == null)
                return;

            float calibForce = bridge != null ? bridge.CalibForce : 0f;
            string emgRestText = FormatFloatArray(bridge != null ? bridge.EmgRest : null);
            string emgRefText = FormatFloatArray(bridge != null ? bridge.EmgRef : null);
            string emgBracingText = FormatFloatArray(bridge != null ? bridge.EmgBracing : null);
            string note = calibrationManager != null ? calibrationManager.lastResponseNote : string.Empty;
            calibrationTxt.text =
                $"CalibForce: {calibForce:0.00}\n" +
                $"SPI Scale: {(calibrationManager != null ? calibrationManager.lastReturnedEmgScale : 0f):0.000}\n" +
                $"Threshold: {(calibrationManager != null ? calibrationManager.lastReturnedThreshold : 0f):0.000}\n" +
                $"EMG Rest: {emgRestText}\n" +
                $"EMG Ref: {emgRefText}\n" +
                $"EMG Bracing: {emgBracingText}\n" +
                $"Note: {note}";
        }

        private string FormatFloatArray(float[] values)
        {
            if (values == null || values.Length == 0)
                return "--";

            string[] parts = new string[values.Length];
            for (int i = 0; i < values.Length; i++)
                parts[i] = values[i].ToString("0.000000");
            return "[" + string.Join(", ", parts) + "]";
        }

        private void UpdateKSlider()
        {
            if (bridge == null)
                return;

            UpdateKSliderValue(kSlider);
            UpdateKSliderValue(kSlider2);
        }

        private void UpdateKSliderValue(Slider slider)
        {
            if (slider == null)
                return;

            slider.minValue = bridge.StiffnessMin;
            slider.maxValue = bridge.StiffnessMax;
            slider.value = Mathf.Clamp(bridge.StiffnessCmd, slider.minValue, slider.maxValue);
        }

        private void UpdateGraspForceText()
        {
            if (graspForceTxt == null)
                return;

            if (bridge == null)
            {
                graspForceTxt.text = "Grasp: -- (no bridge)";
                return;
            }

            graspForceTxt.text = bridge.TryGetGraspForce(out float force)
                ? $"Grasp: {force:0.00} N"
                : $"Grasp: -- ({bridge.FsrStatus})";
        }

        // Try to apply the pending HRI mode if M2 is ready at A after BGIN.
        private void TryApplyPendingM2Setup()
        {
            if (!pendingHriApply) return;
            if (proxy == null || !proxy.IsReady) return;
            if (!bginReady) return;

            proxy.SendCmd(SetHriCmd, new[] { (double)pendingHri });
            pendingHriApply = false;

            SetCommandButtonsInteractable();
        }

        private void OnBeginSession()
        {
            if (proxy == null || !proxy.IsReady)
            {
                Debug.LogWarning("[UI] Proxy not ready; skip BGIN command");
                return;
            }

            proxy.SendCmd("BGIN");
            SetCommandButtonsInteractable();

            Debug.Log("[UI] Sent BGIN");
            if (statusTxt)
            {
                SetStatus(Color.white, "BGIN sent, waiting BGOK...");
            }
        }

        private void OnConfirmUnityMode()
        {
            int unityMode = GetUnityModeCode();

            if (bridge)
            {
                if (unityMode == 1 && bridge.unityMode == M2RoverBridge.UnityDriveMode.Mode2_M2)
                {
                    if (proxy != null && proxy.IsReady) proxy.SendCmd("SESS");
                    bridge.NotifyBlockEnd();
                }

                bridge.SetUnityMode(unityMode);
            }

            if (unityMode == 1)
            {
                // Switching to keyboard: clear pending M2 setup state.
                pendingHriApply = false;
                bginReady = false;
            }

            SetCommandButtonsInteractable();
            if (statusTxt) SetStatus(Color.white, unityMode == 2 ? "Unity mode set: M2" : "Unity mode set: Keyboard");

            Debug.Log($"[UI] Confirm Unity mode: unity={unityMode}");
        }

        private void OnConfirmHriMode()
        {
            int hri = GetHriModeCode();
            pendingHri = hri;

            if (bridge)
                bridge.ApplyHriMode(pendingHri);

            if (!IsM2Mode)
            {
                pendingHriApply = false;
                SetCommandButtonsInteractable();
                if (statusTxt)
                {
                    SetStatus(Color.white, $"HRI selected ({hri}), switch to M2 mode to apply.");
                }
                Debug.Log($"[UI] Confirm HRI (cached): HRI={hri}");
                return;
            }

            pendingHriApply = true;
            TryApplyPendingM2Setup();
            SetCommandButtonsInteractable();

            if (statusTxt)
            {
                SetStatus(Color.white, $"HRI:{hri} (wait BGOK/AT_A to apply)");
            }

            Debug.Log($"[UI] Confirm HRI: HRI={hri}");
        }

        private void OnStartExperiment()
        {
            if (!IsM2Mode)
            {
                if (blockControl == null || blockControl.IsRoundComplete)
                    return;

                if (!blockControl.HasPreparedBlock)
                    blockControl.PrepareRound();
                if (!blockControl.CanStartPreparedBlock())
                    return;

                blockControl.NotifyBlockStarted();
                bridge?.NotifyBlockBegin();
                SetCommandButtonsInteractable();
                if (statusTxt) SetStatus(Color.white, "Keyboard block in progress...");
                return;
            }

            if (proxy == null || !proxy.IsReady)
            {
                Debug.LogWarning("[UI] Proxy not ready; skip start command");
                return;
            }
            if (!bginReady)
            {
                Debug.LogWarning("[UI] BGIN not ready; TRBG blocked");
                if (statusTxt)
                {
                    SetStatus(Color.yellow, "Please press BGIN and wait for BGOK first.");
                }
                return;
            }

            if (pendingHriApply)
            {
                Debug.LogWarning("[UI] Mode setup pending; TRBG blocked");
                if (statusTxt)
                {
                    SetStatus(Color.yellow, "Waiting mode setup before TRBG.");
                }
                return;
            }

            if (bridge != null && bridge.IsEmgHold)
            {
                bool resumed = bridge.ResumeEmgHold();
                SetCommandButtonsInteractable();
                if (statusTxt)
                    SetStatus(resumed ? Color.white : Color.yellow, resumed ? "EMG reconnected. Resumed current trial." : "EMG still disconnected.");
                return;
            }

            if (blockControl == null || !blockControl.CanStartPreparedBlock())
            {
                if (statusTxt)
                    SetStatus(Color.yellow, blockControl != null && blockControl.IsRoundComplete ? "Round completed." : "Waiting for next block.");
                return;
            }

            proxy.SendCmd("TRBG");  // Use TRBG as the default start command for backward compatibility
            Debug.Log($"[UI] Sent start cmd: TRBG");
            if (statusTxt) SetStatus(Color.white, "TRBG");
        }

        public bool PauseOnBlockCompletion()
        {
            if (proxy == null || !proxy.IsReady)
                return false;
            if (!bginReady)
                return false;
            if (blockControl == null)
                return false;

            proxy.SendCmd("RWST"); // RWST: Return to WAIT_START, which ends the current block in Unity-side logic.
            if (bridge)
                bridge.StopBlockMotionImmediate();

            blockControl.MarkAutoPauseRequested();
            SetCommandButtonsInteractable();
            if (statusTxt)
                SetStatus(Color.white, "Block completed. Returning to WAIT_START...");

            Debug.Log("[UI] Auto-sent RWST for completed block");
            return true;
        }

        private void OnToA()
        {
            if (proxy == null || !proxy.IsReady)
            {
                Debug.LogWarning("[UI] Proxy not ready; skip TO_A command");
                return;
            }
            if (!bginReady)
            {
                Debug.LogWarning("[UI] BGIN not ready; TO_A blocked");
                return;
            }

            proxy.SendCmd("TO_A"); // TO_A: Return to A, which is the standby state before starting the next block in Unity-side logic.
            SetCommandButtonsInteractable();
            if (statusTxt)
            {
                SetStatus(Color.white, "TO_A sent, returning to A...");
            }
            Debug.Log("[UI] Sent TO_A");
        }

        private void OnReturnWaitStart()
        {
            if (proxy == null || !proxy.IsReady)
            {
                Debug.LogWarning("[UI] Proxy not ready; skip RWST command");
                return;
            }
            if (!bginReady)
            {
                Debug.LogWarning("[UI] BGIN not ready; RWST blocked");
                return;
            }

            proxy.SendCmd("RWST"); // RWST: Return to WAIT_START, which ends the current block in Unity-side logic.
            if (bridge)
            {
                bridge.StopBlockMotionImmediate();
                bridge.ResetRoverToInitialPose();
            }

            // AbortCurrentBlock() will pending and refresh the block state, so that after RWST, next START can reenter the block.
            blockControl.AbortCurrentBlock();
            SetCommandButtonsInteractable();

            if (statusTxt)
            {
                SetStatus(Color.white, "RWST sent, returning to WAIT_START...");
            }
            Debug.Log("[UI] Sent RWST");
        }

        private void OnEmergencyStop()
        {
            if (proxy == null || !proxy.IsReady)
            {
                Debug.LogWarning("[UI] Proxy not ready; skip SESS command");
                return;
            }

            proxy.SendCmd("SESS"); // SESS: emergency stop that also ends the current block.
                                   // Different from RWST, SESS resets the block/round state,
                                   // while RWST is the normal block-completion path.
                                   // After SESS, user needs to press BGIN again before starting the next block.
            if (bridge)
            {
                bridge.NotifyBlockEnd();
                bridge.SoftResetUnityStateKeepM2Connection();
            }
            if (statusTxt)
            {
                SetStatus(Color.yellow, "Stop requested (SESS).");
            }
            Debug.Log("[UI] Sent stop cmd: SESS");
        }

        private void OnStartRecording()
        {
            if (bridge == null) return;

            if (startRecordingBtn)
                startRecordingBtn.interactable = false; // block double click while handling click

            bridge.StartAuxRecordingManual();
            RecordBtnInteract();
        }

        private void OnStopRecording()
        {
            if (bridge == null) return;

            bridge.CloseAuxRecordingSessions();
            RecordBtnInteract();
        }

        private void OnStartCalibration()
        {
            if (calibrationManager == null)
            {
                if (statusTxt) SetStatus(Color.yellow, "CalibrationManager missing.");
                return;
            }

            bool started = calibrationManager.StartCalibrationSession(!IsM2Mode);
            SetCommandButtonsInteractable();

            if (statusTxt)
                SetStatus(started ? Color.white : Color.yellow, calibrationManager.LastStatus);
        }

        private void OnCalibForceEdit(string value)
        {
            if (bridge != null)
            {
                bridge.SetCalibForce(value);
                RefreshCalibForceInput();
            }
        }

        public void RefreshCalibForceInput()
        {
            if (calibForceInput != null && bridge != null)
                calibForceInput.text = bridge.CalibForce.ToString("0.00");
        }

        // ---------------------------------------------------------------------------------------------
        // ---------------------------------- Unity Lifecycle ------------------------------------------

        void Awake()
        {
            if (m2 == null) { Debug.LogError("[M2UiPanel] Missing CORCM2 reference!"); return; }
            proxy = new M2Proxy(m2);

            if (beginSessionBtn) beginSessionBtn.onClick.AddListener(OnBeginSession);
            if (confirmUnityModeBtn) confirmUnityModeBtn.onClick.AddListener(OnConfirmUnityMode);
            if (confirmModeBtn) confirmModeBtn.onClick.AddListener(OnConfirmHriMode);
            if (startExperimentBtn) startExperimentBtn.onClick.AddListener(OnStartExperiment);
            if (returnWaitStartBtn) returnWaitStartBtn.onClick.AddListener(OnReturnWaitStart);
            if (toAButton) toAButton.onClick.AddListener(OnToA);
            if (emergencyStopBtn) emergencyStopBtn.onClick.AddListener(OnEmergencyStop);
            if (startRecordingBtn) startRecordingBtn.onClick.AddListener(OnStartRecording);
            if (stopRecordingBtn) stopRecordingBtn.onClick.AddListener(OnStopRecording);
            if (startCalibrBtn) startCalibrBtn.onClick.AddListener(OnStartCalibration);
            
            if (calibForceInput)
            {
                calibForceInput.text = (bridge != null) ? bridge.CalibForce.ToString() : "50";
                calibForceInput.onEndEdit.AddListener(OnCalibForceEdit);
            }
            SetCommandButtonsInteractable();

            if (bridge == null)
                bridge = FindFirstObjectByType<M2RoverBridge>();
            if (calibrationManager == null)
                calibrationManager = FindFirstObjectByType<CalibrationManager>();
            blockControl = FindFirstObjectByType<ExperimentBlockControl>();
            SetCommandButtonsInteractable();

            RecordBtnInteract();

        }

        void Update()
        {
            RecordBtnInteract();
            // if (disturbanceTxt) disturbanceTxt.text = $"Disturbance: {ForceField.DisturbanceU:F1}";
            UpdateCalibrationText();
            UpdateKSlider();
            UpdateGraspForceText();

            if (!IsM2Mode)
            {
                SetCommandButtonsInteractable();

                if (blockControl != null && blockControl.ShouldAutoPauseNow)
                {
                    bridge?.NotifyBlockEnd();
                    blockControl.MarkAutoPauseRequested();
                    blockControl.NotifyReturnWaitReady();
                }

                if (blockControl != null && blockControl.TryAdvancePendingCompletion())
                {
                    bridge?.ResetForNextExperimentBlock();
                    SetCommandButtonsInteractable();
                }
                return;
            }

            if (proxy == null || !proxy.IsReady)
            {
                bginReady = false;
                SetCommandButtonsInteractable();
                if (statusTxt)
                {
                    bool showCalibrationStatus = calibrationManager != null && calibrationManager.IsCalibrationActive;
                    if (showCalibrationStatus)
                        SetStatus(Color.white, calibrationManager.LastStatus);
                    else
                        SetStatus(Color.white, IsM2Mode ? "Connecting to Robot..." : "Keyboard mode active.");
                }
                return;
            }

            TryApplyPendingM2Setup();
            SetCommandButtonsInteractable();

            if (blockControl != null && blockControl.ShouldAutoPauseNow)
                PauseOnBlockCompletion();

            if (blockControl != null && blockControl.TryAdvancePendingCompletion())
            {
                if (bridge) bridge.ResetForNextExperimentBlock();
                SetCommandButtonsInteractable();
            }

            var t = proxy.Time;
            var X = proxy.X;
            var dX = proxy.dX;
            var F = proxy.F;

            if (timeTxt) timeTxt.text = $"Time:{t:F3} s";
            if (posTxt) posTxt.text = $"Position: [{X[0]:F3}, {X[1]:F3}]";
            if (velTxt) velTxt.text = $"Velocity: [{dX[0]:F3}, {dX[1]:F3}]";
            if (frcTxt) frcTxt.text = $"Force: [{F[0]:F3}, {F[1]:F3}]";

            if (bridge != null && bridge.IsEmgHold && statusTxt)
                SetStatus(Color.yellow, "EMG disconnected. Reconnecting...");
            else if (calibrationManager != null && calibrationManager.IsCalibrationActive && statusTxt)
                SetStatus(Color.white, calibrationManager.LastStatus);

            var cmds = proxy.DrainCmds();

            foreach (var c in cmds)
            {
                string cmd = (c.cmd ?? string.Empty).TrimEnd('\0');
                var p = c.parameters ?? Array.Empty<double>();
                if (cmd != "OK")
                    Debug.Log($"[UI Received] {cmd} ({p.Length} params)");

                if (cmd == "TRBG") // TRBG: block begin in Unity-side naming.
                {
                    if (statusTxt) SetStatus(Color.white, "Block in progress...");
                    blockControl?.NotifyBlockStarted();
                    if (bridge) bridge.NotifyBlockBegin();
                    SetCommandButtonsInteractable();
                }
                else if (cmd == "BGOK") // BGOK: M2 is ready at A after BGIN, and we can apply pending mode settings and allow block start.
                {
                    bginReady = true;
                    blockControl?.PrepareRound();
                    if (bridge)
                    {
                        bridge.NotifyBlockEnd();
                        bridge.ResetForNextExperimentBlock();
                    }
                    SetCommandButtonsInteractable();
                    if (statusTxt)
                    {
                        SetStatus(Color.green, "BGIN acknowledged!");
                    }
                }
                else if (cmd == "AT_A") // AT_A: M2 is back at A (after TO_A or block end), we can apply pending mode settings and allow block start.
                {
                    TryApplyPendingM2Setup();
                    SetCommandButtonsInteractable();

                    if (statusTxt)
                    {
                        SetStatus(Color.green, "Robot at A!");
                    }
                }
                else if (cmd == "BUSY")
                {
                    if (statusTxt)
                    {
                        SetStatus(Color.yellow, "Command rejected: Busy!");
                    }
                }
                else if (cmd == "OK")
                {
                    if (statusTxt)
                    {
                        SetStatus(Color.green, "Command accepted!");
                    }
                }
                else if (cmd == "RWOK") // RWOK: Acknowledgement for RWST, we can prepare for the next block if we were waiting for block completion.
                {
                    if (bridge) bridge.NotifyBlockEnd();
                    bool hasNextBlock = blockControl != null && blockControl.NotifyReturnWaitReady();
                    if (hasNextBlock && bridge) bridge.ResetForNextExperimentBlock();
                    TryApplyPendingM2Setup();
                    SetCommandButtonsInteractable();
                    if (statusTxt)
                    {
                        SetStatus(Color.green, "Returned to WAIT_START (to A).");
                    }
                }
                else if (cmd == "TRND") // TRND: block end in Unity-side naming.
                {
                    if (statusTxt) SetStatus(Color.white, "Block ended.");
                    if (bridge) bridge.NotifyBlockEnd();
                    SetCommandButtonsInteractable();
                }
                else if (cmd == "SESS") // SESS: Emergency stop, we can update the UI and reset the block/round state.
                {
                    if (statusTxt) SetStatus(Color.white, "Block ended.");
                    if (bridge) bridge.NotifyBlockEnd();
                    blockControl?.ResetRoundState();
                    bginReady = false;
                    SetCommandButtonsInteractable();
                }
            }

        }
    }

}


