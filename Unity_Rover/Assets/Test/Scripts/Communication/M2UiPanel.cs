
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

        [Header("Texts (TMP)")]
        public TMP_Text timeTxt;
        public TMP_Text posTxt;
        public TMP_Text velTxt;
        public TMP_Text frcTxt;
        public TMP_Text statusTxt;

        [Header("Buttons")]
        public Button beginSessionBtn;
        public Button confirmModeBtn;
        public Button startExperimentBtn;
        public Button returnWaitStartBtn;
        public Button toAButton;
        public Button emergencyStopBtn;

        [Header("Dropdowns")]
        public TMP_Dropdown unityModeDropdown; // Mode1 keyboard / Mode2 M2
        public TMP_Dropdown hriModeDropdown;   // V1_HRI / V2_PHRI
        public TMP_Dropdown ctrlModeDropdown;  // V1_POS / V2_VEL

        private IM2Proxy proxy;
        private const string SetHriCmd = "S_MD";
        private const string SetCtrlCmd = "S_CT";
        private bool bginReady = false; // Indicates BGOK received after BGIN, i.e., M2 is ready for trial start and mode application
        private bool atAReady = false; // Indicates AT_A received, i.e., M2 is at A and has accepted mode settings, ready for TRBG and trial start
        private bool pendingModeApply = false; // Indicates that there are pending HRI/CTRL mode settings to apply once M2 is ready at A
        private int pendingHri = 2;
        private int pendingCtrl = 1;

        // Helper to parse double with fallback
        // private static bool TryParse(string s, out double v, double fallback = 0) { if (double.TryParse(s, out v)) return true; v = fallback; return false; }


        // ---------------------------------------------------------------------------------------------
        // ---------------------------------- UI Event Handlers ----------------------------------------

        private int GetHriModeCode()
        {
            int idx = hriModeDropdown ? hriModeDropdown.value : 1; 
            return idx == 0 ? 1 : 2; // 1: V1_HRI, 2: V2_PHRI
        }

        private int GetCtrlModeCode()
        {
            int idx = ctrlModeDropdown ? ctrlModeDropdown.value : 0;
            return idx == 0 ? 1 : 2;  //1: POS, 2: VEL
        }

        private int GetUnityModeCode()
        {
            int idx = unityModeDropdown ? unityModeDropdown.value : 0;
            return idx == 0 ? 1 : 2; // 1: Mode1_Keyboard, 2: Mode2_M2
        }

        private void SetCommandButtonsInteractable()
        {
            bool isM2Mode = bridge != null && bridge.unityMode == M2RoverBridge.UnityDriveMode.Mode2_M2;
            if (confirmModeBtn) confirmModeBtn.interactable = true;
            if (startExperimentBtn) startExperimentBtn.interactable = isM2Mode && bginReady;
            if (returnWaitStartBtn) returnWaitStartBtn.interactable = isM2Mode && bginReady;
            if (toAButton) toAButton.interactable = isM2Mode;
            if (emergencyStopBtn) emergencyStopBtn.interactable = isM2Mode;
        }

        // Try to apply pending HRI/CTRL mode settings if we are waiting for M2 to be ready at A after BGIN
        private void TryApplyPendingM2Setup()
        {
            if (!pendingModeApply) return;
            if (proxy == null || !proxy.IsReady) return;
            if (!bginReady || !atAReady) return;

            proxy.SendCmd(SetHriCmd, new[] { (double)pendingHri });
            proxy.SendCmd(SetCtrlCmd, new[] { (double)pendingCtrl });
            pendingModeApply = false;
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
                statusTxt.color = Color.white;
                statusTxt.text = "BGIN sent, waiting BGOK...";
            }
        }

        private void OnConfirmModes()
        {
            int hri = GetHriModeCode();
            int ctrl = GetCtrlModeCode();
            int unityMode = GetUnityModeCode();

            if (bridge)
            {
                if (unityMode == 1 && bridge.unityMode == M2RoverBridge.UnityDriveMode.Mode2_M2)
                {
                    if (proxy != null && proxy.IsReady) proxy.SendCmd("SESS");
                    bridge.NotifyTrialEnd();
                }

                bridge.SetUnityMode(unityMode);
                bridge.ApplyM2Modes(hri, ctrl);
            }

            if (unityMode == 2)
            {
                pendingHri = hri;
                pendingCtrl = ctrl;
                pendingModeApply = true;

                if (proxy != null && proxy.IsReady && !bginReady)
                {
                    proxy.SendCmd("BGIN");
                    if (statusTxt)
                    {
                        statusTxt.color = Color.white;
                        statusTxt.text = "M2 connected, auto BGIN sent.";
                    }
                }

                TryApplyPendingM2Setup();
                if (statusTxt && (proxy == null || !proxy.IsReady))
                {
                    statusTxt.color = Color.white;
                    statusTxt.text = "Switched to M2 mode, waiting connection...";
                }
                else if (statusTxt)
                {
                    statusTxt.text = $"Unity:{unityMode}, HRI:{hri}, CTRL:{ctrl} (wait BGOK/AT_A)";
                }
            }
            else
            {// If switching to keyboard mode, clear pending mode and ensure UI is updated
                pendingModeApply = false;
                bginReady = false;
                atAReady = false;
                SetCommandButtonsInteractable();
                if (statusTxt) statusTxt.text = $"Unity:{unityMode}";
            }

            Debug.Log($"[UI] Confirm modes: unity={unityMode}, HRI={hri}, CTRL={ctrl}");
        }

        private void OnStartExperiment()
        {
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
                    statusTxt.color = Color.yellow;
                    statusTxt.text = "Please press BGIN and wait for BGOK first.";
                }
                return;
            }

            if (!atAReady)
            {
                Debug.LogWarning("[UI] AT_A not ready; TRBG blocked");
                if (statusTxt)
                {
                    statusTxt.color = Color.yellow;
                    statusTxt.text = "Robot not in WAIT_START at A; TRBG blocked.";
                }
                return;
            }

            proxy.SendCmd("TRBG");  // Use TRBG as the default start command for backward compatibility
            Debug.Log($"[UI] Sent start cmd: TRBG");
            if (statusTxt) statusTxt.text = $"TRBG";
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

            proxy.SendCmd("TO_A");
            SetCommandButtonsInteractable();
            if (statusTxt)
            {
                statusTxt.color = Color.white;
                statusTxt.text = "TO_A sent, returning to A...";
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

            proxy.SendCmd("RWST");
            if (bridge)
            {
                bridge.NotifyTrialEnd();
                bridge.ResetRoverToInitialPose();
            }
            SetCommandButtonsInteractable();

            if (statusTxt)
            {
                statusTxt.color = Color.white;
                statusTxt.text = "RWST sent, returning to WAIT_START...";
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

            proxy.SendCmd("SESS");
            if (bridge) bridge.NotifyTrialEnd();
            if (statusTxt)
            {
                statusTxt.color = Color.yellow;
                statusTxt.text = "Stop requested (SESS).";
            }
            Debug.Log("[UI] Sent stop cmd: SESS");
        }


        // ---------------------------------------------------------------------------------------------
        // ---------------------------------- Unity Lifecycle ------------------------------------------

        void Awake()
        {
            if (m2 == null) { Debug.LogError("[M2UiPanel] Missing CORCM2 reference!"); return; }
            proxy = new M2Proxy(m2);

            if (beginSessionBtn) beginSessionBtn.onClick.AddListener(OnBeginSession);
            if (confirmModeBtn) confirmModeBtn.onClick.AddListener(OnConfirmModes);
            if (startExperimentBtn) startExperimentBtn.onClick.AddListener(OnStartExperiment);
            if (returnWaitStartBtn) returnWaitStartBtn.onClick.AddListener(OnReturnWaitStart);
            if (toAButton) toAButton.onClick.AddListener(OnToA);
            if (emergencyStopBtn) emergencyStopBtn.onClick.AddListener(OnEmergencyStop);
            SetCommandButtonsInteractable();

            if (bridge == null)
                bridge = FindFirstObjectByType<M2RoverBridge>();

        }

        void Update()
        {
            if (proxy == null || !proxy.IsReady)
            {
                bginReady = false;
                atAReady = false;
                SetCommandButtonsInteractable();
                if (statusTxt)
                {
                    bool isM2Mode = bridge != null && bridge.unityMode == M2RoverBridge.UnityDriveMode.Mode2_M2;
                    statusTxt.text = isM2Mode ? "Connecting to Robot..." : "Keyboard mode active.";
                }
                return;
            }

            TryApplyPendingM2Setup();

            var t = proxy.Time;
            var X = proxy.X;
            var dX = proxy.dX;
            var F = proxy.F;

            if (timeTxt) timeTxt.text = $"Time:{t:F3} s";
            if (posTxt) posTxt.text = $"Position: [{X[0]:F3}, {X[1]:F3}]";
            if (velTxt) velTxt.text = $"Velocity: [{dX[0]:F3}, {dX[1]:F3}]";
            if (frcTxt) frcTxt.text = $"Force: [{F[0]:F3}, {F[1]:F3}]";


            var cmds = proxy.DrainCmds();

            foreach (var c in cmds)
            {
                string cmd = (c.cmd ?? string.Empty).TrimEnd('\0');
                var p = c.parameters ?? Array.Empty<double>();
                Debug.Log($"[UI Received] {cmd} ({p.Length} params)");

                if (cmd == "TRBG")
                {
                    if (statusTxt) { statusTxt.color = Color.white; statusTxt.text = "Trial in progress..."; }
                    if (bridge) bridge.NotifyTrialBegin();
                    atAReady = false;
                    SetCommandButtonsInteractable();
                }
                else if (cmd == "BGOK")
                {
                    bginReady = true;
                    if (bridge)
                    {
                        bridge.NotifyTrialEnd();
                        bridge.ResetRoverToInitialPose();
                    }
                    atAReady = false;
                    SetCommandButtonsInteractable();
                    if (statusTxt)
                    {
                        statusTxt.color = Color.green;
                        statusTxt.text = "BGIN acknowledged!";
                    }
                }
                else if (cmd == "AT_A")
                {
                    atAReady = true;
                    TryApplyPendingM2Setup();
                    SetCommandButtonsInteractable();

                    if (statusTxt)
                    {
                        statusTxt.color = Color.green;
                        statusTxt.text = "Robot at A!";
                    }
                }
                else if (cmd == "BUSY")
                {
                    if (statusTxt)
                    {
                        statusTxt.color = Color.yellow;
                        statusTxt.text = "Command rejected: Busy!";
                    }
                }
                else if (cmd == "OK")
                {
                    if (statusTxt)
                    {
                        statusTxt.color = Color.green;
                        statusTxt.text = "Command accepted!";
                    }
                }
                else if (cmd == "RWOK")
                {
                    if (bridge) bridge.NotifyTrialEnd();
                    atAReady = true;
                    SetCommandButtonsInteractable();
                    if (statusTxt)
                    {
                        statusTxt.color = Color.green;
                        statusTxt.text = "Returned to WAIT_START!";
                    }
                }
                else if (cmd == "TRND")
                {
                    if (statusTxt) { statusTxt.color = Color.white; statusTxt.text = "Trial ended."; }
                    if (bridge) bridge.NotifyTrialEnd();
                    atAReady = true;
                    SetCommandButtonsInteractable();
                }
                else if (cmd == "SESS")
                {
                    if (statusTxt) { statusTxt.color = Color.white; statusTxt.text = "Section ended."; }
                    if (bridge) bridge.NotifyTrialEnd();
                    bginReady = false;
                    atAReady = false;
                    SetCommandButtonsInteractable();
                }

            }

        }
    }

}


