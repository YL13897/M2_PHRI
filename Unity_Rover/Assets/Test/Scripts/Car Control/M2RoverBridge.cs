
// Unity
using System;
using System.IO;
using System.Text;
using UnityEngine;
using UnityEngine.SceneManagement;
using TMPro;

/* 
    Unity modes:
    1. Keyboard mode is a local test mode for rover, block, probability, and tracking logic. 
       a. Press T to toggle driving, and press ctrl+R to reset the rover position. 
       b. To check the block setup, click the "Start Block" button to start a block, and click "End Block" to end it.
    2. M2 mode is the experiment mode using M2 input, force, EMG, and communication.

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
        public LeaderHandler leader;
        public M2WorldFollower worldFollower;
        public InteractionEstimator estimator;
        public Transform roverTransform;
        public Rigidbody roverRigidbody;

        [Header("FSR Grasp Force")]
        [SerializeField] private bool fsrEnable = false;
        [SerializeField] private FsrGraspForceReader fsrReader;
        private bool fsrStateApplied = false;
        private bool lastFsrEnable = false;
        private FsrGraspForceReader lastFsrReader = null;

        [Header("Participant Calibration")]
        [SerializeField, Range(0f, 1f)] private float DisturbanceFactor = 0.20f;
        public float CalibForce = 50f;
        public float[] EmgRest = Array.Empty<float>();
        public float[] EmgBracing = Array.Empty<float>();
        public float[] EmgRef = Array.Empty<float>();

        [Header("M2 Standby A")]
        [Tooltip("M2 standby A.x. Runtime changes are applied outside an active block.")]
        [SerializeField] private float standbyX = 0.32f;
        [Tooltip("M2 standby A.y. Runtime changes are applied outside an active block.")]
        [SerializeField] private float standbyY = 0.12f;
        private const float XMin = 0.20f;
        private const float XMax = 0.44f;
        private const float XCenter = 0.32f;
        private const float YMin = 0.12f;
        private const float YMax = 0.38f;
        private const float YCenter = 0.12f;

        public float M2BoundaryMin => XMin;
        public float M2BoundaryMax => XMax;
        public float M2Center => float.IsNaN(sentA.x) ? XCenter : sentA.x;

        public enum UnityDriveMode
        {
            Mode1_Keyboard = 1,
            Mode2_M2 = 2
        }

        public enum DisturbanceForceMode
        {
            Calibrated = 0,
            Manual = 1
        }

        [Flags]
        public enum SpiEmgSlots
        {
            None = 0, // No slots selected
            CC11 = 1 << 0, // Slot 0, 0000000001=1. Take binary 1 and shift it left by 0 positions.
            CC12 = 1 << 1, // Slot 1, 0000000010=2
            CC21 = 1 << 2, // Slot 2, 0000000100=4
            CC22 = 1 << 3, // Slot 3, 0000001000=8
            CC31 = 1 << 4, // Slot 4, 0000010000=16
            CC32 = 1 << 5, // Slot 5, 0000100000=32
            CC41 = 1 << 6, // Slot 6, 0001000000=64
            CC42 = 1 << 7, // Slot 7, 0010000000=128
            CC51 = 1 << 8, // Slot 8, 0100000000=256
            CC52 = 1 << 9, // Slot 9, 1000000000=512
            All = (1 << DelsysEMG.MaxChannels) - 1
        }

        [Header("Unity Mode")]
        public UnityDriveMode unityMode = UnityDriveMode.Mode1_Keyboard;
        bool blockActive = false;
        [Tooltip("1=V1_HRI, 2=V2_PHRI")]
        public int hriModeCode = 2;

        [Header("pHRI Disturbance")]
        [Tooltip("Calibrated uses 40% of CalibForce; Manual uses the value below.")]
        [SerializeField] private DisturbanceForceMode disturbanceForceMode = DisturbanceForceMode.Calibrated;
        [Tooltip("Manual disturbance force in newtons.")]
        [SerializeField, Range(1f, 50f)] private float manualDisturbanceForce = 15f;

        [Header("Common hotkeys")]
        public bool enableHotkeys = true;

        [Header("M2 to Unity mapping")]
        public float targetWorldXScale = 5.0f; // Scaling factor sync/snap mapping between M2 and Unity, considered as the runtime control gain of M2 handle.
        public float worldXCenter = 0.0f; // The world X coordinate that corresponds to the M2 handle center (zero) position. 
        public bool autoSnapOnM2Reconnect = true; // Whether to automatically snap rover to M2-mapped position when M2 reconnects, to prevent jump forces due to position mismatch.

        bool enableKeyboardInMode1 = true;

        // (1) isPaused=true: completely frozen; 
        // (2) isPaused=false, isDriving=true: normal driving; 
        // (3) isPaused=false, isDriving=false: not moving forward but not frozen (can be continued by external logic)
        private bool isDriving = false;
        private bool isPaused = false;

        private bool emgHold = false; // Whether to hold the EMG score and stiffness estimation at their last values, used to hold up when EMG sensor disconnected during blocks.

        private bool lastM2Connected = false; // Track M2 connection status to detect reconnects for auto-snapping
        private Vector2 sentA = new Vector2(float.NaN, float.NaN);
        private bool hasSentDisturbanceState = false; // To track whether we've sent the disturbance state to M2, to avoid redundant commands
        private bool lastDisturbanceState = false; // To track the last disturbance state sent to M2, for change detection
        private Vector3 roverInitialPosition; // To store the initial pose of the rover for resetting, captured in Awake()
        
        // Flag indicating whether the reference point has been initialized.
        // This is used to ensure that we have a valid reference for computing relative handle movements and corresponding rover target positions,
        // especially after mode switches or block starts.
        private bool syncRefReady = false;

        // The M2 handle X position recorded at the start of the block (or after mode switch), used as the zero point for relative movement.
        private float handleXRef = 0.0f; 
        // The rover X position recorded at the start of the block (or after mode switch), used as the reference point for computing target X based on handle movement.
        private float roverXRef = 0.0f;

        // ----------------------------------------------------------------------------------------------------------------------
        
        [Header("Recording")]
        [SerializeField] private bool recordingEnabled = true;
        [SerializeField] private bool emgRecordFlag = true;
        [SerializeField] private bool scoreLogRecordFlag = true;
        [SerializeField] private bool disturbanceLogRecordFlag = true;

        // Delsys EMG background data collection
        [Header("EMG Settings")]
        [SerializeField] private bool delsysEnable;
        [SerializeField] private bool emgReconnectFlag = false;
        [SerializeField] private EMGFilter.EmgSignalView emgSignalMode = EMGFilter.EmgSignalView.Envelope;
        [SerializeField] private float emgScoreDownsampleHz = 100f;
        [SerializeField] private EMGFilter emgFilter;

        [Header("Recording Paths")]
        [SerializeField] private string emgPathOverride = "";
        [SerializeField] private string scoreLogPathOverride = "";
        [SerializeField] private string disturbanceLogPathOverride = "";
        private DelsysEMG delsysEMG = new DelsysEMG();
        private bool emgIsRecording = false;
        private bool scoreLogIsRecording = false;
        private bool disturbanceLogIsRecording = false;

        [Header("EMG Preview")]
        [SerializeField] private int emgPreviewMaxChannels = DelsysEMG.MaxChannels;
        [SerializeField] private float emgPreviewHz = 10f;

        [Header("EMG Channel Mapping")]
        [SerializeField] private int cc11Channel = 1;
        [SerializeField] private int cc12Channel = 2;
        [SerializeField] private int cc21Channel = 3;
        [SerializeField] private int cc22Channel = 4;
        [SerializeField] private int cc31Channel = 5;
        [SerializeField] private int cc32Channel = 6;
        [SerializeField] private int cc41Channel = 7;
        [SerializeField] private int cc42Channel = 8;
        [SerializeField] private int cc51Channel = 9;
        [SerializeField] private int cc52Channel = 10;
        [Tooltip("Logical EMG slots included in calibration fitting and SPI. All configured channels are still filtered and recorded.")]
        [SerializeField] private SpiEmgSlots spiCalibrationSlots = SpiEmgSlots.All;
        [SerializeField] private float stiffnessMin = 10f;
        [SerializeField] private float stiffnessMax = 100f;
        private TMP_Text emgScoreText;
        private double emgScoreTimestamp = 0.0; // Timestamp of the latest EMG score update, used for display and potential synchronization purposes.
        private bool emgShutdown = false; // Flag to indicate whether EMG has been shut down, to prevent multiple shutdown attempts.
        private double nextEmgUpdateTime = 0.0; // To control the update rate of EMG preview in the UI, we track the next allowed update time based on the specified preview Hz rate.
        private float nextEmgReconnectTime = 0f;
        private const float emgReconnectDelay = 5f;
        private string emgSessionFilePath;
        private string scoreLogSessionFilePath;
        private string disturbanceLogSessionFilePath;
        private StreamWriter scoreLogWriter;
        private StreamWriter disturbanceLogWriter;
        private int scoreLogFlushInterval = 200;
        private int scoreLogRowsSinceFlush = 0;
        private double t0 = -1.0;
        private int sec = 0;
        private bool recStarted = false;

        private void ApplyEmgRuntimeSettings()
        {
            delsysEMG.SignalView = emgSignalMode;
        }

        private void ApplyFsrRuntimeSettings()
        {
            bool enableFsrNow = fsrEnable;

            if (fsrStateApplied && enableFsrNow == lastFsrEnable && fsrReader == lastFsrReader)
                return;

            if (lastFsrReader != null && lastFsrReader != fsrReader)
                lastFsrReader.SetReaderEnabled(false);

            if (fsrReader != null)
                fsrReader.SetReaderEnabled(enableFsrNow);

            lastFsrEnable = enableFsrNow;
            lastFsrReader = fsrReader;
            fsrStateApplied = true;
        }

        private bool TryConnectEmg()
        {
            nextEmgReconnectTime = Time.unscaledTime + emgReconnectDelay;
            if (!delsysEMG.Connect())
                return false;

            if (!HasValidEmgMapping())
                ApplyDefaultEmgMapping();

            delsysEMG.SetProcessChannels(GetConfiguredEmgChannels());
            delsysEMG.StartAcquisition();
            if (!delsysEMG.IsRunning())
                return false;

            Debug.LogWarning("Delsys EMG connected and acquisition started.");
            return true;
        }

        #region Private methods
        private int GetCurrentBlockNumber()
        {
            if (ExperimentBlockControl.Instance != null && ExperimentBlockControl.Instance.CurrentBlockNumber > 0)
                return ExperimentBlockControl.Instance.CurrentBlockNumber;

            return 0; // Default to 0 if block number is not available
        }

        private string BuildBlockScopedEmgPath(string directory, string fileStem, string extension)
        {
            int blockNumber = GetCurrentBlockNumber();
            string safeExtension = string.IsNullOrWhiteSpace(extension) ? ".csv" : extension;
            string datedName = $"{fileStem}_{DateTime.Now:yyyyMMdd}_block{blockNumber}{safeExtension}";
            string fullPath = Path.Combine(directory, datedName);

            if (File.Exists(fullPath))
            {
                string dedupedName = $"{fileStem}_{DateTime.Now:yyyyMMdd}_block{blockNumber}_{DateTime.Now:HHmmss}{safeExtension}";
                fullPath = Path.Combine(directory, dedupedName);
            }

            return fullPath;
        }

        private string ConfigEMGFilePath()
        {
            if (!string.IsNullOrEmpty(emgSessionFilePath))
                return emgSessionFilePath;

            if (!string.IsNullOrWhiteSpace(emgPathOverride))
            {
                string overrideDir = emgPathOverride;
                if (Path.HasExtension(overrideDir))
                    overrideDir = Path.GetDirectoryName(overrideDir);
                if (string.IsNullOrWhiteSpace(overrideDir))
                    overrideDir = Directory.GetCurrentDirectory();
                emgSessionFilePath = BuildBlockScopedEmgPath(overrideDir, "EmgLogs", ".csv");
                string resolvedDir = Path.GetDirectoryName(emgSessionFilePath);
                if (!string.IsNullOrEmpty(resolvedDir))
                    Directory.CreateDirectory(resolvedDir);
                return emgSessionFilePath;
            }

            string emgDir = @"D:\yixianglin\Desktop\PHRI_Data"; // Default directory for EMG data.
            Directory.CreateDirectory(emgDir);
            emgSessionFilePath = BuildBlockScopedEmgPath(emgDir, "EmgLogs", ".csv");
            return emgSessionFilePath;
        }

        private string ConfigScoreLogFilePath()
        {
            if (!string.IsNullOrEmpty(scoreLogSessionFilePath))
                return scoreLogSessionFilePath;

            if (!string.IsNullOrWhiteSpace(scoreLogPathOverride))
            {
                scoreLogSessionFilePath = scoreLogPathOverride;
                string overrideDir = Path.GetDirectoryName(scoreLogSessionFilePath);
                if (!string.IsNullOrEmpty(overrideDir))
                    Directory.CreateDirectory(overrideDir);
                return scoreLogSessionFilePath;
            }

            string logDir = @"D:\yixianglin\Desktop\PHRI_Data";
            Directory.CreateDirectory(logDir);
            string fileName = $"ExpLogs_{DateTime.Now:yyyyMMdd_HHmmss}.csv";
            scoreLogSessionFilePath = Path.Combine(logDir, fileName);
            return scoreLogSessionFilePath;
        }

        private string ConfigDisturbanceLogFilePath()
        {
            if (!string.IsNullOrEmpty(disturbanceLogSessionFilePath))
                return disturbanceLogSessionFilePath;

            if (!string.IsNullOrWhiteSpace(disturbanceLogPathOverride))
            {
                disturbanceLogSessionFilePath = disturbanceLogPathOverride;
                string overrideDir = Path.GetDirectoryName(disturbanceLogSessionFilePath);
                if (!string.IsNullOrEmpty(overrideDir))
                    Directory.CreateDirectory(overrideDir);
                return disturbanceLogSessionFilePath;
            }

            string logDir = @"D:\yixianglin\Desktop\PHRI_Data";
            Directory.CreateDirectory(logDir);
            string fileName = $"DisturbanceLogs_{DateTime.Now:yyyyMMdd_HHmmss}.csv";
            disturbanceLogSessionFilePath = Path.Combine(logDir, fileName);
            return disturbanceLogSessionFilePath;
        }

        public bool IsEmgReady()
        {
            return delsysEnable && delsysEMG.IsConnected() && delsysEMG.IsRunning();
        }

        public bool IsScoreLogReady()
        {
            return ScoreManager.Instance != null;
        }

        public bool IsM2Connected()
        {
            return m2 != null && m2.IsInitialised() && m2.Client != null && m2.Client.IsConnected();
        }

        public bool TryGetM2Sample(out double time, double[] position, double[] force)
        {
            time = 0.0;

            // Reuse the existing position and force arrays if they are provided and have the correct length.
            if (position == null || force == null || position.Length < 2 || force.Length < 2)
                return false;

            if (!IsM2Connected())
                return false;

            if (m2.State == null)
                return false;

            var t = m2.State["t"];
            var x = m2.State["X"];
            var f = m2.State["F"];
            if (t == null || x == null || f == null || t.Length < 1 || x.Length < 2 || f.Length < 2)
                return false;

            time = t[0];
            position[0] = x[0];
            position[1] = x[1];
            force[0] = f[0];
            force[1] = f[1];
            return true;
        }

        public float[] GetScoreEnvelopeData()
        {
            if (!delsysEnable || !delsysEMG.IsConnected() || !delsysEMG.IsRunning())
                return Array.Empty<float>();

            return delsysEMG.GetScoreEnvelopeData();
        }

        public void FillConfiguredEmgChannels(int[] map)
        {
            if (map == null) return;
            if (map.Length > 0) map[0] = cc11Channel;
            if (map.Length > 1) map[1] = cc12Channel;
            if (map.Length > 2) map[2] = cc21Channel;
            if (map.Length > 3) map[3] = cc22Channel;
            if (map.Length > 4) map[4] = cc31Channel;
            if (map.Length > 5) map[5] = cc32Channel;
            if (map.Length > 6) map[6] = cc41Channel;
            if (map.Length > 7) map[7] = cc42Channel;
            if (map.Length > 8) map[8] = cc51Channel;
            if (map.Length > 9) map[9] = cc52Channel;
        }

        public bool TryGetScoreEmg(float[] values, int[] map, out double t, out long seq)
        {
            t = 0.0;
            seq = 0;
            if (values == null || map == null)
                return false;

            FillConfiguredEmgChannels(map);
            if (!delsysEnable || !delsysEMG.IsConnected() || !delsysEMG.IsRunning())
                return false;
            return delsysEMG.CopyScoreEnvelope(values, map, out t, out seq);
        }

        public bool TryGetInteractionForceX(out float fx)
        {
            fx = 0f;
            if (m2 == null) return false;
            if (!m2.IsInitialised() || m2.Client == null || !m2.Client.IsConnected()) return false;
            if (m2.State == null || m2.State["F"] == null || m2.State["F"].Length < 1) return false;
            fx = (float)m2.State["F"][0];
            return !float.IsNaN(fx) && !float.IsInfinity(fx);
        }

        public bool TryGetGraspForce(out float force)
        {
            force = 0f;
            return fsrEnable && fsrReader != null && fsrReader.TryGetForce(out force);
        }

        public bool TryGetGraspForceSample(out float voltage, out float force)
        {
            voltage = 0f;
            force = 0f;
            return fsrEnable && fsrReader != null && fsrReader.TryGetSample(out voltage, out force);
        }

        public string FsrStatus
        {
            get
            {
                if (!fsrEnable) return "disabled";
                if (fsrReader == null) return "reader missing";
                return fsrReader.Status;
            }
        }

        public bool RecordingEnabled => recordingEnabled;
        public bool EmgRecordFlag => recordingEnabled && emgRecordFlag;
        public bool EmgIsRecording => emgIsRecording;
        public bool ScoreLogRecordFlag => recordingEnabled && scoreLogRecordFlag;
        public bool ScoreLogIsRecording => scoreLogIsRecording;
        public bool DisturbanceLogRecordFlag => recordingEnabled && disturbanceLogRecordFlag;
        public bool DisturbanceLogIsRecording => disturbanceLogIsRecording;
        public bool CanStartAnyRecording => recordingEnabled &&
            ((emgRecordFlag && IsEmgReady() && !emgIsRecording) ||
             (scoreLogRecordFlag && IsScoreLogReady() && !scoreLogIsRecording) ||
             (disturbanceLogRecordFlag && !disturbanceLogIsRecording));
        public bool AnyRecordingActive => emgIsRecording || scoreLogIsRecording || disturbanceLogIsRecording;
        public bool IsTrialLoggingActive => unityMode == UnityDriveMode.Mode2_M2 && blockActive &&
            ExperimentBlockControl.Instance != null && ExperimentBlockControl.Instance.HasActiveSection;
        public bool IsEmgHold => emgHold;
        public float EmgEffortScore => estimator != null ? estimator.EmgScore : 0f;
        public float StiffnessCmd => estimator != null ? estimator.StiffnessCmd : 0f;
        public float StiffnessMin => stiffnessMin;
        public float StiffnessMax => stiffnessMax;
        public bool HasValidSpi => estimator != null && estimator.HasValidSpi;
        public int[] GetActiveEmgChannels() => delsysEMG.GetChannelsActiveSensor();
        public int[] GetConfiguredEmgChannels() => new[]
        {
            cc11Channel, cc12Channel, cc21Channel, cc22Channel, cc31Channel,
            cc32Channel, cc41Channel, cc42Channel, cc51Channel, cc52Channel
        };
        public int SpiCalibrationSlotMask => (int)spiCalibrationSlots;
        public bool IsSpiSlotEnabled(int slot) =>
            slot >= 0 && slot < DelsysEMG.MaxChannels &&
            (SpiCalibrationSlotMask & (1 << slot)) != 0;

        public bool HasValidEmgMapping()
        {
            int[] activeChannels = delsysEMG.GetChannelsActiveSensor();
            if (activeChannels == null || activeChannels.Length == 0)
                return false;

            int[] map = GetConfiguredEmgChannels();
            for (int i = 0; i < map.Length; i++)
            {
                if (map[i] <= 0)
                    continue;
                if (Array.IndexOf(activeChannels, map[i]) < 0)
                    return false;
            }

            return true;
        }

        private void ApplyDefaultEmgMapping()
        {
            int[] activeChannels = delsysEMG.GetChannelsActiveSensor();
            if (activeChannels == null || activeChannels.Length == 0)
                return;

            cc11Channel = activeChannels.Length > 0 ? activeChannels[0] : 0;
            cc12Channel = activeChannels.Length > 1 ? activeChannels[1] : 0;
            cc21Channel = activeChannels.Length > 2 ? activeChannels[2] : 0;
            cc22Channel = activeChannels.Length > 3 ? activeChannels[3] : 0;
            cc31Channel = activeChannels.Length > 4 ? activeChannels[4] : 0;
            cc32Channel = activeChannels.Length > 5 ? activeChannels[5] : 0;
            cc41Channel = activeChannels.Length > 6 ? activeChannels[6] : 0;
            cc42Channel = activeChannels.Length > 7 ? activeChannels[7] : 0;
            cc51Channel = activeChannels.Length > 8 ? activeChannels[8] : 0;
            cc52Channel = activeChannels.Length > 9 ? activeChannels[9] : 0;

            Debug.Log($"[M2RoverBridge] EMG mapping defaulted from active channels: {string.Join(", ", activeChannels)}");
        }

        private double GetGlobalT()
        {
            double now = Time.timeAsDouble;
            if (t0 < 0.0) t0 = now;
            return now - t0;
        }

        private bool TryStartEmgRecording()
        {
            if (!recordingEnabled || !emgRecordFlag || !IsEmgReady() || emgIsRecording) return false;

            string emgPath = ConfigEMGFilePath();
            sec = ExperimentBlockControl.Instance != null ? ExperimentBlockControl.Instance.CurrentSectionNumber : 0;
            double startT = GetGlobalT();
            delsysEMG.StartRecording(emgPath, startT, sec);
            emgIsRecording = true;
            return true;
        }

        private bool TryStopEmgRecording()
        {
            if (!emgIsRecording) return false;

            if (delsysEMG.IsConnected())
                delsysEMG.StopRecording();

            emgIsRecording = false;
            return true;
        }

        private bool TryStartScoreLogRecording()
        {
            if (!recordingEnabled || !scoreLogRecordFlag || !IsScoreLogReady() || scoreLogIsRecording) return false;

            if (scoreLogWriter == null)
            {
                string logPath = ConfigScoreLogFilePath();
                bool append = File.Exists(logPath) && new FileInfo(logPath).Length > 0;
                scoreLogWriter = new StreamWriter(logPath, append, Encoding.ASCII);
                scoreLogRowsSinceFlush = 0;
                if (!append)
                {
                    string header = "global_t,x_rover,x_leader,handle_fx,track_score,force_score,emg_score,total_score,boundary_penalty,block_index,section_index";
                    scoreLogWriter.WriteLine(header);
                    scoreLogWriter.Flush();
                }
            }

            scoreLogIsRecording = true;
            return true;
        }

        private bool TryStopScoreLogRecording()
        {
            if (!scoreLogIsRecording) return false;

            try { scoreLogWriter?.Flush(); } catch { }
            scoreLogRowsSinceFlush = 0;
            scoreLogIsRecording = false;
            return true;
        }

        private void CloseScoreLogSession()
        {
            try { scoreLogWriter?.Flush(); } catch { }
            try { scoreLogWriter?.Dispose(); } catch { }
            scoreLogWriter = null;
            scoreLogRowsSinceFlush = 0;
            scoreLogIsRecording = false;
        }

        private bool TryStartDisturbanceLogRecording()
        {
            if (!recordingEnabled || !disturbanceLogRecordFlag || disturbanceLogIsRecording) return false;

            if (disturbanceLogWriter == null)
            {
                string logPath = ConfigDisturbanceLogFilePath();
                bool append = File.Exists(logPath) && new FileInfo(logPath).Length > 0;
                disturbanceLogWriter = new StreamWriter(logPath, append, Encoding.ASCII);
                if (!append)
                {
                    disturbanceLogWriter.WriteLine("block,section,probability,direction,triggered");
                    disturbanceLogWriter.Flush();
                }
            }

            disturbanceLogIsRecording = true;
            return true;
        }

        private bool TryStopDisturbanceLogRecording()
        {
            if (!disturbanceLogIsRecording) return false;

            try { disturbanceLogWriter?.Flush(); } catch { }
            disturbanceLogIsRecording = false;
            return true;
        }

        private void CloseDisturbanceLogSession()
        {
            try { disturbanceLogWriter?.Flush(); } catch { }
            try { disturbanceLogWriter?.Dispose(); } catch { }
            disturbanceLogWriter = null;
            disturbanceLogIsRecording = false;
        }

        public bool StartAuxRecordingManual()
        {
            bool changed = false;
            if (TryStartEmgRecording()) changed = true;
            if (TryStartScoreLogRecording()) changed = true;
            if (TryStartDisturbanceLogRecording()) changed = true;
            return changed;
        }

        public bool StopAuxRecordingManual()
        {
            bool changed = false;
            if (TryStopEmgRecording()) changed = true;
            if (TryStopScoreLogRecording()) changed = true;
            if (TryStopDisturbanceLogRecording()) changed = true;
            estimator?.CloseDebugCsv();
            return changed;
        }

        public void CloseAuxRecordingSessions()
        {
            StopAuxRecordingManual();
            CloseScoreLogSession();
            CloseDisturbanceLogSession();
            emgSessionFilePath = null;
            recStarted = false;
        }

        private void ShutdownEMG()
        {
            if (emgShutdown) return;
            emgShutdown = true;

            try { CloseAuxRecordingSessions(); } catch (Exception ex) { Debug.LogWarning($"[M2RoverBridge] recording close during shutdown failed: {ex.Message}"); }
            if (!delsysEnable) return;
            try { if (delsysEMG.IsRunning()) delsysEMG.StopAcquisition(); } catch (Exception ex) { Debug.LogWarning($"[M2RoverBridge] EMG stop acquisition during shutdown failed: {ex.Message}"); }
            try { if (delsysEMG.IsConnected()) delsysEMG.Close(); } catch (Exception ex) { Debug.LogWarning($"[M2RoverBridge] EMG close during shutdown failed: {ex.Message}"); }
        }

        private void UpdateEmgScorePreview()
        {
            double now = Time.unscaledTimeAsDouble;
            double interval = 1.0 / emgPreviewHz;
            if (now < nextEmgUpdateTime) return;
            nextEmgUpdateTime = now + interval;

            bool hasText = emgScoreText != null;

            bool connected = delsysEMG.IsConnected();
            bool running = delsysEMG.IsRunning();
            bool active = delsysEnable && connected && running;
            if (!active)
            {
                string state = !delsysEnable ? "disabled" : (!connected ? "disconnected" : "stopped");
                string inactiveText = $"EMG: {state} | EMGSc: -- | t: {Time.timeAsDouble:0.0}s";
                if (hasText) emgScoreText.text = inactiveText;
                ScoreManager.Instance?.SetEmgPreview(0f, Time.timeAsDouble, inactiveText);
                return;
            }

            float[] emg = delsysEMG.GetScoreEnvelopeData();
            int[] activeChannels = delsysEMG.GetChannelsActiveSensor();
            if (emg == null || emg.Length == 0)
            {
                string emptyText = $"EMGSc: -- | t: {Time.timeAsDouble:0.0}s";
                if (hasText) emgScoreText.text = emptyText;
                ScoreManager.Instance?.SetEmgPreview(0f, Time.timeAsDouble, emptyText);
                return;
            }

            float emgSpi = estimator != null ? estimator.Spi : 0f;
            float emgEffortScore = EmgEffortScore;
            float stiffnessCmd = StiffnessCmd;
            emgScoreTimestamp = Time.timeAsDouble;

            string previewText = BuildEmgPreviewText(
                emg, activeChannels, emgSpi, emgEffortScore, stiffnessCmd);
            if (hasText) emgScoreText.text = previewText;
            ScoreManager.Instance?.SetEmgPreview(emgEffortScore, emgScoreTimestamp, previewText);
        }

        private string BuildEmgPreviewText(
            float[] raw, int[] activeChannels, float emgSpi, float emgEffortScore, float stiffnessCmd)
        {
            StringBuilder sb = new StringBuilder(160);
            sb.Append("SPI: ").Append(emgSpi.ToString("0.0000"))
              .Append(" | EMGSc: ").Append(emgEffortScore.ToString("0.00"))
              .Append(" | k: ").Append(stiffnessCmd.ToString("0.00"))
              .Append(" | t: ").Append(emgScoreTimestamp.ToString("0.0")).Append('s');

            int previewCount = Mathf.Min(
                Mathf.Min(raw.Length, activeChannels.Length),
                Mathf.Clamp(emgPreviewMaxChannels, 1, DelsysEMG.MaxChannels));
            int[] configuredChannels = GetConfiguredEmgChannels();

            for (int i = 0; i < previewCount; i++)
            {
                bool selected = false;
                for (int slot = 0; slot < configuredChannels.Length; slot++)
                {
                    if (configuredChannels[slot] == activeChannels[i] && IsSpiSlotEnabled(slot))
                    {
                        selected = true;
                        break;
                    }
                }

                sb.Append('\n')
                  .Append(selected ? "<color=#00FF00>" : "<color=#FFFFFF>") // Green for selected SPI channels, white for others
                  .Append("S").Append(activeChannels[i].ToString("D2")) //D2: zero-padded 2-digit channel number
                  .Append(": ").Append(raw[i].ToString("0.0000000"))
                  .Append("</color>");
            }

            return sb.ToString();
        }
        #endregion


        public void SetCalibForce(string value)
        {
            if (float.TryParse(value, out float parsed))
            {
                CalibForce = Mathf.Clamp(parsed, 1.0f, 100.0f);
                Debug.Log($"[M2RoverBridge] Participant CalibForce updated to: {CalibForce} N");
            }
        }

        public void SetCalibrationEmg(float[] emgRest, float[] emgBracing, float[] emgRef = null)
        {
            EmgRest = emgRest ?? Array.Empty<float>();
            EmgBracing = emgBracing ?? Array.Empty<float>();
            EmgRef = emgRef != null && emgRef.Length > 0 ? emgRef : EmgBracing;
            Debug.Log($"[M2RoverBridge] Calibration EMG updated. Rest={FormatArrayForLog(EmgRest)} Ref={FormatArrayForLog(EmgRef)} Bracing={FormatArrayForLog(EmgBracing)}");
        }

        private string FormatArrayForLog(float[] values)
        {
            if (values == null || values.Length == 0)
                return "--";

            string[] parts = new string[values.Length];
            for (int i = 0; i < values.Length; i++)
                parts[i] = values[i].ToString("0.000000");
            return "[" + string.Join(", ", parts) + "]";
        }

        // ----------------------------------------------------------------------------------------------------------------------
        // ------ Unity lifecycle methods ------

        void Awake()
        {
            Application.runInBackground = true;
            // Application.targetFrameRate = 50;

            emgIsRecording = false;

            if (rover == null)
                rover = FindFirstObjectByType<RoverHandler>();
            if (leader == null)
                leader = FindFirstObjectByType<LeaderHandler>();
            if (rover != null && roverTransform == null)
                roverTransform = rover.transform;
            if (rover != null && roverRigidbody == null)
                roverRigidbody = rover.GetComponent<Rigidbody>();
            if (worldFollower == null)
                worldFollower = FindFirstObjectByType<M2WorldFollower>();
            if (estimator == null)
                estimator = FindFirstObjectByType<InteractionEstimator>();

            if (roverTransform != null)
            {
                roverInitialPosition = roverTransform.position;
            }

            if (!GlobalRandomSeed.IsInitialized)
                UnityEngine.Random.InitState(GlobalRandomSeed.Seed);


            #region Initialize EMG sensors
            //Initialse Delsys EMG sensor
            if (delsysEnable)
            {
                if (emgFilter == null)
                    emgFilter = FindFirstObjectByType<EMGFilter>();

                delsysEMG.SetDebugSink(GetComponent<EmgDebugCsvLogger>());
                delsysEMG.Init(emgFilter, emgScoreDownsampleHz);
                ApplyEmgRuntimeSettings();
                if (!TryConnectEmg())
                    Debug.LogWarning("[M2RoverBridge] EMG init failed: cannot connect to Delsys server.");
            }
            #endregion

        }

        private void WriteScoreLogRow()
        {
            double globalT = GetGlobalT();

            if (!scoreLogIsRecording || scoreLogWriter == null || ScoreManager.Instance == null) return;

            float xRover = roverTransform != null ? roverTransform.position.x : 0f;
            float xLeader = leader != null ? leader.transform.position.x : 0f;
            int blockIndex = ExperimentBlockControl.Instance != null ? ExperimentBlockControl.Instance.CurrentBlockNumber : 0;
            int sectionIndex = ExperimentBlockControl.Instance != null ? ExperimentBlockControl.Instance.CurrentSectionNumber : 0;

            StringBuilder line = new StringBuilder(160);
            line.Append(globalT.ToString("F6")).Append(',')
                .Append(xRover.ToString("F6")).Append(',')
                .Append(xLeader.ToString("F6")).Append(',')
                .Append(ScoreManager.Instance.HandleFx.ToString("F6")).Append(',')
                .Append(ScoreManager.Instance.TrackSectionScore.ToString("F6")).Append(',')
                .Append(ScoreManager.Instance.ForceSectionScore.ToString("F6")).Append(',')
                .Append(ScoreManager.Instance.EmgSectionScore.ToString("F6")).Append(',')
                .Append(ScoreManager.Instance.DisplaySectionScore.ToString("F6")).Append(',')
                .Append(ScoreManager.Instance.BoundaryPenaltyScore.ToString("F6")).Append(',')
                .Append(blockIndex).Append(',')
                .Append(sectionIndex);

            scoreLogWriter.WriteLine(line.ToString());
            estimator?.WriteDebugCsv(globalT);
            scoreLogRowsSinceFlush++;

            // Flush the writer every scoreLogFlushInterval rows to ensure data is written to disk in a timely manner without flushing on every single write.
            if (scoreLogRowsSinceFlush >= scoreLogFlushInterval)
            {
                scoreLogWriter.Flush();
                scoreLogRowsSinceFlush = 0;
            }
        }

        public void LogDisturbanceDecision(int block, int section, float probability, int direction, bool triggered)
        {
            if (!recordingEnabled || !disturbanceLogRecordFlag || !disturbanceLogIsRecording || disturbanceLogWriter == null) return;

            StringBuilder line = new StringBuilder(64);
            line.Append(block).Append(',')
                .Append(section).Append(',')
                .Append(probability.ToString("F6")).Append(',')
                .Append(direction).Append(',')
                .Append(triggered ? 1 : 0);

            disturbanceLogWriter.WriteLine(line.ToString());
            disturbanceLogWriter.Flush();
        }

        // ----------------------------------------------------------------------------------------------------------------------
        // --- Public methods to be called by UI or other components ---

        // Switch between keyboard control and M2 control modes.
        // When switching from M2 to keyboard, also send SESS command to end M2 session and stop the current block.
        public void SetUnityMode(int mode)
        {
            UnityDriveMode targetMode = mode == 2 ? UnityDriveMode.Mode2_M2 : UnityDriveMode.Mode1_Keyboard;

            // If switching from M2 to keyboard mode, end the M2 session and block to ensure a clean transition.
            if (unityMode == UnityDriveMode.Mode2_M2 && targetMode == UnityDriveMode.Mode1_Keyboard)
            {
                if (m2 != null && m2.IsInitialised() && m2.Client != null && m2.Client.IsConnected())
                    m2.SendCmd("SESS");
                NotifyBlockEnd();
            }

            unityMode = targetMode;

            // Keyboard mode enabled, reset states to prepare for keyboard control.
            if (unityMode == UnityDriveMode.Mode1_Keyboard)
            {   
                blockActive = false;
                emgHold = false;
                enableKeyboardInMode1 = true;
                isPaused = false;
                isDriving = false;
                worldFollower?.ResetBias();
                Debug.Log("Switched to Keyboard control mode.");
            }

            // M2 control mode enabled, reset states to prepare for M2 control.
            else
            {
                blockActive = false;
                emgHold = false;
                enableKeyboardInMode1 = false;
                syncRefReady = false;
                worldFollower?.ResetBias();
                Debug.Log("Switched to M2 control mode.");
            }
        }

        // Apply the selected HRI mode.
        public void ApplyHriMode(int hriMode)
        {
            hriModeCode = hriMode;
        }

        // Start Unity-side block state after an M2 acknowledgement or a local keyboard start.
        public void NotifyBlockBegin()
        {
            blockActive = true;
            emgHold = false;
            if (unityMode == UnityDriveMode.Mode2_M2)
                syncRefReady = false; 
            if (!isPaused) isDriving = true;
            worldFollower?.ResetBias();
            recStarted = false;

            emgSessionFilePath = null;

            if (ScoreManager.Instance != null) ScoreManager.Instance.SetScorePaused(false);
        }

        // This should be called when the current block ends.
        public void NotifyBlockEnd()
        {
            CloseAuxRecordingSessions();

            blockActive = false;
            emgHold = false;
            syncRefReady = false;
            if (isDriving) isDriving = false;
            worldFollower?.ResetBias();
            hasSentDisturbanceState = false;
            lastDisturbanceState = false;
            if (ScoreManager.Instance != null) ScoreManager.Instance.SetScorePaused(true);
        }

        public void StopBlockMotionImmediate()
        {
            emgHold = false;
            isDriving = false;
            ApplyRoverSteer(0f);
            rover?.Brake();
            leader?.Brake();
            if (ScoreManager.Instance != null) ScoreManager.Instance.SetScorePaused(true);
        }

        public bool HoldBlockForEmgDisconnect()
        {
            if (!blockActive || unityMode != UnityDriveMode.Mode2_M2 || !delsysEnable || IsEmgReady())
                return false;

            emgHold = true;
            emgIsRecording = false;
            recStarted = false;
            isDriving = false;
            ApplyRoverSteer(0f);
            rover?.Brake();
            leader?.Brake();
            if (ScoreManager.Instance != null)
                ScoreManager.Instance.SetScorePaused(true);
            Debug.LogWarning("[EMG Guard] EMG disconnected. Auto movement stopped.");
            return true;
        }

        public bool ResumeEmgHold()
        {
            if (!emgHold)
                return false;
            if (delsysEnable && !IsEmgReady())
                return false;

            bool hasSection = ExperimentBlockControl.Instance != null && ExperimentBlockControl.Instance.HasActiveSection;
            if (blockActive && hasSection && recordingEnabled && !recStarted)
            {
                if (StartAuxRecordingManual())
                    recStarted = true;
            }

            emgHold = false;
            isPaused = false;
            isDriving = blockActive;
            if (ScoreManager.Instance != null)
                ScoreManager.Instance.SetScorePaused(!blockActive);
            Debug.Log("[EMG Guard] EMG reconnected. Auto movement resumed.");
            return true;
        }

        // Reset the rover to its initial pose, and clear velocities. This can be called on block start or when needed.
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
            blockActive = false;
            emgHold = false;
            syncRefReady = false;
            hasSentDisturbanceState = false;
            lastDisturbanceState = false;
            isPaused = false;
            isDriving = false;

            ResetRoverToInitialPose();
            ApplyRoverSteer(0f);
            // SendFeedbackForce(0f, 0f);

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

        public void ResetForNextExperimentBlock()
        {
            blockActive = false;
            emgHold = false;
            syncRefReady = false;
            hasSentDisturbanceState = false;
            lastDisturbanceState = false;
            isPaused = false;
            isDriving = false;
            emgSessionFilePath = null;

            ResetRoverToInitialPose();
            ApplyRoverSteer(0f);
            worldFollower?.ResetBias();
            leader?.ResetForBlockStart();

            if (ScoreManager.Instance != null)
            {
                ScoreManager.Instance.ResetBlockScores();
                ScoreManager.Instance.SetScorePaused(true);
            }

            ForceField.ResetFirstEntryFlag();

            var road = FindFirstObjectByType<EndlessRoad>();
            if (road != null)
                road.ResetRoadAroundPlayer();

            var fields = FindObjectsByType<ForceField>(FindObjectsInactive.Include, FindObjectsSortMode.None);

            // Reset all force fields, including inactive ones, to ensure they are in the correct state for the new block.
            foreach (var f in fields) 
            {
                if (f == null) continue;
                var go = f.gameObject;
                bool wasActive = go.activeSelf;
                go.SetActive(false);
                go.SetActive(wasActive);
            }
        }

        // =======================================================================================
        // --- Sync logic  ---

        private void TrySendA()
        {
            if (blockActive) return;
            if (m2 == null || !m2.IsInitialised() || m2.Client == null || !m2.Client.IsConnected()) return;

            float x = float.IsNaN(standbyX) || float.IsInfinity(standbyX)
                ? XCenter
                : Mathf.Clamp(standbyX, XMin, XMax);
            float y = float.IsNaN(standbyY) || float.IsInfinity(standbyY)
                ? YCenter
                : Mathf.Clamp(standbyY, YMin, YMax);
            if (Mathf.Approximately(sentA.x, x) && Mathf.Approximately(sentA.y, y)) return;

            m2.SendCmd("S_A", new double[] { x, y });

            sentA.Set(x, y);
            syncRefReady = false;
            worldFollower?.ResetBias();
            Debug.Log($"[M2RoverBridge] Sent standby A=({x:F3}, {y:F3})");
        }

        // Safely attempt to read the M2 handle X position from the M2 state.
        private bool TryGetM2HandleX(out float handleX)
        {
            handleX = 0f;
            if (m2 == null || !m2.IsInitialised() || m2.Client == null || !m2.Client.IsConnected()) return false;
            if (m2.State == null || m2.State["X"] == null || m2.State["X"].Length == 0) return false;
            handleX = (float)m2.State["X"][0];
            if (float.IsNaN(handleX) || float.IsInfinity(handleX)) return false;
            return true;
        }

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
            // Debug.Log($"time={Time.time:F3}s, deltatime={Time.deltaTime:F3}s");

            if (roverRigidbody != null)
            {
                Vector3 v = roverRigidbody.linearVelocity;
                v.x = 0f;
                roverRigidbody.linearVelocity = v;
                roverRigidbody.angularVelocity = Vector3.zero;
            }
        }

        // =======================================================================================
        // --- M2 feedback logic ---




        // Overload for sending zero force without parameters.
        private void TrySendDisturbanceStateToM2()
        {
            if (!blockActive) return;
            if (m2 == null || !m2.IsInitialised() || m2.Client == null || !m2.Client.IsConnected()) return;

            // Use the same Unity disturbance source u(t) for both local simulation and M2 signaling.
            bool state = ForceField.DisturbanceU > 0.5f;
            int direction = ExperimentBlockControl.Instance != null ? ExperimentBlockControl.Instance.CurrentDirection : -1;
            
            float disturbanceMagnitude = disturbanceForceMode == DisturbanceForceMode.Manual
                ? manualDisturbanceForce
                : CalibForce * DisturbanceFactor;
            disturbanceMagnitude = Mathf.Clamp(disturbanceMagnitude, 1f, 50f);
            double disturbanceCmd = state ? (direction < 0 ? -disturbanceMagnitude : disturbanceMagnitude) : 0.0;

            if (!hasSentDisturbanceState || state != lastDisturbanceState)
            {
                m2.SendCmd("DSTR", new double[] { disturbanceCmd }); // Send disturbance command to M2.
                lastDisturbanceState = state;
                hasSentDisturbanceState = true;
            }
        }

        // =======================================================================================
        // --- Common control application logic ---
        private void ApplyRoverSteer(float steer)
        {
            rover.SetInputRaw(new Vector2(steer, 0f));
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
            bool emgReady = IsEmgReady(); // Check if EMG is connected and running
            
            // EMG is enabled but not ready, this could be due to EMG disconnection or stop.
            if (delsysEnable && !emgReady)
            {
                if (emgIsRecording)
                {
                    emgIsRecording = false;
                    recStarted = false;
                }

                if (unityMode == UnityDriveMode.Mode2_M2 && blockActive && !emgHold)
                    HoldBlockForEmgDisconnect();
            }

            // Attempt to reconnect EMG if it's not ready and either we're in EMG hold or EMG reconnect is enabled and block is not active.
            if (delsysEnable && !emgReady && Time.unscaledTime >= nextEmgReconnectTime &&
                (emgHold || (emgReconnectFlag && !blockActive)))
                TryConnectEmg();

            ApplyFsrRuntimeSettings();

            if (!recordingEnabled)
            {
                if (AnyRecordingActive)
                    StopAuxRecordingManual();
                recStarted = false;
            }

            bool connectedNow = m2 != null && m2.IsInitialised() && m2.Client != null && m2.Client.IsConnected();

            if (connectedNow)
                TrySendA();

            // Detect M2 reconnect to trigger auto-sync if enabled, to prevent large position mismatches that can cause jump forces.
            if (unityMode == UnityDriveMode.Mode2_M2 && connectedNow && !lastM2Connected && autoSnapOnM2Reconnect)
            {
                SyncRoverToCurrentM2MappedX();
                hasSentDisturbanceState = false;
            }
            if (!connectedNow)
            {
                hasSentDisturbanceState = false;
                sentA.y = float.NaN; // Reset sentA.y to NaN to force re-sending standby A when M2 reconnects.
            }
            lastM2Connected = connectedNow;

            if (unityMode == UnityDriveMode.Mode2_M2 && blockActive)
                TrySendDisturbanceStateToM2();

            bool hasSection = ExperimentBlockControl.Instance != null && ExperimentBlockControl.Instance.HasActiveSection;
            sec = hasSection && ExperimentBlockControl.Instance != null ? ExperimentBlockControl.Instance.CurrentSectionNumber : 0;
            delsysEMG.SetSec(sec);

            if (unityMode == UnityDriveMode.Mode2_M2 && blockActive && !emgHold && !recStarted && hasSection)
            {
                if (recordingEnabled)
                {
                    StartAuxRecordingManual();
                    recStarted = true;
                }
            }

            if (enableHotkeys)
            {
                
                bool ctrl = Input.GetKey(KeyCode.LeftControl);
                if (Input.GetKeyDown(KeyCode.T) && !ctrl)
                {
                    Debug.LogWarning("Hotkey T detected");
                    isPaused = false;
                    isDriving = true;
                }
                if (Input.GetKeyDown(KeyCode.T) && ctrl)
                {
                    isPaused = true;
                    isDriving = false;
                }

                if (Input.GetKeyDown(KeyCode.R) && ctrl )
                {   
                    SceneManager.LoadScene(SceneManager.GetActiveScene().name);
                }
            }


            rover.SetPreserveLateralVelocity(false);
            rover.SetPaused(isPaused);
            rover.SetDriving(isDriving && !isPaused && !emgHold);

            leader.SetPreserveLateralVelocity(unityMode == UnityDriveMode.Mode2_M2);
            leader.SetPaused(isPaused);
            leader.SetDriving(isDriving && !isPaused && !emgHold);

            if (ScoreManager.Instance != null)
                ScoreManager.Instance.SetScorePaused(isPaused || !blockActive || emgHold);

            ApplyEmgRuntimeSettings();
            estimator?.Refresh();
            UpdateEmgScorePreview();

        }    


        void FixedUpdate()
        {
            if (rover == null) return; // Safety check

            if (unityMode == UnityDriveMode.Mode1_Keyboard) // Keyboard control mode
            {
                HandleKeyboardModeFixedUpdate();
                return;
            }

            else if (emgHold)
            {
                ApplyRoverSteer(0f);
                return;
            }

            // Can't get handle X, can't proceed with M2 control
            else if (!TryGetM2HandleX(out _) && blockActive)
            {
                // SendFeedbackForce(0f, 0f);
                Debug.LogWarning("[M2RoverBridge] Unable to read M2 handle X position. Check M2 connection and state.");
                return;
            }

            // M2 control mode but block not active, ensure rover is not moving and send zero feedback.
            else if (!blockActive)
            {
                ApplyRoverSteer(0f);
                return;
            }

            // M2 control mode
            else
            {
                ApplyRoverSteer(0f);
                WriteScoreLogRow();
            }
        }

        // Ensure EMG resources are properly released when the object is destroyed or application quits.
        void OnDestroy() 
        {
            ShutdownEMG();
        }

        // Also ensure EMG shutdown on application quit to cover cases where the object might not be destroyed properly.
        private void OnApplicationQuit() 
        {
            ShutdownEMG();
        }

    }
}
