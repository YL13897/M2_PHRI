using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace CORC.Demo
{
    public class CalibrationManager : MonoBehaviour
    {
        [Serializable]
        private class CalibrationSample
        {
            public bool keyboardMode;
            public bool hasM2;
            public double m2Time;
            public double[] m2Position = Array.Empty<double>();
            public double[] m2Force = Array.Empty<double>();
            public float[] emg = Array.Empty<float>();
            public int[] emgChannels = Array.Empty<int>();
            public int[] emgSlots = Array.Empty<int>();
            public int spiSlotMask;
            public bool hasFsr;
            public float fsrVoltage;
            public float graspForceN;
            public float calibForce;
            public long unityTicksUtc;
        }

        [Serializable]
        private class CalibrationResponse
        {
            public float calibForce;
            public float emgScale;
            public float threshold;
            public float[] emgRest = Array.Empty<float>();
            public float[] emgBracing = Array.Empty<float>();
            public float[] emgRef = Array.Empty<float>();
            public float[] emgForceWeights = Array.Empty<float>();
            public int[] emgForceSlots = Array.Empty<int>();
            public float emgForceBias;
            public float spiRest;
            public float spiRef;
            public string note;
        }

        [Header("Refs")]
        public M2RoverBridge bridge;
        public InteractionEstimator estimator;

        [Header("External Launch")]
        public bool autoLaunchExternalProgram = true;
        public string pythonExecutable = "C:\\Users\\yixianglin\\AppData\\Local\\Programs\\Python\\Python313\\python.exe";
        public string externalProgramPath = "D:\\Unity Projects\\Rover\\Assets\\Test\\Calibration\\calibration_client.py";
        public string externalProgramArgs = ""; // Additional command-line arguments for the external program (optional)

        [Header("Local TCP")]
        public string listenAddress = "127.0.0.1";
        public int listenPort = 25001;
        public float sampleSendHz = 40f;
        private const int FlushEverySamples = 10;

        [Header("Last Returned Params")]
        public float lastReturnedEmgScale;
        public float lastReturnedThreshold;
        [TextArea]
        public string lastResponseNote = "";

        public bool IsListenerRunning => listener != null;
        public bool IsClientConnected => client != null && client.Connected;
        public bool IsCalibrationActive => IsClientConnected || (externalProcess != null && !externalProcess.HasExited);
        public static double LastCloseTime { get; private set; } = -1000.0;
        public string LastStatus { get; private set; } = "Idle";

        private TcpListener listener;
        private Thread listenerThread;
        private TcpClient client;
        private StreamWriter clientWriter;
        private StreamReader clientReader;
        private readonly ConcurrentQueue<string> inboundLines = new ConcurrentQueue<string>();
        private readonly object clientLock = new object();
        private Process externalProcess;
        private float nextSendTime;
        private float sampleSendInterval;
        private int samplesSinceFlush;
        private readonly CalibrationSample sample = new CalibrationSample();
        private readonly double[] m2Position = new double[2];
        private readonly double[] m2Force = new double[2];
        private readonly int[] emgSlots = new int[DelsysEMG.MaxChannels];

        public bool CanStartCalibration()
        {
            ClearExitedExternalProcess();
            return !IsCalibrationActive;
        }

        public bool StartCalibrationSession(bool keyboardMode)
        {
            if (!CanStartCalibration())
                return false;

            StopTcpRuntime();
            StartListener();

            if (autoLaunchExternalProgram && !string.IsNullOrWhiteSpace(externalProgramPath))
            {
                if (!StartExternalProcess())
                    return false;
            }

            LastStatus = keyboardMode
                ? "Calibration started in keyboard debug mode."
                : "Calibration started. Waiting for external client...";
            nextSendTime = 0f;
            return true;
        }

        public void StopCalibrationSession()
        {
            StopExternalProcess();
            StopTcpRuntime();
            LastStatus = "Calibration stopped.";
        }

        private void Awake()
        {
            if (bridge == null)
                bridge = FindFirstObjectByType<M2RoverBridge>();
            if (estimator == null)
                estimator = FindFirstObjectByType<InteractionEstimator>();
            sampleSendInterval = sampleSendHz > 0f ? 1f / sampleSendHz : 0.025f;
        }

        private void Update()
        {
            DrainInboundMessages();

            if (!IsClientConnected)
                return;

            if (Time.unscaledTime < nextSendTime)
                return;

            nextSendTime = Time.unscaledTime + sampleSendInterval;
            SendSnapshot();
        }

        private void OnDestroy()
        {
            StopCalibrationSession();
        }

        private void OnApplicationQuit()
        {
            StopCalibrationSession();
        }

        private void StartListener()
        {
            if (listener != null)
                return;

            IPAddress ip = IPAddress.Parse(listenAddress);
            listener = new TcpListener(ip, listenPort);
            listener.Start();

            listenerThread = new Thread(ListenLoop) { IsBackground = true };
            listenerThread.Start();
        }

        private void ListenLoop()
        {
            try
            {
                LastStatus = $"Listening on {listenAddress}:{listenPort}";
                while (listener != null)
                {
                    TcpClient accepted = listener.AcceptTcpClient();
                    accepted.NoDelay = true;
                    ConfigureClient(accepted);
                    LastStatus = "External client connected.";
                }
            }
            catch (SocketException ex)
            {
                if (listener != null)
                {
                    LastStatus = $"Calibration listener error: {ex.Message}";
                    Debug.LogWarning($"[CalibrationManager] Listener socket error: {ex.Message}");
                }
            }
            catch (ObjectDisposedException)
            {
            }
            finally
            {
                listenerThread = null;
            }
        }

        private void ConfigureClient(TcpClient accepted)
        {
            lock (clientLock)
            {
                CloseClientLocked();
                client = accepted;
                NetworkStream stream = client.GetStream();
                clientWriter = new StreamWriter(stream, new UTF8Encoding(false)) { AutoFlush = false };
                clientReader = new StreamReader(stream, Encoding.UTF8);
                samplesSinceFlush = 0;
                var receiveThread = new Thread(ReceiveLoop) { IsBackground = true };
                receiveThread.Start();
            }
        }

        private void ReceiveLoop()
        {
            try
            {
                while (true)
                {
                    StreamReader reader;
                    lock (clientLock)
                    {
                        reader = clientReader;
                    }

                    if (reader == null)
                        break;

                    string line = reader.ReadLine();

                    if (line == null)
                        break;

                    if (!string.IsNullOrWhiteSpace(line))
                        inboundLines.Enqueue(line);
                }
            }
            catch (IOException ex)
            {
                Debug.LogWarning($"[CalibrationManager] Receive loop closed: {ex.Message}");
            }
            finally
            {
                lock (clientLock)
                {
                    CloseClientLocked(false);
                }
                LastStatus = "External client disconnected.";
            }
        }

        private void DrainInboundMessages()
        {
            while (inboundLines.TryDequeue(out string line))
            {
                CalibrationResponse response = null;
                try
                {
                    response = JsonUtility.FromJson<CalibrationResponse>(line);
                }
                catch (Exception ex)
                {
                    Debug.LogWarning($"[CalibrationManager] Failed to parse response: {ex.Message}");
                }

                if (response == null)
                    continue;

                ApplyResponse(response);
            }
        }

        // Applies the received calibration response to the M2RoverBridge and updates the last returned parameters and status message accordingly.
        private void ApplyResponse(CalibrationResponse response)
        {
            if (bridge != null)
            {
                if (response.calibForce > 0f)
                {
                    bridge.CalibForce = response.calibForce;
                    FindFirstObjectByType<M2UiPanel>()?.RefreshCalibForceInput();
                }
                bridge.SetCalibrationEmg(response.emgRest, response.emgBracing, response.emgRef);
            }

            estimator?.ApplyCalibration(
                response.emgForceWeights,
                response.emgForceSlots,
                response.emgForceBias,
                response.spiRest,
                response.spiRef);

            lastReturnedEmgScale = response.emgScale;
            lastReturnedThreshold = response.threshold;
            lastResponseNote = response.note ?? string.Empty;
            LastStatus = string.IsNullOrWhiteSpace(response.note)
                ? "Calibration parameters received."
                : response.note;
        }


        // SendSnapshot() is used to stream current Unity data to the external calibration GUI over TCP.
        // It builds a snapshot of the current calibration state, including M2 data if available, and sends it to the connected client as a JSON string.
        private void SendSnapshot()
        {
            if (bridge == null)
                return;

            sample.keyboardMode = bridge.unityMode == M2RoverBridge.UnityDriveMode.Mode1_Keyboard;
            sample.calibForce = bridge.CalibForce;
            sample.unityTicksUtc = DateTime.UtcNow.Ticks;

            sample.hasM2 = bridge.TryGetM2Sample(out double m2Time, m2Position, m2Force);
            sample.m2Time = m2Time;
            sample.m2Position = sample.hasM2 ? m2Position : Array.Empty<double>();
            sample.m2Force = sample.hasM2 ? m2Force : Array.Empty<double>();

            float[] emg = bridge.GetScoreEnvelopeData();
            sample.emg = emg;
            sample.emgChannels = emg.Length > 0 ? bridge.GetActiveEmgChannels() : Array.Empty<int>();
            bridge.FillConfiguredEmgChannels(emgSlots);
            sample.emgSlots = emgSlots;
            sample.spiSlotMask = bridge.SpiCalibrationSlotMask;
            sample.hasFsr = bridge.TryGetGraspForceSample(out float fsrVoltage, out float graspForce);
            sample.fsrVoltage = fsrVoltage;
            sample.graspForceN = graspForce;

            string line = JsonUtility.ToJson(sample);

            lock (clientLock)
            {
                if (clientWriter == null)
                    return;

                try
                {
                    clientWriter.WriteLine(line);
                    samplesSinceFlush++;
                    if (samplesSinceFlush >= FlushEverySamples)
                    {
                        clientWriter.Flush();
                        samplesSinceFlush = 0;
                    }
                }
                catch (IOException ex)
                {
                    Debug.LogWarning($"[CalibrationManager] Send failed: {ex.Message}");
                    CloseClientLocked(false);
                    LastStatus = "External client disconnected during send.";
                }
            }
        }

        private bool StartExternalProcess()
        {
            ClearExitedExternalProcess();

            if (externalProcess != null && !externalProcess.HasExited)
                return true;

            try
            {
                string args = BuildProcessArgs();
                var startInfo = new ProcessStartInfo
                {
                    FileName = pythonExecutable,
                    Arguments = args,
                    UseShellExecute = false,
                    CreateNoWindow = false,
                    WorkingDirectory = ResolveWorkingDirectory()
                };
                externalProcess = Process.Start(startInfo);
                return externalProcess != null;
            }
            catch (Exception ex)
            {
                LastStatus = $"External launch failed: {ex.Message}";
                Debug.LogWarning($"[CalibrationManager] External launch failed: {ex.Message}");
                return false;
            }
        }

        private void ClearExitedExternalProcess()
        {
            if (externalProcess == null || !externalProcess.HasExited)
                return;

            externalProcess.Dispose();
            externalProcess = null;
        }

        // Builds the Python script arguments and appends any user-defined arguments.
        private string BuildProcessArgs()
        {
            string quotedScript = string.IsNullOrWhiteSpace(externalProgramPath)
                ? string.Empty
                : $"\"{externalProgramPath}\"";
            string baseArgs = $"{quotedScript} --host {listenAddress} --port {listenPort}";

            if (string.IsNullOrWhiteSpace(externalProgramArgs))
                return baseArgs.Trim();
            return $"{baseArgs} {externalProgramArgs}".Trim();
        }

        private string ResolveWorkingDirectory()
        {
            if (!string.IsNullOrWhiteSpace(externalProgramPath))
            {
                string dir = Path.GetDirectoryName(externalProgramPath);
                if (!string.IsNullOrWhiteSpace(dir))
                    return dir;
            }
            return Application.dataPath;
        }

        private void StopExternalProcess()
        {
            if (externalProcess == null)
                return;

            try
            {
                if (!externalProcess.HasExited)
                {
                    externalProcess.CloseMainWindow();
                    if (!externalProcess.WaitForExit(1500))
                        externalProcess.Kill();
                }
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[CalibrationManager] External stop failed: {ex.Message}");
            }
            finally
            {
                externalProcess.Dispose();
                externalProcess = null;
            }
        }

        private void StopTcpRuntime()
        {
            lock (clientLock)
            {
                CloseClientLocked();
            }

            try { listener?.Stop(); } catch { }
            listener = null;

            if (listenerThread != null && listenerThread.IsAlive)
            {
                try { listenerThread.Join(200); } catch { }
            }
            listenerThread = null;
        }

        private void CloseClientLocked(bool flush = true)
        {
            bool hadClient = clientWriter != null || clientReader != null || client != null;
            if (hadClient)
                LastCloseTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;

            if (flush)
            {
                try { clientWriter?.Flush(); } catch { }
            }
            try { clientWriter?.Dispose(); } catch { }
            try { clientReader?.Dispose(); } catch { }
            try { client?.Close(); } catch { }
            clientWriter = null;
            clientReader = null;
            client = null;
            samplesSinceFlush = 0;
        }
    }
}
