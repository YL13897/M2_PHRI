using System;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

namespace CORC.Demo
{
    // Receives grasp force data from the external Python FSR bridge via UDP.
    public sealed class FsrGraspForceReader : MonoBehaviour
    {
        [Header("UDP Input")]
        [SerializeField] private int udpPort = 50110;

        private readonly object dataLock = new object();
        private Thread receiveThread;
        private UdpClient udpClient;
        private volatile bool running;
        private bool hasSample;
        private float voltage;
        private float forceN;
        private int receivedPackets;
        private int rejectedPackets;
        private string status = "Disabled";

        public string Status
        {
            get
            {
                lock (dataLock)
                    return status;
            }
        }

        public void SetReaderEnabled(bool enabled)
        {
            if (enabled)
                StartReader();
            else
                StopReader();
        }

        public void StartReader()
        {
            if (running)
                return;

            running = true;
            ClearSample();
            StartUdp();
        }

        public void StopReader()
        {
            running = false;
            try { udpClient?.Close(); } catch { }
            udpClient = null;

            if (receiveThread != null && receiveThread.IsAlive)
                receiveThread.Join(300);
            receiveThread = null;

            ClearSample();
            SetStatus("Disabled");
        }

        public bool TryGetForce(out float force)
        {
            force = 0f;
            lock (dataLock)
            {
                if (!hasSample)
                    return false;

                force = forceN;
                return true;
            }
        }

        public bool TryGetSample(out float fsrVoltage, out float fsrForceN)
        {
            fsrVoltage = 0f;
            fsrForceN = 0f;
            lock (dataLock)
            {
                if (!hasSample)
                    return false;

                fsrVoltage = voltage;
                fsrForceN = forceN;
                return true;
            }
        }

        private void StartUdp()
        {
            try
            {
                udpClient = new UdpClient(new IPEndPoint(IPAddress.Loopback, Mathf.Max(1, udpPort)));
                receiveThread = new Thread(ReceiveLoop)
                {
                    IsBackground = true,
                    Name = "FSR UDP Receiver"
                };
                receiveThread.Start();
                SetStatus($"UDP listening {udpPort}");
            }
            catch (Exception ex)
            {
                running = false;
                SetStatus($"UDP open failed: {ex.Message}");
            }
        }

        private void ReceiveLoop()
        {
            IPEndPoint remote = new IPEndPoint(IPAddress.Any, 0);
            while (running)
            {
                try
                {
                    byte[] bytes = udpClient.Receive(ref remote);
                    ProcessPacket(Encoding.ASCII.GetString(bytes));
                }
                catch (ObjectDisposedException)
                {
                    break;
                }
                catch (SocketException ex)
                {
                    if (running)
                        SetStatus($"UDP read failed: {ex.Message}");
                    break;
                }
            }
        }

        private void ProcessPacket(string text)
        {
            if (TryParseStatus(text, out string appStatus))
            {
                SetStatus(appStatus);
                return;
            }

            if (TryParsePacket(text, out float fsrVoltage, out float fsrForce))
            {
                lock (dataLock)
                {
                    voltage = fsrVoltage;
                    forceN = fsrForce;
                    hasSample = true;
                    receivedPackets++;
                    status = $"OK udp={receivedPackets} V={fsrVoltage:0.000} F={fsrForce:0.00}N";
                }
                return;
            }

            lock (dataLock)
            {
                rejectedPackets++;
                status = $"UDP rejected #{rejectedPackets}";
            }
        }

        private bool TryParseStatus(string text, out string appStatus)
        {
            appStatus = "";
            if (string.IsNullOrWhiteSpace(text))
                return false;

            string value = text.Trim();
            if (!value.StartsWith("STATUS,", StringComparison.OrdinalIgnoreCase))
                return false;

            appStatus = value.Length > 7 ? value.Substring(7) : "External app status";
            return true;
        }

        private bool TryParsePacket(string text, out float fsrVoltage, out float fsrForce)
        {
            fsrVoltage = 0f;
            fsrForce = 0f;
            if (string.IsNullOrWhiteSpace(text))
                return false;

            string[] parts = text.Trim().Split(',');
            int offset = parts.Length > 0 && parts[0].Trim().Equals("FSR", StringComparison.OrdinalIgnoreCase) ? 1 : 0;
            return parts.Length >= offset + 2
                && float.TryParse(parts[offset], NumberStyles.Float, CultureInfo.InvariantCulture, out fsrVoltage)
                && float.TryParse(parts[offset + 1], NumberStyles.Float, CultureInfo.InvariantCulture, out fsrForce);
        }

        private void ClearSample()
        {
            lock (dataLock)
                hasSample = false;
        }

        private void SetStatus(string value)
        {
            lock (dataLock)
                status = value;
        }

        private void OnDisable()
        {
            StopReader();
        }

        private void OnDestroy()
        {
            StopReader();
        }

        private void OnApplicationQuit()
        {
            StopReader();
        }
    }
}
