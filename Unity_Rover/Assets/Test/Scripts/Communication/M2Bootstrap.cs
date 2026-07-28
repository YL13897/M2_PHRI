// M2Bootstrap.cs (with auto-assign + optional auto-reconnect)
using UnityEngine;
using System.Collections; // for IEnumerator/Coroutine

namespace CORC.Demo
{
    [RequireComponent(typeof(CORC.CORCM2))]
    public class M2Bootstrap : MonoBehaviour
    {
        [Header("Server")]
        // public string serverIp = "192.168.8.104";
        // public string serverIp = "169.254.94.112";
        public string serverIp = "192.168.10.2";
        public int serverPort = 2048;

        [Header("Connection")]
        public bool autoReconnect = true; // automatically try to reconnect if connection is lost
        public float reconnectInterval = 2f; // seconds between reconnect attempts

        [Header("Refs")]
        public CORC.CORCM2 m2;  
        public M2RoverBridge bridge;

        private Coroutine reconnectCo;

        bool ShouldEnableCommunication()
        {
            if (bridge == null)
                bridge = FindFirstObjectByType<M2RoverBridge>();
            return bridge != null && bridge.unityMode == M2RoverBridge.UnityDriveMode.Mode2_M2;
        }

        void Awake()
        {
            // Try to auto-assign CORCM2 if not set in inspector
            if (m2 == null)
            {
                m2 = GetComponent<CORC.CORCM2>();
                if (m2 != null)
                {
                    Debug.Log("Get CORCM2 component.");
                }
            }

        }

        void Start()
        {
            if (m2 == null)
            {
                Debug.LogError("[M2Bootstrap] Please assign CORCM2 component.");
                return;
            }

            if (bridge == null)
                bridge = FindFirstObjectByType<M2RoverBridge>();

            // Only connect if in M2 mode at start, otherwise wait for mode switch (e.g., from keyboard to M2)
            if (ShouldEnableCommunication())
                TryConnectOnce();

            // Start auto-reconnect loop if enabled
            if (autoReconnect)
                reconnectCo = StartCoroutine(AutoReconnectLoop());
        }


        void TryConnectOnce()
        {
            m2.Init(serverIp, serverPort);
        }


        IEnumerator AutoReconnectLoop()
        {
            while (m2 != null)
            {
                if (!ShouldEnableCommunication())
                {
                    if (m2.IsInitialised() || (m2.Client != null && m2.Client.IsConnected()))
                    {
                        Debug.Log("[M2Bootstrap] Keyboard mode active: disconnect communication.");
                        m2.Disconnect();
                    }
                    yield return new WaitForSeconds(reconnectInterval);
                    continue;
                }

                // If not connected, try to reconnect
                if (!m2.IsInitialised() || !m2.Client.IsConnected())
                {
                    Debug.Log("[Reconnect] try " + serverIp + ":" + serverPort);
                    m2.Disconnect();         // Ensure clean state
                    TryConnectOnce();        // Try to connect
                }
                yield return new WaitForSeconds(reconnectInterval);
            }
        }


        void OnApplicationQuit()
        {
            if (reconnectCo != null) StopCoroutine(reconnectCo);

            if (m2 != null)
            {
                m2.Disconnect();
            }
        }

        void OnDisable()
        {
            if (reconnectCo != null) StopCoroutine(reconnectCo);
            if (m2 != null)
            {
                Debug.Log("[M2Bootstrap] OnDisable - disconnecting M2.");
                m2.Disconnect();
            }
        }


    }
}
