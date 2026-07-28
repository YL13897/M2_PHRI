// M2Proxy.cs
using UnityEngine;

// This file defines the M2Proxy class, 
// a proxy to access CORC M2 robot state and send commands, 
// hiding the communication details and allowing to easily switch to another communication implementation if needed.
namespace CORC.Demo
{
    public interface IM2Proxy
    {
        bool IsReady { get; }
        double Time { get; }
        double[] X { get; }   // pos[2]
        double[] dX { get; }  // vel[2]
        double[] F { get; }   // force[2]

        void SendCmd(string cmd4, double[] args = null);
        System.Collections.Generic.List<CORC.FLNLCmd> DrainCmds();
    }

    /// <summary>
    /// Proxy class to access CORC M2 robot state and send commands, 
    /// hiding the communication details and allowing to easily switch to another communication implementation if needed.
    /// </summary>
    public class M2Proxy : IM2Proxy
    {
        private CORC.CORCM2 m2;

        public M2Proxy(CORC.CORCM2 m2)
        {
            this.m2 = m2;
        }

        public bool IsReady => m2 != null && m2.IsInitialised() && m2.Client != null && m2.Client.IsConnected();

        public double Time => SafeGet("t", 0);

        public double[] X => SafeGetVec("X", 2);
        public double[] dX => SafeGetVec("dX", 2);
        public double[] F => SafeGetVec("F", 2);
        public System.Collections.Generic.List<CORC.FLNLCmd> DrainCmds()
        {
            if (!IsReady) return new System.Collections.Generic.List<CORC.FLNLCmd>();
            return m2.Client.DrainAllCmds();
        }
        public void SendCmd(string cmd4, double[] args = null)
        {
            if (!IsReady) return;
            m2.SendCmd(cmd4, args);
        }
        
        // Safe getters to avoid errors when not connected or if state values are missing
        private double SafeGet(string key, int idx)
        {
            if (!IsReady) return 0;
            var arr = m2.State[key];
            if (arr == null || arr.Length <= idx) return 0;
            return arr[idx];
        }
        private double[] SafeGetVec(string key, int len)
        {
            var outv = new double[len];
            if (!IsReady) return outv;
            var arr = m2.State[key];
            if (arr == null) return outv;
            for (int i = 0; i < len && i < arr.Length; i++) outv[i] = arr[i];
            return outv;
        }
    }
}
