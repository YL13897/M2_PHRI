using CORC;
using UnityEngine;

namespace CORC
{
    public class CORCM2 : CORCRobot
    {
        /// <summary>
        /// Specific class to define a CORC M2 robot object and manage communication with CORC server
        /// State dictionnary will contain X: end-effector position, dX: end-effector velocity,
        /// F: end-effector force from M2 motor/model side, t: running time of CORC server
        /// Previous stream used interaction force instead:
        /// ["F"] = new double[2] // F: M2 end-effector interaction force (N)
        /// </summary>

        // public override void Init(string ip = "192.168.8.104", int port = 2048)
        // public override void Init(string ip = "169.254.94.112", int port = 2048)
         public override void Init(string ip = "192.168.10.2", int port = 2048)
        {
            if (Client.IsConnected())
                Client.Disconnect();
        
            if (Client.Connect(ip, port))
            {
                //Define state values to receive (in pre-defined order: should match CORC implementation)
                State = new FixedDictionary
                {
                    ["t"] = new double[1], // t: running time of CORC server (s)
                    ["X"] = new double[2], // X: M2 end-effector position (m)
                    ["dX"] = new double[2], // dX: M2 end-effector velocity (m/s)
                    // ["F"] = new double[2] // F: M2 end-effector interaction force (N)
                    ["F"] = new double[2] // F: M2 force streamed from etInteractionForce() (N)
                };
                State.Init(new string[] { "t", "X", "dX", "F" });
                Initialised = true;
                Debug.Log("TCP Connection to CORC M2 established");
                Debug.Log("Connected: " + Client.IsConnected());
            }
            else
            {
                Initialised = false;
            }
        }

        protected override string BuildLoggingHeader()
        {
            return "t,X,Y,dX,dY,interaction_fx,interaction_fy,block_index,section_index";
            // return "t,X,Y,dX,dY,handle_fx,handle_fy,block_index,section_index";
        }
    }
}
