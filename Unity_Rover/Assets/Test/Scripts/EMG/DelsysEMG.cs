//======= Copyright (c) Melbourne Robotics Lab, All rights reserved. ===============
using System.Collections.Generic;

// TCP includes
using System;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.IO;
// Threading includes
using System.Threading;
// Debug
using UnityEngine;


public class DelsysEMG
{
    //example of creating a list of sensor types to keep track of various TCP streams...
    enum SensorTypes { SensorTrigno, SensorTrignoImu, SensorTrignoMiniHead, NoSensor };
    private List<SensorTypes> _sensors = new List<SensorTypes>();
    private Dictionary<string, SensorTypes> sensorList = new Dictionary<string, SensorTypes>();
    // Active sensor channel
    private List<int> activeSensorChannels = new List<int>();

    //For TCP/IP connections
    private TcpClient commandSocket;
    private TcpClient emgSocket;
    private const int commandPort = 50040;  //server command port
    private const int emgDataPort = 50043; //server emg data port

    //The following are streams and readers/writers for communication
    private NetworkStream commandStream;
    private NetworkStream emgStream;
    private StreamReader commandReader;
    private StreamWriter commandWriter;


    //Server commands
    private const string COMMAND_QUIT = "QUIT";
    private const string COMMAND_GETTRIGGERS = "TRIGGER?";
    private const string COMMAND_SETSTARTTRIGGER = "TRIGGER START";
    private const string COMMAND_SETSTOPTRIGGER = "TRIGGER STOP";
    private const string COMMAND_START = "START";
    private const string COMMAND_STOP = "STOP";
    private const string COMMAND_SENSOR_TYPE = "TYPE?";


    //Threads for acquiring emg and acc data
    private float samplingInterval = 0.0009f; 
    private Thread emgThread;


    //The following are storage for acquired data
    private float[] tempEmgDataList = new float[16];
    private float[] tempFilteredEmgDataList = new float[16];
    private float[] tempRectifiedEmgDataList = new float[16];
    private float[] filterInputFrame = new float[16];
    private float[] filterOutputFrame = new float[16];
    private float[] rectifiedOutputFrame = new float[16];
    private readonly object dataLock = new object();
    private readonly object recordingLock = new object();
    private EMGFilter emgFilter;

    //Sensor status
    private bool connected = false; //true if connected to server
    private bool running = false;   //true when acquiring data
    private bool recording = false; //for EMG data recording
    


    private StreamWriter recordingWriter;
    private long recordingSampleCount = 0;
    private int recordingFlushCounter = 0;
    private const int RecordingFlushInterval = 128;
    private double recordingBridgeStartTime = double.NaN;

    //Reset local state, clear data lists, and reset status flags
    private void ResetLocalState()
    {
        activeSensorChannels.Clear();
        _sensors.Clear();

        for (int i = 0; i < tempEmgDataList.Length; i++)
        {
            tempEmgDataList[i] = 0f;
            tempFilteredEmgDataList[i] = 0f;
            tempRectifiedEmgDataList[i] = 0f;
            filterInputFrame[i] = 0f;
            filterOutputFrame[i] = 0f;
            rectifiedOutputFrame[i] = 0f;
        }

        recording = false;
        running = false;
        if (emgFilter != null)
            emgFilter.ResetFilters();
        CloseRecordingWriter();
    }

    private void CloseRecordingWriter()
    {
        lock (recordingLock)
        {
            recording = false;
            recordingSampleCount = 0;
            recordingFlushCounter = 0;
            recordingBridgeStartTime = double.NaN;

            if (recordingWriter == null) return;

            try { recordingWriter.Flush(); } catch { }
            try { recordingWriter.Dispose(); } catch { }
            recordingWriter = null;
        }
    }

    private string BuildRecordingHeader()
    {
        StringBuilder header = new StringBuilder("t");
        for (int i = 0; i < 16; i++)
        {
            if (_sensors[i] == SensorTypes.SensorTrignoImu || _sensors[i] == SensorTypes.SensorTrigno)
                header.Append(",ch").Append(i + 1);
        }
        return header.ToString();
    }

    private void WriteRecordingFrame(float[] frame)
    {
        lock (recordingLock)
        {
            if (!recording || recordingWriter == null) return;

            StringBuilder line = new StringBuilder(128);
            double t = double.IsNaN(recordingBridgeStartTime)
                ? recordingSampleCount * samplingInterval
                : recordingBridgeStartTime + recordingSampleCount * samplingInterval;
            line.Append(t);

            for (int i = 0; i < 16; i++)
            {
                if (_sensors[i] == SensorTypes.SensorTrignoImu || _sensors[i] == SensorTypes.SensorTrigno)
                    line.Append(',').Append(frame[i]);
            }

            recordingWriter.WriteLine(line.ToString());
            recordingSampleCount++;
            recordingFlushCounter++;

            if (recordingFlushCounter >= RecordingFlushInterval)
            {
                recordingWriter.Flush();
                recordingFlushCounter = 0;
            }
        }
    }

    #region Initialization and connection
    //Initialization
    public void Init(EMGFilter filter = null)
    {
        ResetLocalState();
        emgFilter = filter;
        if (emgFilter != null)
            emgFilter.Configure(tempEmgDataList.Length, samplingInterval > 0f ? 1f / samplingInterval : 1111.1111f);
        sensorList.Clear();
        sensorList.Add("A", SensorTypes.SensorTrigno);
        sensorList.Add("D", SensorTypes.SensorTrigno);
        sensorList.Add("L", SensorTypes.SensorTrignoImu);
        sensorList.Add("J", SensorTypes.SensorTrignoMiniHead);
        sensorList.Add("O", SensorTypes.SensorTrignoImu);

        Debug.Log("Delsys-> Initialize DelsysEMG");
    }

    //Establish sensors connnection
    public bool Connect()
    {
        ResetLocalState();

        try
        {
            //Establish TCP/IP connection to server using URL entered
            commandSocket = new TcpClient("localhost", commandPort);

            //Set up communication streams
            commandStream = commandSocket.GetStream();
            commandReader = new StreamReader(commandStream, Encoding.ASCII);
            commandWriter = new StreamWriter(commandStream, Encoding.ASCII);

            //Get initial response from server and display
            commandReader.ReadLine();
            commandReader.ReadLine();   //get extra line terminator

            

            connected = true;   //indicate that we are connected
        }
        catch (Exception connectException)
        {
            //connection failed, display error message
            Debug.Log("Delsys-> Could not connect.\n" + connectException.Message);
            connected = false;
            return connected;
        }

        //build a list of connected sensor types
        _sensors = new List<SensorTypes>();
        for (int i = 1; i <= 16; i++)
        {
            string query = "SENSOR " + i + " " + COMMAND_SENSOR_TYPE;
            string response = SendCommand(query);
            // Debug.Log("Delsys-> " + query);
            // Debug.Log("<- Server Delsys " + response);
            _sensors.Add(response.Contains("INVALID") ? SensorTypes.NoSensor : sensorList[response]);
        }

        // Add the active sensor channel to a list for query
        for (int i = 0; i < 16; i++)
        {
            if (_sensors[i] == SensorTypes.SensorTrignoImu || _sensors[i] == SensorTypes.SensorTrigno)
            {
                activeSensorChannels.Add(i + 1);
            }
        }

        SendCommand("UPSAMPLE OFF");

        return connected;
    }


    //Close connection
    public void Close()
    {
        if (!connected)
        {
            ResetLocalState();
            return;
        }
        if (running)
        {
            Debug.Log("Delsys-> Can't quit while acquiring data!");
            return;
        }

        //send QUIT command
        SendCommand(COMMAND_QUIT);
        connected = false;  //no longer connected

        //Close all streams and connections
        commandReader.Close();
        commandWriter.Close();
        commandStream.Close();
        commandSocket.Close();
        emgStream.Close();
        emgSocket.Close();

        ResetLocalState();
        Debug.Log("Delsys-> Disconnect from server and quit!");
    }

    private void OnDestroy()
    {
        //StopZMQPusher();
    }

    #endregion

    #region Data acquisition

    //Start acquisition
    public void StartAcquisition()
    {
        if (!connected)
        {
            Debug.Log("Delsys-> EMG Not connected.");
            return;
        }

        
        //Establish data connections and creat streams
        emgSocket = new TcpClient("localhost", emgDataPort);

        //emgStream = emgSocket.GetStream();
        emgStream = emgSocket.GetStream();

        //Create data acquisition threads
        emgThread = new Thread(ImuEmgThreadRoutine);
        emgThread.IsBackground = true;
        //Indicate we are running and start up the acquisition threads
        running = true;

        //emgTimer = new Timer(new TimerCallback(this.ImuEmgThreadRoutine), null, new TimeSpan(10000), new TimeSpan(10000));
        emgThread.Start();


        //Send start command to server to stream data
        string response = SendCommand(COMMAND_START);

        //check response
        if (response.StartsWith("OK"))
        {
            Debug.Log("Delsys-> Server OK to start acquisition!");
        }
        else
        {
            running = false;    //stop threads
            Debug.Log("Delsys-> Server ERROR to start acquisition!");
        }
        
    }

    //Stop acquisition and write to file
    public void StopAcquisition()
    {
        if (!connected)
            return;
        if (running)
        {
            running = false;    //no longer running
                                //Wait for threads to terminate
            emgThread.Join();
        }
        

        //Send stop command to server
        string response = SendCommand(COMMAND_STOP);
        if (!response.StartsWith("OK"))
            Debug.Log("Delsys->: Server failed to stop. Further actions may fail.");
        else
            Debug.Log("Delsys->: Server stops!");
    }


    //Start log to csv
    public void StartRecording(String filename, double bridgeStartTime = double.NaN)
    {
        string dir = Path.GetDirectoryName(filename);
        if (!string.IsNullOrEmpty(dir))
            Directory.CreateDirectory(dir);

        bool append = File.Exists(filename) && new FileInfo(filename).Length > 0;
        StreamWriter writer = new StreamWriter(filename, append, Encoding.ASCII);
        if (!append)
            writer.WriteLine(BuildRecordingHeader());
        writer.Flush();

        lock (recordingLock) // ensure we don't have a race condition where we start a new recording while the old one is still being finalized
        {
            CloseRecordingWriter();
            recordingWriter = writer;
            recordingSampleCount = 0;
            recordingFlushCounter = 0;
            recordingBridgeStartTime = bridgeStartTime;
            recording = true;
        }

        Debug.Log("Delsys-> Recording...");
    }

    //Stop log to csv and output file
    public void StopRecording()
    {
        if (!connected)
            return;

        long sampleCount;
        lock (recordingLock) // ensure we get the final sample count before stopping recording
        {
            sampleCount = recordingSampleCount;
        }

        CloseRecordingWriter();
        Debug.Log("Delsys-> " + sampleCount + " samples recorded.");
    }

    #endregion


    #region Get sensor status and readings
    //Return the latest readings
    public float[] GetRawEMGData()
    {
        lock (dataLock)
        {
            return CopyActiveChannelData(tempEmgDataList);
        }
    }

    //Return the number of active sensor
    public int GetNbActiveSensors(){ return activeSensorChannels.Count;}

    //Set channels of active sensor
    public int[] GetChannelsActiveSensor(){ return activeSensorChannels.ToArray(); }

    // Return status
    public bool IsConnected() { return connected; }
    public bool IsRunning() { return running; }

    private float[] CopyActiveChannelData(float[] source)
    {
        float[] data = new float[activeSensorChannels.Count];

        int activeSensorNb = 0;
        foreach (int element in activeSensorChannels)
        {
            data[activeSensorNb] = source[element - 1];
            activeSensorNb++;
        }

        return data;
    }

    private void MarkDisconnected(string context, Exception ex = null)
    {
        connected = false;
        running = false;
        recording = false;

        Debug.LogWarning(ex == null
            ? $"Delsys-> Connection closed ({context})."
            : $"Delsys-> Connection closed ({context}): {ex.Message}");

        try { commandReader?.Close(); } catch { }
        try { commandWriter?.Close(); } catch { }
        try { commandStream?.Close(); } catch { }
        try { commandSocket?.Close(); } catch { }
        try { emgStream?.Close(); } catch { }
        try { emgSocket?.Close(); } catch { }

        ResetLocalState();
    }

    #endregion


    //Send a command to the server and get the response
    private string SendCommand(string command)
    {
        string response = "";

        //Check if connected
        if (connected)
        {
            try
            {
                //Send the command
                commandWriter.WriteLine(command);
                commandWriter.WriteLine();  //terminate command
                commandWriter.Flush();  //make sure command is sent immediately

                //Read the response line and display
                response = commandReader.ReadLine();
                commandReader.ReadLine();   //get extra line terminator
            }
            catch (IOException ioException)
            {
                MarkDisconnected($"IOException during SendCommand({command})", ioException);
            }
            catch (SocketException socketException)
            {
                MarkDisconnected($"SocketException during SendCommand({command})", socketException);
            }
            catch (ObjectDisposedException disposedException)
            {
                MarkDisconnected($"ObjectDisposedException during SendCommand({command})", disposedException);
            }
        }
        else
            Debug.Log("Delsys-> EMG: Not connected.");
        return response;    //return the response we got
    }

    // Thread for emg acquisition
    private void ImuEmgThreadRoutine(object state)
    {
       emgStream.ReadTimeout = 100000;    //set timeout, unit: ms

        BinaryReader reader = new BinaryReader(emgStream);
        float[] frame = new float[16];
        while (running)
        {
            try
            {
                // Stream the data;
                for (int sn = 0; sn < 16; ++sn)
                    frame[sn] = reader.ReadSingle();

                if (emgFilter != null)
                {
                    for (int sn = 0; sn < 16; ++sn)
                        filterInputFrame[sn] = frame[sn];

                    emgFilter.ProcessFrame(filterInputFrame, filterOutputFrame, rectifiedOutputFrame);
                }

                lock (dataLock)
                {
                    for (int sn = 0; sn < 16; ++sn)
                    {
                        tempEmgDataList[sn] = frame[sn];
                        tempFilteredEmgDataList[sn] = emgFilter != null ? filterOutputFrame[sn] : frame[sn];
                        tempRectifiedEmgDataList[sn] = emgFilter != null ? rectifiedOutputFrame[sn] : Mathf.Abs(frame[sn]);
                    }
                }

                WriteRecordingFrame(frame);

            }
            catch (IOException e)
            {
                Debug.Log(e.ToString());
            }
        }

    }

}
