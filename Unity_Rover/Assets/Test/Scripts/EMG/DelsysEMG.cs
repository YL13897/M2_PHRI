//======= Copyright (c) Melbourne Robotics Lab, All rights reserved. ===============
using System.Collections.Generic;

// TCP includes
using System;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.IO;
using System.Diagnostics;
// Threading includes
using System.Threading;
// Debug
using UnityEngine;
using Debug = UnityEngine.Debug;

// DelsysEMG: A Unity component that manages the connection to a Delsys EMG system, handles data acquisition in a background thread,
//            processes EMG signals using an optional EMGFilter, and records processed frames through EmgRecordWriter.

public class DelsysEMG
{
    private const int FullChannelCount = 16;
    public const int MaxChannels = 10; // Do not need all 16 channels, currently set it to max 10 channels for processing and recording.

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
    private const int commandTimeoutMs = 3000;
    private const int dataTimeoutMs = 5000;

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
    private float[] tempSelectedEmgDataList = new float[FullChannelCount];
    private float[] tempEnvelopeDataList = new float[FullChannelCount];
    private float[] selectedOutputFrame = new float[FullChannelCount];
    private float[] scoreEnvelopeFrame = new float[FullChannelCount];
    private float[] processingInputFrame = new float[MaxChannels];
    private float[] processingOutputFrame = new float[MaxChannels];
    private float[] processingEnvelopeFrame = new float[MaxChannels];
    private int[] processChannels = new int[MaxChannels];
    private int processChannelCount = 0;
    private readonly object dataLock = new object();
    private readonly object recordingLock = new object();
    private const int RecordingQueueLimit = 5000;
    private EMGFilter emgFilter;
    private IEmgDebugSink debugSink;
    private EMGFilter.EmgSignalView signalView = EMGFilter.EmgSignalView.Envelope;
    private float scoreDownsampleHz = 100f;
    private double downSampleAccumulator = 0.0; // Accumulator for downsampling the score envelope output to a lower frequency.
                                                // Samples are added up and published when it reaches the target score period.
    private double acquisitionTime = 0.0;
    private double scoreEnvelopeTime = 0.0;
    private long scoreEnvelopeSeq = 0;

    //Sensor status
    private volatile bool connected = false; //true if connected to server
    private volatile bool running = false;   //true when acquiring data
    private bool recording = false; //for EMG data recording
    


    private readonly EmgRecordWriter recordingWriter = new EmgRecordWriter();
    private readonly Queue<RecordingFrame> recordingQueue = new Queue<RecordingFrame>();
    private Thread recordingThread;
    private long recordingSampleCount = 0;
    private long recordingDroppedCount = 0;
    private double recordingStartT = double.NaN;
    private int recordingSec = 0;
    private bool recordingWriterRunning = false;

    private struct RecordingFrame
    {
        public double Time;
        public int Sec;
        public float[] Values;
    }

    //Reset local state, clear data lists, and reset status flags
    private void ResetLocalState()
    {
        activeSensorChannels.Clear();
        _sensors.Clear();

        for (int i = 0; i < tempSelectedEmgDataList.Length; i++)
        {
            tempSelectedEmgDataList[i] = 0f;
            tempEnvelopeDataList[i] = 0f;
            selectedOutputFrame[i] = 0f;
            scoreEnvelopeFrame[i] = 0f;
        }
        for (int i = 0; i < MaxChannels; i++)
        {
            processingInputFrame[i] = 0f;
            processingOutputFrame[i] = 0f;
            processingEnvelopeFrame[i] = 0f;
            processChannels[i] = 0;
        }
        processChannelCount = 0;
        downSampleAccumulator = 0.0;
        acquisitionTime = 0.0;
        scoreEnvelopeTime = 0.0;
        scoreEnvelopeSeq = 0;

        recording = false;
        running = false;
        if (emgFilter != null)
            emgFilter.ResetFilters();
        CloseRecordingWriter();
    }

    private void CloseRecordingWriter()
    {
        Thread threadToJoin = null; 

        // First lock: safely sets the stop flags, wakes the recording thread, and copies the thread reference.
        lock (recordingLock) 
        {
            recording = false;
            recordingWriterRunning = false;
            Monitor.PulseAll(recordingLock); // Wake up the recording thread if it's waiting for new frames to write
            threadToJoin = recordingThread;
        }

        // If the recording thread exists and is still running, Join() waits until it completely finishes.
        if (threadToJoin != null && threadToJoin.IsAlive)
            threadToJoin.Join();

        // Second lock: after the thread has fully stopped, safely clears the shared queue and resets the recording state.
        lock (recordingLock) 
        {
            recordingThread = null;
            recordingQueue.Clear();
            recordingSampleCount = 0;
            recordingDroppedCount = 0;
            recordingStartT = double.NaN;
            recordingSec = 0;
        }

        recordingWriter.Close();
    }

    private void CloseDataSocket()
    {
        try { emgStream?.Close(); } catch { }
        try { emgSocket?.Close(); } catch { }
        emgStream = null;
        emgSocket = null;
    }

    private void CloseSockets()
    {
        CloseDataSocket();
        try { commandReader?.Close(); } catch { }
        try { commandWriter?.Close(); } catch { }
        try { commandStream?.Close(); } catch { }
        try { commandSocket?.Close(); } catch { }
        commandReader = null;
        commandWriter = null;
        commandStream = null;
        commandSocket = null;
    }

    private string GetSignalLabel()
    {
        if (emgFilter == null)
            return "raw";

        switch (signalView)
        {
            case EMGFilter.EmgSignalView.Raw:
                return "raw";
            case EMGFilter.EmgSignalView.Filtered:
                return "filtered";
            case EMGFilter.EmgSignalView.Rectified:
                return "rectified";
            case EMGFilter.EmgSignalView.Envelope:
                return "envelope";
            default:
                return "envelope";
        }
    }

    private void WriteRecordingFrame(float[] frame)
    {
        lock (recordingLock)
        {
            if (!recording) return;

            double t = double.IsNaN(recordingStartT)
                ? recordingSampleCount * samplingInterval
                : recordingStartT + recordingSampleCount * samplingInterval;

            float[] values = new float[frame.Length];
            Array.Copy(frame, values, frame.Length);
            if (recordingQueue.Count >= RecordingQueueLimit)
            {
                recordingQueue.Dequeue();
                recordingDroppedCount++;
            }

            recordingQueue.Enqueue(new RecordingFrame
            {
                Time = t,
                Sec = recordingSec,
                Values = values
            });
            recordingSampleCount++;
            Monitor.Pulse(recordingLock);
        }
    }

    private void RecordingThreadRoutine()
    {
        while (true)
        {
            RecordingFrame frame;
            lock (recordingLock)
            {
                while (recordingWriterRunning && recordingQueue.Count == 0)
                    Monitor.Wait(recordingLock);

                if (recordingQueue.Count == 0)
                    return;

                frame = recordingQueue.Dequeue();
            }

            try
            {
                recordingWriter.WriteFrame(frame.Time, frame.Sec, frame.Values);
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"Delsys-> EMG recording writer stopped: {ex.Message}");
                lock (recordingLock)
                {
                    recording = false;
                    recordingWriterRunning = false;
                    recordingQueue.Clear();
                    Monitor.PulseAll(recordingLock);
                }
                return;
            }
        }
    }

    public void SetSec(int sec)
    {
        lock (recordingLock)
        {
            recordingSec = sec;
        }
    }

    #region Initialization and connection
    //Initialization
    public void Init(EMGFilter filter = null, float scoreHz = 100f)
    {
        ResetLocalState();
        emgFilter = filter;
        scoreDownsampleHz = Mathf.Max(1f, scoreHz);
        if (emgFilter != null)
            ConfigureFilter();
        else
            Debug.LogWarning("Delsys-> EMGFilter missing; recording raw EMG.");
        sensorList.Clear();
        sensorList.Add("A", SensorTypes.SensorTrigno);
        sensorList.Add("D", SensorTypes.SensorTrigno);
        sensorList.Add("L", SensorTypes.SensorTrignoImu);
        sensorList.Add("J", SensorTypes.SensorTrignoMiniHead);
        sensorList.Add("O", SensorTypes.SensorTrignoImu);

        Debug.Log("Delsys-> Initialize DelsysEMG");
    }

    public EMGFilter.EmgSignalView SignalView
    {
        get => signalView;
        set => signalView = value;
    }

    public void SetDebugSink(IEmgDebugSink sink)
    {
        debugSink = sink;
    }

    public void SetProcessChannels(int[] channels)
    {
        if (running)
        {
            Debug.LogWarning("Delsys-> Cannot change EMG processing channels while acquisition is running.");
            return;
        }

        processChannelCount = 0;
        if (channels != null)
        {
            for (int i = 0; i < channels.Length && processChannelCount < MaxChannels; i++)
            {
                int channel = channels[i];
                if (channel < 1 || channel > FullChannelCount) continue;
                if (activeSensorChannels.Count > 0 && !activeSensorChannels.Contains(channel)) continue;
                if (Array.IndexOf(processChannels, channel, 0, processChannelCount) >= 0) continue;
                processChannels[processChannelCount++] = channel;
            }
        }

        if (processChannelCount == 0)
        {
            for (int i = 0; i < activeSensorChannels.Count && processChannelCount < MaxChannels; i++)
                processChannels[processChannelCount++] = activeSensorChannels[i];
        }

        for (int i = processChannelCount; i < MaxChannels; i++)
            processChannels[i] = 0;

        ConfigureFilter();
        if (processChannelCount > 0)
            Debug.Log($"Delsys-> Processing EMG channels: {string.Join(", ", GetProcessChannels())}");
    }

    private int[] GetProcessChannels()
    {
        int[] channels = new int[processChannelCount];
        for (int i = 0; i < processChannelCount; i++)
            channels[i] = processChannels[i];
        return channels;
    }

    private void ConfigureFilter()
    {
        if (emgFilter == null)
            return;

        int channels = Mathf.Max(1, processChannelCount);
        emgFilter.Configure(channels, samplingInterval > 0f ? 1f / samplingInterval : 1111.1111f);
    }


    //Establish sensors connnection
    public bool Connect()
    {
        running = false;
        CloseSockets();
        if (emgThread != null && emgThread.IsAlive)
            emgThread.Join(commandTimeoutMs);
        emgThread = null;
        ResetLocalState();

        try
        {
            //Establish TCP/IP connection to server using URL entered
            commandSocket = new TcpClient("localhost", commandPort);
            commandSocket.ReceiveTimeout = commandTimeoutMs;
            commandSocket.SendTimeout = commandTimeoutMs;

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
            CloseSockets();
            return connected;
        }

        //build a list of connected sensor types
        _sensors = new List<SensorTypes>();
        for (int i = 1; i <= 16; i++)
        {
            string query = "SENSOR " + i + " " + COMMAND_SENSOR_TYPE;
            string response = SendCommand(query);
            if (!connected)
                return false;
            // Debug.Log("Delsys-> " + query);
            // Debug.Log("<- Server Delsys " + response);
            response = response.Trim();
            _sensors.Add(response.Contains("INVALID") || !sensorList.TryGetValue(response, out SensorTypes sensor)
                ? SensorTypes.NoSensor
                : sensor);
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
            CloseSockets();
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

        CloseSockets();
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
        if (running)
        {
            Debug.Log("Delsys-> EMG acquisition already running.");
            return;
        }

        if (processChannelCount == 0)
            SetProcessChannels(null);

        
        try
        {
            CloseDataSocket();
            emgSocket = new TcpClient("localhost", emgDataPort);
            emgSocket.ReceiveBufferSize = 1024 * 1024;
            emgSocket.ReceiveTimeout = dataTimeoutMs;
            emgSocket.SendTimeout = commandTimeoutMs;
            emgStream = emgSocket.GetStream();
            emgStream.ReadTimeout = dataTimeoutMs;
        }
        catch (Exception ex) when (ex is IOException || ex is SocketException || ex is ObjectDisposedException)
        {
            MarkDisconnected("Could not open EMG data stream", ex);
            return;
        }

        //Send start command to server to stream data
        string response = SendCommand(COMMAND_START);

        //check response
        if (response.StartsWith("OK"))
        {
            emgThread = new Thread(ImuEmgThreadRoutine) 
            { 
                IsBackground = true,
                Priority = System.Threading.ThreadPriority.AboveNormal,
                Name = "EMG TCP Receiver"
            };
            running = true;
            emgThread.Start();
            Debug.Log("Delsys-> Server OK to start acquisition!");
        }
        else
        {
            running = false;    //stop threads
            CloseDataSocket();
            Debug.Log("Delsys-> Server ERROR to start acquisition!");
        }
        
    }

    //Stop acquisition and write to file
    public void StopAcquisition()
    {
        if (!connected)
            return;
        running = false;

        //Send stop command to server
        string response = SendCommand(COMMAND_STOP);
        CloseDataSocket();
        if (emgThread != null && emgThread.IsAlive)
            emgThread.Join(commandTimeoutMs);
        emgThread = null;

        if (!response.StartsWith("OK"))
            Debug.Log("Delsys->: Server failed to stop. Further actions may fail.");
        else
            Debug.Log("Delsys->: Server stops!");
    }


    //Start recording to CSV
    public void StartRecording(string filename, double startT = double.NaN, int sec = 0)
    {
        string dir = Path.GetDirectoryName(filename);
        if (!string.IsNullOrEmpty(dir))
            Directory.CreateDirectory(dir);

        CloseRecordingWriter();

        lock (recordingLock) // ensure we don't have a race condition where we start a new recording while the old one is still being finalized
        {
            recordingSampleCount = 0;
            recordingDroppedCount = 0;
            recordingStartT = startT;
            recordingSec = sec;
            recordingWriter.Open(filename, GetSignalLabel(), GetChannelsActiveSensor());
            recordingWriterRunning = true;
            recordingThread = new Thread(RecordingThreadRoutine)
            {
                IsBackground = true,
                Name = "EMG Recording Writer"
            };
            recording = true;
            recordingThread.Start();
        }

        Debug.Log("Delsys-> Recording...");
    }

    //Stop recording and close output file
    public void StopRecording()
    {
        if (!connected)
            return;

        long sampleCount;
        long droppedCount;
        lock (recordingLock) // ensure we get the final sample count before stopping recording
        {
            sampleCount = recordingSampleCount;
            droppedCount = recordingDroppedCount;
        }

        CloseRecordingWriter();
        if (droppedCount > 0)
            Debug.LogWarning($"Delsys-> {sampleCount} samples received for recording; {droppedCount} dropped before disk write.");
        else
            Debug.Log("Delsys-> " + sampleCount + " samples recorded.");
    }

    #endregion


    #region Get sensor status and readings
    public float[] GetScoreEnvelopeData()
    {
        lock (dataLock)
        {
            return CopyActiveChannelData(tempEnvelopeDataList);
        }
    }

    public bool CopyScoreEnvelope(float[] values, int[] map, out double t, out long seq)
    {
        t = 0.0;
        seq = 0;
        if (values == null || map == null)
            return false;

        lock (dataLock)
        {
            int n = Mathf.Min(values.Length, map.Length);
            for (int i = 0; i < n; i++)
            {
                int channel = map[i];
                values[i] = channel > 0 && activeSensorChannels.Contains(channel)
                    ? tempEnvelopeDataList[channel - 1]
                    : 0f;
            }
            for (int i = n; i < values.Length; i++)
                values[i] = 0f;

            t = scoreEnvelopeTime;
            seq = scoreEnvelopeSeq;
            return seq > 0;
        }
    }

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

        double now = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
        double sinceCalibrationClose = now - CORC.Demo.CalibrationManager.LastCloseTime;
        string calibrationNote = sinceCalibrationClose >= 0.0 && sinceCalibrationClose <= 10.0
            ? $" Recent calibration TCP close {sinceCalibrationClose:F2}s ago."
            : "";

        Debug.LogWarning(ex == null
            ? $"Delsys-> Connection closed ({context}).{calibrationNote}"
            : $"Delsys-> Connection closed ({context}): {ex.Message}{calibrationNote}");

        CloseSockets();
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
                if (response == null)
                    throw new IOException("Command connection closed by server.");
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
       emgStream.ReadTimeout = dataTimeoutMs;

        BinaryReader reader = new BinaryReader(emgStream);
        float[] frame = new float[FullChannelCount];
        while (running)
        {
            try
            {
                // Stream the data;
                for (int sn = 0; sn < FullChannelCount; ++sn)
                    frame[sn] = reader.ReadSingle();

                for (int sn = 0; sn < FullChannelCount; ++sn)
                {
                    selectedOutputFrame[sn] = 0f;
                    scoreEnvelopeFrame[sn] = 0f;
                }

                if (emgFilter != null)
                {
                    for (int i = 0; i < processChannelCount; i++)
                        processingInputFrame[i] = frame[processChannels[i] - 1];

                    emgFilter.ProcessFrame(processingInputFrame, signalView, processingOutputFrame,
                        processingEnvelopeFrame, debugSink, acquisitionTime);

                    for (int i = 0; i < processChannelCount; i++)
                    {
                        int channelIndex = processChannels[i] - 1;
                        selectedOutputFrame[channelIndex] = processingOutputFrame[i];
                        scoreEnvelopeFrame[channelIndex] = processingEnvelopeFrame[i];
                    }
                }
                else
                {
                    for (int i = 0; i < processChannelCount; i++)
                    {
                        int channelIndex = processChannels[i] - 1;
                        selectedOutputFrame[channelIndex] = frame[channelIndex];
                        scoreEnvelopeFrame[channelIndex] = Mathf.Abs(frame[channelIndex]);
                    }
                }

                acquisitionTime += samplingInterval;
                downSampleAccumulator += samplingInterval;
                double scorePeriodSec = 1.0 / scoreDownsampleHz; // scoreDownsampleHz is safely bounded in Init
                bool envelopeScorePublish = downSampleAccumulator >= scorePeriodSec;
                if (envelopeScorePublish)
                    downSampleAccumulator %= scorePeriodSec;

                lock (dataLock)
                {
                    for (int sn = 0; sn < FullChannelCount; ++sn)
                    {
                        tempSelectedEmgDataList[sn] = selectedOutputFrame[sn]; 
                        if (envelopeScorePublish)
                            tempEnvelopeDataList[sn] = scoreEnvelopeFrame[sn];
                    }
                    if (envelopeScorePublish)
                    {
                        scoreEnvelopeTime = acquisitionTime;
                        scoreEnvelopeSeq++;
                    }
                }

                WriteRecordingFrame(selectedOutputFrame);

            }
            catch (IOException e)
            {
                if (running)
                    MarkDisconnected("IOException during EMG stream read", e);
                break;
            }
            catch (SocketException e)
            {
                if (running)
                    MarkDisconnected("SocketException during EMG stream read", e);
                break;
            }
            catch (ObjectDisposedException e)
            {
                if (running)
                    MarkDisconnected("ObjectDisposedException during EMG stream read", e);
                break;
            }
        }

    }

}
