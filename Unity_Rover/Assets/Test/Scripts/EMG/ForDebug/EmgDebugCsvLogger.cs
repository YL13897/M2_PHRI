using System;
using System.Globalization;
using System.IO;
using System.Text;
using System.Threading;
using UnityEngine;

// <summary>
// A simple CSV logger for EMG debug data. It uses a circular buffer to decouple the capture of samples from the writing to disk, which happens in the main thread during Update().

// EmgDebugCsvLogger.cs is only used as an optional debug sink for EMG filtering.
// The usage is in [M2RoverBridge.cs]: in Awake(), it does:
//      delsysEMG.SetDebugSink(GetComponent<EmgDebugCsvLogger>());
//       Then [DelsysEMG.cs] passes that sink into EMGFilter.ProcessFrame(...), and [EMGFilter.cs] calls debugSink.Capture(...) if the sink is not null.
// 
// If you do not want to use it, the simplest way is:
//  remove the EmgDebugCsvLogger component from the GameObject
//  or delete/comment out the SetDebugSink(GetComponent<EmgDebugCsvLogger>()) line in M2RoverBridge.cs. That way, the EMGFilter will not call Capture() and the EmgDebugCsvLogger will not do anything.

// </summary>

public interface IEmgDebugSink
{
    void Capture(double t, float raw1, float filtered1, float envelope1,
        float raw2, float filtered2, float envelope2);
}

public sealed class EmgDebugCsvLogger : MonoBehaviour, IEmgDebugSink
{
    private const int BufferSize = 8192;
    private const int FlushRows = 1024;

    [SerializeField] private string outputDir = @"D:\yixianglin\Desktop\PHRI_Data";

    private readonly Sample[] buffer = new Sample[BufferSize];
    private readonly StringBuilder batch = new StringBuilder(64 * 1024);
    private StreamWriter writer;
    private int readIndex;
    private int writeIndex;
    private int rowsSinceFlush;
    private int dropped;
    private volatile bool accepting;

    private struct Sample
    {
        public double t;
        public float raw1;
        public float filtered1;
        public float envelope1;
        public float raw2;
        public float filtered2;
        public float envelope2;
    }

    private void OnEnable()
    {
        Directory.CreateDirectory(outputDir);
        string path = Path.Combine(outputDir, $"EmgDebug_{DateTime.Now:yyyyMMdd_HHmmss}.csv");
        writer = new StreamWriter(path, false, Encoding.ASCII);
        writer.WriteLine("emg_time,raw_ch1,filtered_ch1,envelope_ch1,raw_ch2,filtered_ch2,envelope_ch2");
        readIndex = 0;
        writeIndex = 0;
        rowsSinceFlush = 0;
        dropped = 0;
        accepting = true;
    }

    public void Capture(double t, float raw1, float filtered1, float envelope1,
        float raw2, float filtered2, float envelope2)
    {
        if (!accepting)
            return;

        int next = writeIndex + 1;
        if (next == BufferSize) next = 0;
        if (next == Volatile.Read(ref readIndex))
        {
            Interlocked.Increment(ref dropped);
            return;
        }

        buffer[writeIndex] = new Sample
        {
            t = t,
            raw1 = raw1,
            filtered1 = filtered1,
            envelope1 = envelope1,
            raw2 = raw2,
            filtered2 = filtered2,
            envelope2 = envelope2
        };
        Volatile.Write(ref writeIndex, next);
    }

    private void Update()
    {
        Drain();
    }

    private void Drain()
    {
        if (writer == null)
            return;

        batch.Clear();
        int write = Volatile.Read(ref writeIndex);
        while (readIndex != write)
        {
            Sample s = buffer[readIndex];
            batch.AppendFormat(CultureInfo.InvariantCulture,
                "{0:F6},{1:R},{2:R},{3:R},{4:R},{5:R},{6:R}\n",
                s.t, s.raw1, s.filtered1, s.envelope1, s.raw2, s.filtered2, s.envelope2);
            int next = readIndex + 1;
            if (next == BufferSize) next = 0;
            Volatile.Write(ref readIndex, next);
            rowsSinceFlush++;
        }

        if (batch.Length > 0)
            writer.Write(batch.ToString());
        if (rowsSinceFlush >= FlushRows)
        {
            writer.Flush();
            rowsSinceFlush = 0;
        }
    }

    private void OnDisable()
    {
        accepting = false;
        Drain();
        writer?.Dispose();
        writer = null;

        if (dropped > 0)
            Debug.LogWarning($"[EmgDebugCsvLogger] Dropped {dropped} samples.");
    }
}
