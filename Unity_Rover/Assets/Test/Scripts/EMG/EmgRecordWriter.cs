using System;
using System.Globalization; // For CultureInfo.InvariantCulture, which ensures consistent formatting of numbers regardless of the system's locale settings.
using System.IO;
using System.Text;

public sealed class EmgRecordWriter : IDisposable
{
    private const int CsvFlushInterval = 2048;

    private readonly object syncRoot = new object();

    private StreamWriter csvWriter;

    private int[] activeChannels = Array.Empty<int>();
    private string signalLabel = "envelope";
    private int csvFlushCounter = 0;
    private bool opened = false;

    public void Open(string path, string targetSignalLabel, int[] targetActiveChannels)
    {
        lock (syncRoot)
        {
            CloseInternal();

            string dir = Path.GetDirectoryName(path);
            if (!string.IsNullOrEmpty(dir))
                Directory.CreateDirectory(dir);

            signalLabel = string.IsNullOrWhiteSpace(targetSignalLabel) ? "envelope" : targetSignalLabel;
            activeChannels = targetActiveChannels != null ? (int[])targetActiveChannels.Clone() : Array.Empty<int>();
            csvFlushCounter = 0;

            bool append = File.Exists(path) && new FileInfo(path).Length > 0;
            csvWriter = new StreamWriter(path, append, Encoding.ASCII);
            if (!append)
            {
                csvWriter.WriteLine(BuildCsvHeader());
                csvWriter.Flush();
            }

            opened = true;
        }
    }

    public void WriteFrame(double timestamp, int sec, float[] frame)
    {
        lock (syncRoot)
        {
            if (!opened || frame == null)
                return;

            WriteCsvFrame(timestamp, sec, frame);
        }
    }

    public void Close()
    {
        lock (syncRoot)
        {
            CloseInternal();
        }
    }

    public void Dispose()
    {
        Close();
    }

    private string BuildCsvHeader()
    {
        StringBuilder header = new StringBuilder("global_t,sec");
        for (int i = 0; i < activeChannels.Length; i++)
            header.Append(',').Append(signalLabel).Append("_ch").Append(activeChannels[i]);
        return header.ToString();
    }

    private void WriteCsvFrame(double timestamp, int sec, float[] frame)
    {
        if (csvWriter == null)
            return;

        StringBuilder line = new StringBuilder(64 + activeChannels.Length * 16);
        line.Append(timestamp.ToString("R", CultureInfo.InvariantCulture)).Append(',').Append(sec);
        for (int i = 0; i < activeChannels.Length; i++)
        {
            int sourceIndex = activeChannels[i] - 1;
            float value = (sourceIndex >= 0 && sourceIndex < frame.Length) ? frame[sourceIndex] : 0f;
            line.Append(',').Append(value.ToString("G9", CultureInfo.InvariantCulture));
        }

        csvWriter.WriteLine(line.ToString());
        csvFlushCounter++;
        if (csvFlushCounter >= CsvFlushInterval)
        {
            csvWriter.Flush();
            csvFlushCounter = 0;
        }
    }

    private void CloseInternal()
    {
        if (!opened && csvWriter == null)
            return;

        if (csvWriter != null)
        {
            try { csvWriter.Flush(); } catch { }
            try { csvWriter.Dispose(); } catch { }
            csvWriter = null;
        }

        activeChannels = Array.Empty<int>();
        csvFlushCounter = 0;
        opened = false;
    }
}
