using UnityEngine;

// Real-time sEMG pre-processing component.
// Pipeline: 4th-order band-pass (two cascaded high-pass + two cascaded low-pass stages),
// full-wave rectification, then low-pass envelope extraction.
// The component also stores a rolling history buffer so display code can render scrolling traces without touching the acquisition path.
public class EMGFilter : MonoBehaviour
{
    private const float ButterworthQ = 0.70710678f; // Butterworth choice: 1 / sqrt(2) 
    private const int DefaultChannelCount = 16;
    private const float DefaultSampleRateHz = 1111.1111f; // 1 / 0.0009
    private const float DefaultHighPassHz = 15f;
    private const float DefaultLowPassHz = 400f;
    private const float DefaultEnvelopeHz = 10f;
    private const int DefaultHistorySamples = 512;

    [Header("sEMG Filter")]
    [SerializeField] private int channelCount = DefaultChannelCount;
    [SerializeField] private float sampleRateHz = DefaultSampleRateHz;
    [SerializeField] private float highPassHz = DefaultHighPassHz;
    [SerializeField] private float lowPassHz = DefaultLowPassHz;
    [SerializeField] private float envelopeHz = DefaultEnvelopeHz;

    [Header("Display Buffer")]
    [SerializeField] private int historySamples = DefaultHistorySamples;

    private bool keepHistory = true;
    
    //  Biquad[] is a compact 2nd-order IIR filter implementation; cascading two stages gives a 4th-order response with minimal state and CPU overhead.
    private Biquad[] highPassStage2;
    private Biquad[] highPassStage1; 
    private Biquad[] lowPassStage1;
    private Biquad[] lowPassStage2;
    private float[] envelopeState;
    private float envelopeAlpha;

    private float[,] rawHistory;
    private float[,] filteredHistory;
    private float[,] rectifiedHistory;
    private float[,] envelopeHistory;
    private int historyWriteIndex; // Single-slot rolling write index for the circular history buffers.
    private int historyCount; // Number of valid samples in the history buffers, up to historySamples.
    private bool coefficientsDirty = true; // Flag to indicate filter coefficients need to be recalculated, e.g. after parameter changes.
    private readonly object syncRoot = new object();

    public int ChannelCount => channelCount;
    public float SampleRateHz => sampleRateHz;


    private void Awake()
    {
        Rebuild();
    }


    // Checking parameters in OnValidate for the filter
    private void OnValidate()
    {
        channelCount = Mathf.Max(1, channelCount);
        sampleRateHz = Mathf.Max(1f, sampleRateHz);
        historySamples = Mathf.Max(8, historySamples);
        envelopeHz = Mathf.Max(0.1f, envelopeHz);
        coefficientsDirty = true;
    }


    // Configure the filter settings. Configure() is a runtime/API entry point. Called by code, updates fields and immediately rebuilds filters.
    public void Configure(int channels, float sampleRate)
    {
        lock (syncRoot)
        {
            channelCount = Mathf.Max(1, channels);
            sampleRateHz = Mathf.Max(1f, sampleRate);
            coefficientsDirty = true;
            Rebuild();
        }
    }


    private void EnsureReady()
    {
        if (highPassStage1 == null || highPassStage1.Length != channelCount || coefficientsDirty)
            Rebuild();
    }


    private void Rebuild()
    {
        channelCount = Mathf.Max(1, channelCount);
        sampleRateHz = Mathf.Max(1f, sampleRateHz);
        historySamples = Mathf.Max(8, historySamples);
        envelopeHz = Mathf.Max(0.1f, envelopeHz);

        AllocateBuffers();

        float nyquist = sampleRateHz * 0.5f;
        float hp = Mathf.Clamp(highPassHz, 0.001f, nyquist - 0.001f);
        float lp = Mathf.Clamp(lowPassHz, hp + 0.001f, nyquist - 0.001f); 
        float env = Mathf.Clamp(envelopeHz, 0.001f, nyquist - 0.001f);
        envelopeAlpha = ComputeEnvelopeAlpha(sampleRateHz, env);

        for (int i = 0; i < channelCount; i++)
        {
            highPassStage1[i].SetHighPass(sampleRateHz, hp, ButterworthQ);
            highPassStage2[i].SetHighPass(sampleRateHz, hp, ButterworthQ);
            lowPassStage1[i].SetLowPass(sampleRateHz, lp, ButterworthQ);
            lowPassStage2[i].SetLowPass(sampleRateHz, lp, ButterworthQ);
        }

        coefficientsDirty = false;
        ResetState();
    }


    public void ResetFilters()
    {
        lock (syncRoot)
        {
            EnsureReady();
            ResetState();
        }
    }

    // ProcessFrame(): the main entry point for processing a block of EMG samples.
    public void ProcessFrame(float[] rawSamples, float[] filteredOut, float[] rectifiedOut)
    {
        lock (syncRoot)
        {
            EnsureReady();
            if (rawSamples == null || filteredOut == null || rectifiedOut == null)
                return;

            int count = Mathf.Min(channelCount, Mathf.Min(rawSamples.Length, Mathf.Min(filteredOut.Length, rectifiedOut.Length)));

            for (int i = 0; i < count; i++)
            {
                float filtered = rawSamples[i];
                filtered = highPassStage1[i].Process(filtered);
                filtered = highPassStage2[i].Process(filtered);
                filtered = lowPassStage1[i].Process(filtered);
                filtered = lowPassStage2[i].Process(filtered);

                float rectified = Mathf.Abs(filtered);
                float envelope = envelopeState[i] + envelopeAlpha * (rectified - envelopeState[i]);
                envelopeState[i] = envelope;
                filteredOut[i] = filtered;
                rectifiedOut[i] = rectified;
            }

            PushHistory(rawSamples, filteredOut, rectifiedOut, count);
        }
    }


    // PushHistory(): stores the latest samples into the rolling history buffers for later retrieval by display code.
    private void PushHistory(float[] rawSamples, float[] filteredSamples, float[] rectifiedSamples, int count)
    {
        if (!keepHistory || rawHistory == null || count <= 0)
            return;

        // Single-slot rolling write, i.e. the history buffer is updated one time-step at a time; readers reconstruct chronological order with historyWriteIndex.
        int slot = historyWriteIndex;
        for (int i = 0; i < count; i++)
        {
            rawHistory[i, slot] = rawSamples[i];
            filteredHistory[i, slot] = filteredSamples[i];
            rectifiedHistory[i, slot] = rectifiedSamples[i];
            envelopeHistory[i, slot] = envelopeState[i];
        }

        historyWriteIndex++;
        if (historyWriteIndex >= historySamples)
            historyWriteIndex = 0;

        if (historyCount < historySamples)
            historyCount++;
    }


    // CopyHistory(): allows the scope to copy a contiguous block of historical samples for a single channel. Returns the number of samples copied.
    public int CopyHistory(EmgSignalView signal, int channel, float[] destination)
    {
        lock (syncRoot)
        {
            EnsureReady();
            if (!keepHistory || destination == null || (uint)channel >= (uint)channelCount || historyCount == 0)
                return 0;

            int count = Mathf.Min(destination.Length, historyCount); // Number of samples to copy.
            int start = historyWriteIndex - count; // Start index of the oldest sample in the circular buffer.
            if (start < 0) 
                start += historySamples;

            for (int i = 0; i < count; i++)
            {
                int index = start + i;
                if (index >= historySamples)
                    index -= historySamples;

                if (signal == EmgSignalView.Raw)
                    destination[i] = rawHistory[channel, index];
                else if (signal == EmgSignalView.Filtered)
                    destination[i] = filteredHistory[channel, index];
                else if (signal == EmgSignalView.Rectified)
                    destination[i] = rectifiedHistory[channel, index];
                else
                    destination[i] = envelopeHistory[channel, index];
            }

            return count;
        }
    }


    // AllocateBuffers(): allocate filter banks and optional history storage.
    private void AllocateBuffers()
    {
        if (highPassStage1 == null || highPassStage1.Length != channelCount)
        {
            highPassStage1 = CreateFilterBank(channelCount);
            highPassStage2 = CreateFilterBank(channelCount);
            lowPassStage1 = CreateFilterBank(channelCount);
            lowPassStage2 = CreateFilterBank(channelCount);
            envelopeState = new float[channelCount];
        }

        if (keepHistory)
        {
            // History is indexed as [channel, sampleSlot] so the scope can copy one channel without additional transforms.
            bool sizeMismatch = rawHistory == null
                || rawHistory.GetLength(0) != channelCount 
                || rawHistory.GetLength(1) != historySamples;

            if (sizeMismatch) // If mismatch, reallocate new dateframe for buffers; otherwise, keep existing buffers.
            {
                rawHistory = new float[channelCount, historySamples];
                filteredHistory = new float[channelCount, historySamples];
                rectifiedHistory = new float[channelCount, historySamples];
                envelopeHistory = new float[channelCount, historySamples];
            }
        }
        else
        {
            rawHistory = null;
            filteredHistory = null;
            rectifiedHistory = null;
            envelopeHistory = null;
        }
    }


    private void ResetState()
    {
        for (int i = 0; i < channelCount; i++)
        {
            highPassStage1[i].Reset();
            highPassStage2[i].Reset();
            lowPassStage1[i].Reset();
            lowPassStage2[i].Reset();
        }

        historyWriteIndex = 0;
        historyCount = 0;
        System.Array.Clear(envelopeState, 0, envelopeState.Length);

        if (!keepHistory || rawHistory == null)
            return;

        System.Array.Clear(rawHistory, 0, rawHistory.Length);
        System.Array.Clear(filteredHistory, 0, filteredHistory.Length);
        System.Array.Clear(rectifiedHistory, 0, rectifiedHistory.Length);
        System.Array.Clear(envelopeHistory, 0, envelopeHistory.Length);
    }


    private static Biquad[] CreateFilterBank(int count)
    {
        Biquad[] bank = new Biquad[count];
        for (int i = 0; i < count; i++)
            bank[i] = new Biquad();
        return bank;
    }


    public enum EmgSignalView
    {
        Raw = 0,
        Filtered = 1,
        Rectified = 2,
        Envelope = 3
    }

    private static float ComputeEnvelopeAlpha(float sampleRate, float cutoffHz)
    {
        float dt = 1f / Mathf.Max(sampleRate, 1f);
        float rc = 1f / (2f * Mathf.PI * Mathf.Max(cutoffHz, 0.001f));
        return dt / (rc + dt);
    }


    // Definition of the Biquad class for the IIR filter implementation
    private sealed class Biquad
    {
        private float b0;
        private float b1;
        private float b2;
        private float a1;
        private float a2;
        private float z1;
        private float z2;

        public float Process(float input)
        {
            // Transposed direct form II keeps the per-channel state compact and efficient for streaming data.
            float output = b0 * input + z1;
            z1 = b1 * input - a1 * output + z2;
            z2 = b2 * input - a2 * output;
            return output;
        }

        public void Reset()
        {
            z1 = 0f;
            z2 = 0f;
        }

        public void SetLowPass(float sampleRate, float cutoffHz, float q)
        {
            SetCoefficients(DesignLowPass(sampleRate, cutoffHz, q));
        }

        public void SetHighPass(float sampleRate, float cutoffHz, float q)
        {
            SetCoefficients(DesignHighPass(sampleRate, cutoffHz, q));
        }

        private void SetCoefficients(Coefficients coefficients)
        {
            b0 = coefficients.b0;
            b1 = coefficients.b1;
            b2 = coefficients.b2;
            a1 = coefficients.a1;
            a2 = coefficients.a2;
        }

        private static Coefficients DesignLowPass(float sampleRate, float cutoffHz, float q)
        {
            float omega = 2f * Mathf.PI * cutoffHz / sampleRate; // omega: normalized angular frequency, where 1.0 corresponds to the Nyquist frequency (half the sample rate).
            float cosOmega = Mathf.Cos(omega); 
            float alpha = Mathf.Sin(omega) / (2f * q); // alpha: controls the bandwidth of the filter; 
            // Q: quality factor, controls the sharpness of the filter's frequency response, higher Q means a narrower bandwidth and a sharper peak in the frequency response.
            // For butterworth choice set q to be 1/sqrt(2) for a maximally flat response.

            float b0 = (1f - cosOmega) * 0.5f;
            float b1 = 1f - cosOmega;
            float b2 = (1f - cosOmega) * 0.5f;
            float a0 = 1f + alpha;
            float a1 = -2f * cosOmega;
            float a2 = 1f - alpha;
            return Normalize(b0, b1, b2, a0, a1, a2);
        }

        private static Coefficients DesignHighPass(float sampleRate, float cutoffHz, float q)
        {
            float omega = 2f * Mathf.PI * cutoffHz / sampleRate;
            float cosOmega = Mathf.Cos(omega);
            float alpha = Mathf.Sin(omega) / (2f * q);

            float b0 = (1f + cosOmega) * 0.5f;
            float b1 = -(1f + cosOmega);
            float b2 = (1f + cosOmega) * 0.5f;
            float a0 = 1f + alpha;
            float a1 = -2f * cosOmega;
            float a2 = 1f - alpha;
            return Normalize(b0, b1, b2, a0, a1, a2);
        }

        private static Coefficients Normalize(float b0, float b1, float b2, float a0, float a1, float a2)
        {
            float invA0 = 1f / a0;
            return new Coefficients
            {
                b0 = b0 * invA0,
                b1 = b1 * invA0,
                b2 = b2 * invA0,
                a1 = a1 * invA0,
                a2 = a2 * invA0
            };
        }

        private struct Coefficients
        {
            public float b0;
            public float b1;
            public float b2;
            public float a1;
            public float a2;
        }
    }
}
