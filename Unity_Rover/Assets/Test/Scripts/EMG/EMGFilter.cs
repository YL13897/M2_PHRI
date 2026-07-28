using UnityEngine;
using UnityEngine.Serialization;

// EMGFilter: Real-time sEMG pre-processing component.
// Pipeline:
// raw EMG -> [EMGFilter.cs] band-pass 20-450 Hz -> full-wave rectification -> 2nd-order low-pass envelope
// -> [DelsysEMG.cs] downsample to 100 Hz -> [InteractionEstimator.cs] computes SPI
// -> optional triangular SPI filter -> EMA SPI filter -> StiffnessCmd / EMG score.

public class EMGFilter : MonoBehaviour
{
    private const float ButterworthQ = 0.70710678f; // Butterworth choice: 1 / sqrt(2) 
    private const int DefaultChannelCount = 16;
    private const float DefaultSampleRateHz = 1111.1111f; // 1 / 0.0009
    private const float DefaultHighPassHz = 20f;
    private const float DefaultLowPassHz = 450f;
    private const float DefaultEnvelopeHz = 10f;

    [Header("sEMG Filter")]
    [SerializeField] private int channelCount = DefaultChannelCount;
    [SerializeField] private float sampleRateHz = DefaultSampleRateHz;
    [SerializeField] private float highPassHz = DefaultHighPassHz;
    [SerializeField] private float lowPassHz = DefaultLowPassHz;
    [FormerlySerializedAs("envelopeWindowMs")]
    [SerializeField] private float envelopeHz = DefaultEnvelopeHz;

    //  Biquad[] is a compact 2nd-order IIR filter implementation; cascading two stages gives a 4th-order response with minimal state and CPU overhead.
    private Biquad[] highPassStage2;
    private Biquad[] highPassStage1; 
    private Biquad[] lowPassStage1;
    private Biquad[] lowPassStage2;
    private Biquad[] envelopeStage;

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
        if (highPassStage1 == null || envelopeStage == null || highPassStage1.Length != channelCount || envelopeStage.Length != channelCount || coefficientsDirty)
            Rebuild();
    }


    private void Rebuild()
    {
        channelCount = Mathf.Max(1, channelCount);
        sampleRateHz = Mathf.Max(1f, sampleRateHz);
        envelopeHz = Mathf.Max(0.1f, envelopeHz);

        AllocateBuffers();

        float nyquist = sampleRateHz * 0.5f;
        float hp = Mathf.Clamp(highPassHz, 0.001f, nyquist - 0.001f);
        float lp = Mathf.Clamp(lowPassHz, hp + 0.001f, nyquist - 0.001f); 
        float env = Mathf.Clamp(envelopeHz, 0.001f, nyquist - 0.001f);

        for (int i = 0; i < channelCount; i++)
        {
            highPassStage1[i].SetHighPass(sampleRateHz, hp, ButterworthQ);
            highPassStage2[i].SetHighPass(sampleRateHz, hp, ButterworthQ);
            lowPassStage1[i].SetLowPass(sampleRateHz, lp, ButterworthQ);
            lowPassStage2[i].SetLowPass(sampleRateHz, lp, ButterworthQ);
            envelopeStage[i].SetLowPass(sampleRateHz, env, ButterworthQ);
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

    // ProcessFrame(): main entry point for processing a new frame of raw EMG samples.
    public void ProcessFrame(float[] rawSamples, EmgSignalView signal, float[] selectedOut,
        float[] envelopeOut = null, IEmgDebugSink debugSink = null, double t = 0.0)
    {
        lock (syncRoot)
        {
            EnsureReady();
            if (rawSamples == null || selectedOut == null)
                return;

            int count = Mathf.Min(channelCount, Mathf.Min(rawSamples.Length, selectedOut.Length));
            bool needsEnvelope = envelopeOut != null || debugSink != null;
            float filtered1 = 0f;
            float filtered2 = 0f;
            float envelope1 = 0f;
            float envelope2 = 0f;

            // If the requested signal view is raw and envelope is not needed, skip all filtering.
            // This allows fast pass-through when raw recording is selected.
            if (signal == EmgSignalView.Raw && !needsEnvelope)
            {
                for (int i = 0; i < count; i++)
                    selectedOut[i] = rawSamples[i];

                return;
            }

            for (int i = 0; i < count; i++)
            {
                float filtered = rawSamples[i];
                filtered = highPassStage1[i].Process(filtered);
                filtered = highPassStage2[i].Process(filtered);
                filtered = lowPassStage1[i].Process(filtered);
                filtered = lowPassStage2[i].Process(filtered);

                if (i == 0) filtered1 = filtered;
                else if (i == 1) filtered2 = filtered;

                float rectified = Mathf.Abs(filtered);
                float envelope = envelopeStage[i].Process(rectified);
                if (i == 0) envelope1 = envelope;
                else if (i == 1) envelope2 = envelope;

                switch (signal)
                {
                    case EmgSignalView.Raw:
                        selectedOut[i] = rawSamples[i];
                        break;
                    case EmgSignalView.Filtered:
                        selectedOut[i] = filtered;
                        break;
                    case EmgSignalView.Rectified:
                        selectedOut[i] = rectified;
                        break;
                    case EmgSignalView.Envelope:
                        selectedOut[i] = envelope;
                        break;
                    default:
                        selectedOut[i] = envelope;
                        break;
                }

                if (envelopeOut != null && i < envelopeOut.Length)
                    envelopeOut[i] = envelope;
            }

            if (debugSink != null && count >= 2)
                debugSink.Capture(t,
                    rawSamples[0], filtered1, envelope1,
                    rawSamples[1], filtered2, envelope2);
        }
    }


    // AllocateBuffers(): allocate filter banks.
    private void AllocateBuffers()
    {
        if (highPassStage1 == null || envelopeStage == null || highPassStage1.Length != channelCount || envelopeStage.Length != channelCount)
        {
            highPassStage1 = CreateFilterBank(channelCount);
            highPassStage2 = CreateFilterBank(channelCount);
            lowPassStage1 = CreateFilterBank(channelCount);
            lowPassStage2 = CreateFilterBank(channelCount);
            envelopeStage = CreateFilterBank(channelCount);
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
            envelopeStage[i].Reset();
        }
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
