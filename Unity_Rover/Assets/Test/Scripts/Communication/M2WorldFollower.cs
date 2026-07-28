// Minimal 3D follower for M2-to-Unity rover alignment.
using UnityEngine;
using CORC.Demo;

public class M2WorldFollower : MonoBehaviour
{
    [Header("Refs")]
    public CORC.CORCM2 m2;
    public M2RoverBridge bridge;
    public Transform marker;
    public Rigidbody markerRb;
    private RoverHandler rover;

    [Header("Unity Mapping")]
    private const float UnityXMin = -4.5f;
    private const float UnityXMax = 4.5f;
    public float unityCenterX = 0f;

    [Header("Motion")]
    [Range(0f, 1f)] public float smooth = 0f; // smoothness factor for position updates
    [Tooltip("How quickly sync bias returns to zero after disturbance.")]
    public float biasRecoverRate = 4f;
    [Tooltip("Clamp on disturbance-induced Unity X bias.")]
    [Range(0f, 10f)] public float maxBias = 2.5f;

    private Vector3 lastPos;
    private float syncXBias = 0f; // disturbance-induced x offset from nominal mapping
    private bool hasBiasBase = false;
    private float biasBaseX = 0f;

    void Awake()
    {
        if (marker == null) marker = transform;
        if (markerRb == null && marker != null) markerRb = marker.GetComponent<Rigidbody>();
        if (bridge == null) bridge = FindFirstObjectByType<M2RoverBridge>();
        rover = marker != null ? marker.GetComponent<RoverHandler>() : null;
        if (rover == null && bridge != null) rover = bridge.rover;
        if (rover != null)
        {
            rover.minX = UnityXMin;
            rover.maxX = UnityXMax;
        }
    }

    void FixedUpdate()
    {
        if (marker == null || markerRb == null) return;

        if (IsKeyboardMode())
        {
            if (rover != null) rover.enableBoundaryClamp = true;
            ApplyKeyboardBias();
            return;
        }

        if (m2 == null || !m2.IsInitialised() || m2.Client == null || !m2.Client.IsConnected()) return;
        if (m2.State == null || m2.State["X"] == null || m2.State["X"].Length == 0) return;

        float m2X = (float)m2.State["X"][0];
        if (float.IsNaN(m2X) || float.IsInfinity(m2X)) return;

        if (rover != null) rover.enableBoundaryClamp = false;
        float nominalX = MapM2ToUnityX(m2X);

        // HRI mode maps stiffness to disturbance-induced bias.
        if (IsM2HriMode())
        {
            if (IsHriDisturbanceActive())
            {
                int dir = ExperimentBlockControl.Instance != null ? ExperimentBlockControl.Instance.CurrentDirection : 0;
                float k = bridge != null ? bridge.StiffnessCmd : 0f;
                float k01 = bridge != null
                    ? Mathf.InverseLerp(bridge.StiffnessMin, bridge.StiffnessMax, k)
                    : 0f;
                syncXBias = dir * maxBias * (1f - k01);
            }
            else
            {
                float recoverAlpha = Mathf.Clamp01(biasRecoverRate * Time.fixedDeltaTime);
                syncXBias = Mathf.Lerp(syncXBias, 0f, recoverAlpha);
            }

            syncXBias = Mathf.Clamp(syncXBias, -maxBias, maxBias);
        }
        else
        {
            syncXBias = 0f;
        }
        hasBiasBase = false;

        float targetX = nominalX + syncXBias;
        rover?.UpdateBoundaryContact(targetX, UnityXMin, UnityXMax);
        Vector3 target = BuildTargetPosition(targetX);
        
        ApplySmoothPosition(target);
    }

    // BuildTargetPosition: construct the target position for the marker from world X target.
    private Vector3 BuildTargetPosition(float worldX)
    {
        float yWorld = marker.position.y;

        return new Vector3(worldX, yWorld, marker.position.z);
    }

    // Map A to the Unity center while preserving both M2 workspace boundaries.
    private float MapM2ToUnityX(float m2X)
    {
        float m2Min = bridge != null ? bridge.M2BoundaryMin : 0.20f;
        float m2Max = bridge != null ? bridge.M2BoundaryMax : 0.44f;
        float m2Center = bridge != null ? bridge.M2Center : 0.32f;

        if (m2X <= m2Center)
        {
            float range = m2Center - m2Min;
            return range > 1e-6f
                ? UnityXMin + (unityCenterX - UnityXMin) * ((m2X - m2Min) / range)
                : unityCenterX;
        }

        float rightRange = m2Max - m2Center;
        return rightRange > 1e-6f
            ? unityCenterX + (UnityXMax - unityCenterX) * ((m2X - m2Center) / rightRange)
            : unityCenterX;
    }

    private bool IsM2HriMode()
    {
        if (bridge == null) return false;
        if (bridge.unityMode != M2RoverBridge.UnityDriveMode.Mode2_M2) return false;
        if (bridge.hriModeCode != 1) return false;
        return true;
    }

    private bool IsKeyboardMode()
    {
        return bridge != null && bridge.unityMode == M2RoverBridge.UnityDriveMode.Mode1_Keyboard;
    }

    private bool IsHriDisturbanceActive()
    {
        if (!IsM2HriMode()) return false;
        return ForceField.DisturbanceU > 0.5f;
    }

    public void ResetBias()
    {
        syncXBias = 0f;
        lastPos = Vector3.zero;
        hasBiasBase = false;
    }

    private void ApplyKeyboardBias()
    {
        bool active = ForceField.DisturbanceU > 0.5f;
        if (active)
        {
            if (!hasBiasBase)
            {
                biasBaseX = markerRb.position.x;
                lastPos = markerRb.position;
                hasBiasBase = true;
            }

            int dir = ExperimentBlockControl.Instance != null ? ExperimentBlockControl.Instance.CurrentDirection : 0;
            if (dir == 0) dir = 1;
            syncXBias = dir * maxBias;
        }
        else
        {
            float recoverAlpha = Mathf.Clamp01(biasRecoverRate * Time.fixedDeltaTime);
            syncXBias = Mathf.Lerp(syncXBias, 0f, recoverAlpha);
            if (Mathf.Abs(syncXBias) < 1e-3f)
            {
                syncXBias = 0f;
                hasBiasBase = false;
                return;
            }
        }

        if (!hasBiasBase) return;
        Vector3 target = BuildTargetPosition(biasBaseX + Mathf.Clamp(syncXBias, -maxBias, maxBias));
        ApplySmoothPosition(target);
    }

    // ApplySmoothPosition: Smoothly move the marker towards the target position. The 'smooth' parameter controls the responsiveness.
    private void ApplySmoothPosition(Vector3 target)
    {
        float alpha = smooth > 0f ? 1f - Mathf.Pow(1f - smooth, Time.fixedDeltaTime * 60f) : 1f; // frame-rate independent smoothing
        lastPos = Vector3.Lerp(lastPos == Vector3.zero ? marker.position : lastPos, target, alpha); // use lastPos to avoid snapping on first frame

        Vector3 next = markerRb.position;
        next.x = lastPos.x;
        markerRb.position = next;
    }

}
