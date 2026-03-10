// Minimal 3D follower with explicit X mapping for M2 -> Unity rover alignment.
using UnityEngine;
using CORC.Demo;

public class M2WorldFollower : MonoBehaviour
{
    [Header("Refs")]
    public CORC.CORCM2 m2;
    public M2RoverBridge bridge;
    public Transform marker;
    public Rigidbody markerRb;

    [Header("X Mapping (exact)")]
    [Tooltip("Unity X range.")]
    public float unityXMin = -10f;
    public float unityXMax = 10f;
    [Tooltip("Mapped M2 X range in meters.")]
    public float m2XMin = 0.2f;
    public float m2XMax = 0.44f;
    [Tooltip("Reference center: M2 A point and Unity center line.")]
    public float m2CenterX = 0.32f;
    public float unityCenterX = 0f;

    [Header("Motion")]
    [Range(0f, 1f)] public float smooth = 0.25f;      // same feel as old 2D version
    [Tooltip("How quickly sync bias follows disturbance-induced offset in HRI mode.")]
    public float biasFollowRate = 10f;

    [Tooltip("Only sync lateral X. Keep forward Z velocity fully controlled by rover logic.")]
    public bool affectOnlyX = true;

    private Vector3 lastPos;
    private float syncXBias = 0f;

    void Reset()
    {
        marker = transform;
        markerRb = GetComponent<Rigidbody>();
    }

    void Awake()
    {
        if (marker == null) marker = transform;
        if (markerRb == null && marker != null) markerRb = marker.GetComponent<Rigidbody>();
        if (bridge == null) bridge = FindFirstObjectByType<M2RoverBridge>();
    }

    void FixedUpdate()
    {
        if (m2 == null || marker == null || !m2.IsInitialised() || m2.Client == null || !m2.Client.IsConnected())
        {
            return;
        }
        if (m2.State == null || m2.State["X"] == null || m2.State["X"].Length < 1) return;

        float m2X = (float)m2.State["X"][0];

        float nominalX = MapM2XToUnityX(m2X);
        bool isHriDisturb = IsHriDisturbanceActive();
        if (isHriDisturb)
        {
            float desiredBias = marker.position.x - nominalX;
            float biasAlpha = Mathf.Clamp01(biasFollowRate * Time.fixedDeltaTime);
            syncXBias = Mathf.Lerp(syncXBias, desiredBias, biasAlpha);
        }

        float targetX = nominalX + syncXBias;
        Vector3 target = BuildTargetPosition(targetX);
        
        ApplySmoothPosition(target);
    }

    // BuildTargetPosition: construct the target position for the marker from world X target.
    private Vector3 BuildTargetPosition(float worldX)
    {
        float yWorld = marker.position.y;

        return new Vector3(worldX, yWorld, marker.position.z);
    }

    // Center-preserving linear map. By default this does NOT clamp input range.
    private float MapM2XToUnityX(float m2X)
    {
        float denom = Mathf.Max(1e-6f, m2XMax - m2XMin);

        float m2Input = m2X;

        // Pure linear scaling around center: m2CenterX -> unityCenterX.
        float slope = (unityXMax - unityXMin) / denom;
        return unityCenterX + (m2Input - m2CenterX) * slope;
    }

    private bool IsHriDisturbanceActive()
    {
        if (bridge == null) return false;
        if (bridge.unityMode != M2RoverBridge.UnityDriveMode.Mode2_M2) return false;
        if (bridge.hriModeCode != 1) return false;
        return ForceField.DisturbanceU > 0.5f;
    }

    public void ResetBias()
    {
        syncXBias = 0f;
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
