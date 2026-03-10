using System;
using UnityEngine;
// using System;

public class ForceField : MonoBehaviour
{
    public static bool HasPlayerEnteredAnyField { get; private set; } = false;
    // Unified disturbance source u(t) generated in Unity. Current design uses binary 0/1.
    public static float DisturbanceU { get; private set; } = 0f;
    // Estimated disturbance duration used by M2 as a safety auto-off timeout.
    public static float DisturbanceDurationSec { get; private set; } = 0.4f;
    public static event Action OnFirstPlayerEnteredAnyField;

    [SerializeField] ForceFieldPreview forceFieldPreview; 
    
    [SerializeField]
    Rigidbody targetRb;
    [SerializeField]
    CORC.Demo.M2RoverBridge bridge;

    public float forceMagnitude = 2e3f;
    public Vector3 fixedDirection = Vector3.left; // direction of the force field effect

    // sampled once per enable / per section reuse
    // get; -> Anyone can read the value
    // private set; Only this class can change the value
    public bool IsActiveThisRun { get; private set; } 
    private bool playerInsideThisActiveField = false;
    private float disturbanceElapsedSec = 0f;

    // ResetFirstEntryFlag(): Resets the static flags related to player entry and disturbance.
    public static void ResetFirstEntryFlag()
    {
        HasPlayerEnteredAnyField = false;
        DisturbanceU = 0f;
    }

    // OnEnable(): is called immediately after an object or script component becomes active.
    void OnEnable()
    {
        IsActiveThisRun = false;
        targetRb = null;
        playerInsideThisActiveField = false;
        disturbanceElapsedSec = 0f;
        if (bridge == null)
            bridge = FindFirstObjectByType<CORC.Demo.M2RoverBridge>();
    }

    // OnDisable(): Robustness protection - In case the field gets disabled while the player is still inside, we should reset the state to avoid "stuck in disturbance" issues.
    // OnDisable() is called when the behaviour becomes disabled or inactive.
    void OnDisable()
    {
        if (playerInsideThisActiveField)
        {
            playerInsideThisActiveField = false;
            DisturbanceU = 0f;
            disturbanceElapsedSec = 0f;
        }
    }


    // ------------------------------------------- Core Logic -----------------------------------
    /* 
    OnTriggerEnter() ensures that when player enters an active disturbance field, Unity disturbance source u(t) is enabled,
    and when the player exits, u(t) is disabled.
    */

    // OnTriggerEnter(): Detects when the player enters the force field trigger, samples the disturbance state based on the defined probability, 
    // and updates the disturbance source u(t) accordingly.
    void OnTriggerEnter(Collider other)
    {
        if (!other.CompareTag("Player")) return;

        // if (!GlobalRandomSeed.IsInitialized)
        //     UnityEngine.Random.InitState(GlobalRandomSeed.Seed);

        if (!HasPlayerEnteredAnyField)
        {
            HasPlayerEnteredAnyField = true;
            OnFirstPlayerEnteredAnyField?.Invoke(); // Notify ScoreSystem to enable boundary clamp on first entry
        }

        // Read the trigger probability for this force field from the ForceFieldPreview component.
        float Probability = forceFieldPreview.triggerProbability; 

        // sample ONCE per section reuse
        IsActiveThisRun = UnityEngine.Random.value < Probability;
        if (!IsActiveThisRun) return;
        targetRb = other.attachedRigidbody; // Cache the player's Rigidbody for applying forces in FixedUpdate.

        if (!playerInsideThisActiveField)
        {
            playerInsideThisActiveField = true;
            disturbanceElapsedSec = 0f;
            DisturbanceU = 1f;
        }
    }

    // OnTriggerExit(): Detects when the player exits the force field trigger and updates the disturbance state accordingly.
    void OnTriggerExit(Collider other)
    {
        if (!other.CompareTag("Player")) return;
        targetRb = null;
        if (playerInsideThisActiveField)
        {
            playerInsideThisActiveField = false;
            DisturbanceU = 0f;
            disturbanceElapsedSec = 0f;
        }
    }

    void FixedUpdate()
    {
        if (playerInsideThisActiveField && targetRb == null)
        {
            playerInsideThisActiveField = false;
            DisturbanceU = 0f;
            disturbanceElapsedSec = 0f;
        }

        if (!IsActiveThisRun) return;
        if (targetRb == null) return;
        if (DisturbanceU <= 0f) return;

        // Safe guard: If disturbance duration is exceeded, automatically turn off the disturbance.
        disturbanceElapsedSec += Time.fixedDeltaTime;
        if (disturbanceElapsedSec >= DisturbanceDurationSec)
        {
            DisturbanceU = 0f;
            return;
        }

        // In M2 pHRI mode, disturbance force is generated on M2 side; skip Unity force to avoid double disturbance.
        if (bridge != null
            && bridge.unityMode == CORC.Demo.M2RoverBridge.UnityDriveMode.Mode2_M2
            && bridge.hriModeCode == 2) return;

        Vector3 dir = fixedDirection.normalized;
        
        targetRb.AddForce(dir * (forceMagnitude * DisturbanceU), ForceMode.Force);
        // targetRb.MovePosition(targetRb.position + dir * speed * Time.fixedDeltaTime);
    }

}
