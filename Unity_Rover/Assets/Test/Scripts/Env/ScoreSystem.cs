// 

using UnityEngine;


public class ScoreSystem : MonoBehaviour
{  
    [Header("Player")]
    [SerializeField] Rigidbody targetRb; // optional manual assignment
   

    [Header("Centerline reference")]
    public float centerX = 0f;

    // [Header("Spring parameters")]
    // public float springK = 2e3f;   // stiffness (N/m)
    // public float damperC = 5e2f;    // damping (N*s/m)

    [Header("Limits (optional)")]
    public float maxForce = 2000f; // clamp to avoid crazy forces

    [Header("Scoring Parameters")]
    public float deviationScoreRate = 1.0f;   // score per meter per second (abs(e)*dt)
    public float forceIntegralWeight = 0.00001f; // score per (N*s) (abs(fx)*dt)
    public float penaltyValue = 500f;
    [Header("Boundary Clamp Control")]
    public bool clampDisabled = true;
    public bool enableBoundaryClamp = true;
    public float minX = -9.0f;
    public float maxX = 9.0f;

    [Header("Penalty Control")]
    public float penaltyCooldown = 1.0f;   // seconds between penalty triggers
    private float lastPenaltyTime = -999f;
    private bool clampEnabledByFirstField = false;


    void Awake()
    {
        // Auto-find Player if not assigned
        if (targetRb == null)
        {
            GameObject player = GameObject.FindGameObjectWithTag("Player");
            if (player != null) targetRb = player.GetComponent<Rigidbody>();
        }

        if (targetRb == null)
        {
            Debug.LogError("[ScoreSystem] No target Rigidbody assigned or found with tag 'Player'.");
        }

        if (clampDisabled)
        {
            enableBoundaryClamp = false;
            clampEnabledByFirstField = false;
            ForceField.ResetFirstEntryFlag();
        }
    }

    void OnEnable()
    {
        ForceField.OnFirstPlayerEnteredAnyField += EnableClampFromFirstField;
    }

    void OnDisable()
    {
        ForceField.OnFirstPlayerEnteredAnyField -= EnableClampFromFirstField;
    }

    private void EnableClampFromFirstField()
    {
        if (!clampDisabled || clampEnabledByFirstField) return;

        enableBoundaryClamp = true;
        clampEnabledByFirstField = true;
        // Debug.Log("[ScoreSystem] Boundary clamp enabled by first ForceField entry.");
    }

    
    void FixedUpdate()
    {
        if (targetRb == null) return;
        if (ScoreManager.Instance == null) return;

        if (ScoreManager.Instance.IsScorePaused)
        {
            targetRb.linearVelocity = Vector3.zero;
            return;
        }

        ScoreManager.Instance.UpdateBoundaryUI(targetRb.position.x, minX, maxX);
        
        bool penaltyFlag = ClampPosition();

        // signed lateral error (positive if to the right of center)
        float e = targetRb.position.x - centerX;

        // lateral velocity (x direction)
        float eDot = targetRb.linearVelocity.x;
        

        // ---- scoring (integrals via dt) ----
        float dt = Time.fixedDeltaTime;

        // deviation integral ~ ∫ |e| dt
        float deviationTerm = deviationScoreRate * Mathf.Abs(e) * dt;

        // float deltaScore = deviationTerm + forceTerm;
        float deltaScore = deviationTerm;

        if(penaltyFlag)
        { 
            // only punish once per cooldown window
            if (Time.time - lastPenaltyTime >= penaltyCooldown)
            {
                ScoreManager.Instance.ApplyBoundaryPenalty(penaltyValue);
                lastPenaltyTime = Time.time;
            }
        }
        else
        {
            ScoreManager.Instance.AddToScores(deltaScore);
        }

    }



    bool ClampPosition()
    {
        if (!enableBoundaryClamp) return false;

        Vector3 pos = targetRb.position;

        if (pos.x < minX)
        {
            pos.x = minX;
            targetRb.position = pos;
            return true;
        }
        if (pos.x > maxX)
        {
            pos.x = maxX;
            targetRb.position = pos;
            return true;
        }
        return false;
    }

}
