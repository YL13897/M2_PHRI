using UnityEngine;

public class LeaderHandler : MonoBehaviour
{
    [Header("Refs")]
    [SerializeField] Rigidbody rb;
    [SerializeField] Rigidbody leaderRB;

    [Header("Motion")]
    [SerializeField] float targetForwardSpeed = 50f;
    [SerializeField] bool isDriving = false;
    [SerializeField] bool isPaused = false;
    [SerializeField] bool preserveLateralVelocity = true;
    [SerializeField] float lateralDamping = 0.5f;

    [Header("Auto Steer")]
    [SerializeField] float steerSpeed = 10f; // m/s
    [SerializeField] float moveDelay = 1.3f; // Unit: second
    [SerializeField] float arriveToleranceX = 0.05f;
    [SerializeField] float leftTargetX = -8f;
    [SerializeField] float centerTargetX = 0f;
    [SerializeField] float rightTargetX = 8f;

    float leftX;
    float centerX;
    float rightX;
    float targetX;
    float moveDelayTimer;
    bool wasRoverInForceField;
    ForceField[] forceFields;

    void Awake()
    {
        rb ??= GetComponent<Rigidbody>();
        TryFindRover();
        ReadLaneReferences();
        targetX = transform.position.x;
        forceFields = FindObjectsByType<ForceField>(FindObjectsInactive.Include, FindObjectsSortMode.None);
    }

    void FixedUpdate()
    {
        if (rb == null) return;

        if (isPaused)
        {
            Brake();
            return;
        }

        if (isDriving) Accelerate();
        else
        {
            Brake();
            return;
        }

        bool roverInForceField = IsRoverInForceField();
        if (roverInForceField)
        {
            wasRoverInForceField = true;
            moveDelayTimer = 0f;
            HoldLateralPosition();
            return;
        }

        if (wasRoverInForceField)
        {
            SelectNextTargetX();
            moveDelayTimer = moveDelay;
            wasRoverInForceField = false;
        }

        if (moveDelayTimer > 0f)
        {
            moveDelayTimer = Mathf.Max(0f, moveDelayTimer - Time.fixedDeltaTime);
            HoldLateralPosition();
            return;
        }

        AutoSteerToTargetX();
    }

    public void Accelerate()
    {
        if (preserveLateralVelocity)
        {
            Vector3 v = rb.linearVelocity;
            v.z = targetForwardSpeed;
            float alpha = Mathf.Clamp01(lateralDamping * Time.fixedDeltaTime);
            v.x = Mathf.Lerp(v.x, 0f, alpha);
            rb.linearVelocity = v;
            return;
        }

        rb.linearVelocity = transform.forward * targetForwardSpeed;
    }

    public void Brake()
    {
        rb.linearVelocity = Vector3.zero;
    }

    public void SetDriving(bool driving) { isDriving = driving; }
    public bool IsDriving() { return isDriving; }
    public void SetPreserveLateralVelocity(bool preserve) { preserveLateralVelocity = preserve; }

    public void SetPaused(bool paused)
    {
        isPaused = paused;
        if (isPaused) isDriving = false;
    }

    void TryFindRover()
    {
        if (leaderRB != null) return;

        GameObject player = GameObject.FindGameObjectWithTag("Player");
        if (player != null) leaderRB = player.GetComponent<Rigidbody>();
    }

    void ReadLaneReferences()
    {
        leftX = leftTargetX;
        centerX = centerTargetX;
        rightX = rightTargetX;
    }

    bool IsRoverInForceField()
    {
        // TryFindRover();
        // if (leaderRB == null) return false;
        if (forceFields == null || forceFields.Length == 0)
            forceFields = FindObjectsByType<ForceField>(FindObjectsInactive.Include, FindObjectsSortMode.None);

        float roverZ = leaderRB.position.z;
        for (int i = 0; i < forceFields.Length; i++)
        {
            ForceField forceField = forceFields[i];
            if (forceField == null || !forceField.isActiveAndEnabled) continue;

            Collider col = forceField.GetComponent<Collider>();
            if (col == null) continue;

            Bounds bounds = col.bounds;
            if (roverZ >= bounds.min.z && roverZ <= bounds.max.z) return true;
        }

        return false;
    }

    void SelectNextTargetX()
    {
        float p = Random.value;
        if (p < 1f / 3f) targetX = rightX;
        else if (p < 2f / 3f) targetX = leftX;
        else targetX = centerX;
    }

    void AutoSteerToTargetX()
    {
        float xError = targetX - rb.position.x;
        Vector3 v = rb.linearVelocity;

        if (Mathf.Abs(xError) <= Mathf.Max(0.001f, arriveToleranceX))
        {
            Vector3 pos = rb.position;
            pos.x = targetX;
            rb.position = pos;
            v.x = 0f;
            rb.linearVelocity = v;
            return;
        }

        v.x = Mathf.Sign(xError) * steerSpeed;
        rb.linearVelocity = v;
    }

    void HoldLateralPosition()
    {
        Vector3 v = rb.linearVelocity;
        v.x = 0f;
        rb.linearVelocity = v;
    }
}
