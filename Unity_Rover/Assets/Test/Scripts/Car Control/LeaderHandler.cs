using UnityEngine;

public class LeaderHandler : MonoBehaviour
{
    [Header("Refs")]
    [SerializeField] Rigidbody rb;

    [Header("Motion")]
    [SerializeField] float targetForwardSpeed = 50f;
    [SerializeField] bool isDriving = false;
    [SerializeField] bool isPaused = false;
    [SerializeField] bool preserveLateralVelocity = true;
    [SerializeField] float lateralDamping = 0.5f;

    [Header("Auto Steer")]
    [SerializeField] float steerSpeed = 10f; // m/s
    [SerializeField] float moveDelay = 1.0f; // second
    [SerializeField] float arriveToleranceX = 0.05f;
    [SerializeField] float leftTargetX = -4f;
    [SerializeField] float centerTargetX = 0f;
    [SerializeField] float rightTargetX = 4f;

    float leftX;
    float centerX;
    float rightX;
    float targetX;
    float moveDelayTimer;
    Vector3 initialPosition;
    // Quaternion initialRotation;

    public float LeftLaneX => leftX;
    public float CenterLaneX => centerX;
    public float RightLaneX => rightX;

    void Awake()
    {
        rb ??= GetComponent<Rigidbody>();
        initialPosition = transform.position;
        ReadLaneReferences();
        targetX = transform.position.x;
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
            float alpha = lateralDamping * Time.fixedDeltaTime;
            if (alpha > 1f) alpha = 1f; // Avoid going below 0, as Time.fixedDeltaTime and damping are positive
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

    public void ResetForBlockStart()
    {
        transform.position = initialPosition;
        // transform.rotation = initialRotation;
        if (rb != null)
        {
            rb.position = initialPosition;
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        moveDelayTimer = 0f;
        targetX = initialPosition.x;
    }

    public void SetTargetLane(float targetLaneX)
    {
        targetX = targetLaneX;
        moveDelayTimer = moveDelay;
    }

    void ReadLaneReferences()
    {
        leftX = leftTargetX;
        centerX = centerTargetX;
        rightX = rightTargetX;
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
