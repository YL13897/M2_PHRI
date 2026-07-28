using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RoverHandler : MonoBehaviour
{

    [SerializeField]
    Rigidbody rb;
    readonly float targetForwardSpeed = 50f; // m/s
    readonly float steeringMultiplier = 1e3f;
    [SerializeField]
    bool isDriving = false;
    [SerializeField]
    bool isPaused = false;
    [SerializeField]
    bool preserveLateralVelocity = true; 
    [SerializeField]
    float lateralDamping = 4.5f;

    [Header("Boundary")]
    public float minX = -4.5f;
    public float maxX = 4.5f;
    public bool enableBoundaryClamp = true;
    const float MinBoundaryTolerance = 0.001f;
    [SerializeField] float boundaryTolerance = MinBoundaryTolerance;
    [SerializeField] ParticleSystem explosionL;
    [SerializeField] ParticleSystem explosionR;
    public bool BoundaryContact { get; private set; }
    private bool boundaryHitLatched;

    // readonly int logEveryNFrames = 50;
    // public float sideDamping = 0.0f;

    // Input
    Vector2 input = Vector2.zero;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        rb ??= GetComponent<Rigidbody>();
        Debug.Assert(rb != null, "[carHandler] Rigidbody missing!");
        // rb.linearDamping = 0f;
    }

    // Accelerate(): Auto move forward with fixed speed by directly setting velocity.
    public void Accelerate()
    {
        if (preserveLateralVelocity)
        // if (true)
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

    // Brake(): Directly set velocity to zero.
    public void Brake()
    {
        rb.linearVelocity = Vector3.zero;
    }

    // Steer(): Apply side force proportional to the input and current forward speed.
    public void Steer()
    {
        if(Mathf.Abs(input.x) > 0)
        {
            if (enableBoundaryClamp && ((rb.position.x <= minX && input.x < 0f) || (rb.position.x >= maxX && input.x > 0f)))
                return;

            rb.AddForce(rb.transform.right * steeringMultiplier * input.x);
        }
    }


    // For M2 input, it is already normalized by the posScale and forceScale, 
    // so we can directly use the raw input without normalization.
    public void SetInputRaw(Vector2 inputVector) { input = inputVector; }
    
    
    // For keyboard input, normalize input to have max magnitude of 1. 
    public void SetInput(Vector2 inputVector) { input = inputVector.normalized; } 

    public void SetDriving(bool driving) { isDriving = driving; } // Handoffed by M2RoverBridge.cs using: rover.SetDriving(isDriving && !isPaused && !emgHold);
    public void SetPreserveLateralVelocity(bool preserve) { preserveLateralVelocity = preserve; }

    public bool IsDriving() { return isDriving; }

    public void PlayBoundaryExplosion()
    {
        float centerX = 0.5f * (minX + maxX);
        if (rb.position.x < centerX)
            explosionL?.Play();
        else
            explosionR?.Play();
    }

    public bool ConsumeBoundaryHit()
    {
        bool hit = boundaryHitLatched;
        boundaryHitLatched = false;
        return hit;
    }

    public void ResetBoundaryHit()
    {
        boundaryHitLatched = false;
    }

    public void UpdateBoundaryContact(float x, float minBoundary, float maxBoundary)
    {
        float tolerance = Mathf.Max(MinBoundaryTolerance, boundaryTolerance);
        BoundaryContact = x <= minBoundary + tolerance || x >= maxBoundary - tolerance;
        if (BoundaryContact)
            boundaryHitLatched = true;
    }

    public void SetPaused(bool paused)
    {
        isPaused = paused;
        if (isPaused)
        {
            isDriving = false;
            input = Vector2.zero;
        }
    }

    void FixedUpdate()
    {
        if (rb == null) return;
        
        if (isPaused)
        {
            Brake();
            ClampBoundary();
            return;
        }

        if (isDriving) Accelerate();
        else Brake();

        Steer();
        ClampBoundary();
        // ApplyDamping();
    }

    void ClampBoundary()
    {
        Vector3 pos = rb.position;
        Vector3 vel = rb.linearVelocity;
        UpdateBoundaryContact(pos.x, minX, maxX);
        if (!enableBoundaryClamp) return;

        bool clamped = false;

        if (pos.x <= minX)
        {
            pos.x = minX;
            if (vel.x < 0f) vel.x = 0f;
            clamped = true;
        }
        else if (pos.x >= maxX)
        {
            pos.x = maxX;
            if (vel.x > 0f) vel.x = 0f;
            clamped = true;
        }

        if (!clamped) return;

        rb.position = pos;
        rb.linearVelocity = vel;
    }

}
