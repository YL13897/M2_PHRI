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
            rb.AddForce(rb.transform.right * steeringMultiplier * input.x);
        }
    }


    // public void ApplyDamping()
    // {
    //     Vector3 vLocal = rb.transform.InverseTransformDirection(rb.linearVelocity);

    //     float vSide = vLocal.x;

    //     // Apply damping only to the steering: F = -k * vSide * right
    //     rb.AddForce(-transform.right * (vSide * sideDamping), ForceMode.Force);
    // }


    // For M2 input, it is already normalized by the posScale and forceScale, 
    // so we can directly use the raw input without normalization.
    public void SetInputRaw(Vector2 inputVector) { input = inputVector; }
    
    
    // For keyboard input, normalize input to have max magnitude of 1. 
    public void SetInput(Vector2 inputVector) { input = inputVector.normalized; } 

    public void SetDriving(bool driving) { isDriving = driving; }
    public void SetPreserveLateralVelocity(bool preserve) { preserveLateralVelocity = preserve; }

    public bool IsDriving() { return isDriving; }

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
            return;
        }

        if (isDriving) Accelerate();
        else Brake();

        Steer();
        // ApplyDamping();
    }


}
