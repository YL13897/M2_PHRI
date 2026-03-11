using UnityEngine;

public class ScoreSystem : MonoBehaviour
{
    [Header("Player")]
    [SerializeField] Rigidbody targetRb;

    [Header("Centerline reference")]
    public float centerX = 0f;

    [Header("Scoring Parameters")]
    public float deviationScoreRate = 1.0f; // score per meter per second (abs(e)*dt)
    public float penaltyValue = 500f;

    [Header("Global Boundary")]
    public float minX = -9.0f;
    public float maxX = 9.0f;
    public bool enableBoundaryClamp = true;

    [Header("Penalty Control")]
    public float penaltyCooldown = 1.0f; // seconds between penalties while touching boundary

    void Awake()
    {
        if (targetRb == null)
        {
            GameObject player = GameObject.FindGameObjectWithTag("Player");
            if (player != null) targetRb = player.GetComponent<Rigidbody>();
        }

        if (targetRb == null)
            Debug.LogError("[ScoreSystem] No target Rigidbody assigned or found with tag 'Player'.");
    }

    void FixedUpdate()
    {
        if (targetRb == null) return;
        if (ScoreManager.Instance == null) return;

        // If the score is currently paused, prevent player movement and skip scoring.
        if (ScoreManager.Instance.IsScorePaused)
        {
            targetRb.linearVelocity = Vector3.zero;
            return;
        }

        // Update the boundary UI with the player's current position and the defined boundaries.
        ScoreManager.Instance.UpdateBoundaryUI(targetRb.position.x, minX, maxX);

        bool boundaryContact = ClampAndDetectBoundaryContact();
        if (boundaryContact)
        {
            // If the player is in contact with the boundary, apply a penalty to the score and skip further scoring for this frame.
            ScoreManager.Instance.TryApplyBoundaryPenalty(penaltyValue, penaltyCooldown);
            return;
        }

        float e = targetRb.position.x - centerX;
        float deltaScore = deviationScoreRate * Mathf.Abs(e) * Time.fixedDeltaTime;
        ScoreManager.Instance.AddToScores(deltaScore);
    }

    bool ClampAndDetectBoundaryContact()
    {
        if (!enableBoundaryClamp) return false;

        Vector3 pos = targetRb.position;
        Vector3 vel = targetRb.linearVelocity;
        bool hitBoundary = false;

        if (pos.x <= minX)
        {
            pos.x = minX;
            if (vel.x < 0f) vel.x = 0f;
            hitBoundary = true;
        }
        else if (pos.x >= maxX)
        {
            pos.x = maxX;
            if (vel.x > 0f) vel.x = 0f;
            hitBoundary = true;
        }

        if (hitBoundary)
        {
            targetRb.position = pos;
            targetRb.linearVelocity = vel;
        }

        return hitBoundary;
    }
}
