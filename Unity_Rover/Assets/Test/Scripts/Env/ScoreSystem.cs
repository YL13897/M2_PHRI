using UnityEngine;

public class ScoreSystem : MonoBehaviour
{
    [Header("Player")]
    [SerializeField] Rigidbody targetRb;

    [Header("Leader")]
    [SerializeField] Rigidbody leaderRb;

    [Header("Scoring Parameters")]
    public float trackingScoreRate = 10.0f; // max score per second when player x stays within the leader width band
    public float CappedWidthX = 1.0f; // full-score band around the leader center
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

        if (leaderRb == null)
        {
            LeaderHandler leader = FindFirstObjectByType<LeaderHandler>();
            if (leader != null) leaderRb = leader.GetComponent<Rigidbody>();
        }

        if (targetRb == null || leaderRb == null)
            Debug.LogError("[ScoreSystem] No Rigidbody assigned or found.");

        if (ScoreManager.Instance == null)
            Debug.LogError("[ScoreSystem] No ScoreManager instance found in the scene.");
    }

    void FixedUpdate()
    {
        // if (targetRb == null || leaderRb == null || ScoreManager.Instance == null) return;

        if (!ScoreManager.Instance.HasStartedScoring || ScoreManager.Instance.IsScorePaused)
        {
            return;
        }

        ScoreManager.Instance.UpdateBoundaryUI(targetRb.position.x, minX, maxX);

        bool boundaryContact = DetectBoundaryContact();
        // Global gate: even if multiple callers fire, penalty can apply at most once per cooldown window.
        if (boundaryContact)
        {   
            ScoreManager.Instance.TryApplyBoundaryPenalty(penaltyValue, penaltyCooldown);
            return;
        }

        float xError = Mathf.Abs(targetRb.position.x - leaderRb.position.x);
        float cappedError = Mathf.Max(0f, xError - CappedWidthX);
        float maxError = Mathf.Max(0.001f, (maxX - minX) - CappedWidthX);
        float closeness = 1f - Mathf.Clamp01(cappedError / maxError);
        float deltaScore = trackingScoreRate * closeness * Time.fixedDeltaTime;
        ScoreManager.Instance.AddToScores(deltaScore);
    }

    bool DetectBoundaryContact()
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
