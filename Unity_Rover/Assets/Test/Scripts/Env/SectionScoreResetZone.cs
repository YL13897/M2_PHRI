using UnityEngine;

public class SectionScoreResetZone : MonoBehaviour
{
    bool triggered = false;

    void Reset()
    {
        var bc = GetComponent<BoxCollider>();
        if (bc != null) bc.isTrigger = true;
    }

    void OnEnable()
    {
        // important for pooled sections
        triggered = false;
    }

    void OnTriggerEnter(Collider other)
    {
        if (triggered) return;
        if (!other.transform.root.CompareTag("Player")) return;

        triggered = true;

        if (ScoreManager.Instance == null) return;

        if (transform.root.name.StartsWith("Section_BGIN"))
            return;

        if (!ScoreManager.Instance.HasStartedScoring)
            ScoreManager.Instance.BeginScoring();
        else
            ScoreManager.Instance.ResetSection();
    }
}
