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
        if (!other.transform.root.CompareTag("Player")) return; // Ensure that the collider belongs to the player.

        triggered = true;

        if (ScoreManager.Instance == null) return;

        if (name == "ScoringZone")
        {
            ScoreManager.Instance.BeginScoring();
            return;
        }

        if (transform.root.name.StartsWith("Section_BGIN")) 
            return; 

        // Notify the block control that we've entered a new section, so it can handle scoring and block progression logic.
        ExperimentBlockControl.Instance?.NotifySectionEntered(transform.root.name); 

        if (ScoreManager.Instance.SectionScoringActive && ScoreManager.Instance.FinalizeAtSectionEnd)
            ScoreManager.Instance.UpdateLastTotalScore();

        ScoreManager.Instance.ResetSection();
    }
}
