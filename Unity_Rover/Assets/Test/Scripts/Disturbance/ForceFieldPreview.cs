using UnityEngine;

public class ForceFieldPreview : MonoBehaviour
{
    [SerializeField] WorldProbPanel worldProbPanel;

    [Header("Probability (shown ex-ante)")]
    [Range(0f, 1f)] public float triggerProbability = 0.3f;

    [Header("Preview Direction")]
    [SerializeField] int previewDirection = -1;

    public int CurrentDirection => previewDirection;

    void Reset()
    {
        GetComponent<BoxCollider>().isTrigger = true;
    }

    void Awake()
    {
        if (name != "ScoringZone" && worldProbPanel == null)
            worldProbPanel = FindFirstObjectByType<WorldProbPanel>();
    }

    void OnTriggerEnter(Collider other)
    {
        if (!other.transform.root.CompareTag("Player")) return;

        if (name == "ScoringZone")
        {
            ScoreManager.Instance?.BeginScoring();
            return;
        }

        if (ExperimentBlockControl.Instance != null && ExperimentBlockControl.Instance.HasPreparedBlock)
        {
            triggerProbability = ExperimentBlockControl.Instance.CurrentProbability;
            previewDirection = ExperimentBlockControl.Instance.CurrentDirection;
        }

        ScoreManager.Instance?.HideTrialScore();

        if (worldProbPanel != null)
            worldProbPanel.Show(triggerProbability, previewDirection);

    }

    void OnTriggerExit(Collider other)
    {
        if (!other.transform.root.CompareTag("Player")) return;
        if (name == "ScoringZone") return;
        if (worldProbPanel != null)
            worldProbPanel.Hide();
    }

}
