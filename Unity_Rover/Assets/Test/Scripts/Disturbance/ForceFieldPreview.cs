using UnityEngine;

public class ForceFieldPreview : MonoBehaviour
{

    [SerializeField] WorldProbPanel worldProbPanel;  // drag the panel (on the car) or a manager

    [Header("Probability (shown ex-ante)")]
    [Range(0f, 1f)] public float triggerProbability = 0.3f;

    void Reset()
    {
        GetComponent<BoxCollider>().isTrigger = true;
    }

    void OnTriggerEnter(Collider other)
    {
        if (!other.CompareTag("Player")) return;
        triggerProbability = Random.value;
        // Show probability (ex-ante cue)
        // Debug.Log($"Preview Check: {triggerProbability}");
        if (WorldProbPanel.Instance != null)
            {WorldProbPanel.Instance.Show(triggerProbability);}

    }

    void OnTriggerExit(Collider other)
    {
        if (!other.CompareTag("Player")) return;
         if (WorldProbPanel.Instance != null)
            {WorldProbPanel.Instance.Hide();}
    }

}
