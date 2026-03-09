using UnityEngine;
using TMPro;
using UnityEngine.UI;

public class ScoreManager : MonoBehaviour
{
    public static ScoreManager Instance { get; private set; }

    [Header("UI")]
    [SerializeField] TextMeshProUGUI globalScoreText;
    [SerializeField] TextMeshProUGUI sectionScoreText;
    [SerializeField] Slider sectionBar;

    [Header("Section bar settings")]
    // [SerializeField] float sectionBarMax = 9f; // tune
    // [SerializeField] bool clampSectionBar = true;

    public float GlobalScore { get; private set; }
    public float SectionScore { get; private set; }
    public bool IsScorePaused { get; private set; }

    void Awake()
    {
        if (Instance != null && Instance != this) { Destroy(gameObject); return; }
        Instance = this;

        // Optional: persist across scene reloads
        // DontDestroyOnLoad(gameObject);

        if (globalScoreText != null) globalScoreText.raycastTarget = false;
        if (sectionScoreText != null) sectionScoreText.raycastTarget = false;

        ResetAll();
    }

    public void ResetAll()
    {
        GlobalScore = 0f;
        IsScorePaused = false;
        ResetSection();
        RefreshUI();
    }

    public void SetScorePaused(bool paused)
    {
        IsScorePaused = paused;
    }

    public void ResetSection()
    {
        SectionScore = 0f;
        if (sectionBar != null)
        {
            sectionBar.minValue = 0f;
            // sectionBar.maxValue = sectionBarMax;
            sectionBar.maxValue = 9f;
            sectionBar.value = 0f;
        }
        RefreshUI();
    }

    public void AddToScores(float delta)
    {
        GlobalScore += delta;
        SectionScore += delta;

        RefreshUI();
    }

    public void ApplyBoundaryPenalty(float penalty)
    {
        GlobalScore -= penalty;
        SectionScore -= penalty;

        RefreshUI();
    }


    // UI update
    public void UpdateBoundaryUI(float x, float minX, float maxX)
    {
        float halfWidth = 0.5f * (maxX - minX);
        sectionBar.maxValue = halfWidth;
        float distFromCenter = Mathf.Abs(x);
        // Debug.Log($"{distFromCenter}");
        float fill = Mathf.Clamp(distFromCenter, 0f, halfWidth);
        sectionBar.value = fill; 

    }

    void RefreshUI()
    {
        if (globalScoreText != null)
            globalScoreText.text = $"ToSc: {GlobalScore:0}";

        if (sectionScoreText != null)
            sectionScoreText.text = $"SeSc: {SectionScore:0}";

        // if (sectionBar != null)
        // {   
        //     float v = SectionScore;
        //     if (clampSectionBar) v = Mathf.Clamp(v, 0f, sectionBarMax);
        //     sectionBar.value = v;
        // }
    }
}
