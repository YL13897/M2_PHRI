using UnityEngine;
using TMPro;
using UnityEngine.UI;

public class ScoreManager : MonoBehaviour
{
    public static ScoreManager Instance { get; private set; }

    [Header("UI")]
    [SerializeField] TextMeshProUGUI globalScoreText;
    [SerializeField] TextMeshProUGUI sectionScoreText;
    [SerializeField] TextMeshProUGUI emgScoreText;
    [SerializeField] Slider sectionBar;

    public float GlobalScore { get; private set; }
    public float SectionScore { get; private set; }
    public float EmgScore { get; private set; }
    public double EmgTimestamp { get; private set; }
    public string EmgDetails { get; private set; }
    public bool HasStartedScoring { get; private set; }
    public bool IsScorePaused { get; private set; }
    private double nextPenaltytime = 0.0;

    void Awake()
    {
        if (Instance != null && Instance != this) { Destroy(gameObject); return; }
        Instance = this;

        // Optional: persist across scene reloads
        // DontDestroyOnLoad(gameObject);

        if (globalScoreText != null) globalScoreText.raycastTarget = false;
        if (sectionScoreText != null) sectionScoreText.raycastTarget = false;
        if (emgScoreText != null) emgScoreText.raycastTarget = false;

        ResetAll();
    }

    public void ResetAll()
    {
        GlobalScore = 0f;
        EmgScore = 0f;
        EmgTimestamp = 0.0;
        EmgDetails = "";
        HasStartedScoring = false;
        IsScorePaused = false;
        nextPenaltytime = 0.0;
        ResetSection();
        RefreshUI();
    }

    public void BeginScoring()
    {
        if (HasStartedScoring) return;
        HasStartedScoring = true;
        ResetSection();
    }

    public void SetEmgScore(float score, double timestamp)
    {
        EmgScore = Mathf.Max(0f, score);
        EmgTimestamp = timestamp;
        RefreshUI();
    }

    public void SetEmgPreview(float score, double timestamp, string details)
    {
        EmgScore = Mathf.Max(0f, score);
        EmgTimestamp = timestamp;
        EmgDetails = details ?? "";
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

    public bool TryApplyBoundaryPenalty(float penalty, float cooldownSec)
    {
        double now = Time.realtimeSinceStartupAsDouble;
        float cooldown = Mathf.Max(0.5f, cooldownSec);
        if (now < nextPenaltytime) return false;

        ApplyBoundaryPenalty(Mathf.Abs(penalty));
        nextPenaltytime = now + cooldown;
        return true;
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

        if (emgScoreText != null)
            emgScoreText.text = string.IsNullOrEmpty(EmgDetails)
                ? $"EMG: {EmgScore:0.00} | t: {EmgTimestamp:0.000}s"
                : EmgDetails;
    }
}
