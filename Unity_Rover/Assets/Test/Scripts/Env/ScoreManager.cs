using UnityEngine;
using TMPro;
using UnityEngine.UI;

public class ScoreManager : MonoBehaviour
{
    public enum ScoringWindowMode
    {
        FullSection,
        RiskOnly
    }

    public static ScoreManager Instance { get; private set; }

    [Header("UI")]
    [SerializeField] TextMeshProUGUI emgScoreText;
    [SerializeField] TextMeshProUGUI TrialScoreText;
    [SerializeField] TextMeshProUGUI DebugTrialScoreText;
    [SerializeField] bool alwaysShowTrialScore = false;
    [SerializeField] bool showTotalOnly = false;
    [SerializeField] bool useTrackScore = true;
    [SerializeField] bool useStiffnessScore = true;
    [SerializeField] bool useForceScore = false;
    [SerializeField] Slider sectionBar;

    [Header("Scoring Window")]
    [SerializeField] ScoringWindowMode scoringWindowMode = ScoringWindowMode.FullSection;
    const float fullScoreDuration = 1.4f;
    const float riskOnlyDuration = 0.4f;

    public float SectionScore { get; private set; }
    public float EmgScore { get; private set; }
    public float TrackSectionScore { get; private set; }
    public float ForceSectionScore { get; private set; }
    public float ForceAvg { get; private set; }
    public float EmgSectionScore { get; private set; }
    public float BoundaryPenaltyScore { get; private set; }
    public double EmgTimestamp { get; private set; }
    public string EmgDetails { get; private set; }
    public float HandleFx { get; private set; }
    public float TrackCost { get; private set; }
    public float WTrack { get; private set; }
    public float WForce { get; private set; }
    public float WEmg { get; private set; }
    public float TotalCost { get; private set; }
    public bool HasStartedScoring { get; private set; }
    public bool SectionScoringActive { get; private set; }
    public bool IsScorePaused { get; private set; }
    public float ScoreDuration => fullScoreDuration;
    public float DisplaySectionScore => IsDisplayActive ? displaySectionScore : 0f;
    public bool FinalizeAtSectionEnd => scoringWindowMode == ScoringWindowMode.FullSection;

    private bool boundaryPenaltyApplied;
    private bool displayActive;
    private float displayElapsed;
    private float lastTrackScore;
    private float lastForceScore;
    private float lastEmgScore;
    private float lastTotalScore;
    private float displaySectionScore;
    private float lastDisplayTrackScore;
    private float lastDisplayForceScore;
    private float lastDisplayEmgScore;
    private float lastDisplayScore;
    private bool showLiveScore;
    private CORC.Demo.M2RoverBridge bridge;
    private float DisplayDuration =>
        scoringWindowMode == ScoringWindowMode.RiskOnly ? riskOnlyDuration : fullScoreDuration;
    private bool IsDisplayActive => scoringWindowMode != ScoringWindowMode.RiskOnly || displayActive;
    private float DisplayScale => fullScoreDuration / DisplayDuration;
    private float DisplayTrackSectionScore => IsDisplayActive ? TrackSectionScore * DisplayScale : 0f;
    private float DisplayForceSectionScore => IsDisplayActive ? ForceSectionScore * DisplayScale : 0f;
    private float DisplayEmgSectionScore => IsDisplayActive ? EmgSectionScore * DisplayScale : 0f;

    void Awake()
    {
        if (Instance != null && Instance != this) { Destroy(gameObject); return; }
        Instance = this;

        // Optional: persist across scene reloads
        // DontDestroyOnLoad(gameObject);

        if (emgScoreText != null)
        {
            emgScoreText.raycastTarget = false; // raycastTarget = false to allow clicks to pass through
            emgScoreText.richText = true; // Enable rich text formatting for colour coding
        }
        if (TrialScoreText != null) TrialScoreText.raycastTarget = false;
        if (DebugTrialScoreText != null) DebugTrialScoreText.raycastTarget = false;
        bridge = FindFirstObjectByType<CORC.Demo.M2RoverBridge>();

        ResetAll();
    }

    void Update()
    {
        if (scoringWindowMode != ScoringWindowMode.RiskOnly) return;
        if (!SectionScoringActive || !displayActive || IsScorePaused) return;

        displayElapsed += Time.deltaTime;
        if (displayElapsed >= riskOnlyDuration)
        {
            UpdateLastTotalScore();
            displayActive = false;
            displayElapsed = 0f;
        }
    }

    public void ResetAll()
    {
        EmgScore = 0f;
        EmgTimestamp = 0.0;
        EmgDetails = "";
        ResetCompositeMetrics();
        ClearTrialScore();
        HasStartedScoring = false;
        SectionScoringActive = false;
        IsScorePaused = false;
        displayActive = false;
        displayElapsed = 0f;
        ResetSection();
        UpdateTrialScoreVisible();
        RefreshUI();
    }

    public void BeginScoring()
    {
        ClearCurrentSectionScores();
        HasStartedScoring = true;
        SectionScoringActive = true;
        displayActive = true;
        displayElapsed = 0f;
        showLiveScore = true;
        if (!alwaysShowTrialScore)
            HideTrialScore();
        RefreshUI();
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

    public void SetCompositeMetrics(float handleFx, float trackCost, float wTrack, float wForce, float wEmg, float totalCost)
    {
        HandleFx = handleFx;
        TrackCost = trackCost;
        WTrack = wTrack;
        WForce = wForce;
        WEmg = wEmg;
        TotalCost = totalCost;
    }

    public void ResetSection()
    {
        ClearCurrentSectionScores();
        RefreshUI();
    }

    void ClearCurrentSectionScores()
    {
        SectionScore = 0f;
        displaySectionScore = 0f;
        TrackSectionScore = 0f;
        ForceSectionScore = 0f;
        ForceAvg = 0f;
        EmgSectionScore = 0f;
        BoundaryPenaltyScore = 0f;
        SectionScoringActive = false;
        displayActive = false;
        displayElapsed = 0f;
        boundaryPenaltyApplied = false;
        ResetCompositeMetrics();
        if (sectionBar != null)
        {
            sectionBar.minValue = 0f;
            sectionBar.maxValue = 9f;
            sectionBar.value = 0f;
        }
    }

    public void ResetBlockScores()
    {
        ClearCurrentSectionScores();
        ClearTrialScore();
        HasStartedScoring = false;
        UpdateTrialScoreVisible();

        if (sectionBar != null)
        {
            sectionBar.minValue = 0f;
            // sectionBar.maxValue = 9f;
            sectionBar.maxValue = 4.5f;
            sectionBar.value = 0f;
        }

        RefreshUI();
    }

    public void AddToScores(float trackDelta, float forceDelta, float emgDelta)
    {
        float delta = trackDelta + forceDelta + emgDelta;
        SectionScore += delta;
        if (IsDisplayActive)
            displaySectionScore += SelectedScore(trackDelta, forceDelta, emgDelta) * DisplayScale;

        TrackSectionScore += trackDelta;
        ForceSectionScore += forceDelta;
        EmgSectionScore += emgDelta;

        RefreshUI();
    }

    public void SetForceAvg(float forceAvg)
    {
        ForceAvg = Mathf.Max(0f, forceAvg);
        RefreshUI();
    }

    public void ApplyBoundaryPenalty(float penalty)
    {
        float penaltyAbs = Mathf.Abs(penalty);
        SectionScore -= penaltyAbs;
        BoundaryPenaltyScore -= penaltyAbs;
        if (useTrackScore && IsDisplayActive)
        {
            displaySectionScore -= penaltyAbs;
        }

        RefreshUI();
    }

    public bool TryApplyBoundaryPenalty(float penalty)
    {
        if (boundaryPenaltyApplied) return false;

        ApplyBoundaryPenalty(Mathf.Abs(penalty));
        boundaryPenaltyApplied = true;
        return true;
    }

    public void UpdateLastTotalScore()
    {
        lastTrackScore = TrackSectionScore;
        lastForceScore = ForceSectionScore;
        lastEmgScore = EmgSectionScore;
        lastTotalScore = SectionScore;
        lastDisplayTrackScore = DisplayTrackSectionScore;
        lastDisplayForceScore = DisplayForceSectionScore;
        lastDisplayEmgScore = DisplayEmgSectionScore;
        lastDisplayScore = DisplaySectionScore;
        showLiveScore = false;
        ShowTrialScore(false);
    }

    public void HideTrialScore()
    {
        if (TrialScoreText != null && !alwaysShowTrialScore)
            TrialScoreText.gameObject.SetActive(false);
    }

    void ShowTrialScore(bool live)
    {
        if (TrialScoreText == null) return;

        TrialScoreText.text = FormatTrialScore(live, showTotalOnly);
        TrialScoreText.gameObject.SetActive(true);
    }

    void UpdateTrialScoreVisible()
    {
        if (alwaysShowTrialScore)
            ShowTrialScore(SectionScoringActive);
        else
            HideTrialScore();
    }

    void ClearTrialScore()
    {
        lastTrackScore = 0f;
        lastForceScore = 0f;
        lastEmgScore = 0f;
        lastTotalScore = 0f;
        lastDisplayTrackScore = 0f;
        lastDisplayForceScore = 0f;
        lastDisplayEmgScore = 0f;
        lastDisplayScore = 0f;
        showLiveScore = false;
    }

    void ResetCompositeMetrics()
    {
        HandleFx = 0f;
        TrackCost = 0f;
        WTrack = 0f;
        WForce = 0f;
        WEmg = 0f;
        TotalCost = 0f;
    }

    float SelectedScore(float track, float force, float stiffness)
    {
        float total = 0f;
        float count = 0f;
        if (useTrackScore) { total += track; count++; }
        if (useStiffnessScore) { total += stiffness; count++; }
        if (useForceScore) { total += force; count++; }
        return count > 0f ? total / count : 0f;
    }

    string FormatTrialScore(bool live, bool totalOnly = false)
    {
        float track = live ? DisplayTrackSectionScore : lastDisplayTrackScore;
        float force = live ? DisplayForceSectionScore : lastDisplayForceScore;
        float emg = live ? DisplayEmgSectionScore : lastDisplayEmgScore;
        float total = live ? DisplaySectionScore : lastDisplayScore;

        if (totalOnly)
            return $"Total: {total:0.0}";

        string text = "";
        if (useTrackScore) text += $"Track: {track:0.0}\n";
        if (useStiffnessScore) text += $"Stiffness: {emg:0.0}\n";
        if (useForceScore) text += $"Force: {force:0.0}\n";
        return text + $"Total: {total:0.0}";
    }

    string FormatDebugTrialScore(bool live)
    {
        float track = live ? TrackSectionScore : lastTrackScore;
        float force = live ? ForceSectionScore : lastForceScore;
        float emg = live ? EmgSectionScore : lastEmgScore;
        float total = live ? SectionScore : lastTotalScore;

        return
            $"Track: {track:0.0}\n" +
            $"Force: {force:0.0}\n" +
            $"EMG: {emg:0.0}\n" +
            $"Total: {total:0.0}";
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
        if (emgScoreText != null)
            emgScoreText.text = string.IsNullOrEmpty(EmgDetails)
                ? $"EMG: {EmgScore:0.00} | t: {EmgTimestamp:0.000}s"
                : EmgDetails;

        if (alwaysShowTrialScore)
            ShowTrialScore(SectionScoringActive);

        if (DebugTrialScoreText != null)
            DebugTrialScoreText.text = FormatDebugTrialScore(showLiveScore);
    }
}
