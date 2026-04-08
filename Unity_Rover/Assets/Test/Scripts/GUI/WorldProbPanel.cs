using UnityEngine;
using TMPro;

public class WorldProbPanel : MonoBehaviour
{
    public static WorldProbPanel Instance { get; private set; }

    [SerializeField] TextMeshProUGUI probabilityText;
    [SerializeField] CanvasGroup canvasGroup;

    void Awake()
    {
        Instance = this;

        if (canvasGroup == null)
            {canvasGroup = GetComponent<CanvasGroup>();}

        Hide(); // hide visually, but keep object active
    }

    public void Show(float probability)
    {
        probabilityText.text = $"{(probability * 100f):0}%";
        canvasGroup.alpha = 1f;
        canvasGroup.interactable = false;
        canvasGroup.blocksRaycasts = false;
    }

    public void Hide()
    {
        canvasGroup.alpha = 0f;
        canvasGroup.interactable = false;
        canvasGroup.blocksRaycasts = false;
    }
}
