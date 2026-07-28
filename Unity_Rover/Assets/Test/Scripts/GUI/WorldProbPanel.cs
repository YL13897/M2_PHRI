using UnityEngine;
using TMPro;
using UnityEngine.UI;

public class WorldProbPanel : MonoBehaviour
{
    [SerializeField] TextMeshProUGUI probabilityText;
    [SerializeField] CanvasGroup canvasGroup;
    [SerializeField] Image leftArrow;
    [SerializeField] Image rightArrow;

    void Awake()
    {
        if (canvasGroup == null)
            canvasGroup = GetComponent<CanvasGroup>();

        Hide(); // hide visually, but keep object active
    }

    public void Show(float probability, int direction)
    {
        probabilityText.text = $"{Mathf.RoundToInt(probability * 100f)}%";
        SetArrowDirection(direction);
        canvasGroup.alpha = 1f;
        canvasGroup.interactable = false;
        canvasGroup.blocksRaycasts = false;
    }

    public void SetArrowDirection(int direction)
    {
        if (leftArrow != null)
            leftArrow.gameObject.SetActive(direction < 0);

        if (rightArrow != null)
            rightArrow.gameObject.SetActive(direction > 0);
    }

    public void Hide()
    {
        SetArrowDirection(0);
        canvasGroup.alpha = 0f;
        canvasGroup.interactable = false;
        canvasGroup.blocksRaycasts = false;
    }
}
