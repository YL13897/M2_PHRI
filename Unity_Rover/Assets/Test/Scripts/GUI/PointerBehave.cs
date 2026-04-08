using UnityEngine;
using UnityEngine.UI;

public class PointerBehave : MonoBehaviour
{
    [SerializeField] Transform pointerTransform;
    [SerializeField] Transform leaderTransform;
    [SerializeField] SpriteRenderer fillSpriteRenderer;
    [SerializeField] float alignedHalfWidth = 1f;
    [SerializeField] Color alignedColor = Color.green;
    [SerializeField] Color misalignedColor = Color.red;

    void Awake()
    {
        pointerTransform ??= transform;
        fillSpriteRenderer ??= GetComponent<SpriteRenderer>();
    }

    void Update()
    {
        PointerBehaveUpdate();
    }

    void PointerBehaveUpdate()
    {
        // if (pointerTransform == null || leaderTransform == null) return;
        float pointerX = pointerTransform.position.x;
        float leaderX = leaderTransform.position.x;
        bool isAligned = Mathf.Abs(pointerX - leaderX) <= alignedHalfWidth;
        Color targetColor = isAligned ? alignedColor : misalignedColor;
        fillSpriteRenderer.color = targetColor;
    }
}
