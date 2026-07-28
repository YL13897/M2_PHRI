using UnityEngine;

public class PointerBehave : MonoBehaviour
{
    [SerializeField] Transform pointerTransform;
    [SerializeField] Transform leaderTransform;
    [SerializeField] SpriteRenderer fillSpriteRenderer;
    [SerializeField] float whiteError = 1f;
    [SerializeField] float redError = 2f;
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
        if (pointerTransform == null || leaderTransform == null || fillSpriteRenderer == null) return;

        float deviation = Mathf.Abs(pointerTransform.position.x - leaderTransform.position.x);
        Color targetColor = deviation <= whiteError
            ? Color.Lerp(alignedColor, Color.white, Mathf.InverseLerp(0f, whiteError, deviation))
            : Color.Lerp(Color.white, misalignedColor, Mathf.InverseLerp(whiteError, redError, deviation));

        fillSpriteRenderer.color = targetColor;
    }
}
