using UnityEngine;
using UnityEngine.UI;

[RequireComponent(typeof(CanvasRenderer), typeof(RectTransform))]
public class EMGScopeUIRenderer : MaskableGraphic
{
    private static readonly Color BackgroundColor = new Color(0f, 1f, 1f, 0.08f);
    private static readonly Color BaselineColor = new Color(1f, 1f, 1f, 0.35f);

    private float lineThickness = 6f;
    private float[] samples;
    private int sampleCount;
    private float yScale = 1f;

    public void Configure(float width, float height, float baseline, float thickness)
    {
        lineThickness = Mathf.Max(1f, thickness);

        RectTransform rect = rectTransform;
        rect.anchorMin = new Vector2(0f, 1f);
        rect.anchorMax = new Vector2(0f, 1f);
        rect.pivot = new Vector2(0f, 1f);
        rect.sizeDelta = new Vector2(Mathf.Max(1f, width), Mathf.Max(1f, height));
        rect.anchoredPosition = new Vector2(0f, baseline);

        SetVerticesDirty();
    }

    public void Render(float[] sourceSamples, int count, float scale)
    {
        samples = sourceSamples;
        sampleCount = count;
        yScale = scale;
        SetVerticesDirty();
    }

    public void Clear()
    {
        sampleCount = 0;
        SetVerticesDirty();
    }


    // OnPopulateMesh(): Core rendering logic to convert sample data into a visual scope representation using Unity's UI system.
    protected override void OnPopulateMesh(VertexHelper vh)
    {
        vh.Clear();

        Rect rect = rectTransform.rect;
        float xMin = rect.xMin;
        float xMax = rect.xMax;
        float centerY = rect.center.y;
        float halfHeight = rect.height * 0.5f;
        float halfThickness = lineThickness * 0.5f;

        // Draw background and baseline.
        AddQuad(vh, new Vector2(xMin, rect.yMin), new Vector2(xMax, rect.yMax), BackgroundColor);
        AddSegment(vh, new Vector2(xMin, centerY), new Vector2(xMax, centerY), halfThickness, BaselineColor);

        if (samples == null || sampleCount <= 1)
            return;

        float xStep = sampleCount > 1 ? (xMax - xMin) / (sampleCount - 1) : 0f;

        for (int i = 0; i < sampleCount - 1; i++)
        {
            float x0 = xMin + i * xStep;
            float x1 = xMin + (i + 1) * xStep;
            float y0 = centerY + Mathf.Clamp(samples[i] * yScale, -halfHeight, halfHeight);
            float y1 = centerY + Mathf.Clamp(samples[i + 1] * yScale, -halfHeight, halfHeight);
            AddSegment(vh, new Vector2(x0, y0), new Vector2(x1, y1), halfThickness, color);
        }
    }

    
    // AddQuad() and AddSegment(): Helper methods to construct the visual elements of the scope, 
    // such as the background, baseline, and signal trace segments, by adding vertices and triangles to the VertexHelper.
    private void AddQuad(VertexHelper vh, Vector2 min, Vector2 max, Color quadColor)
    {
        int index = vh.currentVertCount;
        UIVertex vertex = UIVertex.simpleVert;
        vertex.color = quadColor;

        vertex.position = new Vector2(min.x, min.y);
        vh.AddVert(vertex);
        vertex.position = new Vector2(min.x, max.y);
        vh.AddVert(vertex);
        vertex.position = new Vector2(max.x, max.y);
        vh.AddVert(vertex);
        vertex.position = new Vector2(max.x, min.y);
        vh.AddVert(vertex);

        vh.AddTriangle(index, index + 1, index + 2);
        vh.AddTriangle(index, index + 2, index + 3);
    }

    private void AddSegment(VertexHelper vh, Vector2 start, Vector2 end, float halfThickness, Color segmentColor)
    {
        Vector2 dir = end - start;
        float length = dir.magnitude;
        if (length <= 0.0001f)
            return;

        dir /= length;
        Vector2 normal = new Vector2(-dir.y, dir.x) * halfThickness;

        int index = vh.currentVertCount;
        UIVertex vertex = UIVertex.simpleVert;
        vertex.color = segmentColor;

        vertex.position = start - normal;
        vh.AddVert(vertex);
        vertex.position = start + normal;
        vh.AddVert(vertex);
        vertex.position = end + normal;
        vh.AddVert(vertex);
        vertex.position = end - normal;
        vh.AddVert(vertex);

        vh.AddTriangle(index, index + 1, index + 2);
        vh.AddTriangle(index, index + 2, index + 3);
    }
}
