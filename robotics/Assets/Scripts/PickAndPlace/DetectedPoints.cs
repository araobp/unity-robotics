using UnityEngine;
using UnityEngine.UI;

public class DetectedPoints : MonoBehaviour
{
    [Tooltip("The prefab for the label that will be instantiated to mark detected objects.")]
    [SerializeField]
    private GameObject detectedPointPrefab;

    [Tooltip("Optional: The RawImage to place detected points on. If not set, points are drawn on the Canvas.")]
    [SerializeField]
    private RawImage targetRawImage;

    void Start()
    {
        // Ensure the label prefab is assigned.
        if (detectedPointPrefab == null)
        {
            Debug.LogError("Label prefab is not assigned.");
        }
    }

    /// <summary>
    /// Clears all displayed detected points from the target RawImage.
    /// </summary>
    public void clear()
    {
        // Destroy all existing detected point instances.
        foreach (Transform child in targetRawImage.transform)
        {
            Destroy(child.gameObject);
        }
    }

    /// <summary>
    /// Displays a detected object on the target RawImage by instantiating a label prefab at its position.
    /// </summary>
    public void displayDetectionPosition(DetectedObject detectedObject)
    {
        // Determine the parent transform for the detected points.
        Transform parent = targetRawImage.transform;

        // Instantiate the label prefab and set its parent to the target RawImage.
        GameObject detectedPointInstance = Instantiate(detectedPointPrefab, parent);

        // Set the label's text to the detected object's name.
        TMPro.TMP_Text labelText = detectedPointInstance.GetComponentInChildren<TMPro.TMP_Text>();

        // Get a color based on the label hash for consistent coloring.
        var random = new System.Random(detectedObject.label.GetHashCode());
        float hue = (float)random.NextDouble();
        // Avoid dark (low value) and light/grey (low saturation) colors.
        float saturation = 0.6f + (float)random.NextDouble() * 0.4f;
        float value = 0.6f + (float)random.NextDouble() * 0.4f;
        Color color = Color.HSVToRGB(hue, saturation, value);

        if (labelText != null)
        {
            // Set the label text.
            labelText.text = detectedObject.label;

            // Set the label's color.
            labelText.color = color;
        }
        else
        {
            Debug.LogError("Label prefab does not contain a TMP_Text component.");
        }

        // Set the color of the detected point's RawImage.
        detectedPointInstance.GetComponentInChildren<RawImage>().color = color;

        // Convert the API's normalized coordinates (0-1000) to Unity's normalized coordinates (0-1).
        float normalizedX = detectedObject.point.x / 1000f;
        float normalizedY = detectedObject.point.y / 1000f;

        // Calculate the local position within the target RawImage.
        RectTransform rawRect = targetRawImage.GetComponent<RectTransform>();
        float localX = rawRect.rect.x + (normalizedX * rawRect.rect.width);
        // Invert the Y coordinate because the API uses a top-left origin, while Unity UI uses bottom-left.
        float localY = rawRect.rect.y + (rawRect.rect.height - normalizedY * rawRect.rect.height);
        // Set the local position of the label.
        detectedPointInstance.transform.localPosition = new Vector3(localX, localY, 0);
    }
}
