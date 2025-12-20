using UnityEngine;
using UnityEngine.UI;

public class DetectedObjects : MonoBehaviour
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

    public void displayDetectionPosition(DetectedObject[] detectedObjects)
    {
        foreach (DetectedObject obj in detectedObjects)
        {
            Transform parent = targetRawImage.transform;

            // Instantiate a new detected point from the prefab.
            GameObject detectedPointInstance = Instantiate(detectedPointPrefab, parent);

            // Set the label's text to the detected object's name.
            TMPro.TMP_Text labelText = detectedPointInstance.GetComponentInChildren<TMPro.TMP_Text>();
            if (labelText != null)
            {
                labelText.text = $"<mark=#FFFF00AA>{obj.label}</mark>";
            }
            else
            {
                Debug.LogError("Label prefab does not contain a TMP_Text component.");
            }

            // The points from the Gemini API are normalized to a range of 0-1000.
            // This needs to be converted to normalized coordinates from 0-1 first.
            float normalizedX = obj.point.x / 1000f;
            float normalizedY = obj.point.y / 1000f;

            RectTransform rawRect = targetRawImage.GetComponent<RectTransform>();
            float localX = rawRect.rect.x + (normalizedX * rawRect.rect.width);
            // The Y coordinate is inverted because the API's origin is the top-left corner,
            // while the UI's origin is the bottom-left.
            float localY = rawRect.rect.y + (rawRect.rect.height - normalizedY * rawRect.rect.height);
            // Use the base transform property which is always present on a GameObject.
            // This avoids the MissingComponentException if the prefab is not a UI element.
            detectedPointInstance.transform.localPosition = new Vector3(localX, localY, 0);
        }
    }



}
