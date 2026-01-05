using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;
using System.Linq;
using System;

/// <summary>
/// Manages the visualization of objects detected by the Gemini API on a UI RawImage.
/// This class handles the creation, placement, and clearing of labels for detected objects.
/// </summary>
public class DetectedPoints : MonoBehaviour
{
    [Tooltip("The prefab for the label that will be instantiated to mark detected objects.")]
    [SerializeField]
    private GameObject detectedPointPrefab;

    [Tooltip("Optional: The RawImage to place detected points on. If not set, points are drawn on the Canvas.")]
    [SerializeField]
    private RawImage targetRawImage;

    /// <summary>
    /// A list to store the objects detected in the most recent analysis.
    /// </summary>
    private List<DetectedObject> _detectedObjects = new List<DetectedObject>();

    /// <summary>
    /// Ensures that the necessary prefab is assigned before any operations are performed.
    /// </summary>
    void Start()
    {
        // Ensure the label prefab is assigned.
        if (detectedPointPrefab == null)
        {
            Debug.LogError("Label prefab is not assigned.");
        }
    }

    /// <summary>
    /// Clears all displayed detected points from the UI and empties the list of detected objects.
    /// </summary>
    public void clear()
    {
        _detectedObjects.Clear();
        // Destroy all existing detected point instances to clean the UI.
        foreach (Transform child in targetRawImage.transform)
        {
            Destroy(child.gameObject);
        }
    }

    /// <summary>
    /// Displays a visual marker for a detected object on the target RawImage.
    /// This involves instantiating a label prefab and positioning it based on the object's coordinates.
    /// </summary>
    /// <param name="detectedObject">The object information received from the detection API.</param>
    public void displayDetectionPosition(DetectedObject detectedObject)
    {
        _detectedObjects.Add(detectedObject);

        // Determine the parent transform for the detected points.
        Transform parent = targetRawImage.transform;

        // Instantiate the label prefab and set its parent to the target RawImage.
        GameObject detectedPointInstance = Instantiate(detectedPointPrefab, parent);

        // Set the label's text to the detected object's name.
        TMPro.TMP_Text labelText = detectedPointInstance.GetComponentInChildren<TMPro.TMP_Text>();

        // Generate a consistent color based on the hash of the object's label.
        var random = new System.Random(detectedObject.label.GetHashCode());
        float hue = (float)random.NextDouble();
        // Adjust saturation and value to avoid colors that are too dark or too washed out.
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

        // The API returns coordinates in a 0-1000 range. Convert this to a 0-1 normalized scale.
        float normalizedX = detectedObject.point.x / 1000f;
        float normalizedY = detectedObject.point.y / 1000f;

        // Calculate the local position within the target RawImage's coordinate system.
        RectTransform rawRect = targetRawImage.GetComponent<RectTransform>();
        float localX = rawRect.rect.x + (normalizedX * rawRect.rect.width);
        // Invert the Y coordinate because the API uses a top-left origin, while Unity's UI uses a bottom-left origin.
        float localY = rawRect.rect.y + (rawRect.rect.height - normalizedY * rawRect.rect.height);
        
        // Set the local position of the label.
        detectedPointInstance.transform.localPosition = new Vector3(localX, localY, 0);
    }

    /// <summary>
    /// Finds all detected objects where the label contains a specific word, ignoring case.
    /// </summary>
    /// <param name="labelName">The word to search for within the object labels.</param>
    /// <param name="detectedObjects">An enumerable of detected objects that match the label name.</param>
    /// <returns>True if any matching objects were found, false otherwise.</returns>
    public bool TryGetDetectedObjects(string labelName, out IEnumerable<DetectedObject> detectedObjects)
    {
        // Find all detected objects where at least one word in the label matches the labelName (case-insensitive).
        detectedObjects = _detectedObjects.Where(o =>
            o.label != null &&
            o.label.Split(' ').Contains(labelName, StringComparer.OrdinalIgnoreCase));

        return detectedObjects.Any();
    }
}
