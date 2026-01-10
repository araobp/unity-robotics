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
    /// A list to store the move operations generated from the most recent instruction.
    /// </summary>
    private List<Move> _moves = new List<Move>();


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
        _moves.Clear();

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

        // Generate a consistent color based on the hash of the object's label.
        var random = new System.Random(detectedObject.label.GetHashCode());
        float hue = (float)random.NextDouble();
        // Adjust saturation and value to avoid colors that are too dark or too washed out.
        float saturation = 0.6f + (float)random.NextDouble() * 0.4f;
        float value = 0.6f + (float)random.NextDouble() * 0.4f;
        Color color = Color.HSVToRGB(hue, saturation, value);

        DisplayPoint(detectedObject.point, detectedObject.label, color);
    }

    /// <summary>
    /// Displays visual markers for a move operation on the target RawImage.
    /// This involves instantiating labels for the start and end points of the move.
    /// </summary>
    /// <param name="move">The move information, including from and to coordinates and a label.</param>
    public void displayMove(Move move)
    {
        _moves.Add(move);

        // Generate a consistent color based on the hash of the object's label.
        var random = new System.Random(move.label.GetHashCode());
        float hue = (float)random.NextDouble();
        float saturation = 0.6f + (float)random.NextDouble() * 0.4f;
        float value = 0.6f + (float)random.NextDouble() * 0.4f;
        Color color = Color.HSVToRGB(hue, saturation, value);

        // Display the 'from' point
        DisplayPoint(move.from, $"{move.label} (from)", color);

        // Display the 'to' point
        DisplayPoint(move.to, $"{move.label} (to)", color);
    }

    /// <summary>
    /// Creates and displays a single point marker on the UI, converting normalized coordinates
    /// to the local space of the target RawImage.
    /// </summary>
    /// <param name="point">The 0-1000 normalized coordinates of the point.</param>
    /// <param name="label">The text to display on the marker.</param>
    /// <param name="color">The color of the marker.</param>
    private void DisplayPoint(Vector2Int point, string label, Color color)
    {
        Transform parent = targetRawImage.transform;
        GameObject detectedPointInstance = Instantiate(detectedPointPrefab, parent);

        TMPro.TMP_Text labelText = detectedPointInstance.GetComponentInChildren<TMPro.TMP_Text>();
        if (labelText != null)
        {
            labelText.text = label;
            labelText.color = color;
        }
        else
        {
            Debug.LogError("Label prefab does not contain a TMP_Text component.");
        }

        detectedPointInstance.GetComponentInChildren<RawImage>().color = color;

        // The API returns coordinates in a 0-1000 range. Convert this to a 0-1 normalized scale.
        float normalizedX = point.x / 1000f;
        float normalizedY = point.y / 1000f;

        // Calculate the local position within the target RawImage's coordinate system.
        RectTransform rawRect = targetRawImage.GetComponent<RectTransform>();
        float localX = rawRect.rect.x + (normalizedX * rawRect.rect.width);
        // Invert the Y coordinate because the API uses a top-left origin, while Unity's UI uses a bottom-left origin.
        float localY = rawRect.rect.y + (rawRect.rect.height - normalizedY * rawRect.rect.height);
        
        // Set the local position of the instantiated marker.
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
