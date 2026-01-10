using TMPro;
using UnityEngine;

/// <summary>
/// Manages a UI element for displaying 3D coordinates (X, Y, Z) using TextMeshPro components.
/// </summary>
public class Coordinate : MonoBehaviour
{
    TMP_Text x;
    TMP_Text y;
    TMP_Text z;

    /// <summary>
    /// Initialize references to the UI text components.
    /// </summary>
    void Start()
    {
        x = transform.Find("X").GetComponent<TMP_Text>();
        y = transform.Find("Y").GetComponent<TMP_Text>();
        z = transform.Find("Z").GetComponent<TMP_Text>();
    }

    /// <summary>
    /// Updates the text fields to display the provided 3D vector coordinates, formatted to three decimal places.
    /// </summary>
    /// <param name="fingerPos">The 3D position vector to display.</param>
    public void UpdatePositionText(Vector3 fingerPos)
    {
        if (x == null || y == null || z == null)
            return;
        x.text = $"x: {fingerPos.x.ToString("F3")}";
        y.text = $"y: {fingerPos.y.ToString("F3")}";
        z.text = $"z: {fingerPos.z.ToString("F3")}";
    }

}
