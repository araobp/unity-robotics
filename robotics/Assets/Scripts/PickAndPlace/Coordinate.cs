using TMPro;
using UnityEngine;

public class Coordinate : MonoBehaviour
{
    TMP_Text x;
    TMP_Text y;
    TMP_Text z;

    // Initialize references to the UI text components.
    void Start()
    {
        x = transform.Find("X").GetComponent<TMP_Text>();
        y = transform.Find("Y").GetComponent<TMP_Text>();
        z = transform.Find("Z").GetComponent<TMP_Text>();
    }

    public void UpdatePositionText(Vector3 fingerPos)
    {
        x.text = $"x: {fingerPos.x.ToString("F3")}";
        y.text = $"y: {fingerPos.y.ToString("F3")}";
        z.text = $"z: {fingerPos.z.ToString("F3")}";
    }

}
