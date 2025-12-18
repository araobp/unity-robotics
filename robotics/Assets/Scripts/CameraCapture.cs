using UnityEngine;
using System;

/// <summary>
/// This component handles capturing an image from a specified Unity Camera
/// and converting it into a Base64-encoded string.
/// </summary>
public class CameraCapture : MonoBehaviour
{
    [Tooltip("The camera to capture the image from. If not set, it will default to the main camera.")]
    [SerializeField]
    private Camera captureCamera;

    [Tooltip("The desired width of the captured image.")]
    [SerializeField]
    private int imageWidth = 1280;

    [Tooltip("The desired height of the captured image.")]
    [SerializeField]
    private int imageHeight = 720;

    private string _lastCaptureBase64;

    /// <summary>
    /// The last captured image, encoded as a Base64 string.
    /// </summary>
    public string LastCaptureBase64 => _lastCaptureBase64;

    void Start()
    {
        if (captureCamera == null)
        {
            captureCamera = Camera.main;
            if (captureCamera == null)
            {
                Debug.LogError("No camera found. Please assign a camera to the CameraCapture script or ensure you have a main camera in the scene.");
                enabled = false;
            }
        }
    }

    /// <summary>
    /// Captures an image from the assigned camera and returns it as a Base64 encoded string.
    /// </summary>
    /// <returns>A Base64 encoded string of the captured JPG image.</returns>
    public string CaptureAsBase64()
    {
        // Create a RenderTexture with the specified dimensions.
        RenderTexture renderTexture = RenderTexture.GetTemporary(imageWidth, imageHeight, 24);
        
        // Temporarily assign the RenderTexture to the camera.
        var previousTargetTexture = captureCamera.targetTexture;
        captureCamera.targetTexture = renderTexture;

        // Render the camera's view to our RenderTexture.
        captureCamera.Render();

        // Set the active RenderTexture to the one we just rendered to.
        RenderTexture.active = renderTexture;

        // Create a new Texture2D to hold the captured image.
        Texture2D capturedImage = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);

        // Read the pixels from the active RenderTexture into the Texture2D.
        capturedImage.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        capturedImage.Apply();

        // Reset the camera's target texture and the active RenderTexture, and release the temporary one.
        captureCamera.targetTexture = previousTargetTexture;
        RenderTexture.active = null;
        RenderTexture.ReleaseTemporary(renderTexture);

        // Encode the Texture2D to a JPG byte array and then to a Base64 string.
        byte[] imageBytes = capturedImage.EncodeToJPG();
        _lastCaptureBase64 = Convert.ToBase64String(imageBytes);

        // Clean up the temporary Texture2D.
        Destroy(capturedImage);

        return _lastCaptureBase64;
    }
}