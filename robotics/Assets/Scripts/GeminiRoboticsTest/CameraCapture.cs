using UnityEngine;
using System;
using UnityEngine.UI;

/// <summary>
/// Captures an image from a specified Unity Camera, converts it to a Base64-encoded string,
/// and optionally displays it on a RawImage UI component.
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

    [Tooltip("A UI RawImage component to display the captured image. This is optional.")]
    [SerializeField] RawImage outputRawImage;

    private string _lastCaptureBase64;

    /// <summary>
    /// Gets the last captured image encoded as a Base64 string.
    /// </summary>
    public string LastCaptureBase64 => _lastCaptureBase64;

    /// <summary>
    /// Initializes the component by finding the main camera if no capture camera is assigned.
    /// </summary>
    void Start()
    {
        if (captureCamera == null)
        {
            captureCamera = Camera.main;
            if (captureCamera == null)
            {
                Debug.LogError("No camera found. Please assign a camera to the CameraCapture script or ensure you have a main camera in the scene.");
                enabled = false;
                return;
            }
        }
    }

    /// <summary>
    /// Captures a frame from the assigned camera and returns it as a Base64 encoded string.
    /// </summary>
    /// <returns>A Base64 encoded string of the captured JPG image.</returns>
    public string CaptureAsBase64()
    {
        // Create a temporary RenderTexture to hold the camera's view.
        RenderTexture renderTexture = RenderTexture.GetTemporary(imageWidth, imageHeight, 24);

        // Set the camera's target to our temporary RenderTexture.
        var previousTargetTexture = captureCamera.targetTexture;
        captureCamera.targetTexture = renderTexture;

        // Manually render the camera's view.
        captureCamera.Render();

        // Set the temporary RenderTexture as the active one to read from.
        RenderTexture.active = renderTexture;

        // Create a new Texture2D to receive the pixel data.
        Texture2D capturedImage = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);

        // Read the pixels from the active RenderTexture into the Texture2D.
        capturedImage.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        capturedImage.Apply();

        // Clean up: reset the camera's target and release the temporary RenderTexture.
        captureCamera.targetTexture = previousTargetTexture;
        RenderTexture.active = null;
        RenderTexture.ReleaseTemporary(renderTexture);

        // If an output RawImage is assigned, display the captured image.
        if (outputRawImage != null)
        {
            // To prevent memory leaks, destroy the old texture before assigning a new one.
            if (outputRawImage.texture != null)
#if UNITY_EDITOR
                // Use DestroyImmediate in the editor to avoid errors about destroying assets.
                DestroyImmediate(outputRawImage.texture, true);
#else
                Destroy(outputRawImage.texture);
#endif
            outputRawImage.texture = capturedImage;
        }

        // Encode the image to JPG format and then convert to a Base64 string.
        byte[] imageBytes = capturedImage.EncodeToJPG();
        _lastCaptureBase64 = Convert.ToBase64String(imageBytes);

        // If the image is not being displayed, we should destroy the texture to free up memory.
        if (outputRawImage == null)
#if UNITY_EDITOR
            // Use DestroyImmediate in the editor to avoid errors about destroying assets.
            DestroyImmediate(capturedImage, true);
#else
            Destroy(capturedImage);
#endif

        return _lastCaptureBase64;
    }
}