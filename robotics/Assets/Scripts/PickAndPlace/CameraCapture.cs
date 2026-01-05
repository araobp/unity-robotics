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
    [SerializeField] private Camera captureCamera;
    private int imageWidth;
    private int imageHeight;

    [Tooltip("A UI RawImage component to display the captured image. This is optional.")]
    [SerializeField] RawImage outputRawImage;

    private string _lastCaptureBase64;

    /// <summary>
    /// Gets the last captured image encoded as a Base64 string.
    /// </summary>
    public string LastCaptureBase64 => _lastCaptureBase64;

    float fovHorizontal; // Horizontal field of view (degrees)
    float fovVertical;   // Vertical field of view (degrees)
    private float H;           // Camera height (Y)
    private float L_cam;       // Camera installation position (Z)
    private float theta;        // Angle of depression (angle measured downward from the horizontal)

    [Header("Work Area")]
    [SerializeField] GameObject workArea;

    [Header("Debugging")]
    [SerializeField] float debug_u = 320; // OpenCV X
    [SerializeField] float debug_v = 240; // OpenCV Y

    /// <summary>
    /// Initializes the component by finding the main camera if no capture camera is assigned.
    /// </summary>
    void Start()
    {
        if (captureCamera == null)
        {
            Debug.LogError("No camera found. Please assign a camera to the CameraCapture script or ensure you have a main camera in the scene.");
            enabled = false;
            return;
        }
        else
        {
            imageWidth = captureCamera.pixelWidth;
            imageHeight = captureCamera.pixelHeight;

            fovVertical = captureCamera.fieldOfView;
            fovHorizontal = 2f * Mathf.Atan(Mathf.Tan(fovVertical * Mathf.Deg2Rad / 2f) * captureCamera.aspect) * Mathf.Rad2Deg;

            // 1. Get Camera position relative to Work Area
            Vector3 localPos = workArea.transform.InverseTransformPoint(captureCamera.transform.position);
            H = localPos.y;     // Height above the work area plane
            L_cam = localPos.z; // Depth offset from work area origin

            // 2. Fix Theta Calculation
            // We want the angle between the camera's forward vector and the work area's horizontal plane.
            // Vector3.Angle gives the angle between 0 and 180.
            float angleToGround = Vector3.Angle(captureCamera.transform.forward, -workArea.transform.up); 
    // If angleToGround is 0, camera is looking straight down. 
    // theta (angle from horizontal) = 90 - angleToGround.
    theta = 90f - angleToGround;

            // Debugging
            Debug.Log($"[Projection] H: {H}, L_cam: {L_cam}, Theta: {theta}");
            Debug.Log("[Projection] Image Dimensions: " + imageWidth + "x" + imageHeight);
            Debug.Log("[Projection] FOV H: " + fovHorizontal.ToString("F4") + ", V: " + fovVertical.ToString("F4"));
            Debug.Log("[Projection] Camera H: " + H.ToString("F4") + ", L_cam: " + L_cam.ToString("F4") + ", Theta: " + theta.ToString("F4"));
            Vector3 worldPos = ProjectToWorld(debug_u, debug_v);
            Debug.Log($"[Projection] Debug Projection - u: {debug_u}, v: {debug_v} => World Position: {worldPos}");
        }
    }

    /// <summary>
    /// Captures a frame from the assigned camera and returns it as a Base64 encoded string.
    /// </summary>
    /// <returns>A Base64 encoded string of the captured JPG image.</returns>
    public string CaptureAsBase64()
    {
        // Render the camera's view to a temporary RenderTexture.
        RenderTexture renderTexture = RenderTexture.GetTemporary(imageWidth, imageHeight, 24);

        var previousTargetTexture = captureCamera.targetTexture;
        captureCamera.targetTexture = renderTexture;

        captureCamera.Render();

        RenderTexture.active = renderTexture;

        // Read the pixels from the RenderTexture into a new Texture2D.
        Texture2D capturedImage = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);

        capturedImage.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        capturedImage.Apply();

        // Restore the camera's target texture and release the temporary RenderTexture.
        captureCamera.targetTexture = previousTargetTexture;
        RenderTexture.active = null;
        RenderTexture.ReleaseTemporary(renderTexture);

        // If an output RawImage is assigned, display the captured image.
        if (outputRawImage != null)
        {
            // Destroy the previous texture to prevent memory leaks before assigning the new one.
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

    /// <summary>
    /// Converts OpenCV pixel coordinates to Unity world coordinates.
    /// </summary>
    /// <param name="u">OpenCV X (0 to Width)</param>
    /// <param name="v">OpenCV Y (0 to Height)</param>
    /// <returns>Unity World Position (X, 0, Z)</returns>
    public Vector3 ProjectToWorld(float u, float v)
    {
        float u_prime = u - imageWidth * 0.5f;
        float v_prime = imageHeight * 0.5f - v;

        float alpha = Mathf.Atan((v_prime / (imageHeight * 0.5f)) * Mathf.Tan(fovVertical * 0.5f * Mathf.Deg2Rad));
        float beta = Mathf.Atan((u_prime / (imageWidth * 0.5f)) * Mathf.Tan(fovHorizontal * 0.5f * Mathf.Deg2Rad));

        float gamma = (theta * Mathf.Deg2Rad) + alpha;

        if (gamma <= 0.001f) return new Vector3(0, -999, 0);

        float z_dist = H / Mathf.Tan(gamma);
        Debug.Log($"[Projection] z_dist: {H / Mathf.Tan((theta) * Mathf.Deg2Rad)}");

        float localZ = L_cam - z_dist;
        Debug.Log($"[Projection] localZ: {localZ}");

        float horizontalDistToPoint = H / Mathf.Sin(gamma);
        float localX = horizontalDistToPoint * Mathf.Tan(beta);
        Debug.Log($"[Projection] localX: {localX}");
        return new Vector3(localX, 0, localZ);
    }
}
