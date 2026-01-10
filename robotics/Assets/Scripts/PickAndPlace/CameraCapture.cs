using UnityEngine;
using System;
using UnityEngine.UI;

/// <summary>
/// Handles camera operations, including capturing images and projecting 2D image coordinates
/// into 3D world space. It can capture from a specified Unity Camera, convert the image
/// to a Base64-encoded string, and optionally display it on a RawImage UI component.
/// It also provides a method to project pixel coordinates from the captured image onto
/// a defined work area plane in the 3D scene.
/// </summary>
public class CameraCapture : MonoBehaviour
{
    [Tooltip("The camera to capture the image from. If not set, it will default to the main camera.")]
    [SerializeField] private Camera captureCamera;
    public int ImageWidth => _imageWidth;
    public int ImageHeight => _imageHeight;
    private int _imageWidth;
    private int _imageHeight;

    [Tooltip("A UI RawImage component to display the captured image. This is optional.")]
    [SerializeField] RawImage outputRawImage;

    private string _lastCaptureBase64;

    /// <summary>
    /// Gets the last captured image encoded as a Base64 string.
    /// </summary>
    public string LastCaptureBase64 => _lastCaptureBase64;

    /// <summary>
    /// Horizontal field of view in degrees.
    /// </summary>
    float fovHorizontal;
    /// <summary>
    /// Vertical field of view in degrees.
    /// </summary>
    float fovVertical;
    /// <summary>
    /// Camera height (Y-axis) relative to the work area.
    /// </summary>
    private float H;
    /// <summary>
    /// Camera installation position (Z-axis) relative to the work area.
    /// </summary>
    private float L_cam;
    /// <summary>
    /// Angle of depression (angle measured downward from the horizontal).
    /// </summary>
    private float theta;

    [Header("Work Area")]
    [SerializeField] GameObject workArea;

    [Header("Debugging")]
    [SerializeField] float debug_u = 320; // OpenCV X
    [SerializeField] float debug_v = 240; // OpenCV Y

    /// <summary>
    /// Initializes camera parameters for projection calculations. This includes calculating
    /// the horizontal field of view and determining the camera's position and orientation
    /// relative to the work area.
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
            _imageWidth = captureCamera.pixelWidth;
            _imageHeight = captureCamera.pixelHeight;

            fovVertical = captureCamera.fieldOfView;
            fovHorizontal = 2f * Mathf.Atan(Mathf.Tan(fovVertical * Mathf.Deg2Rad / 2f) * captureCamera.aspect) * Mathf.Rad2Deg;

            // Get the camera's position relative to the work area's local coordinate system.
            Vector3 localPos = workArea.transform.InverseTransformPoint(captureCamera.transform.position);
            H = localPos.y;     // Height (Y-axis) of the camera above the work area plane.
            L_cam = localPos.z; // Depth (Z-axis) offset of the camera from the work area origin.

            // Calculate the camera's depression angle (theta).
            // We want the angle between the camera's forward vector and the work area's horizontal plane.
            // Vector3.Angle gives the angle between 0 and 180.
            float angleToGround = Vector3.Angle(captureCamera.transform.forward, -workArea.transform.up);

            // If angleToGround is 0, camera is looking straight down. 
            // theta (angle from horizontal) = 90 - angleToGround.
            theta = 90f - angleToGround;

            // Debugging
            Debug.Log($"[Projection] H: {H}, L_cam: {L_cam}, Theta: {theta}");
            Debug.Log("[Projection] Image Dimensions: " + ImageWidth + "x" + ImageHeight);
            Debug.Log("[Projection] FOV H: " + fovHorizontal.ToString("F4") + ", V: " + fovVertical.ToString("F4"));
            Debug.Log("[Projection] Camera H: " + H.ToString("F4") + ", L_cam: " + L_cam.ToString("F4") + ", Theta: " + theta.ToString("F4"));
            Vector3 worldPos = ProjectToWorkAreaLocal(debug_u, debug_v);
            Debug.Log($"[Projection] Debug Projection - u: {debug_u}, v: {debug_v} => World Position: {worldPos}");
        }
    }

    /// <summary>
    /// Captures a single frame from the assigned camera, encodes it as a JPG, and returns it
    /// as a Base64 encoded string. If an output RawImage is assigned, the captured image is
    /// also displayed on it.
    /// </summary>
    /// <returns>A Base64 encoded string representing the captured JPG image.</returns>
    public string CaptureAsBase64()
    {
        // Create a temporary RenderTexture to hold the camera's view.
        RenderTexture renderTexture = RenderTexture.GetTemporary(ImageWidth, ImageHeight, 24);

        var previousTargetTexture = captureCamera.targetTexture;
        captureCamera.targetTexture = renderTexture;

        captureCamera.Render();

        RenderTexture.active = renderTexture;

        // Create a new Texture2D and read the pixels from the active RenderTexture.
        Texture2D capturedImage = new Texture2D(ImageWidth, ImageHeight, TextureFormat.RGB24, false);

        capturedImage.ReadPixels(new Rect(0, 0, ImageWidth, ImageHeight), 0, 0);
        capturedImage.Apply();

        // Restore the camera's original target texture and release the temporary one.
        captureCamera.targetTexture = previousTargetTexture;
        RenderTexture.active = null;
        RenderTexture.ReleaseTemporary(renderTexture);

        // If an output RawImage is provided, display the captured image on it.
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

        // Encode the captured image to JPG format and then convert it to a Base64 string.
        byte[] imageBytes = capturedImage.EncodeToJPG();
        _lastCaptureBase64 = Convert.ToBase64String(imageBytes);

        // If the captured image is not being displayed on a UI element, destroy it to free up memory.
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
    /// Projects a 2D pixel coordinate from the camera's view onto the 3D work area plane.
    /// This uses the camera's intrinsic (FOV) and extrinsic (position, rotation) parameters
    /// calculated during Start().
    /// </summary>
    /// <param name="u">The horizontal pixel coordinate (from 0 to ImageWidth). Corresponds to OpenCV's x-coordinate.</param>
    /// <param name="v">The vertical pixel coordinate (from 0 to ImageHeight). Corresponds to OpenCV's y-coordinate.</param>
    /// <returns>The projected 3D position in the work area's local coordinate system, with Y set to 0.</returns>
    public Vector3 ProjectToWorkAreaLocal(float u, float v)
    {
        // Convert pixel coordinates (top-left origin) to image-center-relative coordinates.
        float u_prime = u - ImageWidth * 0.5f;
        float v_prime = ImageHeight * 0.5f - v;

        // Calculate the vertical (alpha) and horizontal (beta) angles of the pixel relative to the camera's forward axis.
        float alpha = Mathf.Atan((v_prime / (ImageHeight * 0.5f)) * Mathf.Tan(fovVertical * 0.5f * Mathf.Deg2Rad));
        float beta = Mathf.Atan((u_prime / (ImageWidth * 0.5f)) * Mathf.Tan(fovHorizontal * 0.5f * Mathf.Deg2Rad));

        // Gamma is the final downward angle from the horizontal plane to the point on the ground.
        float gamma = (theta * Mathf.Deg2Rad) + alpha;

        // If the point is at or above the horizon, it cannot be projected onto the work plane.
        if (gamma <= 0.001f) return new Vector3(0, -999, 0);

        // Use trigonometry to find the distance from the camera's ground-projection to the target point.
        float z_dist = H / Mathf.Tan(gamma);
        Debug.Log($"[Projection] z_dist: {H / Mathf.Tan((theta) * Mathf.Deg2Rad)}");

        // Calculate the final local X and Z coordinates on the work area plane.
        float localZ = L_cam - z_dist;
        Debug.Log($"[Projection] localZ: {localZ}");

        float horizontalDistToPoint = H / Mathf.Sin(gamma);
        float localX = horizontalDistToPoint * Mathf.Tan(beta);
        Debug.Log($"[Projection] localX: {localX}");
        return new Vector3(localX, 0, localZ);
    }
}
