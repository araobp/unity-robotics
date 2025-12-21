using UnityEngine;
using System.Threading;
using System.Threading.Tasks;
using Vector3 = UnityEngine.Vector3;
using UnityEngine.UI;

/// <summary>
/// Implements inverse kinematics (IK) for a robot arm to pick and place objects.
/// It calculates the required joint angles to reach a target and animates the movement.
/// </summary>
public class PickAndPlace : MonoBehaviour
{
    // --- Fields ---

    [Header("IK Test Mode")]
    [SerializeField] bool ikTestMode = false;

    // The target GameObject that the robot arm's end effector will attempt to reach.
    [Header("IK Target")]
    [SerializeField] GameObject work;

    // Inverse kinematics behavior settings.
    [Header("IK Settings")]
    [Tooltip("The duration in seconds for the IK movement to complete.")]
    [SerializeField] private float ikMoveDuration = 2.0f;

    [Header("Robot Base")]
    [SerializeField] GameObject robotBase;

    // GameObjects representing the robot's joints.
    [Header("Robot Joints")]
    [SerializeField] GameObject swingAxis;
    [SerializeField] GameObject boomAxis;
    [SerializeField] GameObject armAxis;
    [SerializeField] GameObject handAxis;

    [Header("End Effector")]
    [SerializeField] GameObject finger;

    [Header("Coordinate Display")]
    [SerializeField] Coordinate coordinateFinger;
    [SerializeField] Coordinate coordinatePick;
    [SerializeField] Coordinate coordinatePlace;

    [Header("Operation buttons")]
    [SerializeField] Button buttonDetect;
    [SerializeField] Button buttonPick;
    [SerializeField] Button buttonPlace;
    [SerializeField] Button buttonReset;

    [Header("Components")]
    [SerializeField] private CameraCapture cameraCapture;
    [SerializeField] private DetectedPoints detectedPoints;
    private GeminiRoboticsApi geminiRoboticsApi;

    // Used to cancel the in-progress asynchronous movement task.
    private CancellationTokenSource _ikMoveCts;

    // Stores the initial rotation of each joint, allowing for relative
    // calculations. These are initialized in the Start() method.
    Quaternion initialSwingRotation;
    Quaternion initialBoomRotation;
    Quaternion initialArmRotation;
    Quaternion initialHandRotation;

    // Robot arm segment lengths (in meters).
    const float AB = 0.169f;
    const float CD = 0.273f;
    const float HANDSIZE = 0.325f;
    const float GF = HANDSIZE - CD;
    const float FE = 0.49727f;
    const float ED = 0.70142f;

    /// <summary>
    /// Initializes the robot arm's joint rotations, sets the initial pose,
    /// and subscribes to UI button events.
    /// </summary>
    void Start()
    {
        // Store the initial local rotations to use as a reference for relative movements.
        initialSwingRotation = swingAxis.transform.localRotation;
        initialBoomRotation = boomAxis.transform.localRotation;
        initialArmRotation = armAxis.transform.localRotation;
        initialHandRotation = handAxis.transform.localRotation;

        // Set the initial pose of the robot arm.
        setPose(Mathf.PI / 2, Mathf.PI / 2, Mathf.PI / 2, Mathf.PI / 2);

        // If in test mode, trigger the IK test sequence after a short delay.
        if (ikTestMode)
        {
            Invoke(nameof(PerformIK), 2f);
        }

        // Instantiate the GeminiRoboticsApi
        geminiRoboticsApi = new GeminiRoboticsApi();

        // Add a listener to the detect button to trigger the object detection process.
        buttonDetect.onClick.AddListener(OnDetectButtonClicked);
    }

    /// <summary>
    /// Called every frame. Updates the displayed coordinates of the end effector.
    /// </summary>
    void Update()
    {
        Vector3 fingerPos = robotBase.transform.InverseTransformPoint(finger.transform.position);
        coordinateFinger.UpdatePositionText(fingerPos);
    }

    /// <summary>
    /// Called when the MonoBehaviour is destroyed. Cancels any ongoing asynchronous tasks.
    /// </summary>
    void OnDestroy()
    {
        // Cancel and dispose the CancellationTokenSource to prevent memory leaks.
        _ikMoveCts?.Cancel();
        _ikMoveCts?.Dispose();
    }

    /// <summary>
    /// Handles the click event for the "Detect" button.
    /// Captures an image, sends it to the Gemini API for object detection, and logs the results.
    /// </summary>
    private async void OnDetectButtonClicked()
    {
        if (cameraCapture == null || geminiRoboticsApi == null)
        {
            Debug.LogError("CameraCapture or GeminiRoboticsApi is not assigned.");
            return;
        }

        // Capture the image from the camera as a base64 string.
        string b64Image = cameraCapture.CaptureAsBase64();

        // Send the image to the Gemini API and wait for the detected objects.
        var objectObjects = await geminiRoboticsApi.DetectObjects(b64Image);

        // Log the detected objects to the console.
        Debug.Log($"Detected {objectObjects.Length} objects.");

        // Clear previous detections and display the new ones.
        detectedPoints.clear();

        // Display each detected object and log its details.
        foreach (var obj in objectObjects)
        {
            Debug.Log($"- Label: {obj.label}, Point: ({obj.point.x}, {obj.point.y})");
            // Display the detected object on the UI.
            detectedPoints.displayDetectionPosition(obj);
        }
    }

    /// <summary>
    /// Performs inverse kinematics to calculate the joint angles required to reach the target.
    /// It solves a 2D planar IK problem based on the geometric relationships of the robot arm's segments
    /// and then initiates the robot arm's movement.
    /// </summary>
    public async void PerformIK()
    {
        // Calculate the target position relative to the robot base.
        Vector3 A = work.transform.localPosition;
        Debug.Log("Work position: " + A.ToString("F4"));

        float theta1 = Mathf.Atan2(A.z, A.x);
        Debug.Log("Theta1: " + (theta1 * Mathf.Rad2Deg).ToString("F4"));

        float AC = Mathf.Sqrt(A.x * A.x + A.z * A.z);
        float theta3 = Mathf.Asin(AB / AC);
        Debug.Log("Theta3: " + (theta3 * Mathf.Rad2Deg).ToString("F4"));

        float BC = AC * Mathf.Cos(theta3);
        Debug.Log("BC: " + BC.ToString("F4"));

        float theta2 = theta1 - theta3;
        Debug.Log("Theta2: " + (theta2 * Mathf.Rad2Deg).ToString("F4"));

        Vector3 B = new Vector3(BC * Mathf.Cos(theta2), A.y, BC * Mathf.Sin(theta2));
        Vector3 G = new Vector3(B.x, B.y + CD, B.z);

        float r = Mathf.Sqrt(BC * BC + GF * GF);
        Debug.Log("r: " + r.ToString("F4"));

        // Calculate internal angles using the Law of Cosines.
        float theat6 = Mathf.Acos((FE * FE - ED * ED - r * r) / (-2 * ED * r));
        float theat7 = Mathf.Acos((r * r - FE * FE - ED * ED) / (-2 * FE * ED));
        Debug.Log("Theta6: " + (theat6 * Mathf.Rad2Deg).ToString("F4"));
        Debug.Log("Theta7: " + (theat7 * Mathf.Rad2Deg).ToString("F4"));

        float theat5 = Mathf.Atan2(GF, BC);
        float theat4 = theat5 + theat6;
        Debug.Log("Theta4: " + (theat4 * Mathf.Rad2Deg).ToString("F4"));
        Debug.Log("Theta5: " + (theat5 * Mathf.Rad2Deg).ToString("F4"));

        float theat8 = 3 * Mathf.PI / 2 - theat4 - theat7;
        Debug.Log("Theta8: " + (theat8 * Mathf.Rad2Deg).ToString("F4"));

        // Log the initial rotation values for debugging.
        Debug.Log($"{initialSwingRotation.y}, {initialSwingRotation.z}");
        Debug.Log($"{initialBoomRotation.y}, {initialBoomRotation.z}");
        Debug.Log($"{initialArmRotation.y}, {initialArmRotation.z}");

        // Cancel any existing movement task before starting a new one.
        if (_ikMoveCts != null)
        {
            _ikMoveCts.Cancel();
            _ikMoveCts.Dispose();
        }

        // Start the asynchronous movement task, allowing for cancellation.
        _ikMoveCts = new CancellationTokenSource();
        var newTargets = CalculateTargetPose(theta2, theat4, theat7, theat8);
        await MoveToTargets(newTargets.swing, newTargets.boom, newTargets.arm, newTargets.hand, ikMoveDuration, _ikMoveCts.Token);
    }

    /// <summary>
    /// Directly sets the pose of the robot arm's joints to the specified angles without animation.
    /// </summary>
    /// <param name="swingAngle">The angle for the swing joint in radians.</param>
    /// <param name="boomAngle">The angle for the boom joint in radians.</param>
    /// <param name="armAngle">The angle for the arm joint in radians.</param>
    /// <param name="handAngle">The angle for the hand joint in radians.</param>
    void setPose(float swingAngle, float boomAngle, float armAngle, float handAngle)
    {
        // Apply rotations relative to the initial state.
        swingAxis.transform.localRotation = initialSwingRotation * UnityEngine.Quaternion.AngleAxis(swingAngle * Mathf.Rad2Deg, -Vector3.up);
        boomAxis.transform.localRotation = initialBoomRotation * UnityEngine.Quaternion.AngleAxis(boomAngle * Mathf.Rad2Deg, -Vector3.up);
        armAxis.transform.localRotation = initialArmRotation * UnityEngine.Quaternion.AngleAxis(armAngle * Mathf.Rad2Deg, Vector3.up);
        handAxis.transform.localRotation = initialHandRotation * UnityEngine.Quaternion.AngleAxis(handAngle * Mathf.Rad2Deg, Vector3.up);

        var newTargets = CalculateTargetPose(swingAngle, boomAngle, armAngle, handAngle);
    }

    /// <summary>
    /// Calculates the target world-space rotations for each joint based on the IK solver's angle results.
    /// </summary>
    /// <returns>A tuple containing the target quaternions for each joint.</returns>
    (UnityEngine.Quaternion swing, UnityEngine.Quaternion boom, UnityEngine.Quaternion arm, UnityEngine.Quaternion hand) CalculateTargetPose(float swingAngle, float boomAngle, float armAngle, float handAngle)
    {
        // Calculate the target rotation for each joint relative to its initial orientation.
        var swing = initialSwingRotation * Quaternion.AngleAxis(swingAngle * Mathf.Rad2Deg, -Vector3.up);
        var boom = initialBoomRotation * Quaternion.AngleAxis(boomAngle * Mathf.Rad2Deg, -Vector3.up);
        var arm = initialArmRotation * Quaternion.AngleAxis(armAngle * Mathf.Rad2Deg, Vector3.up);
        var hand = initialHandRotation * Quaternion.AngleAxis(handAngle * Mathf.Rad2Deg, Vector3.up);
        return (swing, boom, arm, hand);
    }

    /// <summary>
    /// Asynchronously moves the robot's joints smoothly from their current rotation to the target rotations over a specified duration.
    /// </summary>
    /// <param name="swingTarget">The target rotation for the swing joint.</param>
    /// <param name="boomTarget">The target rotation for the boom joint.</param>
    /// <param name="armTarget">The target rotation for the arm joint.</param>
    /// <param name="handTarget">The target rotation for the hand joint.</param>
    /// <param name="duration">The time in seconds the movement should take.</param>
    /// <param name="cancellationToken">A token to allow for cancellation of the movement task.</param>
    private async Task MoveToTargets(UnityEngine.Quaternion swingTarget, UnityEngine.Quaternion boomTarget, UnityEngine.Quaternion armTarget, UnityEngine.Quaternion handTarget, float duration, CancellationToken cancellationToken)
    {
        // Capture the starting rotation of each joint.
        Quaternion startSwing = swingAxis.transform.localRotation;
        Quaternion startBoom = boomAxis.transform.localRotation;
        Quaternion startArm = armAxis.transform.localRotation;
        Quaternion startHand = handAxis.transform.localRotation;
        float elapsedTime = 0f;

        try
        {
            // Loop until the elapsed time reaches the specified duration.
            while (elapsedTime < duration)
            {
                cancellationToken.ThrowIfCancellationRequested();

                // Smoothly interpolate joint rotations.
                float t = elapsedTime / duration;
                swingAxis.transform.localRotation = UnityEngine.Quaternion.Slerp(startSwing, swingTarget, t);
                boomAxis.transform.localRotation = UnityEngine.Quaternion.Slerp(startBoom, boomTarget, t);
                armAxis.transform.localRotation = UnityEngine.Quaternion.Slerp(startArm, armTarget, t);
                handAxis.transform.localRotation = UnityEngine.Quaternion.Slerp(startHand, handTarget, t);

                elapsedTime += Time.deltaTime;
                // Wait for the next frame before continuing the loop.
                await Task.Yield();
            }

            // Snap to the final target rotations to ensure precision.
            swingAxis.transform.localRotation = swingTarget;
            boomAxis.transform.localRotation = boomTarget;
            armAxis.transform.localRotation = armTarget;
            handAxis.transform.localRotation = handTarget;
        }
        catch (TaskCanceledException)
        {
            // Gracefully handle task cancellation (e.g., if a new movement command interrupts this one).
        }
    }
}
