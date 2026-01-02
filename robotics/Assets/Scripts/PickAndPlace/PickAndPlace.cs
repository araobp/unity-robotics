using UnityEngine;
using System.Threading;
using System.Threading.Tasks;
using Vector3 = UnityEngine.Vector3;
using UnityEngine.UI;

/// <summary>
/// Implements inverse kinematics (IK) for a robot arm to pick and place objects.
/// It calculates the required articulation angles to reach a target and animates the movement.
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

    // GameObjects representing the robot's articulations.
    [Header("Robot Articulations")]
    [SerializeField] GameObject swing;
    [SerializeField] GameObject boom;
    [SerializeField] GameObject arm;
    [SerializeField] GameObject hand;
    [SerializeField] GameObject wrist;

    [Header("End Effector")]
    [SerializeField] GameObject endEffector;
    [SerializeField] bool isAlignedToTable = true;
    [SerializeField] float targetForce = 3.0f;
    [SerializeField] float targetAngularVelocity = 30.0f;

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

    private ArticulationBody swingAb;
    private ArticulationBody boomAb;
    private ArticulationBody armAb;
    private ArticulationBody handAb;
    private ArticulationBody wristAb;

    // Robot arm segment lengths (in meters).
    const float AB = 0.169f;
    const float CD = 0.273f;
    const float END_EFFECTOR_SIZE = 0.41f;
    const float FE = 0.49727f;
    const float ED = 0.70142f;

    /// <summary>
    /// Initializes the robot arm's articulation components, sets the initial pose,
    /// and subscribes to UI button events.
    /// </summary>
    async void Start()
    {
        swingAb = swing.GetComponent<ArticulationBody>();
        boomAb = boom.GetComponent<ArticulationBody>();
        armAb = arm.GetComponent<ArticulationBody>();
        handAb = hand.GetComponent<ArticulationBody>();
        wristAb = wrist.GetComponent<ArticulationBody>();
        try
        {
            // Set the initial pose of the robot arm.
            await setPose(Mathf.PI / 2, Mathf.PI / 2, Mathf.PI / 2, Mathf.PI / 2, isAlignedToTable ? 0 : Mathf.PI / 2);

            // If in test mode, trigger the IK test sequence after a short delay.
            if (ikTestMode)
            {
                await TestIKSequence();
            }
        }
        catch (System.Exception e)
        {
            Debug.LogException(e);
        }

        // Instantiate the GeminiRoboticsApi
        geminiRoboticsApi = new GeminiRoboticsApi();

        // Add a listener to the detect button to trigger the object detection process.
        buttonDetect.onClick.AddListener(OnDetectButtonClicked);
    }

    private async Task TestIKSequence()
    {
        await Task.Delay(2000);
        await PerformIK(work.transform.localPosition); 
        await Close(targetForce, targetAngularVelocity);
        await Task.Delay(1000);
        await PerformIK(work.transform.localPosition + new Vector3(-0.3f, 0.8f, -0.3f));
    }

    /// <summary>
    /// Called every frame. Updates the displayed coordinates of the end effector.
    /// </summary>
    void Update()
    {
        Vector3 fingerPos = robotBase.transform.InverseTransformPoint(wrist.transform.position);
        coordinateFinger.UpdatePositionText(fingerPos);
    }

    /// <summary>
    /// Called at a fixed time interval, used for physics-related updates.
    /// Handles the continuous gripping logic and force feedback.
    /// </summary>
    void FixedUpdate()
    {
        // The gripping logic is now handled by the ParallelGripper script.
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
    /// Performs inverse kinematics to calculate the articulation angles required to reach the target.
    /// It solves a 2D planar IK problem based on the geometric relationships of the robot arm's segments
    /// and then initiates the robot arm's movement.
    /// </summary>
    public async Task PerformIK(Vector3 workPosition)
    {
        // Calculate the target position relative to the robot base.
        Vector3 A = workPosition;
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

        float GF = END_EFFECTOR_SIZE - CD + A.y;

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

        // Cancel any existing movement task before starting a new one.
        if (_ikMoveCts != null)
        {
            _ikMoveCts.Cancel();
            _ikMoveCts.Dispose();
        }

        // Start the asynchronous movement task, allowing for cancellation.
        _ikMoveCts = new CancellationTokenSource();

        float swingTarget = -theta2 * Mathf.Rad2Deg;
        float boomTarget = -theat4 * Mathf.Rad2Deg;
        float armTarget = theat7 * Mathf.Rad2Deg;
        float handTarget = theat8 * Mathf.Rad2Deg;

        await MoveToTargets(swingTarget, boomTarget, armTarget, handTarget, ikMoveDuration, _ikMoveCts.Token);
    }

    /// <summary>
    /// Closes the gripper fingers to the defined closed angle.
    /// </summary>
    public async Task Close(float targetForce, float angularVelocity)
    {
        if (endEffector == null) return;
        await endEffector.GetComponent<ParallelGripper>().Close(targetForce, angularVelocity);
    }

    public async Task Open(float angularVelocity)
    {
        if (endEffector == null) return;
        await endEffector.GetComponent<ParallelGripper>().Open(angularVelocity);
    }

    /// <summary>
    /// Directly sets the pose of the robot arm's articulations to the specified angles without animation.
    /// </summary>
    /// <param name="swingAngle">The angle for the swing articulation in radians.</param>
    /// <param name="boomAngle">The angle for the boom articulation in radians.</param>
    /// <param name="armAngle">The angle for the arm articulation in radians.</param>
    /// <param name="handAngle">The angle for the hand articulation in radians.</param>
    /// <param name="wristAngle">The angle for the wrist articulation in radians.</param>
    async Task setPose(float swingAngle, float boomAngle, float armAngle, float handAngle, float wristAngle)
    {
        SetArticulationTarget(swingAb, -swingAngle * Mathf.Rad2Deg);
        SetArticulationTarget(boomAb, -boomAngle * Mathf.Rad2Deg);
        SetArticulationTarget(armAb, armAngle * Mathf.Rad2Deg);
        SetArticulationTarget(handAb, handAngle * Mathf.Rad2Deg);
        SetArticulationTarget(wristAb, -wristAngle * Mathf.Rad2Deg);
        await Task.Delay(100);
        await Open(100f);
    }

    /// <summary>
    /// Sets the target position for the given articulation body's drive.
    /// </summary>
    /// <param name="articulation">The articulation body to update.</param>
    /// <param name="target">The target position in degrees.</param>
    void SetArticulationTarget(ArticulationBody articulation, float target)
    {
        var drive = articulation.xDrive;
        drive.target = target;
        articulation.xDrive = drive;
    }

    /// <summary>
    /// Asynchronously moves the robot's articulations smoothly from their current angle to the target angles over a specified duration.
    /// </summary>
    /// <param name="swingTarget">The target angle for the swing articulation in degrees.</param>
    /// <param name="boomTarget">The target angle for the boom articulation in degrees.</param>
    /// <param name="armTarget">The target angle for the arm articulation in degrees.</param>
    /// <param name="handTarget">The target angle for the hand articulation in degrees.</param>
    /// <param name="wristTarget">The target angle for the end effector in degrees.</param>
    /// <param name="duration">The time in seconds the movement should take.</param>
    /// <param name="cancellationToken">A token to allow for cancellation of the movement task.</param>
    private async Task MoveToTargets(float swingTarget, float boomTarget, float armTarget, float handTarget, float duration, CancellationToken cancellationToken)
    {
        // Capture the starting angle of each articulation.
        float swingStart = swingAb.xDrive.target;
        float boomStart = boomAb.xDrive.target;
        float armStart = armAb.xDrive.target;
        float handStart = handAb.xDrive.target;

        float elapsedTime = 0f;

        try
        {
            // Loop until the elapsed time reaches the specified duration.
            while (elapsedTime < duration)
            {
                cancellationToken.ThrowIfCancellationRequested();

                // Smoothly interpolate articulation angles.
                float t = elapsedTime / duration;
                float swingRotation = Mathf.Lerp(swingStart, swingTarget, t);
                SetArticulationTarget(swingAb, swingRotation);
                SetArticulationTarget(boomAb, Mathf.Lerp(boomStart, boomTarget, t));
                SetArticulationTarget(armAb, Mathf.Lerp(armStart, armTarget, t));
                SetArticulationTarget(handAb, Mathf.Lerp(handStart, handTarget, t));
                SetArticulationTarget(wristAb, isAlignedToTable? swingRotation -90: swingRotation);

                elapsedTime += Time.deltaTime;
                // Wait for the next frame before continuing the loop.
                await Task.Yield();
            }

            // Snap to the final target angles to ensure precision.
            SetArticulationTarget(swingAb, swingTarget);
            SetArticulationTarget(boomAb, boomTarget);
            SetArticulationTarget(armAb, armTarget);
            SetArticulationTarget(handAb, handTarget);
            SetArticulationTarget(wristAb, isAlignedToTable? swingTarget -90: swingTarget);
        }
        catch (TaskCanceledException)
        {
            // Gracefully handle task cancellation (e.g., if a new movement command interrupts this one).
        }
    }
}
