using UnityEngine;
using System.Threading;
using System.Threading.Tasks;
using Vector3 = UnityEngine.Vector3;
using UnityEngine.UI;

/// <summary>
/// This class implements inverse kinematics (IK) for a 6-axis robot arm to pick and place objects.
/// It calculates the required joint angles to reach a target position and orientation,
/// and then animates the robot's movement to the calculated pose. It also integrates with
/// a camera and a generative AI model to detect objects in the scene.
/// </summary>
public class PickAndPlace : MonoBehaviour
{
    // --- Fields ---

    [Header("IK Test Mode")]
    [SerializeField] bool ikTestMode = false;

    [Header("Placement Target")]
    [SerializeField] GameObject placeTarget;

    // The target GameObject that the robot arm's end effector will attempt to reach.
    [Header("IK Target")]
    [SerializeField] GameObject work;

    // Inverse kinematics behavior settings.
    [Header("IK Settings")]
    [Tooltip("The angular speed of the fastest joint in degrees per second during an IK movement.")]
    [SerializeField] private float ikAngularSpeed = 60.0f;
    [Tooltip("Whether to use Ease-in/Ease-out interpolation for smoother movement.")]
    [SerializeField] bool useEaseinEaseout = true;

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
    private float _endEffectorSize = 0f;
    const float FE = 0.49727f;
    const float ED = 0.70142f;

    /// <summary>
    /// Initializes the robot arm by caching its ArticulationBody components, setting an initial pose,
    /// and subscribing to UI button events. If in IK test mode, it will also run a test sequence.
    /// </summary>
    async void Start()
    {
        // Cache ArticulationBody components for each robot joint to improve performance.
        swingAb = swing.GetComponent<ArticulationBody>();
        boomAb = boom.GetComponent<ArticulationBody>();
        armAb = arm.GetComponent<ArticulationBody>();
        handAb = hand.GetComponent<ArticulationBody>();
        wristAb = wrist.GetComponent<ArticulationBody>();

        // Get the end effector size from the ParallelGripper component.
        _endEffectorSize = endEffector.GetComponent<IEndEffector>().EndEffectorSize;
        Debug.Log("End Effector Size: " + _endEffectorSize);

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

    /// <summary>
    /// Runs a pre-defined sequence of movements to test the inverse kinematics and gripper functionality.
    /// </summary>
    private async Task TestIKSequence() 
    {
        await Task.Delay(2000);
        await PerformIK(work.transform.localPosition + new Vector3(0f, 0.4f, 0f));
        await Task.Delay(1000);
        await PerformIK(work.transform.localPosition);
        await Task.Delay(1000);
        await Close(targetForce, targetAngularVelocity);
        await Task.Delay(1000);
        await PerformIK(work.transform.localPosition + new Vector3(0f, 0.4f, 0f));
        await Task.Delay(1000);
        await PerformIK(placeTarget.transform.localPosition + new Vector3(0f, 0.4f, 0f));
        await Task.Delay(1000);
        await PerformIK(placeTarget.transform.localPosition + new Vector3(0f, 0.15f, 0f));
        await Task.Delay(1000);
        await Open(targetAngularVelocity);
        await Task.Delay(1000);
        await PerformIK(placeTarget.transform.localPosition + new Vector3(0f, 0.4f, 0f));
        await Task.Delay(1000);
    }

    /// <summary>
    /// Called every frame. Updates the UI to display the current coordinates of the end effector (wrist).
    /// </summary>
    void Update()
    {
        Vector3 fingerPos = robotBase.transform.InverseTransformPoint(wrist.transform.position);
        coordinateFinger.UpdatePositionText(fingerPos);
    }

    /// <summary>
    /// This method is called at a fixed time interval and is used for physics-related updates.
    /// In this script, the continuous gripping logic has been moved to the ParallelGripper script.
    /// </summary>
    void FixedUpdate()
    {
        // The gripping logic is now handled by the ParallelGripper script.
    }

    /// <summary>
    /// Called when the MonoBehaviour is destroyed. This is used to cancel any ongoing asynchronous
    /// movement tasks to prevent errors or unexpected behavior when the object is removed from the scene.
    /// </summary>
    void OnDestroy()
    {
        // Cancel and dispose the CancellationTokenSource to prevent memory leaks.
        _ikMoveCts?.Cancel();
        _ikMoveCts?.Dispose();
    }

    /// <summary>
    /// Handles the click event from the "Detect" button. This function captures an image from the
    /// scene's camera, sends it to the Gemini API for object detection, and then displays the
    /// results on the UI.
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
    /// Performs inverse kinematics (IK) to calculate the required joint angles for the robot arm
    /// to reach the specified target position. This implementation solves a 2D planar IK problem
    /// based on the geometric relationships of the robot arm's segments and then initiates the movement.
    /// </summary>
    /// <param name="targetPosition">The target position for the end effector in the robot's local space.</param>
    /// <param name="duration">Optional. The duration of the movement. If not provided, it's calculated based on the required joint rotation and ikAngularSpeed.</param>
    public async Task PerformIK(Vector3 targetPosition, float duration = 0)
    {
        // Calculate the target position relative to the robot base.
        Vector3 A = targetPosition;
        Debug.Log("Work position: " + A.ToString("F4"));

        // --- 2D Planar Inverse Kinematics Calculation ---

        // theta1 is the base swing angle around the Y-axis.
        float theta1 = Mathf.Atan2(A.z, A.x);
        Debug.Log("Theta1: " + (theta1 * Mathf.Rad2Deg).ToString("F4"));

        // Horizontal distance from the base to the target point in the XZ plane.
        float AC = Mathf.Sqrt(A.x * A.x + A.z * A.z);
        // Angle correction to account for the boom's pivot offset from the base's center.
        float theta3 = Mathf.Asin(AB / AC);
        Debug.Log("Theta3: " + (theta3 * Mathf.Rad2Deg).ToString("F4"));

        // Corrected horizontal distance from the boom pivot to the target's projection.
        float BC = AC * Mathf.Cos(theta3);
        Debug.Log("BC: " + BC.ToString("F4"));

        // The final, corrected swing angle for the base articulation.
        float theta2 = theta1 - theta3;
        Debug.Log("Theta2: " + (theta2 * Mathf.Rad2Deg).ToString("F4"));

        // Vertical distance from the boom pivot to the wrist joint.
        float GF = _endEffectorSize - CD + A.y;

        // The direct distance from the boom pivot to the wrist joint, forming a triangle with the boom and arm.
        float r = Mathf.Sqrt(BC * BC + GF * GF);
        Debug.Log("r: " + r.ToString("F4"));

        // Calculate internal angles of the arm triangle (boom, arm, r) using the Law of Cosines.
        // theat6 is the angle at the wrist joint.
        float theat6 = Mathf.Acos((FE * FE - ED * ED - r * r) / (-2 * ED * r));
        // theat7 is the angle at the arm joint (elbow).
        float theat7 = Mathf.Acos((r * r - FE * FE - ED * ED) / (-2 * FE * ED));
        Debug.Log("Theta6: " + (theat6 * Mathf.Rad2Deg).ToString("F4"));
        Debug.Log("Theta7: " + (theat7 * Mathf.Rad2Deg).ToString("F4"));

        // Calculate the final angles for the boom and hand articulations.
        // theat5 is the angle of the line 'r' with the horizontal plane.
        float theat5 = Mathf.Atan2(GF, BC);
        // theat4 is the angle for the boom articulation.
        float theat4 = theat5 + theat6;
        Debug.Log("Theta4: " + (theat4 * Mathf.Rad2Deg).ToString("F4"));
        Debug.Log("Theta5: " + (theat5 * Mathf.Rad2Deg).ToString("F4"));

        // Calculate the hand angle to keep the end effector level.
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

        float moveDuration = duration;
        if (moveDuration <= 0 && ikAngularSpeed > 0)
        {
            // Calculate the duration based on the largest joint angle change required.
            // This ensures a more consistent speed across different types of movements.
            float swingStart = swingAb.xDrive.target;
            float boomStart = boomAb.xDrive.target;
            float armStart = armAb.xDrive.target;
            float handStart = handAb.xDrive.target;

            float maxAngleChange = 0f;
            maxAngleChange = Mathf.Max(maxAngleChange, Mathf.Abs((-theta2 * Mathf.Rad2Deg) - swingStart));
            maxAngleChange = Mathf.Max(maxAngleChange, Mathf.Abs((-theat4 * Mathf.Rad2Deg) - boomStart));
            maxAngleChange = Mathf.Max(maxAngleChange, Mathf.Abs((theat7 * Mathf.Rad2Deg) - armStart));
            maxAngleChange = Mathf.Max(maxAngleChange, Mathf.Abs((theat8 * Mathf.Rad2Deg) - handStart));
            moveDuration = maxAngleChange / ikAngularSpeed;
        }

        float swingTarget = -theta2 * Mathf.Rad2Deg;
        float boomTarget = -theat4 * Mathf.Rad2Deg;
        float armTarget = theat7 * Mathf.Rad2Deg;
        float handTarget = theat8 * Mathf.Rad2Deg;

        await MoveToTargets(swingTarget, boomTarget, armTarget, handTarget, moveDuration, _ikMoveCts.Token);
    }

    /// <summary>
    /// Commands the gripper to close, applying a specified force and angular velocity.
    /// </summary>
    /// <param name="targetForce">The desired force in Newtons to apply when gripping.</param>
    /// <param name="angularVelocity">The speed at which the gripper should close.</param>
    public async Task Close(float targetForce, float angularVelocity)
    {
        if (endEffector == null) return;
        await endEffector.GetComponent<IGripper>().Close(targetForce, angularVelocity);
    }

    /// <summary>
    /// Commands the gripper to open at a specified angular velocity.
    /// </summary>
    /// <param name="angularVelocity">The speed at which the gripper should open.</param>
    public async Task Open(float angularVelocity)
    {
        if (endEffector == null) return;
        await endEffector.GetComponent<IGripper>().Open(angularVelocity);
    }

    /// <summary>
    /// Instantly sets the pose of the robot arm's articulations to the specified angles without animation,
    /// and then opens the gripper.
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
    /// A helper function to set the target position for a given articulation body's primary drive (xDrive).
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
    /// Asynchronously moves the robot's articulations smoothly from their current angles to the target angles
    /// over a specified duration using linear interpolation (Lerp).
    /// </summary>
    /// <param name="swingTarget">The target angle for the swing articulation in degrees.</param>
    /// <param name="boomTarget">The target angle for the boom articulation in degrees.</param>
    /// <param name="armTarget">The target angle for the arm articulation in degrees.</param>
    /// <param name="handTarget">The target angle for the hand articulation in degrees.</param>
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
                // Use SmoothStep to create an ease-in and ease-out effect for a more industrial feel.
                if (useEaseinEaseout)
                {
                    // The SmoothStep function provides a smooth interpolation between 0 and 1.
                    t = Mathf.SmoothStep(0f, 1f, t);
                }

                float swingRotation = Mathf.Lerp(swingStart, swingTarget, t);
                SetArticulationTarget(swingAb, swingRotation);
                SetArticulationTarget(boomAb, Mathf.Lerp(boomStart, boomTarget, t));
                SetArticulationTarget(armAb, Mathf.Lerp(armStart, armTarget, t));
                SetArticulationTarget(handAb, Mathf.Lerp(handStart, handTarget, t));
                SetArticulationTarget(wristAb, isAlignedToTable ? swingRotation - 90 : swingRotation);

                elapsedTime += Time.deltaTime;
                // Wait for the next frame before continuing the loop.
                await Task.Yield();
            }

            // Snap to the final target angles to ensure precision.
            SetArticulationTarget(swingAb, swingTarget);
            SetArticulationTarget(boomAb, boomTarget);
            SetArticulationTarget(armAb, armTarget);
            SetArticulationTarget(handAb, handTarget);
            SetArticulationTarget(wristAb, isAlignedToTable ? swingTarget - 90 : swingTarget);
        }
        catch (TaskCanceledException)
        {
            // Gracefully handle task cancellation (e.g., if a new movement command interrupts this one).
        }
    }
}
