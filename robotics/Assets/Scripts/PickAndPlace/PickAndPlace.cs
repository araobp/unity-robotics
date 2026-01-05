using UnityEngine;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine.UI;
using System.Collections.Generic;
using System.Linq;

/// <summary>
/// This class implements inverse kinematics (IK) for a 6-axis robot arm, enabling it to pick and place objects.
/// It calculates the necessary joint angles to reach a target position and orientation, then animates the robot's
/// movement to the calculated pose. The class also integrates with a camera and the Gemini API to detect
/// objects in the scene, allowing for dynamic interaction with the environment. It supports a test mode for IK
/// validation and provides UI controls for triggering detection and pick-and-place operations.
/// </summary>
public class PickAndPlace : MonoBehaviour
{
    // --- Fields ---

    [Header("IK Test Mode")]
    [Tooltip("If true, the robot will perform a pre-defined pick-and-place sequence for testing purposes.")]
    [SerializeField] bool ikTestMode = false;

    [Tooltip("The target GameObject where the robot arm will place the picked object.")]
    [Header("Placement Target")]
    [SerializeField] GameObject placeTarget;

    [Tooltip("The target GameObject that the robot arm's end effector will attempt to reach in test mode.")]
    [Header("IK Target")]
    [SerializeField] GameObject work;

    // Inverse kinematics behavior settings.
    [Header("IK Settings")]
    [Tooltip("The angular speed of the fastest joint in degrees per second during an IK movement.")]
    [SerializeField] private float ikAngularSpeed = 60.0f;

    [Tooltip("Whether to use Ease-in/Ease-out interpolation for smoother movement.")]
    [SerializeField] bool useEaseinEaseout = true;

    [Tooltip("The workarea")]
    [Header("Work Area")]
    [SerializeField] GameObject workArea;

    // GameObjects representing the robot's articulations.
    [Header("Robot Articulations")]
    [SerializeField] GameObject swing;
    [SerializeField] GameObject boom;
    [SerializeField] GameObject arm;
    [SerializeField] GameObject hand;
    [SerializeField] GameObject wrist;

    [Tooltip("The end effector (gripper) of the robot arm.")]
    [Header("End Effector")]
    [SerializeField] GameObject endEffector;

    [Tooltip("If true, the wrist will attempt to stay parallel to the table.")]
    [SerializeField] bool isAlignedToTable = true;

    [Tooltip("The force applied by the gripper when closing.")]
    [SerializeField] float targetForce = 3.0f;

    [Tooltip("The angular velocity of the gripper when opening or closing.")]
    [SerializeField] float targetAngularVelocity = 30.0f;

    // UI elements for displaying coordinates.
    [Header("Coordinate Display")]
    [SerializeField] Coordinate coordinateEndEffector;
    [SerializeField] Coordinate coordinatePick;
    [SerializeField] Coordinate coordinatePlace;

    private Transform _endEffectorEdgeTransform;

    // UI buttons for triggering robot actions.
    [Header("Operation buttons")]
    [SerializeField] Button buttonDetect;
    [SerializeField] Button buttonPickAndPlace;
    [SerializeField] Button buttonReset;

    [Tooltip("The label of the object to be detected and picked up.")]
    [Header("Detection target")]
    [SerializeField] string detectionTargetLabel = "cube";

    // References to other components used for detection and display.
    [Header("Components")]
    [SerializeField] private CameraCapture cameraCapture;
    [SerializeField] private DetectedPoints detectedPoints;

    // API for interacting with the Gemini Robotics service.
    private GeminiRoboticsApi geminiRoboticsApi; // API for interacting with the Gemini vision and language models.

    // A collection to store the workpieces detected by the vision system.
    private IEnumerable<DetectedObject> detectedWorkpieces;

    // Used to cancel the in-progress asynchronous movement task.
    private CancellationTokenSource _ikMoveCts;

    // Cached ArticulationBody components for each joint.
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

    // --- Unity Lifecycle Methods ---

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

        _endEffectorEdgeTransform = endEffector.GetComponent<IEndEffector>().EndEffectorEdgeTransform;

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
        buttonPickAndPlace.onClick.AddListener(OnPickAndPlaceButtonClicked);

    }
    
    /// <summary>
    /// Called every frame. Updates the UI to display the current coordinates of the end effector (wrist).
    /// </summary>
    void Update()
    {
        // Get the current position of the wrist relative to the robot base and update the UI display.
        coordinateEndEffector.UpdatePositionText(workArea.transform.InverseTransformPoint(_endEffectorEdgeTransform.position));
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

    // --- Public Methods ---

    /// <summary>
    /// Performs inverse kinematics (IK) to calculate the required joint angles for the robot arm
    /// to reach the specified target position. This implementation solves a 2D planar IK problem
    /// based on the geometric relationships of the robot arm's segments and then initiates the movement.
    /// </summary>
    /// <param name="targetPosition">The target position for the end effector in the robot's local space.</param>
    /// <param name="duration">Optional. The duration of the movement in seconds. If not provided, it's calculated based on the required joint rotation and ikAngularSpeed.</param>
    /// <param name="delay">Optional delay in milliseconds before the movement starts.</param>
    public async Task PerformIK(Vector3 targetPosition, float duration = 0, int delay = 1000)
    {
        // Introduce a delay before starting the IK calculation and movement.
        await Task.Delay(delay);

        // Calculate the target position relative to the robot base.
        Vector3 A = targetPosition;
        Debug.Log("Work position: " + A.ToString("F4"));

        // --- 2D Planar Inverse Kinematics Calculation --- //

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
        // theta6 is the angle at the wrist joint.
        float theta6 = Mathf.Acos((FE * FE - ED * ED - r * r) / (-2 * ED * r));
        // theta7 is the angle at the arm joint (elbow).
        float theta7 = Mathf.Acos((r * r - FE * FE - ED * ED) / (-2 * FE * ED));
        Debug.Log("Theta6: " + (theta6 * Mathf.Rad2Deg).ToString("F4"));
        Debug.Log("Theta7: " + (theta7 * Mathf.Rad2Deg).ToString("F4"));

        // Calculate the final angles for the boom and hand articulations.
        // theta5 is the angle of the line 'r' with the horizontal plane.
        float theta5 = Mathf.Atan2(GF, BC);
        // theta4 is the angle for the boom articulation.
        float theta4 = theta5 + theta6;
        Debug.Log("Theta4: " + (theta4 * Mathf.Rad2Deg).ToString("F4"));
        Debug.Log("Theta5: " + (theta5 * Mathf.Rad2Deg).ToString("F4"));

        // Calculate the hand angle to keep the end effector level.
        float theta8 = 3 * Mathf.PI / 2 - theta4 - theta7;
        Debug.Log("Theta8: " + (theta8 * Mathf.Rad2Deg).ToString("F4"));

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
            maxAngleChange = Mathf.Max(maxAngleChange, Mathf.Abs((-theta4 * Mathf.Rad2Deg) - boomStart));
            maxAngleChange = Mathf.Max(maxAngleChange, Mathf.Abs((theta7 * Mathf.Rad2Deg) - armStart));
            maxAngleChange = Mathf.Max(maxAngleChange, Mathf.Abs((theta8 * Mathf.Rad2Deg) - handStart));
            moveDuration = maxAngleChange / ikAngularSpeed;
        }

        float swingTarget = -theta2 * Mathf.Rad2Deg;
        float boomTarget = -theta4 * Mathf.Rad2Deg;
        float armTarget = theta7 * Mathf.Rad2Deg;
        float handTarget = theta8 * Mathf.Rad2Deg;

        await MoveToTargets(swingTarget, boomTarget, armTarget, handTarget, moveDuration, _ikMoveCts.Token);
    }

    /// <summary>
    /// Commands the gripper to open at a specified angular velocity.
    /// </summary>
    /// <param name="angularVelocity">The speed at which the gripper should open.</param>
    /// <param name="delay">Optional delay in milliseconds before opening.</param>
    public async Task Open(float angularVelocity, int delay = 1000)
    {
        // Ensure the end effector is assigned before attempting to open.
        if (endEffector == null) return;
        // Introduce a delay before opening the gripper.
        await Task.Delay(delay);
        await endEffector.GetComponent<IGripper>().Open(angularVelocity);
    }

    /// <summary>
    /// Commands the gripper to close, applying a specified force and angular velocity.
    /// </summary>
    /// <param name="targetForce">The desired force in Newtons to apply when gripping.</param>
    /// <param name="angularVelocity">The speed at which the gripper should close.</param>
    /// <param name="delay">Optional delay in milliseconds before closing.</param>
    public async Task Close(float targetForce, float angularVelocity, int delay = 1000)
    {
        // Ensure the end effector is assigned before attempting to close.
        if (endEffector == null) return;
        // Introduce a delay before closing the gripper.
        await Task.Delay(delay);
        await endEffector.GetComponent<IGripper>().Close(targetForce, angularVelocity);
    }

    // --- UI Event Handlers ---

    /// <summary>
    /// Handles the click event from the "Detect" button. This function captures an image from the
    /// scene's camera, sends it to the Gemini API for object detection, and then displays the
    /// results on the UI.
    /// </summary>
    private async void OnDetectButtonClicked()
    {
        // Ensure required components are assigned.
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

        // Find the target object and store it.
        if (detectedPoints.TryGetDetectedObjects(detectionTargetLabel, out detectedWorkpieces))
        {
            Debug.Log($"Found target workpiece: {detectedWorkpieces.First().label} at ({detectedWorkpieces.First().point.x}, {detectedWorkpieces.First().point.y})");
        }
        else
        {
            Debug.LogWarning($"Target workpiece with label '{detectionTargetLabel}' not found.");
        }
    }

    /// <summary>
    /// Handles the click event for the "Pick and Place" button. It iterates through the
    /// detected workpieces and initiates the pick-and-place sequence for each one.
    /// </summary>
    private async void OnPickAndPlaceButtonClicked()
    {
        // Check if there are any detected workpieces to process.
        if (detectedWorkpieces != null && detectedWorkpieces.Any())
        {
            // Iterate through each detected object.
            foreach (var obj in detectedWorkpieces)
            {   
                Debug.Log($"Workpiece: {obj.label} at ({obj.point.x}, {obj.point.y})");
                // Convert normalized 0-1000 coordinates to pixel coordinates.
                float u = obj.point.x / 1000f * cameraCapture.ImageWidth;
                float v = obj.point.y / 1000f * cameraCapture.ImageHeight;

                // Project the 2D pixel coordinates to a 3D position in the robot's work area.
                Vector3 workpiecePos = cameraCapture.ProjectToWorkAreaLocal(u, v);
                Debug.Log($"[PickAndPlace] World Position of detected object '{obj.label}': {workpiecePos}");
                // Start the pick-and-place sequence for the current object.
                await pickAndPlace(workpiecePos, placeTarget.transform.localPosition);
            }
        }
        else
        {
            Debug.LogWarning("No detected workpieces to pick and place.");
        }
    }

    /// <summary>
    /// Runs a pre-defined sequence of movements to test the inverse kinematics and gripper functionality.
    /// </summary>
    private async Task TestIKSequence()
    {
        await pickAndPlace(work.transform.localPosition, placeTarget.transform.localPosition);
    }

    /// <summary>
    /// Executes a full pick-and-place sequence for a single object, moving from a pick position to a place position.
    /// </summary>
    /// <param name="pickPos">The position where the object will be picked from.</param>
    /// <param name="placePos">The position where the object will be placed.</param>
    private async Task pickAndPlace(Vector3 pickPos, Vector3 placePos)
    {
        // 2. Move above the object.
        Vector3 pickPositionAbove = new Vector3(pickPos.x, pickPos.y + 0.2f, pickPos.z);
        await PerformIK(pickPositionAbove);

        // 3. Move down to the object.
        Vector3 pickPositionAt = new Vector3(pickPos.x, pickPos.y + 0.02f, pickPos.z);
        await PerformIK(pickPositionAt);

        // 4. Close the gripper to pick up the object.
        await Close(targetForce, targetAngularVelocity);

        coordinatePick.UpdatePositionText(_endEffectorEdgeTransform.position);

        // 5. Move back up with the object.
        Vector3 pickPositionLift = new Vector3(pickPos.x, pickPos.y + 0.4f, pickPos.z);
        await PerformIK(pickPositionLift);

        Vector3 placePositionLift = new Vector3(placePos.x, placePos.y + 0.4f, placePos.z);
        await PerformIK(placePositionLift);

        // 6. Move to the place target above position.
        Vector3 placePositionAbove = new Vector3(placePos.x, placePos.y + 0.15f, placePos.z);
        await PerformIK(placePositionAbove);

        // 8. Open the gripper to release the object.
        await Open(targetAngularVelocity);

        coordinatePlace.UpdatePositionText(_endEffectorEdgeTransform.position);

        // 9. Move back up from the place position.
        await PerformIK(placePositionLift);
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
    private async Task setPose(float swingAngle, float boomAngle, float armAngle, float handAngle, float wristAngle)
    {
        // Set the target angle for each articulation body, converting from radians to degrees.
        SetArticulationTarget(swingAb, -swingAngle * Mathf.Rad2Deg);
        SetArticulationTarget(boomAb, -boomAngle * Mathf.Rad2Deg);
        SetArticulationTarget(armAb, armAngle * Mathf.Rad2Deg);
        SetArticulationTarget(handAb, handAngle * Mathf.Rad2Deg);
        SetArticulationTarget(wristAb, -wristAngle * Mathf.Rad2Deg);
        // Wait for a short period to allow the joints to settle.
        await Task.Delay(100);
        // Open the gripper.
        await Open(100f);
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

        // Keep track of the time elapsed during the movement.
        float elapsedTime = 0f;

        try
        {
            // Loop until the elapsed time reaches the specified duration.
            while (elapsedTime < duration)
            {
                // Check if the task has been cancelled.
                cancellationToken.ThrowIfCancellationRequested();

                // Smoothly interpolate articulation angles.
                float t = elapsedTime / duration;
                // Use SmoothStep to create an ease-in and ease-out effect for a more industrial feel.
                if (useEaseinEaseout)
                {
                    // The SmoothStep function provides a smooth interpolation between 0 and 1.
                    t = Mathf.SmoothStep(0f, 1f, t);
                }

                // Calculate the interpolated angle for each joint.
                float swingRotation = Mathf.Lerp(swingStart, swingTarget, t);
                SetArticulationTarget(swingAb, swingRotation);
                SetArticulationTarget(boomAb, Mathf.Lerp(boomStart, boomTarget, t));
                SetArticulationTarget(armAb, Mathf.Lerp(armStart, armTarget, t));
                SetArticulationTarget(handAb, Mathf.Lerp(handStart, handTarget, t));
                // Adjust the wrist to keep it aligned (e.g., parallel to the table).
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

    // --- Private Helper Methods ---

    /// <summary>
    /// A helper function to set the target position for a given articulation body's primary drive (xDrive).
    /// </summary>
    /// <param name="articulation">The articulation body to update.</param>
    /// <param name="target">The target position in degrees.</param>
    void SetArticulationTarget(ArticulationBody articulation, float target)
    {
        // Get the current drive settings.
        var drive = articulation.xDrive;
        drive.target = target;
        articulation.xDrive = drive;
    }
}
