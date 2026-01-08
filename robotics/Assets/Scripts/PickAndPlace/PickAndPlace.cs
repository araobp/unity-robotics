using UnityEngine;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Generic;
using System.Linq;
using UnityEngine.UI;
using TMPro;
using System.Runtime.CompilerServices;

/// <summary>
/// This class manages the behavior of a 6-axis robot arm, including inverse kinematics (IK) for movement,
/// object detection using a camera and the Gemini API, and pick-and-place operations. It provides UI controls
/// for user interaction, such as detecting objects and executing pick-and-place sequences based on either
/// pre-defined targets or natural language commands.
/// </summary>
public class PickAndPlace : MonoBehaviour
{
    // --- Enums ---
    public enum InterpolationType
    {
        Joint,
        Linear
    }

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
    [Tooltip("The interpolation method to use for movement.")]
    [SerializeField] private InterpolationType interpolationType = InterpolationType.Linear;

    [Tooltip("The speed of the end effector in meters per second during a linear IK movement.")]
    [SerializeField] private float ikLinearSpeed = 0.01f;

    [Tooltip("The angular speed of the fastest joint in degrees per second during a joint-based IK movement.")]
    [SerializeField] private float ikJointAngularSpeed = 60.0f;

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


    [Header("Menu and Panels")]
    [SerializeField] TMP_Dropdown dropdownMenu;
    [SerializeField] GameObject panelObjectDetection;
    [SerializeField] GameObject panelChat;
    // UI elements for displaying coordinates.
    [Header("Coordinate Display")]
    [SerializeField] Coordinate coordinateEndEffector;
    [SerializeField] Coordinate coordinatePick;
    [SerializeField] Coordinate coordinatePlace;

    [Header("Chat UI")]
    [SerializeField] TMP_InputField chatInputField;
    [SerializeField] TMP_InputField chatOutputField;

    // The Transform representing the precise point of interaction for the end effector,
    // often located at the center of the gripper's fingers. The IK solver targets this point.
    private Transform _toolCenterPoint;

    // The last recorded local position of the end effector edge, set by setPose().
    private Vector3 _endEffectorEdgeRestPosition;

    // The last commanded target position for the end effector. This is used to ensure smooth transitions
    // between movements by starting new trajectories from the last intended destination, rather than the
    // current physical position, which may have a slight "gravity sag".
    private Vector3 _lastTargetPosition;

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
    private GeminiRoboticsApi _geminiRoboticsApi;

    // A collection to store the workpieces detected by the vision system.
    private IEnumerable<DetectedObject> _detectedWorkpieces;

    // Used to cancel the in-progress asynchronous movement task.
    private CancellationTokenSource _ikMoveCts;

    // Cached ArticulationBody components for each joint.
    private ArticulationBody _swingAb;
    private ArticulationBody _boomAb;
    private ArticulationBody _armAb;
    private ArticulationBody _handAb;
    private ArticulationBody _wristAb;

    // Robot arm segment lengths (in meters).
    const float AB = 0.1689f;
    const float CD = 0.27312f;

    // FB = HAND_AND_WRITE_SIZE + _toolSize
    const float HAND_AND_WRISTE_SIZE = 0.17259f;

    // Tool size attached to the wrist of the end effector
    private float _toolSize = 0f;

    // A small vertical offset added to target positions to ensure the gripper doesn't collide with the table
    // or the object itself. This margin provides a safe clearance.
    Vector3 TCP_MARGIN = new Vector3(0f, 0.01f, 0f);
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
        _swingAb = swing.GetComponent<ArticulationBody>();
        _boomAb = boom.GetComponent<ArticulationBody>();
        _armAb = arm.GetComponent<ArticulationBody>();
        _handAb = hand.GetComponent<ArticulationBody>();
        _wristAb = wrist.GetComponent<ArticulationBody>();

        // Get the end effector size from the ParallelGripper component.
        _toolCenterPoint = endEffector.GetComponent<IEndEffector>().ToolCenterPoint;
        _toolSize = (wrist.transform.position - _toolCenterPoint.position).magnitude;
        Debug.Log("End Effector Size: " + _toolSize);

        // Instantiate the GeminiRoboticsApi
        _geminiRoboticsApi = new GeminiRoboticsApi();

        // Add a listener to the detect button to trigger the object detection process.
        buttonDetect.onClick.AddListener(OnDetectButtonClicked);
        buttonPickAndPlace.onClick.AddListener(OnPickAndPlaceButtonClicked);
        chatInputField.onSubmit.AddListener(OnSubmitChat);

        // Add a listener to the dropdown menu to handle panel switching.
        dropdownMenu.onValueChanged.AddListener(OnDropdownValueChanged);
        // Set the initial state of the panels based on the dropdown's default value.
        OnDropdownValueChanged(dropdownMenu.value);

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
    }

    /// <summary>
    /// Called every frame. Updates the UI to display the current coordinates of the end effector (wrist).
    /// </summary>
    void Update()
    {
        // Get the current position of the wrist relative to the robot base and update the UI display.
        coordinateEndEffector.UpdatePositionText(workArea.transform.InverseTransformPoint(_toolCenterPoint.position));
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
    /// Handles the event when the dropdown menu selection changes. It activates and deactivates
    /// UI panels based on the selected option.
    /// </summary>
    /// <param name="index">The index of the selected option in the dropdown.</param>
    private void OnDropdownValueChanged(int index)
    {
        string selectedOption = dropdownMenu.options[index].text;

        if (selectedOption == "Object Detection")
        {
            if (panelObjectDetection != null) panelObjectDetection.SetActive(true);
            if (panelChat != null) panelChat.SetActive(false);
        }
        else if (selectedOption == "Chat")
        {
            if (panelObjectDetection != null) panelObjectDetection.SetActive(false);
            if (panelChat != null) panelChat.SetActive(true);
        }
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
        await Task.Delay(delay);

        // Cancel any existing movement task before starting a new one.
        if (_ikMoveCts != null)
        {
            _ikMoveCts.Cancel();
            _ikMoveCts.Dispose();
        }
        _ikMoveCts = new CancellationTokenSource();

        Debug.Log($"Performing IK...: {interpolationType}");
        if (interpolationType == InterpolationType.Linear)
        {
            await MoveToTargetLinearly(targetPosition, ikLinearSpeed, _ikMoveCts.Token);
        }
        else // Joint interpolation
        {
            // Determine the closest IK solution family at the start of the movement.
            var currentArmAngle = _armAb.xDrive.target;
            (float _, float _, float normalArmAngle, float _) = CalculateIKAngles(targetPosition, useAlternate: false);
            (float _, float _, float alternateArmAngle, float _) = CalculateIKAngles(targetPosition, useAlternate: true);

            bool useAlternate = Mathf.Abs(currentArmAngle - alternateArmAngle) < Mathf.Abs(currentArmAngle - normalArmAngle);


            (float swingTarget, float boomTarget, float armTarget, float handTarget) = CalculateIKAngles(targetPosition, useAlternate);

            float moveDuration = duration;
            if (moveDuration <= 0 && ikJointAngularSpeed > 0)
            {
                // Calculate the duration based on the largest joint angle change required.
                // This ensures a more consistent speed across different types of movements.
                float swingStart = _swingAb.xDrive.target;
                float boomStart = _boomAb.xDrive.target;
                float armStart = _armAb.xDrive.target;
                float handStart = _handAb.xDrive.target;

                float maxAngleChange = 0f;
                maxAngleChange = Mathf.Max(maxAngleChange, Mathf.Abs(swingTarget - swingStart));
                maxAngleChange = Mathf.Max(maxAngleChange, Mathf.Abs(boomTarget - boomStart));
                maxAngleChange = Mathf.Max(maxAngleChange, Mathf.Abs(armTarget - armStart));
                maxAngleChange = Mathf.Max(maxAngleChange, Mathf.Abs(handTarget - handStart));
                moveDuration = maxAngleChange / ikJointAngularSpeed;
            }

            await MoveToTargets(swingTarget, boomTarget, armTarget, handTarget, moveDuration, _ikMoveCts.Token);
            _lastTargetPosition = targetPosition;
        }
    }

    /// <summary>
    /// Handles the submission of a command from the chat input field. It captures an image,
    /// sends it along with the user's instruction to the Gemini API to get movement commands,
    /// and then executes the pick-and-place sequence for each move.
    /// </summary>
    /// <param name="instruction">The natural language instruction from the user.</param>
    private async void OnSubmitChat(string instruction)
    {
        if (chatOutputField != null) chatOutputField.text = "";

        if (string.IsNullOrWhiteSpace(instruction))
        {
            Debug.LogWarning("Instruction is empty.");
            if (chatOutputField != null) chatOutputField.text += "Please enter an instruction!";
            return;
        }

        if (cameraCapture == null || _geminiRoboticsApi == null)
        {
            Debug.LogError("CameraCapture or GeminiRoboticsApi is not assigned.");
            return;
        }

        // Clear previous detections and display the new ones.
        detectedPoints.clear();

        if (chatOutputField != null)
        {
            chatInputField.text = "";
            chatOutputField.text += $"Processing instruction: \"{instruction}\"\n";
        }

        string b64Image = cameraCapture.CaptureAsBase64();
        var moves = await _geminiRoboticsApi.DetectAndMoveObjects(b64Image, instruction);

        if (moves != null && moves.Any())
        {
            if (chatOutputField != null) chatOutputField.text += $"Found {moves.Length} moves. Executing...\n";
            foreach (var move in moves)
            {
                detectedPoints.displayMove(move);
                Vector3 pickPos = cameraCapture.ProjectToWorkAreaLocal(move.from.x / 1000f * cameraCapture.ImageWidth, move.from.y / 1000f * cameraCapture.ImageHeight);
                Vector3 placePos = cameraCapture.ProjectToWorkAreaLocal(move.to.x / 1000f * cameraCapture.ImageWidth, move.to.y / 1000f * cameraCapture.ImageHeight);

                Debug.Log($"[PickAndPlace] Move '{move.label}': from {pickPos} to {placePos}");
                if (chatOutputField != null) chatOutputField.text += $"  - Moving '{move.label}' from {pickPos:F2} to {placePos:F2}.\n";
                await PerformPickAndPlace(pickPos, placePos);
            }
            if (chatOutputField != null) chatOutputField.text += "Finished executing moves.\n";
        }
        else
        {
            if (chatOutputField != null) chatOutputField.text += "Sorry, I couldn't determine any moves from your instruction.\n";
        }
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
        if (cameraCapture == null || _geminiRoboticsApi == null)
        {
            Debug.LogError("CameraCapture or GeminiRoboticsApi is not assigned.");
            return;
        }

        // Capture the image from the camera as a base64 string.
        string b64Image = cameraCapture.CaptureAsBase64();

        // Send the image to the Gemini API and wait for the detected objects.
        var objectObjects = await _geminiRoboticsApi.DetectObjects(b64Image);

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
        if (detectedPoints.TryGetDetectedObjects(detectionTargetLabel, out _detectedWorkpieces))
        {
            Debug.Log($"Found target workpiece: {_detectedWorkpieces.First().label} at ({_detectedWorkpieces.First().point.x}, {_detectedWorkpieces.First().point.y})");
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
        if (_detectedWorkpieces != null && _detectedWorkpieces.Any())
        {
            // Iterate through each detected object.
            foreach (var obj in _detectedWorkpieces)
            {
                Debug.Log($"Workpiece: {obj.label} at ({obj.point.x}, {obj.point.y})");
                // Convert normalized 0-1000 coordinates to pixel coordinates.
                float u = obj.point.x / 1000f * cameraCapture.ImageWidth;
                float v = obj.point.y / 1000f * cameraCapture.ImageHeight;

                // Project the 2D pixel coordinates to a 3D position in the robot's work area.
                Vector3 workpiecePos = cameraCapture.ProjectToWorkAreaLocal(u, v);
                Debug.Log($"[PickAndPlace] World Position of detected object '{obj.label}': {workpiecePos}");
                // Start the pick-and-place sequence for the current object.
                await PerformPickAndPlace(workpiecePos, placeTarget.transform.localPosition);
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
        // 1. Get the current local position of the 'work' object (the item to be picked)
        Vector3 pickPos = work.transform.localPosition;

        // 2. Adjust the vertical coordinate (y) to 0. 
        // This ensures the IK target is set to the base of the object/ground level 
        // rather than the center of the object's mesh.
        pickPos = new Vector3(pickPos.x, 0f, pickPos.z);

        // 3. Execute the asynchronous Pick and Place logic.
        // Moves from the calculated ground-level pick position to the local position of 'placeTarget'.
        await PerformPickAndPlace(pickPos, placeTarget.transform.localPosition);
    }

    /// <summary>
    /// Executes a full pick-and-place sequence for a single object. The sequence involves:
    /// 1. Opening the gripper to prepare for picking.
    /// 2. Moving to a safe position above the pick location.
    /// 3. Moving down to the pick location.
    /// 4. Closing the gripper to grasp the object.
    /// 5. Lifting the object to a safe height.
    /// 6. Moving to a safe height above the place location.
    /// 7. Moving down to the place location.
    /// 8. Opening the gripper to release the object.
    /// 9. Moving back up to a safe height.
    /// </summary>
    /// <param name="pickPos">The position where the object will be picked from.</param>
    /// <param name="placePos">The position where the object will be placed.</param>
    private async Task PerformPickAndPlace(Vector3 pickPos, Vector3 placePos)
    {
        // 1. Move to a safe position above the object.
        Vector3 pickPositionAbove = new Vector3(pickPos.x, pickPos.y + 0.2f, pickPos.z);
        await PerformIK(pickPositionAbove);

        // 2. Open the gripper to prepare for picking.
        await Open(targetAngularVelocity);

        // 3. Move down to the object to be picked.
        Vector3 pickPositionAt = new Vector3(pickPos.x, pickPos.y, pickPos.z) + TCP_MARGIN;
        await PerformIK(pickPositionAt);

        // 4. Close the gripper to pick up the object.
        await Close(targetForce, targetAngularVelocity);
        coordinatePick.UpdatePositionText(workArea.transform.InverseTransformPoint(_toolCenterPoint.position));

        // 5. Lift the object to a safe height.
        Vector3 pickPositionLift = new Vector3(pickPos.x, pickPos.y + 0.4f, pickPos.z);
        await PerformIK(pickPositionLift);

        // 6. Move to a safe height above the place position.
        Vector3 placePositionLift = new Vector3(placePos.x, placePos.y + 0.4f, placePos.z);
        await PerformIK(placePositionLift);

        // 7. Move down to the place position.
        Vector3 placePositionAbove = new Vector3(placePos.x, placePos.y + 0.15f, placePos.z) + TCP_MARGIN;
        await PerformIK(placePositionAbove);

        // 8. Open the gripper to release the object.
        await Open(targetAngularVelocity);
        coordinatePlace.UpdatePositionText(workArea.transform.InverseTransformPoint(_toolCenterPoint.position));

        // 9. Move back to the safe height above the place position.
        await PerformIK(placePositionLift);

        // 10. Close the gripper.
        await Close(targetForce, targetAngularVelocity);

        // 11. Move back to the rest position.
        await PerformIK(_endEffectorEdgeRestPosition);
    }

    /// <summary>
    /// Instantly sets the pose of the robot arm's articulations to the specified angles without animation,
    /// memorizes the end effector's position, and then opens the gripper.
    /// </summary>
    /// <param name="swingAngle">The angle for the swing articulation in radians.</param>
    /// <param name="boomAngle">The angle for the boom articulation in radians.</param>
    /// <param name="armAngle">The angle for the arm articulation in radians.</param>
    /// <param name="handAngle">The angle for the hand articulation in radians.</param>
    /// <param name="wristAngle">The angle for the wrist articulation in radians.</param>
    private async Task setPose(float swingAngle, float boomAngle, float armAngle, float handAngle, float wristAngle)
    {
        // Set the target angle for each articulation body, converting from radians to degrees.
        SetArticulationTarget(_swingAb, -swingAngle * Mathf.Rad2Deg);
        SetArticulationTarget(_boomAb, -boomAngle * Mathf.Rad2Deg);
        SetArticulationTarget(_armAb, armAngle * Mathf.Rad2Deg);
        SetArticulationTarget(_handAb, handAngle * Mathf.Rad2Deg);
        SetArticulationTarget(_wristAb, -wristAngle * Mathf.Rad2Deg);
        // Wait for a short period to allow the joints to settle.
        await Task.Delay(100);

        // Memorize the end effector's local position after setting the pose.
        _endEffectorEdgeRestPosition = workArea.transform.InverseTransformPoint(_toolCenterPoint.position);
        _lastTargetPosition = _endEffectorEdgeRestPosition;
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
        float swingStart = _swingAb.xDrive.target;
        float boomStart = _boomAb.xDrive.target;
        float armStart = _armAb.xDrive.target;
        float handStart = _handAb.xDrive.target;

    // Unwrap the swing angle to ensure the shortest path is taken.
    while (Mathf.Abs(swingTarget - swingStart) > 180.0f)
    {
        if (swingTarget > swingStart)
            swingTarget -= 360.0f;
        else
            swingTarget += 360.0f;
    }

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
                SetArticulationTarget(_swingAb, swingRotation);
                SetArticulationTarget(_boomAb, Mathf.Lerp(boomStart, boomTarget, t));
                SetArticulationTarget(_armAb, Mathf.Lerp(armStart, armTarget, t));
                SetArticulationTarget(_handAb, Mathf.Lerp(handStart, handTarget, t));
                // Adjust the wrist to keep it aligned (e.g., parallel to the table).
                SetArticulationTarget(_wristAb, isAlignedToTable ? swingRotation - 90 : swingRotation);

                elapsedTime += Time.deltaTime;
                // Wait for the next frame before continuing the loop.
                await Task.Yield();
            }

            // Snap to the final target angles to ensure precision.
            SetArticulationTarget(_swingAb, swingTarget);
            SetArticulationTarget(_boomAb, boomTarget);
            SetArticulationTarget(_armAb, armTarget);
            SetArticulationTarget(_handAb, handTarget);
            SetArticulationTarget(_wristAb, isAlignedToTable ? swingTarget - 90 : swingTarget);
        }
        catch (TaskCanceledException)
        {
            // Gracefully handle task cancellation (e.g., if a new movement command interrupts this one).
        }
    }

    /// <summary>
    /// Moves the robot's end-effector in a straight line from its current position to the target position.
    /// This is achieved by calculating a series of intermediate points along the line and solving the
    /// inverse kinematics for each point.
    /// </summary>
    /// <param name="targetPosition">The destination position in the robot's local space.</param>
    /// <param name="speed">The desired speed of the end-effector in meters per second.</param>
    /// <param name="token">A cancellation token to stop the movement prematurely.</param>
    private async Task MoveToTargetLinearly(Vector3 targetPosition, float speed, CancellationToken token)
    {
        // We read the arm's current physical position to determine the starting point of the movement.
        Vector3 startPosition = workArea.transform.InverseTransformPoint(this._toolCenterPoint.position);

        // --- Gravity Sag Compensation ---
        // If the robot arm is holding a position, its actual physical location might be slightly
        // lower than its commanded target position due to "gravity sag" or "droop".
        // If we start a new movement from this slightly sagged physical position, the physics controller
        // might lose its "holding force", causing a small drop or jerk at the start of the new movement.
        // To prevent this, we check if the current physical position is reasonably close to the last
        // *commanded* target position. If it is, we start the new movement from that last commanded
        // target, ensuring a smooth, continuous trajectory without the jerk.
        if (Vector3.Distance(startPosition, _lastTargetPosition) < 0.1f)
        {
            startPosition = _lastTargetPosition;
        }

        float distance = Vector3.Distance(startPosition, targetPosition);

        // --- IK Solution Continuity ---
        // For any given target position, there are two possible "elbow up" and "elbow down" solutions.
        // To prevent the arm from suddenly "flipping" between these solutions during a movement,
        // we determine which solution family is closer to the arm's current pose at the very
        // beginning of the movement. We then stick with that solution for the entire trajectory.
        var currentArmAngle = _armAb.xDrive.target;
        (float _, float _, float normalArmAngle, float _) = CalculateIKAngles(startPosition, useAlternate: false);
        (float _, float _, float alternateArmAngle, float _) = CalculateIKAngles(startPosition, useAlternate: true);

        bool useAlternate = Mathf.Abs(currentArmAngle - alternateArmAngle) < Mathf.Abs(currentArmAngle - normalArmAngle);

        if (distance <= 0.001f)
        {
            return;
        }

        // Calculate duration based on distance and speed.
        // If speed is not positive, snap to target instantly.
        float duration = (speed > 0) ? distance / speed : 0;

        if (duration <= 0)
        {
            (float finalSwing, float finalBoom, float finalArm, float finalHand) = CalculateIKAngles(targetPosition, useAlternate);
            SetArticulationTarget(_swingAb, finalSwing);
            SetArticulationTarget(_boomAb, finalBoom);
            SetArticulationTarget(_armAb, finalArm);
            SetArticulationTarget(_handAb, finalHand);
            SetArticulationTarget(_wristAb, isAlignedToTable ? finalSwing - 90 : finalSwing);
            _lastTargetPosition = targetPosition;
            return;
        }

        float elapsedTime = 0f;
        float previousSwingTarget = _swingAb.xDrive.target; // Initialize with the current angle

        try
        {
            while (elapsedTime < duration)
            {
                token.ThrowIfCancellationRequested();

                float t = elapsedTime / duration;
                if (useEaseinEaseout)
                {
                    t = Mathf.SmoothStep(0f, 1f, t);
                }

                Vector3 intermediatePoint = Vector3.Lerp(startPosition, targetPosition, t);

                (float swingTarget, float boomTarget, float armTarget, float handTarget) = CalculateIKAngles(intermediatePoint, useAlternate);

                // --- Angle Unwrapping for Swing Joint ---
                // The Atan2 function in the IK calculation returns an angle between -180 and +180 degrees.
                // If the robot's path crosses this 180-degree seam, the target angle can suddenly jump
                // from +179 to -179 degrees. A simple interpolation would then try to move the arm the
                // "long way around" (a nearly 360-degree turn). This "unwrapping" logic ensures the
                // robot always takes the shortest path for the swing joint.
                while (Mathf.Abs(swingTarget - previousSwingTarget) > 180.0f)
                {
                    if (swingTarget > previousSwingTarget)
                        swingTarget -= 360.0f;
                    else
                        swingTarget += 360.0f;
                }
                previousSwingTarget = swingTarget;

                // Set targets directly without using MoveToTargets to avoid nested async loops and ensure linear path
                SetArticulationTarget(_swingAb, swingTarget);
                SetArticulationTarget(_boomAb, boomTarget);
                SetArticulationTarget(_armAb, armTarget);
                SetArticulationTarget(_handAb, handTarget);
                SetArticulationTarget(_wristAb, isAlignedToTable ? swingTarget - 90 : swingTarget);

                elapsedTime += Time.deltaTime;
                await Task.Yield();
            }

            // Final movement to the target to ensure it reaches the destination
            (float finalSwing, float finalBoom, float finalArm, float finalHand) = CalculateIKAngles(targetPosition, useAlternate);
            SetArticulationTarget(_swingAb, finalSwing);
            SetArticulationTarget(_boomAb, finalBoom);
            SetArticulationTarget(_armAb, finalArm);
            SetArticulationTarget(_handAb, finalHand);
            SetArticulationTarget(_wristAb, isAlignedToTable ? finalSwing - 90 : finalSwing);
            _lastTargetPosition = targetPosition;
        }
        catch (TaskCanceledException)
        {
            // Gracefully handle task cancellation
        }
    }

    // --- Private Helper Methods ---

    /// <summary>
    /// Calculates the necessary joint angles for the robot arm to reach a given target position
    /// using 2D planar inverse kinematics.
    /// </summary>
    /// <param name="targetPosition">The target position for the end effector in the robot's local space.</param>
    /// <returns>A tuple containing the calculated target angles (in degrees) for the swing, boom, arm, and hand joints.</returns>
    private (float swing, float boom, float arm, float hand) CalculateIKAngles(Vector3 targetPosition, bool useAlternate = false)
    {
        Vector3 A = targetPosition;

        // --- 2D Planar Inverse Kinematics Calculation --- //
        float theta1 = Mathf.Atan2(A.z, A.x);
        float AC = Mathf.Sqrt(A.x * A.x + A.z * A.z);
        float val = AB / AC;
        if (val > 1.0f || val < -1.0f)
        {
            Debug.LogWarning($"Target position {targetPosition} is unreachable. AC is too small.");
            // Return current angles to avoid movement
            return (_swingAb.xDrive.target, _boomAb.xDrive.target, _armAb.xDrive.target, _handAb.xDrive.target);
        }
        float theta3 = Mathf.Asin(val);
        float BC = AC * Mathf.Cos(theta3);
        float theta2 = theta1 - theta3;
        float GF = HAND_AND_WRISTE_SIZE + _toolSize - CD + A.y;
        float r = Mathf.Sqrt(BC * BC + GF * GF);

        // Check if the target is reachable
        if (r > FE + ED)
        {
            Debug.LogWarning($"Target position {targetPosition} is unreachable. r ({r}) > FE ({FE}) + ED ({ED})");
            // Return current angles to avoid movement
            return (_swingAb.xDrive.target, _boomAb.xDrive.target, _armAb.xDrive.target, _handAb.xDrive.target);
        }

        // Use law of cosines to solve for the angles of the triangle formed by the boom, arm, and the line 'r'.
        // Clamp the arguments to Acos to prevent NaN errors due to floating point inaccuracies.
        float theta6 = Mathf.Acos(Mathf.Clamp((FE * FE - ED * ED - r * r) / (-2 * ED * r), -1.0f, 1.0f));
        float theta7 = Mathf.Acos(Mathf.Clamp((r * r - FE * FE - ED * ED) / (-2 * FE * ED), -1.0f, 1.0f));

        // The IK problem has two solutions (elbow up/down). 'useAlternate' allows us to choose one.
        // We negate the angles to get the other valid configuration.
        if (useAlternate)
        {
            theta6 = -theta6;
            theta7 = -theta7;
        }
        float theta5 = Mathf.Atan2(GF, BC);
        float theta4 = theta5 + theta6;
        float theta8 = 3 * Mathf.PI / 2 - theta4 - theta7;

        return (-theta2 * Mathf.Rad2Deg, -theta4 * Mathf.Rad2Deg, theta7 * Mathf.Rad2Deg, theta8 * Mathf.Rad2Deg);
    }

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
