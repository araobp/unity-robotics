using TMPro;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;

/// <summary>
/// Controls a robotic arm's movement and grip based on keyboard input.
/// This script manipulates various ArticulationBody components to control the arm.
/// </summary>
public class ArmControllerArticulated : MonoBehaviour
{
    /// <summary>
    /// The GameObject for the slider mechanism.
    /// </summary>
    [SerializeField] GameObject slider;

    /// <summary>
    /// The GameObject for the main arm.
    /// </summary>
    [SerializeField] GameObject arm;

    /// <summary>
    /// The GameObject for the hand.
    /// </summary>
    [SerializeField] GameObject hand;

    /// <summary>
    /// The GameObject for the left finger.
    /// </summary>
    [SerializeField] GameObject fingerL;
    /// <summary>
    /// The GameObject for the right finger.
    /// </summary>
    [SerializeField] GameObject fingerR;

    /// <summary>
    /// The speed at which the arm components move.
    /// </summary>
    [SerializeField] float speed = 0.1f;

    /// <summary>
    /// The speed at which the fingers open and close.
    /// </summary>
    [SerializeField] float gripSpeed = 50.0f;

    /// <summary>
    /// The force threshold for the pressure sensors to stop the grip from closing.
    /// </summary>
    [SerializeField] float targetForce = 1.0f;

    /// <summary>
    /// The small gap between the min and max limits of the articulation body for the fingers.
    /// </summary>
    [SerializeField]
    private float minMaxGap = 0.01f;

    /// <summary>
    /// The denominator used to slow down the grip speed when a collision is detected.
    /// </summary>
    [SerializeField] private float slowDownDenominator = 200.0f;

    /// <summary>
    /// The denominator used to reduce the force feedback effect on the grip.
    /// </summary>
    [SerializeField]
    private float forceReductionDenominator = 500.0f;

    // Articulation bodies for controlling the fingers.
    private ArticulationBody _fingerL;
    private ArticulationBody _fingerR;
    // The target angle for the finger grip.
    private float _targetGripAngle = 0.0f;

    // The angle of the fingers when they are fully closed.
    private float _fingerClosedAngle;
    // The angle of the fingers when they are fully open.
    private float _fingerOpenAngle;

    // Articulation bodies for controlling the arm, hand, and slider.
    private ArticulationBody _armJoint;
    private ArticulationBody _handJoint;
    private ArticulationBody _sliderJoint;

    // Pressure sensors on each finger to detect contact force.
    private PressureSensor _pressureSensorL;
    private PressureSensor _pressureSensorR;

    // Stores the grip angle from the previous frame to detect changes.
    private float _lastGripAngle = 0.0f;


    /// <summary>
    /// UI Text element to display arm's status information.
    /// </summary>
    [SerializeField] TMP_Text textInfo;
    /// <summary>
    /// UI Text element to display the target force.
    /// </summary>
    [SerializeField] TMP_Text textTargetForce;
    /// <summary>
    /// UI Slider to control the target force.
    /// </summary>
    [SerializeField] Slider sliderTargetForce;

    /// <summary>
    /// Initializes components and settings.
    /// </summary>
    void Start()
    {
        _sliderJoint = slider?.GetComponent<ArticulationBody>();
        _armJoint = arm?.GetComponent<ArticulationBody>();
        _handJoint = hand?.GetComponent<ArticulationBody>();

        SetupFinger(fingerL, out _fingerL, out _pressureSensorL);
        SetupFinger(fingerR, out _fingerR, out _pressureSensorR);

        if (_fingerL != null)
        {
            _fingerClosedAngle = _fingerL.xDrive.upperLimit;
            _fingerOpenAngle = _fingerL.xDrive.lowerLimit;
        }

        targetForce = sliderTargetForce.value;
        sliderTargetForce.onValueChanged.AddListener((value) =>
        {
            targetForce = value;
        });
    }


    /// <summary>
    /// Sets up the finger components, including the articulation body and pressure sensor.
    /// </summary>
    private void SetupFinger(GameObject finger, out ArticulationBody body, out PressureSensor sensor)
    {
        body = null;
        sensor = null;
        if (finger == null) return;

        body = finger.GetComponent<ArticulationBody>();
        sensor = finger.GetComponent<PressureSensor>();
    }

    /// <summary>
    /// Updates the UI text with the current status of the arm's joints and sensors each frame.
    /// </summary>
    void Update()
    {
        if (textInfo != null)
        {
            // Calculate average force, mass, and friction from both pressure sensors.
            float forceL = _pressureSensorL?.LastForce ?? 0.0f;
            float forceR = _pressureSensorR?.LastForce ?? 0.0f;
            float force = (forceL + forceR) / 2.0f;
            float mass = ((_pressureSensorL?.LastMass ?? 0.0f) + (_pressureSensorR?.LastMass ?? 0.0f)) / 2.0f;
            float friction = ((_pressureSensorL?.LastFriction ?? 0.0f) + (_pressureSensorR?.LastFriction ?? 0.0f)) / 2.0f;

            // Prepare color codes for UI text based on friction force comparison.
            string whiteCode = ColorUtility.ToHtmlStringRGBA(Color.white);
            string greenCode = ColorUtility.ToHtmlStringRGBA(Color.green);
            string redCode = ColorUtility.ToHtmlStringRGBA(Color.red);

            // Calculate the friction force.
            // The total normal force is `force * 2.0f`.
            // The second `* 2.0f` is an empirical multiplier to better match real-world physics observations.
            float frictionForce = force * friction * 2.0f * 2.0f;
            float gravityForce = mass * Physics.gravity.magnitude;

            // Determine color coding based on friction force relative to gravity force.
            string colorCode;
            if (frictionForce > 0)
            {
                colorCode = frictionForce >= gravityForce ? greenCode : redCode;
            }
            else
            {
                colorCode = whiteCode;
            }

            // Update the UI text with formatted information.
            textInfo.text = $"Grip Angle: {_targetGripAngle:F1}(deg)\n\n" +
                        $"Measured Force:\n" +
                        $"- Force L: {forceL:F1}(N)\n" +
                        $"- Force R: {forceR:F1}(N)\n" +
                        $"- Force Avg: {force:F1}(N)\n\n" +
                        $"Friction Info:\n" +
                        $"Mass(kg): {mass:F1}(kg)\n" +
                        $"Unity Physics μ: {friction:F2}\n" +
                        $"Real Physics μ: {friction * 2.0f:F2}\n" +
                        $"Friction(2μF): <color=#{colorCode}>{frictionForce:F1}</color>(N)\n" +
                        $"Gravity(mg): {gravityForce:F1}(N)";

            // Update the target force display.
            textTargetForce.text = $"Target Force: {targetForce:F1}(N)";
        }
    }

    /// <summary>
    /// Called at a fixed time interval for physics-related updates.
    /// This handles all the physics-based movements of the arm, including joint movements
    /// and grip adjustments based on keyboard input and sensor feedback.
    /// </summary>
    void FixedUpdate()
    {
        // Exit if there is no keyboard input detected.
        if (Keyboard.current == null) return;

        // Update the positions of the slider, arm, and hand joints based on keyboard input.
        UpdateDriveTarget(_sliderJoint, Key.A, Key.D, 'x');
        UpdateDriveTarget(_armJoint, Key.W, Key.S, 'y');
        UpdateDriveTarget(_handJoint, Key.DownArrow, Key.UpArrow, 'z');

        // Adjust the target grip angle based on left and right arrow key presses.
        float gripSpeedAdjustment = _pressureSensorL.IsColliding && _pressureSensorR.IsColliding ? gripSpeed / slowDownDenominator : gripSpeed;
        if (Keyboard.current.leftArrowKey.isPressed)
        {
            _targetGripAngle = Mathf.MoveTowards(_targetGripAngle, _fingerClosedAngle, gripSpeedAdjustment * Time.deltaTime);
        }
        else if (Keyboard.current.rightArrowKey.isPressed)
        {
            _targetGripAngle = Mathf.MoveTowards(_targetGripAngle, _fingerOpenAngle, gripSpeed * Time.deltaTime);
        }

        // Force feedback: if closing and both fingers detect a force greater than the threshold, stop closing.
        if (_targetGripAngle - _lastGripAngle > 0 && _pressureSensorL.LastForce > targetForce && _pressureSensorR.LastForce > targetForce)
        {
            _targetGripAngle = _lastGripAngle - (_pressureSensorL.LastForce + _pressureSensorR.LastForce - 2 * targetForce)
                / forceReductionDenominator;
        }

        // Update the hinge joint limits to reflect the new target grip angle.
        if (_fingerL != null)
        {
            var drive = _fingerL.xDrive;
            drive.lowerLimit = _targetGripAngle - minMaxGap;
            drive.upperLimit = _targetGripAngle;
            _fingerL.xDrive = drive;
        }
        if (_fingerR != null)
        {
            var drive = _fingerR.xDrive;
            drive.lowerLimit = _targetGripAngle - minMaxGap;
            drive.upperLimit = _targetGripAngle;
            _fingerR.xDrive = drive;
        }

        // Update the last grip angle for the next frame's comparison.
        _lastGripAngle = _targetGripAngle;
    }

    /// <summary>
    /// Updates the target of an ArticulationBody's drive to move it along a specified axis.
    /// </summary>
    /// <param name="joint">The joint to be moved.</param>
    /// <param name="negativeKey">The key for movement in the negative direction.</param>
    /// <param name="positiveKey">The key for movement in the positive direction.</param>
    /// <param name="axis">The axis of movement ('x', 'y', or 'z').</param>
    private void UpdateDriveTarget(ArticulationBody joint, Key negativeKey, Key positiveKey, char axis)
    {
        if (joint == null) return;

        ArticulationDrive drive;

        switch (axis)
        {
            case 'y':
                drive = joint.yDrive;
                break;
            case 'z':
                drive = joint.zDrive;
                break;
            default:
                drive = joint.xDrive;
                break;
        }

        float value = drive.target;

        if (Keyboard.current[negativeKey].isPressed && value > drive.lowerLimit)
        {
            value -= speed * Time.deltaTime;
        }
        else if (Keyboard.current[positiveKey].isPressed && value < drive.upperLimit)
        {
            value += speed * Time.deltaTime;
        }

        drive.target = value;

        switch (axis)
        {
            case 'y': joint.yDrive = drive; break;
            case 'z': joint.zDrive = drive; break;
            default: joint.xDrive = drive; break;
        }
    }
}
