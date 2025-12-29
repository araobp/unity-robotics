using TMPro;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;

/// <summary>
/// Controls a robotic arm's movement and grip based on keyboard input.
/// This script manipulates various joints to control different parts of the arm.
/// </summary>
public class ArmController : MonoBehaviour
{
    /// <summary>
    /// The maximum position for the slider joint.
    /// </summary>
    const float SLIDER_MAX = 0.414f;
    
    /// <summary>
    /// The minimum position for the slider joint.
    /// </summary>
    const float SLIDER_MIN = -0.414f;

    /// <summary>
    /// The maximum position for the arm joint.
    /// </summary>
    const float ARM_MAX = 0.377f;
    
    /// <summary>
    /// The minimum position for the arm joint.
    /// </summary>
    const float ARM_MIN = -0.377f;

    /// <summary>
    /// The maximum position for the hand joint.
    /// </summary>
    const float HAND_MAX = 0.158f;

    /// <summary>
    /// The minimum position for the hand joint.
    /// </summary>
    const float HAND_MIN = -0.032f;

    // The GameObject representing the slider mechanism.
    [SerializeField] GameObject slider;

    // The GameObject representing the main arm.
    [SerializeField] GameObject arm;

    // The GameObject representing the hand part of the arm.
    [SerializeField] GameObject hand;

    // GameObjects for the left and right fingers.
    [SerializeField] GameObject fingerL;
    [SerializeField] GameObject fingerR;

    /// <summary>
    /// The speed at which the arm components move.
    /// </summary>
    [SerializeField] float speed = 1.0f; // Speed of movement

    /// <summary>
    /// The small gap between the min and max limits of the hinge joint for the fingers.
    /// </summary>
    [SerializeField]
    private float minMaxGap = 0.01f;

    /// <summary>
    /// The speed at which the fingers open and close.
    /// </summary>
    [SerializeField] float gripSpeed = 50.0f;

    /// <summary>
    /// The angle of the finger joints when the grip is fully closed.
    /// </summary>
    [SerializeField] float closedAngle = 10.0f;

    /// <summary>
    /// The angle of the finger joints when the grip is fully open.
    /// </summary>
    [SerializeField] float openAngle = -20.0f;

    /// <summary>
    /// The force threshold for the pressure sensors to stop the grip from closing.
    /// </summary>
    [SerializeField] float targetForce = 1.0f;

    /// <summary>
    /// The denominator used to slow down the grip speed when a collision is detected.
    /// </summary>
    [SerializeField] private float slowDownDenominator = 200.0f;

    /// <summary>
    /// The denominator used to reduce the force feedback effect on the grip.
    /// </summary>
    [SerializeField]
    private float forceReductionDenominator = 500.0f;

    // Hinge joints for controlling the fingers.
    private HingeJoint _hingeL;
    private HingeJoint _hingeR;
    // The target angle for the finger grip.
    private float _targetGripAngle = 0.0f;

    // Configurable joints for controlling the arm, hand, and slider.
    private ConfigurableJoint _armJoint;
    private ConfigurableJoint _handJoint;
    private ConfigurableJoint _sliderJoint;

    // Pressure sensors on each finger to detect contact force.
    private PressureSensor _pressureSensorL;
    private PressureSensor _pressureSensorR;

    // Stores the grip angle from the previous frame to detect changes.
    private float _lastGripAngle = 0.0f;


    // UI Text element to display arm's status information.
    [SerializeField] TMP_Text textInfo;
    [SerializeField] TMP_Text textTargetForce;
    [SerializeField] Slider sliderTargetForce;

    /// <summary>
    /// Called once before the first frame update. Initializes components and settings.
    /// </summary>
    void Start()
    {
        _sliderJoint = slider?.GetComponent<ConfigurableJoint>();
        _armJoint = arm?.GetComponent<ConfigurableJoint>();
        _handJoint = hand?.GetComponent<ConfigurableJoint>();

        SetupFinger(fingerL, out _hingeL, out _pressureSensorL);
        SetupFinger(fingerR, out _hingeR, out _pressureSensorR);

        targetForce = sliderTargetForce.value;
        sliderTargetForce.onValueChanged.AddListener((value) =>
        {
            targetForce = value;
        });
    }


    /// <summary>
    /// Sets up the finger components, including the hinge joint and pressure sensor.
    /// </summary>
    private void SetupFinger(GameObject finger, out HingeJoint hinge, out PressureSensor sensor)
    {
        hinge = null;
        sensor = null;
        if (finger == null) return;

        hinge = finger.GetComponent<HingeJoint>();
        sensor = finger.GetComponent<PressureSensor>();
    }

    /// <summary>
    /// Updates the UI text with the current status of the arm's joints and sensors.
    /// </summary>
    /// <remarks>
    /// This method is called every frame to display real-time information about the arm's state, including forces from the pressure sensors and calculated friction data.
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
                        $"- Force L: {_pressureSensorL.LastForce:F1}(N)\n" +
                        $"- Force R: {_pressureSensorR.LastForce:F1}(N)\n" +
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
    /// Called at a fixed time interval, used for physics-related updates.
    /// </summary>
    /// <remarks>
    /// This handles all the physics-based movements of the arm, including joint movements and grip adjustments based on keyboard input and sensor feedback.
    /// </summary>
    void FixedUpdate()
    {
        // Exit if there is no keyboard input detected.
        if (Keyboard.current == null) return;

        // Update the positions of the slider, arm, and hand joints based on keyboard input.
        UpdateJointAnchor(_sliderJoint, Key.A, Key.D, SLIDER_MIN, SLIDER_MAX, 'x');
        UpdateJointAnchor(_armJoint, Key.W, Key.S, ARM_MIN, ARM_MAX, 'y');
        UpdateJointAnchor(_handJoint, Key.DownArrow, Key.UpArrow, HAND_MIN, HAND_MAX, 'z');

        // Adjust the target grip angle based on left and right arrow key presses.
        float gripSpeedAdjustment = _pressureSensorL.IsColliding && _pressureSensorR.IsColliding ? gripSpeed / slowDownDenominator : gripSpeed;
        if (Keyboard.current.leftArrowKey.isPressed)
        {
            _targetGripAngle = Mathf.MoveTowards(_targetGripAngle, closedAngle, gripSpeedAdjustment * Time.deltaTime);
        }
        else if (Keyboard.current.rightArrowKey.isPressed)
        {
            _targetGripAngle = Mathf.MoveTowards(_targetGripAngle, openAngle, gripSpeed * Time.deltaTime);
        }

        // Force feedback: if closing and both fingers detect a force greater than the threshold, stop closing.
        if (_targetGripAngle - _lastGripAngle > 0 && _pressureSensorL.LastForce > targetForce && _pressureSensorR.LastForce > targetForce)
        {
            _targetGripAngle = _lastGripAngle - (_pressureSensorL.LastForce + _pressureSensorR.LastForce - 2 * targetForce)
                / forceReductionDenominator;
        }

        // Update the hinge joint limits to reflect the new target grip angle.
        SetHingeLimits(_hingeL, _targetGripAngle);
        SetHingeLimits(_hingeR, _targetGripAngle);

        // Update the last grip angle for the next frame's comparison.
        _lastGripAngle = _targetGripAngle;
    }

    /// <summary>
    /// Updates the connected anchor of a ConfigurableJoint to move it along a specified axis.
    /// </summary>
    /// <param name="joint">The joint to be moved.</param>
    /// <param name="negativeKey">The key for movement in the negative direction.</param>
    /// <param name="positiveKey">The key for movement in the positive direction.</param>
    /// <param name="min">The minimum allowed value for the anchor position on the axis.</param>
    /// <param name="max">The maximum allowed value for the anchor position on the axis.</param>
    /// <param name="axis">The axis of movement ('x', 'y', or 'z').</param>
    private void UpdateJointAnchor(ConfigurableJoint joint, Key negativeKey, Key positiveKey, float min, float max, char axis)
    {
        if (joint == null) return;

        var anchor = joint.connectedAnchor;
        float value = 0;

        switch (axis)
        {
            case 'x': value = anchor.x; break;
            case 'y': value = anchor.y; break;
            case 'z': value = anchor.z; break;
        }

        if (Keyboard.current[negativeKey].isPressed && value > min)
        {
            value -= speed * Time.deltaTime;
        }
        else if (Keyboard.current[positiveKey].isPressed && value < max)
        {
            value += speed * Time.deltaTime;
        }

        switch (axis)
        {
            case 'x': anchor.x = value; break;
            case 'y': anchor.y = value; break;
            case 'z': anchor.z = value; break;
        }
        joint.connectedAnchor = anchor;
    }

    /// <summary>
    /// Sets the limits of a HingeJoint to control the finger's angle.
    /// </summary>
    /// <param name="hinge">The HingeJoint to be configured.</param>
    /// <param name="angle">The target angle for the joint.</param>
    private void SetHingeLimits(HingeJoint hinge, float angle)
    {
        if (hinge == null || !hinge.useLimits) return;

        var limits = hinge.limits;
        limits.min = angle - minMaxGap;
        limits.max = angle;
        hinge.limits = limits;
    }
}
