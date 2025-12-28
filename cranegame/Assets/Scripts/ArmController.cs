using TMPro;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;

/// <summary>
/// Controls a robotic arm's movement and grip using keyboard input.
/// This script manipulates various joints to control different parts of the arm.
/// </summary>
public class ArmController : MonoBehaviour
{
    // The GameObject representing the hand part of the arm.
    [SerializeField] GameObject hand;
    /// <summary>
    /// The speed at which the arm components move.
    /// </summary>
    [SerializeField] float speed = 1.0f; // Speed of movement
    // The GameObject representing the main arm.
    [SerializeField] GameObject arm;

    // The GameObject representing the slider mechanism.
    [SerializeField] GameObject slider;

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

    // GameObjects for the left and right fingers.
    [SerializeField] GameObject fingerL;
    [SerializeField] GameObject fingerR;

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

    /// <summary>
    /// Updates the UI text with the current status of the arm's joints and sensors.
    /// </summary>
    void Update()
    {
        if (textInfo != null && _sliderJoint != null && _armJoint != null && _handJoint != null)
        {
            float forceL = _pressureSensorL != null ? _pressureSensorL.LastForce : 0.0f;
            float forceR = _pressureSensorR != null ? _pressureSensorR.LastForce : 0.0f;
            float force = (forceL + forceR) / 2.0f;
            float mass = (_pressureSensorL != null && _pressureSensorR != null) ?
                (_pressureSensorL.LastMass + _pressureSensorR.LastMass) / 2.0f : 0.0f;

            // Assuming the real friction value in the real world is multiplied by 2
            float friction = _pressureSensorL != null && _pressureSensorR != null ?
                (_pressureSensorL.LastFriction + _pressureSensorR.LastFriction) / 2.0f * 2.0f : 0.0f;

            string whiteCode = ColorUtility.ToHtmlStringRGBA(Color.white);
            string greenCode = ColorUtility.ToHtmlStringRGBA(Color.green);
            string redCode = ColorUtility.ToHtmlStringRGBA(Color.red);

            float frictionForce = force * friction * 2.0f;
            float gravityForce = mass * Physics.gravity.magnitude;

            string colorCode;
            if (frictionForce > 0)
            {
                colorCode = frictionForce >= gravityForce ? greenCode : redCode;
            }
            else
            {
                colorCode = whiteCode;
            }

            textInfo.text = $"Grip Angle: {_targetGripAngle:F1}(deg)\n\n" +
                        $"Measured Force:\n" +
                        $"- Force L: {_pressureSensorL.LastForce:F1}(N)\n" +
                        $"- Force R: {_pressureSensorR.LastForce:F1}(N)\n" +
                        $"- Force Avg: {force:F1}(N)\n\n" +
                        $"Friction Info:\n" +
                        $"Mass(kg): {mass:F1}(kg)\n" +
                        $"Friction Coef(μ): {friction:F1}\n" +
                        $"Friction(2μF): <color=#{colorCode}>{frictionForce:F1}</color>(N) \n" + // Assuming the real friction value in the real world is multiplied by 2
                        $"Gravity(mg): {gravityForce:F1}(N)";

            textTargetForce.text = $"Target Force: {targetForce:F1}(N)";
        }
    }

    /// <summary>
    /// Called once before the first frame update. Initializes components and settings.
    /// </summary>
    void Start()
    {
        // Set a fixed time step for physics calculations to ensure consistent behavior.
        Time.fixedDeltaTime = 0.01f;

        // Get the ConfigurableJoint component from the slider GameObject.
        if (slider != null)
        {
            _sliderJoint = slider.GetComponent<ConfigurableJoint>();
        }

        // Get the ConfigurableJoint component from the arm GameObject.
        if (arm != null)
        {
            _armJoint = arm.GetComponent<ConfigurableJoint>();
        }

        // Get the ConfigurableJoint component from the hand GameObject.
        if (hand != null)
        {
            _handJoint = hand.GetComponent<ConfigurableJoint>();
        }

        // Get HingeJoint and PressureSensor components from the left finger.
        if (fingerL != null)
        {
            _hingeL = fingerL.GetComponent<HingeJoint>();
            _pressureSensorL = fingerL.GetComponent<PressureSensor>();
        }
        // Get HingeJoint and PressureSensor components from the right finger.
        if (fingerR != null)
        {
            _hingeR = fingerR.GetComponent<HingeJoint>();
            _pressureSensorR = fingerR.GetComponent<PressureSensor>();
        }

        targetForce = sliderTargetForce.value;
        sliderTargetForce.onValueChanged.AddListener((value) =>
        {
            targetForce = value;
        });
    }

    /// <summary>
    /// Called at a fixed time interval, used for physics-related updates.
    /// </summary>
    void FixedUpdate()
    {
        // Exit if there is no keyboard input detected.
        if (Keyboard.current == null) return;

        if (_sliderJoint != null)
        {
            // Move the slider along the x-axis based on 'A' and 'D' key presses.
            var anchor = _sliderJoint.connectedAnchor;
            if (Keyboard.current.aKey.isPressed)
            {
                if (anchor.x <= SLIDER_MIN) return;
                anchor.x -= speed * Time.deltaTime;
            }
            else if (Keyboard.current.dKey.isPressed)
            {
                if (anchor.x >= SLIDER_MAX) return;
                anchor.x += speed * Time.deltaTime;

            }
            _sliderJoint.connectedAnchor = anchor;
        }

        if (_armJoint != null)
        {
            // Move the arm along the y-axis based on 'W' and 'S' key presses.
            var anchor = _armJoint.connectedAnchor;
            if (Keyboard.current.wKey.isPressed)
            {
                if (anchor.y <= ARM_MIN) return;
                anchor.y -= speed * Time.deltaTime;
            }
            else if (Keyboard.current.sKey.isPressed)
            {
                if (anchor.y >= ARM_MAX) return;
                anchor.y += speed * Time.deltaTime;
            }
            _armJoint.connectedAnchor = anchor;
        }

        if (_handJoint != null)
        {
            // Move the hand along the z-axis based on up and down arrow key presses.
            var anchor = _handJoint.connectedAnchor;
            if (Keyboard.current.upArrowKey.isPressed)
            {
                if (anchor.z >= HAND_MAX) return;
                anchor.z += speed * Time.deltaTime;
            }
            else if (Keyboard.current.downArrowKey.isPressed)
            {
                if (anchor.z <= HAND_MIN) return;
                anchor.z -= speed * Time.deltaTime;
            }
            _handJoint.connectedAnchor = anchor;
        }

        // Adjust the target grip angle based on left and right arrow key presses.
        float gripSpeedAdjustment = _pressureSensorL.IsColliding && _pressureSensorR.IsColliding ? gripSpeed / 200.0f : gripSpeed;
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
            _targetGripAngle = _lastGripAngle - 0.002f * (_pressureSensorL.LastForce + _pressureSensorR.LastForce - 2 * targetForce);
        }

        // Apply the target grip angle to the left finger's hinge joint limits.
        if (_hingeL != null && _hingeL.useLimits)
        {
            var limits = _hingeL.limits;
            limits.min = _targetGripAngle - 0.01f;
            limits.max = _targetGripAngle;
            _hingeL.limits = limits;
        }

        // Apply the target grip angle to the right finger's hinge joint limits.
        if (_hingeR != null && _hingeR.useLimits)
        {
            var limits = _hingeR.limits;
            limits.min = _targetGripAngle - 0.01f;
            limits.max = _targetGripAngle;
            _hingeR.limits = limits;
        }

        // Update the last grip angle for the next frame's comparison.
        _lastGripAngle = _targetGripAngle;

    }

}
