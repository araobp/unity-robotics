using TMPro;
using UnityEngine;
using UnityEngine.InputSystem;

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
    public float speed = 1.0f; // Speed of movement
    // The GameObject representing the main arm.
    [SerializeField] GameObject arm;

    // The GameObject representing the slider mechanism.
    [SerializeField] GameObject slider;

    /// <summary>
    /// The speed at which the fingers open and close.
    /// </summary>
    public float gripSpeed = 50.0f;
    /// <summary>
    /// The angle of the finger joints when the grip is fully closed.
    /// </summary>
    public float closedAngle = 10.0f;
    /// <summary>
    /// The angle of the finger joints when the grip is fully open.
    /// </summary>
    public float openAngle = -20.0f;

    /// <summary>
    /// The force threshold for the pressure sensors to stop the grip from closing.
    /// </summary>
    public float targetForce = 3.0f;

    // GameObjects for the left and right fingers.
    [SerializeField] GameObject fingerL;
    [SerializeField] GameObject fingerR;

    // Hinge joints for controlling the fingers.
    private HingeJoint hingeL;
    private HingeJoint hingeR;
    // The target angle for the finger grip.
    private float targetGripAngle;

    // Configurable joints for controlling the arm, hand, and slider.
    private ConfigurableJoint armJoint;
    private ConfigurableJoint handJoint;
    private ConfigurableJoint sliderJoint;

    // Pressure sensors on each finger to detect contact force.
    private PressureSensor pressureSensorL;
    private PressureSensor pressureSensorR;

    // Stores the grip angle from the previous frame to detect changes.
    private float lastGripAngle = 0.0f;


    [SerializeField] TMP_Text info;

    const float SLIDER_MAX = 0.414f;
    const float SLIDER_MIN = -0.414f;

    const float ARM_MAX = 0.377f;
    const float ARM_MIN = -0.377f;

    const float HAND_MAX = 0.158f;
    const float HAND_MIN = -0.032f;

    void Update()
    {
        if (info != null && sliderJoint != null && armJoint != null && handJoint != null)
        {
            info.text = $"Slider: {sliderJoint.connectedAnchor.x:F3}\n" +
                        $"Arm: {armJoint.connectedAnchor.y:F3}\n" +
                        $"Hand: {handJoint.connectedAnchor.z:F3}\n" +
                        $"Grip Angle: {targetGripAngle:F1}\n" +
                        $"Force L: {pressureSensorL.LastForce:F2}\n" +
                        $"Force R: {pressureSensorR.LastForce:F2}";
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
            sliderJoint = slider.GetComponent<ConfigurableJoint>();
        }

        // Get the ConfigurableJoint component from the arm GameObject.
        if (arm != null)
        {
            armJoint = arm.GetComponent<ConfigurableJoint>();
        }

        // Get the ConfigurableJoint component from the hand GameObject.
        if (hand != null)
        {
            handJoint = hand.GetComponent<ConfigurableJoint>();
        }

        // Get HingeJoint and PressureSensor components from the left finger.
        if (fingerL != null)
        {
            hingeL = fingerL.GetComponent<HingeJoint>();
            pressureSensorL = fingerL.GetComponent<PressureSensor>();
        }
        // Get HingeJoint and PressureSensor components from the right finger.
        if (fingerR != null)
        {
            hingeR = fingerR.GetComponent<HingeJoint>();
            pressureSensorR = fingerR.GetComponent<PressureSensor>();
        }
    }

    /// <summary>
    /// Called at a fixed time interval, used for physics-related updates.
    /// </summary>
    void FixedUpdate()
    {
        // Exit if there is no keyboard input detected.
        if (Keyboard.current == null) return;

        if (sliderJoint != null)
        {
            // Move the slider along the x-axis based on 'A' and 'D' key presses.
            var anchor = sliderJoint.connectedAnchor;
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
            sliderJoint.connectedAnchor = anchor;
        }

        if (armJoint != null)
        {
            // Move the arm along the y-axis based on 'W' and 'S' key presses.
            var anchor = armJoint.connectedAnchor;
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
            armJoint.connectedAnchor = anchor;
        }

        if (handJoint != null)
        {
            // Move the hand along the z-axis based on up and down arrow key presses.
            var anchor = handJoint.connectedAnchor;
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
            handJoint.connectedAnchor = anchor;
        }

        // Adjust the target grip angle based on left and right arrow key presses.
        float gripSpeedAdjustment = pressureSensorL.OnCollisionEntered && pressureSensorR.OnCollisionEntered ? gripSpeed / 200.0f : gripSpeed;    
        if (Keyboard.current.leftArrowKey.isPressed)
        {
            targetGripAngle = Mathf.MoveTowards(targetGripAngle, closedAngle, gripSpeedAdjustment * Time.deltaTime);
        }
        else if (Keyboard.current.rightArrowKey.isPressed)
        {
            targetGripAngle = Mathf.MoveTowards(targetGripAngle, openAngle, gripSpeed * Time.deltaTime);
        }

        // Force feedback: if closing and both fingers detect a force greater than the threshold, stop closing.
        if (targetGripAngle - lastGripAngle > 0 && pressureSensorL.LastForce  > targetForce && pressureSensorR.LastForce > targetForce)
        {
            targetGripAngle = lastGripAngle - 0.001f * (pressureSensorL.LastForce + pressureSensorR.LastForce - 2 * targetForce);
        }

        // Apply the target grip angle to the left finger's hinge joint limits.
        if (hingeL != null && hingeL.useLimits)
        {
            var limits = hingeL.limits;
            limits.min = targetGripAngle-0.01f;
            limits.max = targetGripAngle;
            hingeL.limits = limits;
        }

        // Apply the target grip angle to the right finger's hinge joint limits.
        if (hingeR != null && hingeR.useLimits)
        {
            var limits = hingeR.limits;
            limits.min = targetGripAngle-0.01f;
            limits.max = targetGripAngle;
            hingeR.limits = limits;
        }

        // Update the last grip angle for the next frame's comparison.
        lastGripAngle = targetGripAngle;

    }

}
