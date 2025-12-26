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
    public float closedAngle = 45.0f;
    /// <summary>
    /// The angle of the finger joints when the grip is fully open.
    /// </summary>
    public float openAngle = 0.0f;

    /// <summary>
    /// The force threshold for the pressure sensors to stop the grip from closing.
    /// </summary>
    public float targetForce = 1.0f;

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
                anchor.x -= speed * Time.deltaTime;
            }
            else if (Keyboard.current.dKey.isPressed)
            {
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
                anchor.y -= speed * Time.deltaTime;
            }
            else if (Keyboard.current.sKey.isPressed)
            {
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
                anchor.z += speed * Time.deltaTime;
            }
            else if (Keyboard.current.downArrowKey.isPressed)
            {
                anchor.z -= speed * Time.deltaTime;
            }
            handJoint.connectedAnchor = anchor;
        }

        // Adjust the target grip angle based on left and right arrow key presses.
        if (Keyboard.current.leftArrowKey.isPressed)
        {
            targetGripAngle = Mathf.MoveTowards(targetGripAngle, closedAngle, gripSpeed * Time.deltaTime);
        }
        else if (Keyboard.current.rightArrowKey.isPressed)
        {
            targetGripAngle = Mathf.MoveTowards(targetGripAngle, openAngle, gripSpeed * Time.deltaTime);
        }

        // Force feedback: if closing and both fingers detect a force greater than the threshold, stop closing.
        if (targetGripAngle - lastGripAngle > 0 && pressureSensorL.LastForce > targetForce && pressureSensorR.LastForce > targetForce)
        {
            targetGripAngle = lastGripAngle;
        }

        // Apply the target grip angle to the left finger's hinge joint limits.
        if (hingeL != null && hingeL.useLimits)
        {
            var limits = hingeL.limits;
            limits.min = targetGripAngle - 1;
            limits.max = targetGripAngle;
            hingeL.limits = limits;
        }

        // Apply the target grip angle to the right finger's hinge joint limits.
        if (hingeR != null && hingeR.useLimits)
        {
            var limits = hingeR.limits;
            limits.min = targetGripAngle - 1;
            limits.max = targetGripAngle;
            hingeR.limits = limits;
        }

        // Update the last grip angle for the next frame's comparison.
        lastGripAngle = targetGripAngle;

    }

}
