using UnityEngine;
using System.Threading.Tasks;

/// <summary>
/// Implements the <see cref="IGripper"/> and <see cref="IEndEffector"/> interfaces to control
/// a parallel gripper mechanism using ArticulationBody components. This script provides
/// asynchronous opening and closing functionality with integrated force control feedback.
/// </summary>
public class ParallelGripper : MonoBehaviour, IEndEffector, IGripper
{
    /// <summary>
    /// The ArticulationBody for the main component of the right finger.
    /// </summary>
    private ArticulationBody _fingerR;
    /// <summary>
    /// The ArticulationBody for the plate component of the right finger.
    /// </summary>
    private ArticulationBody _plateR;
    /// <summary>
    /// The ArticulationBody for the secondary component of the right finger.
    /// </summary>
    private ArticulationBody _fingerR2;

    /// <summary>
    /// The ArticulationBody for the main component of the left finger.
    /// </summary>
    private ArticulationBody _fingerL;
    /// <summary>
    /// The ArticulationBody for the plate component of the left finger.
    /// </summary>
    private ArticulationBody _plateL;
    /// <summary>
    /// The ArticulationBody for the secondary component of the left finger.
    /// </summary>
    private ArticulationBody _fingerL2;

    /// <summary>
    /// The pressure sensor on the right finger, used to detect contact and measure applied force.
    /// </summary>
    private PressureSensor _pressureSensorR;
    /// <summary>
    /// The pressure sensor on the left finger, used to detect contact and measure applied force.
    /// </summary>
    private PressureSensor _pressureSensorL;

    /// <summary>
    /// The lower limit of the gripper's primary joint drive, representing the fully open state.
    /// </summary>
    private float _lowerLimit = 0f;
    /// <summary>
    /// The upper limit of the gripper's primary joint drive, representing the fully closed state.
    /// </summary>
    private float _upperLimit = 0f;
 
    [Header("Gripper Settings")]
    [Tooltip("The compliance gap for the gripper's articulation drives when holding an object. This allows for a more stable grip.")]
    [SerializeField] private float minMaxGap = 0.1f;

    [Tooltip("A factor used to reduce the applied force during a hold, preventing excessive pressure on the gripped object.")]
    [SerializeField] private float forceReductionDenominator = 500.0f;
    [Tooltip("A factor used to slow down the gripper's movement upon collision, before the target force is reached.")]
    [SerializeField] private float slowDownDenominator = 200.0f;
 
    // --- State variables ---

    /// <summary>
    /// A state flag that is true when the gripper is actively moving towards a target position (opening or closing).
    /// </summary>
    private bool _isMoving = false;
    /// <summary>
    /// A state flag that is true when the gripper is actively holding an object by applying a continuous force.
    /// </summary>
    private bool _isHolding = false;

    // --- Movement and Target variables ---

    /// <summary>
    /// The current target position for the gripper's master joint drive.
    /// </summary>
    private float _currentTarget;
    /// <summary>
    /// The final target position for the current movement, which is typically either the open or closed limit.
    /// </summary>
    private float _targetMasterValue;
    /// <summary>
    /// The angular velocity (speed) for the current gripper movement.
    /// </summary>
    private float _angularVelocity;
    /// <summary>
    /// The desired force in Newtons to apply and maintain when in a holding state.
    /// </summary>
    private float _targetForce;

    /// <summary>
    /// Gets the characteristic size of the end effector in meters. For a gripper, this is
    /// typically the length from the wrist joint to the gripping point. This value is crucial
    /// for accurate Inverse Kinematics (IK) calculations to position the end effector correctly.
    /// </summary>
    public float EndEffectorSize => 0.392f;

    /// <summary>
    /// Gets the current target position of the gripper's master joint drive.
    /// The private setter applies this target value to all individual finger articulation bodies,
    /// correctly handling the opposing motion required for a parallel gripper and applying a compliance gap when in a holding state.
    /// </summary>
    public float CurrentDriveTarget
    {
        get { return _currentTarget; }
        private set
        {
            bool applyGap = _isHolding;
            // The right side moves in the positive direction to close the gripper.
            SetDriveProperties(_fingerR, value, applyGap);
            SetDriveProperties(_plateR, value, applyGap);
            SetDriveProperties(_fingerR2, value, applyGap);
            // The left side moves in the positive direction to close the gripper.
            SetDriveProperties(_fingerL, value, applyGap);
            SetDriveProperties(_plateL, value, applyGap);
            SetDriveProperties(_fingerL2, value, applyGap);
            _currentTarget = value;
        }
    }

    /// <summary>
    /// Gets the last force value read from the left finger's pressure sensor.
    /// </summary>
    public float LeftFingerForce => _pressureSensorL.LastForce;
    /// <summary>
    /// Gets the last force value read from the right finger's pressure sensor.
    /// </summary>
    public float RightFingerForce => _pressureSensorR.LastForce;


    /// <summary>
    /// Initializes component references and gripper joint limits upon startup.
    /// </summary>
    private void Start()
    {        
        _fingerR = transform.Find("Palm/J1R/FingerR").GetComponent<ArticulationBody>();
        _plateR = transform.Find("Palm/J1R/FingerR/J2R/PlateR").GetComponent<ArticulationBody>();
        _fingerR2 = transform.Find("Palm/J1R/FingerR/J2R/PlateR/J3R/Finger2R").GetComponent<ArticulationBody>();
        _fingerL = transform.Find("Palm/J1L/FingerL").GetComponent<ArticulationBody>();
        _plateL = transform.Find("Palm/J1L/FingerL/J2L/PlateL").GetComponent<ArticulationBody>();
        _fingerL2 = transform.Find("Palm/J1L/FingerL/J2L/PlateL/J3L/Finger2L").GetComponent<ArticulationBody>();

        _pressureSensorR = transform.Find("Palm/J1R/FingerR/J2R/PlateR").GetComponent<PressureSensor>();
        _pressureSensorL = transform.Find("Palm/J1L/FingerL/J2L/PlateL").GetComponent<PressureSensor>();

        _lowerLimit = _fingerR.xDrive.lowerLimit;
        _upperLimit = _fingerR.xDrive.upperLimit;
        _currentTarget = _fingerR.xDrive.target;
    }

    /// <summary>
    /// Handles the physics-based movement and force application of the gripper on a fixed timestep.
    /// </summary>
    private void FixedUpdate()
    {
        // State: Moving towards a target position (opening or closing).
        if (_isMoving)
        {
            // During a close operation with force, check for contact and target force achievement.
            if (_targetForce > 0f && IsColliding())
            {
                // Read the force currently being applied by each finger.
                float forceL = _pressureSensorL != null ? _pressureSensorL.LastForce : 0f;
                float forceR = _pressureSensorR != null ? _pressureSensorR.LastForce : 0f;

                // If we have achieved the target force, switch to holding mode and stop the movement task.
                if (forceL > _targetForce && forceR > _targetForce)
                {
                    Debug.Log($"Gripper achieved target force: Left={forceL}, Right={forceR}, Target={_targetForce}");
                    _isMoving = false; // This will terminate the async task in Close() or Open().
                    _isHolding = true;
                    // The holding logic will start next frame.
                    // The Close() task will now complete.
                    return;
                }
            }

            float currentSpeed = _angularVelocity;
            // If colliding but not yet at the target force, slow down the movement.
            if (IsColliding())
            {
                currentSpeed /= slowDownDenominator;
            }

            // Move towards the target position
            float newMasterValue = Mathf.MoveTowards(_currentTarget, _targetMasterValue, currentSpeed * Time.fixedDeltaTime);
            CurrentDriveTarget = newMasterValue;

            // If we have reached the target position (fully open or fully closed), stop moving.
            if (Mathf.Approximately(_currentTarget, _targetMasterValue))
            {
                _isMoving = false;
                _isHolding = false; // If we reached the limit, we are not holding anything.
            }
        }
        // State: Actively holding an object with a specific force.
        else if (_isHolding)
        {
            // Continue to move slowly towards the target to maintain a firm grip.
            float currentSpeed = _angularVelocity / slowDownDenominator;
            float newMasterValue = Mathf.MoveTowards(_currentTarget, _targetMasterValue, currentSpeed * Time.fixedDeltaTime);

            // Read forces from sensors
            float forceL = _pressureSensorL != null ? _pressureSensorL.LastForce : 0f;
            float forceR = _pressureSensorR != null ? _pressureSensorR.LastForce : 0f;

            // If both fingers are applying more force than desired, slightly release the grip
            // to maintain the target force. This creates a force-feedback loop.
            if (forceL > _targetForce && forceR > _targetForce)
            {
                // Calculate the adjustment based on the excess force.
                float adjustment = (forceL + forceR - 2 * _targetForce) / forceReductionDenominator;
                newMasterValue -= adjustment;
            }

            CurrentDriveTarget = newMasterValue;
        }
    }

    /// <summary>
    /// Asynchronously opens the gripper.
    /// </summary>
    /// <param name="angularVelocity">The speed at which to open the gripper.</param>
    /// <returns>A task that completes when the gripper is fully open.</returns>
    public async Task Open(float angularVelocity)
    {
        // If we were holding something, reset the joint limits to allow full range of motion.
        if (_isHolding)
        {
            ResetDriveLimits();
        }
        _isHolding = false;
        _angularVelocity = angularVelocity;
        _targetMasterValue = _lowerLimit;
        _isMoving = true;

        // Wait until the movement is complete.
        while (_isMoving)
        {
            await Task.Yield();
        }
    }

    /// <summary>
    /// Asynchronously closes the gripper with a specified target force.
    /// </summary>
    /// <param name="targetForce">The desired force to apply upon gripping an object. If 0, it will close until it hits the limit.</param>
    /// <param name="angularVelocity">The speed at which to close the gripper.</param>
    /// <returns>A task that completes when the gripper has made contact with an object or has fully closed.</returns>
    public async Task Close(float targetForce, float angularVelocity)
    {
        _angularVelocity = angularVelocity;
        _targetForce = targetForce;
        _targetMasterValue = _upperLimit;
        _isHolding = false;
        _isMoving = true;

        // Wait until the movement is complete (either fully closed or holding an object).
        while (_isMoving)
        {
            await Task.Yield();
        }
    }

    /// <summary>
    /// Immediately stops any movement or holding action of the gripper.
    /// </summary>
    public void Stop()
    {
        _isMoving = false;
        _isHolding = false;
        ResetDriveLimits();
    }

    /// <summary>
    /// Checks if both gripper fingers are currently colliding with something.
    /// </summary>
    /// <returns>True if both pressure sensors are colliding, false otherwise.</returns>
    private bool IsColliding()
    {
        return _pressureSensorR.IsColliding && _pressureSensorL.IsColliding;
    }

    /// <summary>
    /// Sets the target for an ArticulationBody's xDrive.
    /// </summary>
    /// <remarks>
    /// When applyGap is true, it also sets the drive limits to create a small
    /// compliance range, allowing the gripper to "give" slightly when holding an object.
    /// </remarks>
    /// <param name="body">The articulation body to modify.</param>
    /// <param name="target">The new target value.</param>
    /// <param name="applyGap">Whether to apply the minMaxGap to the drive limits.</param>
    private void SetDriveProperties(ArticulationBody body, float target, bool applyGap)
    {
        var drive = body.xDrive;
        drive.target = target;

        if (applyGap)
        {
            // Create a small, one-sided compliance gap to allow for a more stable grip.
            // This is based on the logic from the original PickAndPlace script.
            // It allows the joint to move back from the target, but not past it.
            drive.lowerLimit = target - minMaxGap;
            drive.upperLimit = target;
        }

        body.xDrive = drive;
    }

    /// <summary>
    /// Resets the drive limits for all gripper articulation bodies to their original values.
    /// This is necessary after a grip-hold operation to allow for full range of motion.
    /// </summary>
    private void ResetDriveLimits()
    {
        var drive = _fingerR.xDrive;
        drive.lowerLimit = _lowerLimit;
        drive.upperLimit = _upperLimit;
        _fingerR.xDrive = drive;
        _plateR.xDrive = drive;
        _fingerR2.xDrive = drive;

        // The left side has inverted limits relative to the right side.
        drive.lowerLimit = -_upperLimit;
        drive.upperLimit = -_lowerLimit;
        _fingerL.xDrive = drive;
        _plateL.xDrive = drive;
        _fingerL2.xDrive = drive;
    }
}
