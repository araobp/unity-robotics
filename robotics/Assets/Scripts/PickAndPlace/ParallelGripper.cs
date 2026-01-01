using UnityEngine;
using System.Threading.Tasks;

/// <summary>
/// Controls a parallel gripper mechanism using ArticulationBody components.
/// It allows for asynchronous opening and closing of the gripper with force control.
/// </summary>
public class ParallelGripper : MonoBehaviour
{
    // Articulation bodies for the right side of the gripper
    private ArticulationBody _fingerR;
    private ArticulationBody _plateR;
    private ArticulationBody _fingerR2;

    // Articulation bodies for the left side of the gripper
    private ArticulationBody _fingerL;
    private ArticulationBody _plateL;
    private ArticulationBody _fingerL2;

    // Pressure sensors to detect contact and force
    private PressureSensor _pressureSensorR;
    private PressureSensor _pressureSensorL;

    // Gripper joint limits
    private float _lowerLimit = 0f;
    private float _upperLimit = 0f;
 
    // The compliance gap for the gripper articulations when holding an object.
    [SerializeField] private float minMaxGap = 0.01f;

    // Denominator for reducing force to prevent excessive force application.
    [SerializeField] private float forceReductionDenominator = 500.0f;
    // Denominator for slowing down gripper movement upon collision.
    [SerializeField] private float slowDownDenominator = 200.0f;
 
    // State flags
    // True if the gripper is actively opening or closing.
    private bool _isMoving = false;
    // True if the gripper is holding an object with force.
    private bool _isHolding = false;

    // Target and movement variables
    // The current target position of the gripper.
    private float _currentTarget;
    // The desired final target position for the current movement.
    private float _targetMasterValue;
    // The speed at which the gripper moves.
    private float _angularVelocity;
    // The desired force to apply when gripping an object.
    private float _targetForce;

    /// <summary>
    /// Gets the current target position of the gripper's joints.
    /// The setter is private and applies the target value to all finger articulation bodies.
    /// </summary>
    public float CurrentTarget
    {
        get { return _currentTarget; }
        private set
        {
            bool applyGap = _isHolding;
            // set x drive target for right finger
            SetDriveProperties(_fingerR, value, applyGap);
            SetDriveProperties(_plateR, value, applyGap);
            SetDriveProperties(_fingerR2, value, applyGap);
            // set x drive target for left finger
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


    // Initializes component references and gripper limits.
    void Start()
    {        
        _fingerR = transform.Find("EndEffectorBase/Hand/P1R/FingerR").GetComponent<ArticulationBody>();
        _plateR = transform.Find("EndEffectorBase/Hand/P1R/FingerR/P2R/PlateR").GetComponent<ArticulationBody>();
        _fingerR2 = transform.Find("EndEffectorBase/Hand/P1R/FingerR/P2R/PlateR/P3R/FingerR2").GetComponent<ArticulationBody>();
        _fingerL = transform.Find("EndEffectorBase/Hand/P1L/FingerL").GetComponent<ArticulationBody>();
        _plateL = transform.Find("EndEffectorBase/Hand/P1L/FingerL/P2L/PlateL").GetComponent<ArticulationBody>();
        _fingerL2 = transform.Find("EndEffectorBase/Hand/P1L/FingerL/P2L/PlateL/P3L/FingerL2").GetComponent<ArticulationBody>();

        _pressureSensorR = transform.Find("EndEffectorBase/Hand/P1R/FingerR/P2R/PlateR").GetComponent<PressureSensor>();
        _pressureSensorL = transform.Find("EndEffectorBase/Hand/P1L/FingerL/P2L/PlateL").GetComponent<PressureSensor>();

        _lowerLimit = _fingerR.xDrive.lowerLimit;
        _upperLimit = _fingerR.xDrive.upperLimit;
        _currentTarget = _fingerR.xDrive.target;
    }

    // Handles the physics-based movement and force application of the gripper.
    void FixedUpdate()
    {
        if (_isMoving) // This is for opening, or closing before contact
        {
            // If we are closing and make contact, switch to holding mode
            if (IsColliding() && _targetForce > 0f)
            {
                _isMoving = false;
                _isHolding = true;
                return;
            }

            float currentSpeed = _angularVelocity;
            // Slow down if we are colliding but not yet in holding mode (e.g. gentle close)
            if (IsColliding())
            {
                currentSpeed /= slowDownDenominator;
            }

            // Move towards the target position
            float newMasterValue = Mathf.MoveTowards(_currentTarget, _targetMasterValue, currentSpeed * Time.fixedDeltaTime);
            CurrentTarget = newMasterValue;

            // Stop moving if we have reached the target
            if (Mathf.Approximately(_currentTarget, _targetMasterValue))
            {
                _isMoving = false;
            }
        }
        else if (_isHolding) // This is for continuous gripping with force feedback
        {
            // Move slowly towards the target to maintain grip
            float currentSpeed = _angularVelocity / slowDownDenominator;
            float newMasterValue = Mathf.MoveTowards(_currentTarget, _targetMasterValue, currentSpeed * Time.fixedDeltaTime);

            // Read forces from sensors
            float forceL = _pressureSensorL != null ? _pressureSensorL.LastForce : 0f;
            float forceR = _pressureSensorR != null ? _pressureSensorR.LastForce : 0f;

            // If both fingers are applying more force than the target, adjust the grip
            if (forceL > _targetForce && forceR > _targetForce)
            {
                float gripDirection = Mathf.Sign(_targetMasterValue - _currentTarget);
                if (gripDirection == 0) gripDirection = 1;
                // Calculate adjustment based on excess force
                float adjustment = (forceL + forceR - 2 * _targetForce) / forceReductionDenominator;
                newMasterValue -= gripDirection * adjustment;
            }

            CurrentTarget = newMasterValue;
        }
    }

    /// <summary>
    /// Asynchronously opens the gripper.
    /// </summary>
    /// <param name="angularVelocity">The speed at which to open the gripper.</param>
    /// <returns>A task that completes when the gripper is fully open.</returns>
    public async Task Open(float angularVelocity)
    {
        _isHolding = false;
        _angularVelocity = angularVelocity;
        _targetMasterValue = _lowerLimit;
        _isMoving = true;

        while (_isMoving)
        {
            await Task.Yield();
        }
    }

    /// <summary>
    /// Asynchronously closes the gripper with a specified target force.
    /// </summary>
    /// <param name="angularVelocity">The speed at which to close the gripper.</param>
    /// <param name="targetForce">The desired force to apply upon gripping an object. If 0, it will close until it hits the limit.</param>
    /// <returns>A task that completes when the gripper has made contact with an object or has fully closed.</returns>
    public async Task Close(float angularVelocity, float targetForce)
    {
        _angularVelocity = angularVelocity;
        _targetForce = targetForce;
        _targetMasterValue = _upperLimit;
        _isHolding = false;
        _isMoving = true;

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
            // Create a small compliance gap to allow for a more stable grip.
            // This is based on the logic from the original PickAndPlace script.
            drive.lowerLimit = target - minMaxGap;
            drive.upperLimit = target + minMaxGap;
        }

        body.xDrive = drive;
    }
}
