using System.Threading.Tasks;

/// <summary>
/// Defines the basic properties of a robot's end effector.
/// </summary>
public interface IEndEffector
{
    /// <summary>
    /// Gets the size of the end effector in meters. This is used for Inverse Kinematics calculations.
    /// </summary>
    float EndEffectorSize { get; }

    /// <summary>
    /// Gets the current target position of the end effector's primary drive.
    /// </summary>
    float CurrentDriveTarget { get; }
}

/// <summary>
/// Extends the <see cref="IEndEffector"/> interface with gripper-specific functionality,
/// including force sensing and open/close commands.
/// </summary>
public interface IGripper : IEndEffector
{
    /// <summary>
    /// Gets the force currently being applied by the left finger's pressure sensor.
    /// </summary>
    float LeftFingerForce { get; }
    /// <summary>
    /// Gets the force currently being applied by the right finger's pressure sensor.
    /// </summary>
    float RightFingerForce { get; }

    /// <summary>
    /// Asynchronously opens the gripper.
    /// </summary>
    /// <param name="targetAngularVelocity">The speed at which to open the gripper.</param>
    /// <returns>A task that completes when the gripper is fully open.</returns>
    Task Open(float targetAngularVelocity);

    /// <summary>
    /// Asynchronously closes the gripper with a specified target force.
    /// </summary>
    /// <param name="targetForce">The desired force to apply upon gripping an object. If 0, it will close until it hits the limit.</param>
    /// <param name="targetAngularVelocity">The speed at which to close the gripper.</param>
    /// <returns>A task that completes when the gripper has made contact with an object or has fully closed.</returns>
    Task Close(float targetForce, float targetAngularVelocity);
}