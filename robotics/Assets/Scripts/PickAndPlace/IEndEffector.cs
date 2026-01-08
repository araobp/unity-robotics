using System.Threading.Tasks;
using UnityEngine;

/// <summary>
/// Defines the basic properties of a robot's end effector. This interface provides essential
/// information for positioning and interacting with the end effector in a simulated environment.
/// </summary>
public interface IEndEffector
{
    /// <summary>
    /// Gets the Transform of the edge of the end effector, which represents the precise
    /// point of interaction (e.g., the tip of the gripper fingers).
    /// </summary>
    Transform ToolCenterPoint { get; }

    /// <summary>
    /// Gets the current target position of the end effector's primary drive. For a gripper,
    /// this might represent the target separation of the fingers.
    /// </summary>
    float CurrentDriveTarget { get; }
}

/// <summary>
/// Extends the <see cref="IEndEffector"/> interface with gripper-specific functionality,
/// including force sensing and asynchronous open/close commands. This allows for more
/// detailed control and feedback from a parallel gripper mechanism.
/// </summary>
public interface IGripper : IEndEffector
{
    /// <summary>
    /// Gets the force currently being applied by the left finger's pressure sensor, in Newtons.
    /// </summary>
    float LeftFingerForce { get; }
    /// <summary>
    /// Gets the force currently being applied by the right finger's pressure sensor, in Newtons.
    /// </summary>
    float RightFingerForce { get; }

    /// <summary>
    /// Asynchronously commands the gripper to open.
    /// </summary>
    /// <param name="targetAngularVelocity">The angular velocity at which the gripper fingers should open.</param>
    /// <returns>A task that completes when the gripper is fully open.</returns>
    Task Open(float targetAngularVelocity);

    /// <summary>
    /// Asynchronously commands the gripper to close, applying a specified target force.
    /// </summary>
    /// <param name="targetForce">The desired force in Newtons to apply when gripping an object. If set to 0, the gripper will close completely until it reaches its physical limit.</param>
    /// <param name="targetAngularVelocity">The angular velocity at which the gripper fingers should close.</param>
    /// <returns>A task that completes when the gripper has either made contact with an object and achieved the target force, or has fully closed.</returns>
    Task Close(float targetForce, float targetAngularVelocity);
}