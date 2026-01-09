using UnityEngine;
using System.Threading;
using System.Threading.Tasks;
using Vector3 = UnityEngine.Vector3;
using Quaternion = UnityEngine.Quaternion;

// This class implements inverse kinematics (IK) for a robot arm, calculating the joint angles required to reach a target and smoothly animating the robot's movement.
public class RobotTest : MonoBehaviour
{
    // --- Fields ---

    // The target GameObject that the robot arm's end effector will attempt to reach.
    [Header("IK Target")]
    [SerializeField] GameObject work;

    // GameObjects representing the robot's joints.
    [Header("Robot Joints")]
    [SerializeField] GameObject j1;
    [SerializeField] GameObject j2;
    [SerializeField] GameObject j3;
    [SerializeField] GameObject j5;

    // Inverse kinematics behavior settings.
    [Header("IK Settings")]
    [Tooltip("The duration in seconds for the IK movement to complete.")]
    [SerializeField] private float ikMoveDuration = 1.0f;

    // A CancellationTokenSource for canceling the in-progress asynchronous movement task.
    private CancellationTokenSource _ikMoveCts;

    // Stores the initial rotation of each joint, allowing for relative calculations.
    Quaternion initialJ1Rotation;
    Quaternion initialJ2Rotation;
    Quaternion initialJ3Rotation;
    Quaternion initialJ5Rotation;

    // Called once before the first frame update. Initializes the robot's joint rotations and schedules the first IK calculation.
    void Start()
    {
        // Capture the initial local rotation of each bone.
        initialJ1Rotation = j1.transform.localRotation;
        initialJ2Rotation = j2.transform.localRotation;
        initialJ3Rotation = j3.transform.localRotation;
        initialJ5Rotation = j5.transform.localRotation;

        // Set the initial pose of the robot arm.
        setPose(Mathf.PI/2, Mathf.PI/2, Mathf.PI/2, Mathf.PI/2);

        // Call the IKTest method after 2 seconds to start the IK process.
        Invoke("IKTest", 2f);
    }

    // Called when the MonoBehaviour is destroyed.
    void OnDestroy()
    {
        // Ensures the CancellationTokenSource is cancelled and disposed to prevent memory leaks.
        _ikMoveCts?.Cancel();
        _ikMoveCts?.Dispose();
    }

    /* This function is the core implementation of the inverse kinematics (IK) for the robot arm.
     * It calculates the necessary joint angles (theta values) to position the end effector (hand)
     * at the target position defined by the 'work' GameObject.
     * The calculation is based on the geometric relationships of the robot arm's segments,
     * solving a 2D planar IK problem.
     */
    public async void IKTest()
    {
        // Define the fixed lengths of each segment of the robot arm.
        const float AB = 0.169f;
        const float CD = 0.273f;
        const float HANDSIZE = 0.42f;
        const float GF = HANDSIZE - CD;
        const float FE = 0.49727f;
        const float ED = 0.70142f;

        // Get the target position from the 'work' GameObject in local space and perform geometric calculations.
        Vector3 A = work.transform.localPosition;
        Debug.Log("Work position: " + A.ToString("F4"));

        float theta1 = Mathf.Atan2(A.z, A.x);
        Debug.Log("Theta1: " + (theta1 * Mathf.Rad2Deg).ToString("F4"));

        float AC = Mathf.Sqrt(A.x * A.x + A.z * A.z);
        float theta3 = Mathf.Asin(AB / AC);
        Debug.Log("Theta3: " + (theta3 * Mathf.Rad2Deg).ToString("F4"));

        float BC = AC * Mathf.Cos(theta3);
        Debug.Log("BC: " + BC.ToString("F4"));

        float theta2 = theta1 - theta3;
        Debug.Log("Theta2: " + (theta2 * Mathf.Rad2Deg).ToString("F4"));

        Vector3 B = new Vector3(BC * Mathf.Cos(theta2), A.y, BC * Mathf.Sin(theta2));
        Vector3 G = new Vector3(B.x, B.y + CD, B.z);

        float r = Mathf.Sqrt(BC * BC + GF * GF);
        Debug.Log("r: " + r.ToString("F4"));

        // Use the law of cosines to find the internal angles of the arm's triangles.
        float theat6 = Mathf.Acos((FE * FE - ED * ED - r * r) / (-2 * ED * r));
        float theat7 = Mathf.Acos((r * r - FE * FE - ED * ED) / (-2 * FE * ED));
        Debug.Log("Theta6: " + (theat6 * Mathf.Rad2Deg).ToString("F4"));
        Debug.Log("Theta7: " + (theat7 * Mathf.Rad2Deg).ToString("F4"));

        float theat5 = Mathf.Atan2(GF, BC);
        float theat4 = theat5 + theat6;
        Debug.Log("Theta4: " + (theat4 * Mathf.Rad2Deg).ToString("F4"));
        Debug.Log("Theta5: " + (theat5 * Mathf.Rad2Deg).ToString("F4"));

        float theat8 = 3 * Mathf.PI / 2 - theat4 - theat7;
        Debug.Log("Theta8: " + (theat8 * Mathf.Rad2Deg).ToString("F4"));

        // Log the initial rotation values for debugging.
        Debug.Log($"{initialJ1Rotation.y}, {initialJ1Rotation.z}");
        Debug.Log($"{initialJ2Rotation.y}, {initialJ2Rotation.z}");
        Debug.Log($"{initialJ3Rotation.y}, {initialJ3Rotation.z}");

        // Cancel any existing movement task before starting a new one.
        if (_ikMoveCts != null)
        {
            _ikMoveCts.Cancel();
            _ikMoveCts.Dispose();
        }

        // Create a new CancellationTokenSource and start the movement task.
        _ikMoveCts = new CancellationTokenSource();
        var newTargets = CalculateTargetPose(theta2, theat4, theat7, theat8);
        await MoveToTargets(newTargets.j1Q, newTargets.j2Q, newTargets.j3Q, newTargets.j5Q, ikMoveDuration, _ikMoveCts.Token);
    }

    // Directly sets the pose of the robot arm's joints to the specified angles.
    void setPose(float j1Angle, float j2Angle, float j3Angle, float j5Angle)
    {
        // Apply rotations relative to the initial orientation of each joint.
        j1.transform.localRotation = initialJ1Rotation * Quaternion.AngleAxis(j1Angle * Mathf.Rad2Deg, -Vector3.up);
        j2.transform.localRotation = initialJ2Rotation * Quaternion.AngleAxis(j2Angle * Mathf.Rad2Deg, -Vector3.up);
        j3.transform.localRotation = initialJ3Rotation * Quaternion.AngleAxis(j3Angle * Mathf.Rad2Deg, Vector3.up);
        j5.transform.localRotation = initialJ5Rotation * Quaternion.AngleAxis(j5Angle * Mathf.Rad2Deg, Vector3.up);
    }

    // Calculates the target rotations determined by the IK solver.
    (Quaternion j1Q, Quaternion j2Q, Quaternion j3Q, Quaternion j5Q) CalculateTargetPose(float j1Angle, float j2Angle, float j3Angle, float j5Angle)
    {
        // Calculate the target rotation for each joint relative to its initial orientation.
        var j1Q = initialJ1Rotation * Quaternion.AngleAxis(j1Angle * Mathf.Rad2Deg, -Vector3.up);
        var j2Q = initialJ2Rotation * Quaternion.AngleAxis(j2Angle * Mathf.Rad2Deg, -Vector3.up);
        var j3Q = initialJ3Rotation * Quaternion.AngleAxis(j3Angle * Mathf.Rad2Deg, Vector3.up);
        var j5Q = initialJ5Rotation * Quaternion.AngleAxis(j5Angle * Mathf.Rad2Deg, Vector3.up);
        return (j1Q, j2Q, j3Q, j5Q);
    }

    // Asynchronously moves the robot's joints smoothly from their current rotation to the target rotations over a specified duration.
    private async Task MoveToTargets(Quaternion j1Target, Quaternion j2Target, Quaternion j3Target, Quaternion j5Target, float duration, CancellationToken cancellationToken)
    {
        // Capture the starting rotation of each joint.
        Quaternion startJ1Q = j1.transform.localRotation;
        Quaternion startJ2Q = j2.transform.localRotation;
        Quaternion startj3Q = j3.transform.localRotation;
        Quaternion startJ5Q = j5.transform.localRotation;
        float elapsedTime = 0f;
        
        try
        {
            // Loop until the elapsed time reaches the specified duration.
            while (elapsedTime < duration)
            {
                cancellationToken.ThrowIfCancellationRequested();

                // Interpolate the rotation of each joint using Slerp.
                float t = elapsedTime / duration;
                j1.transform.localRotation = Quaternion.Slerp(startJ1Q, j1Target, t);
                j2.transform.localRotation = Quaternion.Slerp(startJ2Q, j2Target, t);
                j3.transform.localRotation = Quaternion.Slerp(startj3Q, j3Target, t);
                j5.transform.localRotation = Quaternion.Slerp(startJ5Q, j5Target, t);
                
                elapsedTime += Time.deltaTime;
                // Wait for the next frame before continuing the loop.
                await Task.Yield();
            }

            // Ensure the final rotation is set exactly to the target in case of any minor timing inaccuracies.
            j1.transform.localRotation = j1Target;
            j2.transform.localRotation = j2Target;
            j3.transform.localRotation = j3Target;
            j5.transform.localRotation = j5Target;
        }
        catch (TaskCanceledException)
        {
            // The task was canceled, which is an expected behavior when a new move command is issued.
        }
    }
}
