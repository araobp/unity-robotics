using UnityEngine;
using System.Threading;
using System.Threading.Tasks;
using Vector3 = UnityEngine.Vector3;
using Quaternion = UnityEngine.Quaternion;
using TMPro;

// This class implements inverse kinematics (IK) for a robot arm, calculating the joint angles required to reach a target and smoothly animating the robot's movement.
public class GeminiRoboticsTest : MonoBehaviour
{
    // --- Fields ---

    // The target GameObject that the robot arm's end effector will attempt to reach.
    [Header("IK Target")]
    [SerializeField] GameObject work;

    [SerializeField] GameObject finger;

    [Header("Robot Base")]
    [SerializeField] GameObject robotBase;

    // GameObjects representing the robot's joints.
    [Header("Robot Joints")]
    [SerializeField] GameObject swingAxis;
    [SerializeField] GameObject boomAxis;
    [SerializeField] GameObject armAxis;
    [SerializeField] GameObject handAxis;

    // Inverse kinematics behavior settings.
    [Header("IK Settings")]
    [Tooltip("The duration in seconds for the IK movement to complete.")]
    [SerializeField] private float ikMoveDuration = 1.0f;

    [Header("Hand target coordinates")]
    [SerializeField] TMP_Text handTargetPosX;
    [SerializeField] TMP_Text handTargetPosY;
    [SerializeField] TMP_Text handTargetPosZ;

    // 進行中の非同期移動タスクをキャンセルするためのトークンソース。
    private CancellationTokenSource _ikMoveCts;

    // Stores the initial rotation of each joint, allowing for relative calculations.
    Quaternion initialSwingRotation;
    Quaternion initialBoomRotation;
    Quaternion initialArmRotation;
    Quaternion initialHandRotation;

    // Called once before the first frame update. Initializes the robot's joint rotations and schedules the first IK calculation.
    void Start()
    {
        // Capture the initial local rotation of each bone.
        initialSwingRotation = swingAxis.transform.localRotation;
        initialBoomRotation = boomAxis.transform.localRotation;
        initialArmRotation = armAxis.transform.localRotation;
        initialHandRotation = handAxis.transform.localRotation;

        // Set the initial pose of the robot arm.
        setPose(Mathf.PI/2, Mathf.PI/2, Mathf.PI/2, Mathf.PI/2);

        // Call the IKTest method after 2 seconds to start the IK process.
        Invoke("IKTest", 2f);
    }

    void Update()
    {
        Vector3 handTargetPos = robotBase.transform.InverseTransformPoint(finger.transform.position);

        handTargetPosX.text = $"x: {handTargetPos.x.ToString("F3")}";
        handTargetPosY.text = $"y: {handTargetPos.y.ToString("F3")}";
        handTargetPosZ.text = $"z: {handTargetPos.z.ToString("F3")}";
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
        const float HANDSIZE = 0.325f;
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
        Debug.Log($"{initialSwingRotation.y}, {initialSwingRotation.z}");
        Debug.Log($"{initialBoomRotation.y}, {initialBoomRotation.z}");
        Debug.Log($"{initialArmRotation.y}, {initialArmRotation.z}");

        // Cancel any existing movement task before starting a new one.
        if (_ikMoveCts != null)
        {
            _ikMoveCts.Cancel();
            _ikMoveCts.Dispose();
        }

        // Create a new CancellationTokenSource and start the movement task.
        _ikMoveCts = new CancellationTokenSource();
        var newTargets = CalculateTargetPose(theta2, theat4, theat7, theat8);
        await MoveToTargets(newTargets.swing, newTargets.boom, newTargets.arm, newTargets.hand, ikMoveDuration, _ikMoveCts.Token);
    }

    // Directly sets the pose of the robot arm's joints to the specified angles.
    void setPose(float swingAngle, float boomAngle, float armAngle, float handAngle)
    {
        // Apply rotations relative to the initial orientation of each joint.
        swingAxis.transform.localRotation = initialSwingRotation * Quaternion.AngleAxis(swingAngle * Mathf.Rad2Deg, -Vector3.up);
        boomAxis.transform.localRotation = initialBoomRotation * Quaternion.AngleAxis(boomAngle * Mathf.Rad2Deg, -Vector3.up);
        armAxis.transform.localRotation = initialArmRotation * Quaternion.AngleAxis(armAngle * Mathf.Rad2Deg, Vector3.up);
        handAxis.transform.localRotation = initialHandRotation * Quaternion.AngleAxis(handAngle * Mathf.Rad2Deg, Vector3.up);

        // When setting the pose directly, also update the target rotations to prevent unintended smooth movements by Update().
        var newTargets = CalculateTargetPose(swingAngle, boomAngle, armAngle, handAngle);
    }

    // Calculates the target rotations determined by the IK solver.
    (Quaternion swing, Quaternion boom, Quaternion arm, Quaternion hand) CalculateTargetPose(float swingAngle, float boomAngle, float armAngle, float handAngle)
    {
        // Calculate the target rotation for each joint relative to its initial orientation.
        var swing = initialSwingRotation * Quaternion.AngleAxis(swingAngle * Mathf.Rad2Deg, -Vector3.up);
        var boom = initialBoomRotation * Quaternion.AngleAxis(boomAngle * Mathf.Rad2Deg, -Vector3.up);
        var arm = initialArmRotation * Quaternion.AngleAxis(armAngle * Mathf.Rad2Deg, Vector3.up);
        var hand = initialHandRotation * Quaternion.AngleAxis(handAngle * Mathf.Rad2Deg, Vector3.up);
        return (swing, boom, arm, hand);
    }

    // Asynchronously moves the robot's joints smoothly from their current rotation to the target rotations over a specified duration.
    private async Task MoveToTargets(Quaternion swingTarget, Quaternion boomTarget, Quaternion armTarget, Quaternion handTarget, float duration, CancellationToken cancellationToken)
    {
        // Capture the starting rotation of each joint.
        Quaternion startSwing = swingAxis.transform.localRotation;
        Quaternion startBoom = boomAxis.transform.localRotation;
        Quaternion startArm = armAxis.transform.localRotation;
        Quaternion startHand = handAxis.transform.localRotation;
        float elapsedTime = 0f;
        
        try
        {
            // Loop until the elapsed time reaches the specified duration.
            while (elapsedTime < duration)
            {
                cancellationToken.ThrowIfCancellationRequested();

                // Interpolate the rotation of each joint using Slerp.
                float t = elapsedTime / duration;
                swingAxis.transform.localRotation = Quaternion.Slerp(startSwing, swingTarget, t);
                boomAxis.transform.localRotation = Quaternion.Slerp(startBoom, boomTarget, t);
                armAxis.transform.localRotation = Quaternion.Slerp(startArm, armTarget, t);
                handAxis.transform.localRotation = Quaternion.Slerp(startHand, handTarget, t);
                
                elapsedTime += Time.deltaTime;
                // Wait for the next frame before continuing the loop.
                await Task.Yield();
            }

            // Ensure the final rotation is set exactly to the target in case of any minor timing inaccuracies.
            swingAxis.transform.localRotation = swingTarget;
            boomAxis.transform.localRotation = boomTarget;
            armAxis.transform.localRotation = armTarget;
            handAxis.transform.localRotation = handTarget;
        }
        catch (TaskCanceledException)
        {
            // The task was canceled, which is an expected behavior when a new move command is issued.
        }
    }
}
