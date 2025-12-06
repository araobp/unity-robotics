using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem.XR;
using System;
using System.Numerics;
using Vector3 = UnityEngine.Vector3;
using Quaternion = UnityEngine.Quaternion;
using Newtonsoft.Json.Bson;

public class RobotTest : MonoBehaviour
{

    [SerializeField] GameObject work;

    [SerializeField] Button buttonSwingR;
    [SerializeField] Button buttonSwingL;

    [SerializeField] Button buttonBoomR;
    [SerializeField] Button buttonBoomL;

    [SerializeField] Button buttonArmR;
    [SerializeField] Button buttonArmL;

    [SerializeField] Button buttonHandR;
    [SerializeField] Button buttonHandL;

    [SerializeField] private float rotationSwingSpeed = 100f;
    [SerializeField] GameObject boneSwing;

    [SerializeField] private float rotationBoomSpeed = 100f;
    [SerializeField] GameObject boneBoom;

    [SerializeField] private float rotationArmSpeed = 100f;
    [SerializeField] GameObject boneArm;

    [SerializeField] private float rotationHandSpeed = 100f;
    [SerializeField] GameObject boneHand;

    [SerializeField] Toggle toggleLookDown;

    private bool isSwingButtonRPressed = false;
    private bool isSwingButtonLPressed = false;

    private bool isBoomButtonRPressed = false;
    private bool isBoomButtonLPressed = false;

    private bool isArmButtonRPressed = false;
    private bool isArmButtonLPressed = false;

    private bool isHandButtonRPressed = false;
    private bool isHandButtonLPressed = false;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        SetupButtonEvents(buttonSwingR, (isPressed) => isSwingButtonRPressed = isPressed);
        SetupButtonEvents(buttonSwingL, (isPressed) => isSwingButtonLPressed = isPressed);

        SetupButtonEvents(buttonBoomR, (isPressed) => isBoomButtonRPressed = isPressed);
        SetupButtonEvents(buttonBoomL, (isPressed) => isBoomButtonLPressed = isPressed);

        SetupButtonEvents(buttonArmR, (isPressed) => isArmButtonRPressed = isPressed);
        SetupButtonEvents(buttonArmL, (isPressed) => isArmButtonLPressed = isPressed);

        SetupButtonEvents(buttonHandR, (isPressed) => isHandButtonRPressed = isPressed);
        SetupButtonEvents(buttonHandL, (isPressed) => isHandButtonLPressed = isPressed);

        IKTest();
    }


    // Update is called once per frame
    void Update()
    {
        if (isSwingButtonRPressed)
        {
            boneSwing.transform.Rotate(0f, rotationSwingSpeed * Time.deltaTime, 0f);
        }
        else if (isSwingButtonLPressed)
        {
            boneSwing.transform.Rotate(0f, -rotationSwingSpeed * Time.deltaTime, 0f);
        }

        if (isBoomButtonRPressed)
        {
            boneBoom.transform.Rotate(0f, rotationBoomSpeed * Time.deltaTime, 0f);
        }
        else if (isBoomButtonLPressed)
        {
            boneBoom.transform.Rotate(0f, -rotationBoomSpeed * Time.deltaTime, 0f);
        }

        if (isArmButtonRPressed)
        {
            boneArm.transform.Rotate(0f, -rotationArmSpeed * Time.deltaTime, 0f);
        }
        else if (isArmButtonLPressed)
        {
            boneArm.transform.Rotate(0f, rotationArmSpeed * Time.deltaTime, 0f);
        }

        if (isHandButtonRPressed)
        {
            boneHand.transform.Rotate(0f, rotationHandSpeed * Time.deltaTime, 0f);
        }
        else if (isHandButtonLPressed)
        {
            boneHand.transform.Rotate(0f, -rotationHandSpeed * Time.deltaTime, 0f);
        }

        if (toggleLookDown.isOn)
        {
            makeHandLookDown();            
        }
    }

    void makeHandLookDown()
    {

        Transform parentTransform = boneHand.transform.parent;
        // Get the parent's world-space right axis
        Vector3 parentRight = parentTransform.right;

        // Define the desired forward direction in world space (pointing straight up)
        Vector3 worldUpward = Vector3.up;

        // Calculate the target world rotation. This rotation orients the boneHand so that its
        // forward vector points upwards (worldUpward), and its up vector aligns with its
        // parent's right vector.
        Quaternion targetWorldRotation = Quaternion.LookRotation(worldUpward, parentRight);

        // Convert the world rotation to local rotation relative to the parent
        boneHand.transform.localRotation = Quaternion.Inverse(parentTransform.rotation) * targetWorldRotation;
    }

    private void SetupButtonEvents(Button button, System.Action<bool> setPressedState)
    {
        EventTrigger trigger = button.gameObject.GetComponent<EventTrigger>() ?? button.gameObject.AddComponent<EventTrigger>();

        var pointerDown = new EventTrigger.Entry { eventID = EventTriggerType.PointerDown };
        pointerDown.callback.AddListener((e) => setPressedState(true));
        trigger.triggers.Add(pointerDown);

        var pointerUp = new EventTrigger.Entry { eventID = EventTriggerType.PointerUp };
        pointerUp.callback.AddListener((e) => setPressedState(false));
        trigger.triggers.Add(pointerUp);
    }

    public void onButtonSwingPressed()
    {
        Debug.Log("Swing Clicked");
    }


    /* This function is my original implementation of Inverse Kinematics for the robot arm.
     * It calculates the necessary joint angles to position the end effector (hand)
     * at the target position defined by the 'work' GameObject.
     * The calculations are based on the geometric relationships of the robot arm's segments.
     */ 
    public void IKTest()
    {
        // Constants
        const float AB = 0.169f;
        const float Bs = 0.273f;
        const float HANDSIZE = 0.325f;
        const float GF = HANDSIZE - Bs;
        const float FE = 0.49727f;
        const float ED = 0.70142f;

        // Work position
        Vector3 A = work.transform.position;
        Debug.Log("Work position: " + A.ToString("F4"));
        
        float theta1 = Mathf.Atan2(A.z, A.x);
        Debug.Log("Theta1: " + (theta1 * Mathf.Rad2Deg).ToString("F4"));

        float AC = A.x / Mathf.Cos(theta1);
        float theta3 = Mathf.Asin(AB/AC);
        Debug.Log("Theta3: " + (theta3 * Mathf.Rad2Deg).ToString("F4"));

        float BC = AC * Mathf.Cos(theta3);
        Debug.Log("BC: " + BC.ToString("F4"));

        float theta2 = theta1 - theta3;
        Debug.Log("Theta2: " + (theta2 * Mathf.Rad2Deg).ToString("F4"));

        Vector3 B = new Vector3(BC * Mathf.Cos(theta2), A.y, BC * Mathf.Sin(theta2));
        Vector3 G = new Vector3(B.x, B.y + Bs, B.z);

        float r = Mathf.Sqrt(BC * BC  + GF * GF);
        Debug.Log("r: " + r.ToString("F4"));

        // Cosine theorem
        float theat6 = Mathf.Acos((FE * FE - ED * ED - r * r) / (- 2 * ED * r));
        float theat7 = Mathf.Acos((r * r - FE * FE - ED * ED) / (- 2 * FE * ED));
        Debug.Log("Theta6: " + (theat6 * Mathf.Rad2Deg).ToString("F4"));
        Debug.Log("Theta7: " + (theat7 * Mathf.Rad2Deg).ToString("F4"));

        float theat5 = Mathf.Atan2(GF, BC);
        float theat4 = theat5 + theat6;
        Debug.Log("Theta4: " + (theat4 * Mathf.Rad2Deg).ToString("F4"));
        Debug.Log("Theta5: " + (theat5 * Mathf.Rad2Deg).ToString("F4"));

        // Set rotations of the bones of the robot
        boneSwing.transform.localEulerAngles = new Vector3(theta2 * Mathf.Rad2Deg, 90f, 90f);
        boneBoom.transform.localEulerAngles = new Vector3(theat4 * Mathf.Rad2Deg,90f,90f);
        boneArm.transform.localEulerAngles = new Vector3(-theat7 * Mathf.Rad2Deg,-90f,90f);
    }
}
