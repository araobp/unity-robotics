using UnityEngine;
using UnityEngine.InputSystem;

public class EndEffectorController : MonoBehaviour
{
    [SerializeField] GameObject hand;
    public float speed = 1.0f; // Speed of movement

    [SerializeField] GameObject fingerL;
    [SerializeField] GameObject fingerR;

    private HingeJoint hingeL;
    private HingeJoint hingeR;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        if (fingerL != null)
            hingeL = fingerL.GetComponent<HingeJoint>();
        if (fingerR != null)
            hingeR = fingerR.GetComponent<HingeJoint>();

        if (hingeL == null && fingerL != null)
            Debug.LogWarning("HingeJoint component not found on fingerL GameObject.", fingerL);

        if (hingeR == null && fingerR != null)
            Debug.LogWarning("HingeJoint component not found on fingerR GameObject.", fingerR);
        else if (hingeR != null && !hingeR.useSpring)
        {
            Debug.LogWarning("HingeJoint on fingerR must have 'Use Spring' enabled for rotation copying to work. Enabling it now.", fingerR);
            hingeR.useSpring = true;
        }

    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if (Keyboard.current == null) return;

        if (Keyboard.current.upArrowKey.isPressed)
        {
            hand.transform.Translate(Vector3.up * speed * Time.deltaTime, Space.World);
        }
        else if (Keyboard.current.downArrowKey.isPressed)
        {
            hand.transform.Translate(Vector3.down * speed * Time.deltaTime, Space.World);
        }

        if (hingeL != null && hingeR != null && hingeR.useSpring)
        {
            var springR = hingeR.spring;
            springR.targetPosition = hingeL.angle;
            hingeR.spring = springR;
        }

    }
}
