using UnityEngine;
using System.Threading.Tasks;

public class ParallelGripper : MonoBehaviour
{

    private ArticulationBody _fingerR;
    private ArticulationBody _plateR;
    private ArticulationBody _fingerR2;

    private ArticulationBody _fingerL;
    private ArticulationBody _plateL;
    private ArticulationBody _fingerL2;

    private float _lowerLimit = 0f;
    private float _upperLimit = 0.04f;

    private bool _isMoving = false;
    private float _masterValue;
    private float _targetMasterValue;
    private float _angularVelocity;


    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {        
        _fingerR = transform.Find("EndEffectorBase/Hand/P1R/FingerR").GetComponent<ArticulationBody>();
        _plateR = transform.Find("EndEffectorBase/Hand/P1R/FingerR/P2R/PlateR").GetComponent<ArticulationBody>();
        _fingerR2 = transform.Find("EndEffectorBase/Hand/P1R/FingerR/P2R/PlateR/P3R/FingerR2").GetComponent<ArticulationBody>();
        _fingerL = transform.Find("EndEffectorBase/Hand/P1L/FingerL").GetComponent<ArticulationBody>();
        _plateL = transform.Find("EndEffectorBase/Hand/P1L/FingerL/P2L/PlateL").GetComponent<ArticulationBody>();
        _fingerL2 = transform.Find("EndEffectorBase/Hand/P1L/FingerL/P2L/PlateL/P3L/FingerL2").GetComponent<ArticulationBody>();

        _lowerLimit = _fingerR.xDrive.lowerLimit;
        _upperLimit = _fingerR.xDrive.upperLimit;
        _masterValue = _fingerR.xDrive.target;
    }

    void FixedUpdate()
    {
        if (_isMoving)
        {
            float newMasterValue = Mathf.MoveTowards(_masterValue, _targetMasterValue, _angularVelocity * Time.fixedDeltaTime);
            target = newMasterValue;

            if (Mathf.Approximately(_masterValue, _targetMasterValue))
            {
                _isMoving = false;
            }
        }
    }

    private void SetTarget(ArticulationBody body, float target)
    {
        var drive = body.xDrive;
        drive.target = target;
        body.xDrive = drive;
    }

    public float target
    {
        get { return _masterValue; }
        set
        {
            // set x drive target for right finger
            SetTarget(_fingerR, value);
            SetTarget(_plateR, value);
            SetTarget(_fingerR2, value);
            SetTarget(_fingerL, value);
            SetTarget(_plateL, value);
            SetTarget(_fingerL2, value);
            _masterValue = value;
        }
    }

    public float lowerLimit
    {
        get { return _lowerLimit; }
    }   

    public float upperLimit
    {
        get { return _upperLimit; }
    }

    public async Task open(float angularVelocity)
    {
        _targetMasterValue = lowerLimit;
        _angularVelocity = angularVelocity;
        _isMoving = true;

        while (_isMoving)
        {
            await Task.Yield();
        }
    }

    public async Task close(float angularVelocity)
    {
        _targetMasterValue = upperLimit;
        _angularVelocity = angularVelocity;
        _isMoving = true;

        while (_isMoving)
        {
            await Task.Yield();
        }
    }
}
