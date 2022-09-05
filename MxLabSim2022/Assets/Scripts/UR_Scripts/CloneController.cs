using UnityEngine;
using Unity.Robotics.UrdfImporter.Control;

public class CloneController : MonoBehaviour
{
    public XRControllerCapture xrCapture;
    public TrajPlanCaller service;

    [HideInInspector] public float jointInput, gripInput;
    [HideInInspector] public int selectedIndex;
    [HideInInspector] public bool ready;
    [InspectorReadOnly(hideInEditMode = true)] public string selectedJoint;
    public ControlMode mode;
    private readonly int[] revoluteJoints = { 2, 3, 4, 5, 6, 7 };
    private ArticulationBody[] artiBodies;

    public ControlType control = ControlType.PositionControl;
    public float stiffness = 10000;
    public float damping = 1000;
    public float forceLimit = 1000;
    public float speed = 20f; // Units: degree/s
    public float torque = 100f; // Units: Nm or N
    public float acceleration = 5f;// Units: m/s^2 / degree/s^2

    [InspectorReadOnly(hideInEditMode = true)] public float[] q;

    void Awake()
    {
        ready = false;
        mode = ControlMode.Auto; //for testing
        this.gameObject.AddComponent<FKRobot>();
        artiBodies = this.GetComponentsInChildren<ArticulationBody>();
        int defDynamicVal = 10;
        foreach (ArticulationBody joint in artiBodies)
        {
            joint.gameObject.AddComponent<URJointControl>();
            joint.jointFriction = defDynamicVal;
            joint.angularDamping = defDynamicVal;
            ArticulationDrive currentDrive = joint.xDrive;
            currentDrive.forceLimit = forceLimit;
            joint.xDrive = currentDrive;
        }
        if (q == null) q = GetJointAngles();
    }

    void Update()
    {
        if (mode == ControlMode.Auto)
        {
            if (xrCapture.rightTrigger == true) service.CallService();
            if (q != null)
                TrajExecute(q);
            else
                TrajExecute(GetJointAngles());

            if (GetJointAngles() == q) ready = true; 
        }
    }

    void AutoMove(int jointIndex, float current, float target)
    {
        URJointControl joint = artiBodies[revoluteJoints[jointIndex]].GetComponent<URJointControl>();
        if (current < target)
            joint.direction = RotationDirection.Positive;
        else if (current > target)
            joint.direction = RotationDirection.Negative;
        else
            joint.direction = RotationDirection.None;
    }

    public void TrajExecute(float[] targets)
    {
        for (int i = 0; i < revoluteJoints.Length; i++)
        {
            AutoMove(i, GetJointAngles()[i], targets[i]);
        }
    }

    public float[] GetJointAngles()
    {
        float[] currentAngles = { 0, 0, 0, 0, 0, 0 };
        for (int i = 0; i < revoluteJoints.Length; i++)
        {
            currentAngles[i] = artiBodies[revoluteJoints[i]].xDrive.target;
        }

        return currentAngles;
    }

    public void UpdateControlType(URJointControl joint)
    {
        joint.controltype = control;
        if (control == ControlType.PositionControl)
        {
            ArticulationDrive drive = joint.joint.xDrive;
            drive.stiffness = stiffness;
            drive.damping = damping;
            joint.joint.xDrive = drive;
        }
    }

    public enum ControlMode
    {
        Auto
    }
}
