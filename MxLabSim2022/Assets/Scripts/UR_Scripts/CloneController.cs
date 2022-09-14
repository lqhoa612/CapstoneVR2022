using UnityEngine;
using Unity.Robotics.UrdfImporter.Control;

public class CloneController : MonoBehaviour
{
    public XRControllerCapture xrCapture;
    public TrajPlanCaller service;

    [HideInInspector] public bool ready = true;
    [HideInInspector] public int selectedIndex;
    [HideInInspector] public float[] q = null;

    public ControlType control = ControlType.PositionControl;
    public float stiffness = 10000;
    public float damping = 1000;
    public float forceLimit = 1000;
    public float speed = 50f; // Units: degree/s
    public float torque = 100f; // Units: Nm or N
    public float acceleration = 5f;// Units: m/s^2 / degree/s^2

    private readonly int[] revoluteJoints = { 2, 3, 4, 5, 6, 7 };
    private ArticulationBody[] artiBodies;
    private MeshRenderer[] meshRenderers;

    void Start()
    {
        this.gameObject.AddComponent<FKRobot>();
        artiBodies = this.GetComponentsInChildren<ArticulationBody>();
        meshRenderers = this.GetComponentsInChildren<MeshRenderer>();
        int defDynamicVal = 10;
        foreach (ArticulationBody joint in artiBodies)
        {
            joint.gameObject.AddComponent<CloneJointControl>();
            joint.jointFriction = defDynamicVal;
            joint.angularDamping = defDynamicVal;
            ArticulationDrive currentDrive = joint.xDrive;
            currentDrive.forceLimit = forceLimit;
            joint.xDrive = currentDrive;
        }
        //ToggleCloneMesh(false);
    }

    void Update()
    {
        if (service.q != null)
        {
            TrajExecute(service.q);

            if (CompareJointAngles(service.q) == true)
            {
                q = service.q;
                service.q = null;
            }
        }
        else
        {
            TrajExecute(GetJointAngles());
        }
    }

    void AutoMove(int jointIndex, float current, float target)
    {
        CloneJointControl joint = artiBodies[revoluteJoints[jointIndex]].GetComponent<CloneJointControl>();
        if (current < target)
            joint.direction = RotationDirection.Positive;
        else if (current > target)
            joint.direction = RotationDirection.Negative;
        else
            joint.direction = RotationDirection.None;
    }

    public bool CompareJointAngles(float[] q)
    {
        int jointReached = 0;
        for (int i = 0; i < q.Length; i++)
        {
            if (q[i] - GetJointAngles()[i] <= 0.3)
            {
                jointReached++;
            }
        }

        return jointReached == 6;
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

    public void UpdateControlType(CloneJointControl joint)
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

    public void ToggleCloneMesh(bool toggle)
    {
        if (toggle == true)
        {
            for (int i = 0; i < meshRenderers.Length; i++)
            {
                meshRenderers[i].enabled = true;
            }
        }

        if (toggle == false)
        {
            for (int i = 0; i < meshRenderers.Length; i++)
            {
                meshRenderers[i].enabled = false;
            }
        }
    }
}
