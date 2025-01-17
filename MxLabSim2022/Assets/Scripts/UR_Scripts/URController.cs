using Unity.Robotics.UrdfImporter.Control;
using UnityEngine;
using UnityEngine.SceneManagement;

public class URController : MonoBehaviour
{
    public XRControllerCapture xrCapture;
    public CloneController cloneController;

    [HideInInspector] public float jointInput;
    [HideInInspector] public int selectedIndex;

    [InspectorReadOnly(hideInEditMode = true)] public string selectedJoint;

    [HideInInspector] public ControlMode mode;
    [HideInInspector] public bool ready = true;
    [HideInInspector] public float[] q = null;
    [HideInInspector] public string collisionMsg = null;

    public ControlType control = ControlType.PositionControl;
    public float stiffness = 10000;
    public float damping = 1000;
    public float forceLimit = 1000;
    public float speed = 20f; // Units: degree/s
    public float torque = 100f; // Units: Nm or N
    public float acceleration = 5f;// Units: m/s^2 / degree/s^2

    private readonly int[] revoluteJoints = { 2, 3, 4, 5, 6, 7 };
    private float timerA, timerB;
    private ArticulationBody[] artiBodies;

    public float[] test_q = { 0, 0, 0, 0, 0, 0 };

    void Start()
    {
        if (SceneManager.GetActiveScene().name == "URScene") mode = ControlMode.Manual;
        else if (SceneManager.GetActiveScene().name == "URAutoScene") mode = ControlMode.Auto;
        else mode = ControlMode.Stop;
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

    }

    void Update()
    {
        switch (mode)
        {
            case ControlMode.Manual:
                jointInput = xrCapture.rightJoy.x;
                JointIndexNav();
                DisplaySelectedJoint(selectedIndex);
                JointMover(selectedIndex);
                break;

            case ControlMode.Auto:
                if (q != null)
                {
                    TrajExecute(q);
                }
                else
                {
                    TrajExecute(GetJointAngles());
                }

                break;

            case ControlMode.Pause:
                StopAll();
                if (xrCapture.AisPressed == true)
                {
                    ResetRobot();
                }
                if (xrCapture.BisPressed == true)
                {
                    if (SceneManager.GetActiveScene().name == "URScene") mode = ControlMode.Manual;
                    if (SceneManager.GetActiveScene().name == "URAutoScene") mode = ControlMode.Auto;
                }
                break;

            case ControlMode.Stop:
                StopAll();
                break;

            default:
                break;
        }
    }

    void JointIndexNav()
    {
        if (selectedIndex >= revoluteJoints.Length) selectedIndex = 0;
        if (selectedIndex < 0) selectedIndex = revoluteJoints.Length - 1;

        if (xrCapture.BisPressed == true && timerB == 0.00f)
        {
            timerB += Time.deltaTime;
            selectedIndex++;
        }
        if (xrCapture.BisPressed == false) timerB = 0;

        if (xrCapture.AisPressed == true && timerA == 0.00f)
        {
            timerA += Time.deltaTime;
            selectedIndex--;
        }
        if (xrCapture.AisPressed == false) timerA = 0;
    }

    void DisplaySelectedJoint(int index)
    {
        if (index < 0 || index >= revoluteJoints.Length) return;
        selectedJoint = artiBodies[revoluteJoints[index]].name + " (" + index + " )";
    }

    void JointMover(int index)
    {
        if (index < 0 || index >= revoluteJoints.Length) return;
        URJointControl joint = artiBodies[revoluteJoints[index]].GetComponent<URJointControl>();

        if (jointInput > 0) joint.direction = RotationDirection.Positive;
        else if (jointInput < 0) joint.direction = RotationDirection.Negative;
        else joint.direction = RotationDirection.None;
    }

    //void GripMover()
    //{
    //    // Get right gripper parts
    //    URJointControl rightInner = artiBodies[10].GetComponent<URJointControl>();
    //    URJointControl rightOuter = artiBodies[11].GetComponent<URJointControl>();
    //    URJointControl rightFinger = artiBodies[13].GetComponent<URJointControl>();
    //    // Get left gripper parts
    //    URJointControl leftInner = artiBodies[15].GetComponent<URJointControl>();
    //    URJointControl leftOuter = artiBodies[16].GetComponent<URJointControl>();
    //    URJointControl leftFinger = artiBodies[18].GetComponent<URJointControl>();

    //    if (gripInput >= 0.9)
    //    {
    //        rightInner.direction = RotationDirection.Positive;
    //        leftInner.direction = RotationDirection.Positive;

    //        rightOuter.direction = RotationDirection.Positive;
    //        leftOuter.direction = RotationDirection.Positive;

    //        rightFinger.direction = RotationDirection.Positive;
    //        leftFinger.direction = RotationDirection.Positive;
    //    }
    //    else if (gripInput <= 0.1)
    //    {
    //        rightInner.direction = RotationDirection.Negative;
    //        leftInner.direction = RotationDirection.Negative;

    //        rightOuter.direction = RotationDirection.Negative;
    //        leftOuter.direction = RotationDirection.Negative;

    //        rightFinger.direction = RotationDirection.Negative;
    //        leftFinger.direction = RotationDirection.Negative;
    //    }
    //    else
    //    {
    //        rightInner.direction = RotationDirection.None;
    //        leftInner.direction = RotationDirection.None;

    //        rightOuter.direction = RotationDirection.None;
    //        leftOuter.direction = RotationDirection.None;

    //        rightFinger.direction = RotationDirection.None;
    //        leftFinger.direction = RotationDirection.None;
    //    }
    //}

    void AutoMove(int jointIndex, float current, float target)
    {
        URJointControl joint = artiBodies[revoluteJoints[jointIndex]].GetComponent<URJointControl>();

        if (current < target)
        {
            if (Mathf.Abs(target - current) > 30f) speed = 50f;
            else speed = 10f;
            joint.direction = RotationDirection.Positive;
        }

        else if (current > target)
        {
            if (Mathf.Abs(current - target) > 30f) speed = 50f;
            else speed = 10f;
            joint.direction = RotationDirection.Negative;
        }

        else
            joint.direction = RotationDirection.None;
    }

    void ResetRobot()
    {
        speed = 20f;
        TrajExecute(new float[] { 0, 0, 0, 0, 0, 0 });
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

    public void StopAll()
    {
        speed = 0;
    }

    public string GetJointName()
    {
        return artiBodies[revoluteJoints[selectedIndex]].name;
    }

    public enum ControlMode
    {
        Manual,
        Auto,
        Pause,
        Stop
    }

    public void SetControlMode(string modeName)
    {
        switch (modeName)
        {
            case "Manual":
                mode = ControlMode.Manual;
                break;
            case "Auto":
                mode = ControlMode.Auto;
                break;
            default:
                mode = ControlMode.Manual;
                break;
        }
    }

}
