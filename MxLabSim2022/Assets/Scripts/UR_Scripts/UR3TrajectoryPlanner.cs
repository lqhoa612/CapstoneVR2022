//using RosMessageTypes.Geometry;
//using RosMessageTypes.HoaUnityRos;
//using System.Collections;
//using System.Linq;
//using Unity.Robotics.ROSTCPConnector;
//using Unity.Robotics.ROSTCPConnector.ROSGeometry;
//using UnityEngine;

//public class UR3TrajectoryPlanner : MonoBehaviour
//{
//    // Hardcoded variables
//    const int k_NumRobotJoints = 6;
//    const float k_JointAssignmentWait = .1f;
//    const float k_PoseAssignmentWait = .5f;

//    // Variables required for ROS communication
//    [SerializeField] string m_RosServiceName = "ur3_moveit";

//    public GameObject m_UR;
//    public GameObject m_Target;
//    public GameObject m_Goal;

//    // Assures that the gripper is always positioned above the m_Target cube before grasping.
//    readonly Vector3 m_PickPoseOffset = Vector3.up * .1f;
//    readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);

//    // Articulation Bodies
//    ArticulationBody[] m_JointArticulationBodies;

//    // ROS Connector
//    ROSConnection m_Ros;

//    /// <summary>
//    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
//    ///     Find left and right finger joints and assign them to their respective articulation body objects.
//    /// </summary>
//    void Start()
//    {
//        // Get ROS connection static instance
//        m_Ros = ROSConnection.GetOrCreateInstance();
//        m_Ros.RegisterRosService<MoverServiceRequest, MoverServiceResponse>(m_RosServiceName);

//        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

//        var linkName = string.Empty;
//        for (var i = 0; i < k_NumRobotJoints; i++)
//        {
//            linkName += SourceDestinationPublisher.LinkNames[i];
//            m_JointArticulationBodies[i] = m_UR.transform.Find(linkName).GetComponent<ArticulationBody>();
//        }
//    }

//    /// <summary>
//    ///     Get the current values of the robot's joint angles.
//    /// </summary>
//    /// <returns>NiryoMoveitJoints</returns>
//    UR3MoveitJointsMsg CurrentJointConfig()
//    {
//        var joints = new UR3MoveitJointsMsg();

//        for (var i = 0; i < k_NumRobotJoints; i++)
//        {
//            joints.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
//        }

//        return joints;
//    }

//    /// <summary>
//    ///     Create a new MoverServiceRequest with the current values of the robot's joint angles,
//    ///     the target cube's current position and rotation, and the targetPlacement position and rotation.
//    ///     Call the MoverService using the ROSConnection and if a trajectory is successfully planned,
//    ///     execute the trajectories in a coroutine.
//    /// </summary>
//    public void PublishJoints()
//    {
//        var request = new MoverServiceRequest();
//        request.joints_input = CurrentJointConfig();

//        // Pick Pose
//        request.pick_pose = new PoseMsg
//        {
//            position = (m_Target.transform.position + m_PickPoseOffset).To<FLU>(),

//            // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
//            orientation = Quaternion.Euler(90, m_Target.transform.eulerAngles.y, 0).To<FLU>()
//        };

//        // Place Pose
//        request.place_pose = new PoseMsg
//        {
//            position = (m_Goal.transform.position + m_PickPoseOffset).To<FLU>(),
//            orientation = m_PickOrientation.To<FLU>()
//        };

//        m_Ros.SendServiceMessage<MoverServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
//    }

//    void TrajectoryResponse(MoverServiceResponse response)
//    {
//        if (response.trajectories.Length > 0)
//        {
//            Debug.Log("Trajectory returned.");
//            StartCoroutine(ExecuteTrajectories(response));
//        }
//        else
//        {
//            Debug.LogError("No trajectory returned from MoverService.");
//        }
//    }

//    /// <summary>
//    ///     Execute the returned trajectories from the MoverService.
//    ///     The expectation is that the MoverService will return four trajectory plans,
//    ///     PreGrasp, Grasp, PickUp, and Place,
//    ///     where each plan is an array of robot poses. A robot pose is the joint angle values
//    ///     of the six robot joints.
//    ///     Executing a single trajectory will iterate through every robot pose in the array while updating the
//    ///     joint values on the robot.
//    /// </summary>
//    /// <param name="response"> MoverServiceResponse received from niryo_moveit mover service running in ROS</param>
//    /// <returns></returns>
//    IEnumerator ExecuteTrajectories(MoverServiceResponse response)
//    {
//        if (response.trajectories != null)
//        {
//            // For every trajectory plan returned
//            for (var poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
//            {
//                // For every robot pose in trajectory plan
//                foreach (var t in response.trajectories[poseIndex].joint_trajectory.points)
//                {
//                    var jointPositions = t.positions;
//                    var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

//                    // Set the joint values for every joint
//                    for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
//                    {
//                        var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
//                        joint1XDrive.target = result[joint];
//                        m_JointArticulationBodies[joint].xDrive = joint1XDrive;
//                    }

//                    // Wait for robot to achieve pose for all joint assignments
//                    yield return new WaitForSeconds(k_JointAssignmentWait);
//                }

//                // Wait for the robot to achieve the final pose from joint assignment
//                yield return new WaitForSeconds(k_PoseAssignmentWait);
//            }
//        }
//    }

//    enum Poses
//    {
//        PreGraps,
//        Graps,
//        PickUp,
//        Place
//    }
//}
