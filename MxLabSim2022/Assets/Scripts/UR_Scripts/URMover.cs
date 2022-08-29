using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class URMover : MonoBehaviour
{
    //public Unity.Robotics.UrdfImporter.Control.Controller m_RobotController;
    private readonly int[] m_revolutionJoints = { 2, 3, 4, 5, 6, 7 };
    public float[] targetAngles;

    public IKSolver m_IK;

    private void Start()
    {
        //m_RobotController.SetTrajectory(targetAngles, m_revolutionJoints);
    }

    private void Update()
    {
        if (m_IK.m_jointAngles != null)
        {
            //m_RobotController.SetTrajectory(m_IK.m_jointAngles, m_revolutionJoints);
        }
    }
}
