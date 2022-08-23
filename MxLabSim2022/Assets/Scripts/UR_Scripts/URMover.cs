using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class URMover : MonoBehaviour
{
    public Unity.Robotics.UrdfImporter.Control.Controller m_RobotController;
    private readonly int[] m_revolutionJoints = { 2, 3, 4, 5, 6, 7, 10 };
    public float[] targetAngles;

    

    private void Update()
    {
        m_RobotController.SetTrajectory(targetAngles, m_revolutionJoints);
    }
}
