using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class URController : MonoBehaviour
{
    public Unity.Robotics.UrdfImporter.Control.Controller m_RobotController;
    public XRControllerCapture m_XRCapture;

    [HideInInspector] public int m_index;

    private float m_timerA, m_timerB;
    private readonly int[] m_revolutionJoints = {2,3,4,5,6,7,10,11,13,15,16,18};

    // Update is called once per frame
    void Update()
    {
        if (m_index < 0) m_index = m_revolutionJoints.Length - 1;
        if (m_index > m_revolutionJoints.Length - 1) m_index = 0; 

        m_RobotController.selectedIndex = m_revolutionJoints[m_index];
        m_RobotController.moveDirection = m_XRCapture.rightJoy.x;

        if (m_XRCapture.BisPressed == true && m_timerB == 0.00f)
        {
            m_timerB += Time.deltaTime;
            m_index++;
        }
        if (m_XRCapture.BisPressed == false) m_timerB = 0.00f;

        if (m_XRCapture.AisPressed == true && m_timerA == 0.00f)
        {
            m_timerA += Time.deltaTime;
            m_index--;
        }
        if (m_XRCapture.AisPressed == false) m_timerA = 0.00f;


    }

    public string CurrentJointName(int jointIndex)
    {
        return jointIndex switch
        {
            2 => "Shoulder",
            3 => "Upper Arm",
            4 => "Fore Arm",
            5 => "Wrist 1",
            6 => "Wrist 2",
            7 => "Wrist 3",
            10 => "Right Inner Knuckle",
            11 => "Right Outer Knuckle",
            13 => "Right Inner Finger",
            15 => "Left Inner Knuckle",
            16 => "Left Outer Knuckle",
            18 => "Left Inner Finger",
            _ => "Shoulder",
        };
    }
    
}
