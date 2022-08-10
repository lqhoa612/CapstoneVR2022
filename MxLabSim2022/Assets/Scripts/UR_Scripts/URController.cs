using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class URController : MonoBehaviour
{
    public Unity.Robotics.UrdfImporter.Control.Controller m_RobotController;
    public XRControllerCapture m_XRCapture;

    [HideInInspector] public int _index;
    private float timerA, timerB;

    // Start is called before the first frame update
    void OnEnable()
    {
        _index = 2;
    }

    // Update is called once per frame
    void Update()
    {
        if (_index < 2) _index = 7;
        if (_index > 7) _index = 2;

        m_RobotController.selectedIndex = _index;
        m_RobotController.moveDirection = m_XRCapture.rightJoy.x;

        if (m_XRCapture.BisPressed == true && timerB == 0.00f)
        {
            timerB += Time.deltaTime;
            _index++;
        }
        if (m_XRCapture.BisPressed == false) timerB = 0.00f;

        if (m_XRCapture.AisPressed == true && timerA == 0.00f)
        {
            timerA += Time.deltaTime;
            _index--;
        }
        if (m_XRCapture.AisPressed == false) timerA = 0.00f;


    }

    public string CurrentJointName(int jointIndex)
    {
        switch (jointIndex)
        {
            case 2: return "Shoulder";
            case 3: return "Upper Arm";
            case 4: return "Fore Arm";
            case 5: return "Wrist 1";
            case 6: return "Wrist 2";
            case 7: return "Wrist 3";
        }
        return "Shoulder";
    }

    
}
