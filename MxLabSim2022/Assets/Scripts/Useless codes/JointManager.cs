using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class JointManager : MonoBehaviour
{
    public ArticulationJointController m_URArticulation;
    public JointManager m_child;
    public JointManager GetChild()
    {
        return m_child;
    }

    public void Rotate(float _angle)
    {

        //transform.Rotate(m_URArticulationBody.anchorRotation.eulerAngles * _angle);
    }
}
