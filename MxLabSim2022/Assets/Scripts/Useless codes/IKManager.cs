using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class IKManager : MonoBehaviour
{
    public ArticulationJointController m_root;

    public ArticulationJointController m_end;

    public GameObject m_target;

    public float m_threshold = .05f;
    public float m_rate = 5.0f;

    float CalculateSlope(ArticulationJointController _joint)
    {
        float deltaTheta = .01f;
        float dist1 = GetDistance(m_end.transform.position, m_target.transform.position);
        _joint.RotateTo(deltaTheta);
        float dist2 = GetDistance(m_end.transform.position, m_target.transform.position);
        _joint.RotateTo(-deltaTheta);

        return (dist2 - dist1) / deltaTheta;
    }
    // Update is called once per frame
    void Update()
    {
        if (GetDistance(m_end.transform.position, m_target.transform.position) > m_threshold)
        {
            ArticulationJointController current = m_root;
            while(current != null)
            {
                float slope = CalculateSlope(current);
                current.RotateTo(-slope * m_rate);
            }
        }
    }

    float GetDistance(Vector3 _pointA, Vector3 _pointB)
    {
        return Vector3.Distance(_pointA, _pointB);
    }
}
