using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class URPosRot : MonoBehaviour
{
    [SerializeReference] GameObject UR_TrackPoint;

    //private Vector3 pos;
    //private Quaternion rot;
    //void FixedUpdate()
    //{
    //    if (pos != UR_TrackPoint.transform.position)
    //        Debug.Log(UR_TrackPoint.transform.position);

    //    if (rot != UR_TrackPoint.transform.rotation)
    //    Debug.Log(UR_TrackPoint.transform.rotation);

    //    pos = UR_TrackPoint.transform.position;
    //    rot = UR_TrackPoint.transform.rotation;
    //}

    public Vector3 GetPosition()
    {
        return UR_TrackPoint.transform.position;
    }

    public Quaternion GetRotation()
    {
        return UR_TrackPoint.transform.rotation;
    }
}
